#!/usr/bin/env python3
#
#   Copyright (C) 2026  b43-tools contributors
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License version 2
#   as published by the Free Software Foundation.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
"""Queue a frame in a TX FIFO of the virtual chip (txe.py) and show what the
ucode did with it, up to the TX status it reports.

The frame is a TX header and frame as b43-next queued them, taken from a
capture: lines "b43txhcap <phy> rev R ring N hdr H len L", "h<off> <hex>"
and "data <hex>" (the first bytes of the frame, padded with zeros to L).
fwmatrix.py's beacon setup and rxframe.py's host side come first. With
--ack the peer acknowledges: an ACK frame reaches the chip after the frame
went out. With --ampdu N (corerev 40+) N copies of the frame are queued as
b43_txhdr_ac_ampdu() marks an MPDU it may aggregate, each with its own frame
ID and sequence number, and with --ack the peer answers with a compressed
Block Ack (--ba-bitmap, default all N). Printed: the TX engine's events, the
conditions it raised, the TX status words, the SHM counters that moved.

    txframe.py FWDIR UCODE CAPTURE [--index I] [--ack] [--us 2000] [--rev R]
               [--ampdu N [--ba-bitmap B]]
"""

import argparse
import re
import struct
import zlib

import fwmatrix
import rxe
import rxframe


def captured(path, corerev):
    """The (ring, header, frame) of each TX header capture of corerev."""
    head = re.compile(r"b43txhcap (\S+) rev (\d+) ring (\d+) hdr (\d+) len (\d+)")
    out, cur = [], None
    for line in open(path):
        m = head.search(line)
        if m:
            cur = None
            if int(m.group(2)) == corerev:
                cur = [int(m.group(3)), b"", int(m.group(5))]
                out.append(cur)
            continue
        if cur is None:
            continue
        m = re.search(r"b43txhcap\s+h[0-9a-f]{3} ([0-9a-f]+)", line)
        if m:
            cur[1] += bytes.fromhex(m.group(1))
        m = re.search(r"b43txhcap\s+data ([0-9a-f]+)", line)
        if m:
            d = bytes.fromhex(m.group(1))
            cur[2] = d + bytes(cur[2] - len(d))
            cur = None
    return [(r, h, f) for r, h, f in out if isinstance(f, bytes)]


def ack_frame(ra):
    mpdu = struct.pack("<HH", 0x00D4, 0) + ra
    mpdu += struct.pack("<I", zlib.crc32(mpdu))
    return struct.pack("<I", 0xB | len(mpdu) << 5) + b"\0\0" + mpdu


def block_ack(ra, ta, tid, ssn, bitmap):
    """A compressed Block Ack from ta to ra."""
    mpdu = struct.pack("<HH", 0x0094, 0) + ra + ta
    mpdu += struct.pack("<HHQ", 0x0004 | tid << 12, ssn << 4, bitmap)
    mpdu += struct.pack("<I", zlib.crc32(mpdu))
    return struct.pack("<I", 0xB | len(mpdu) << 5) + b"\0\0" + mpdu


def ampdu_copies(hdr, frame, n):
    """n copies of an AC TX header and QoS data frame marked as b43-next marks
    an MPDU it may aggregate (b43_txhdr_ac_ampdu(); SVHT only off one), with
    consecutive frame IDs and sequence numbers."""
    seq0 = struct.unpack_from("<H", frame, 22)[0] >> 4
    out = []
    for k in range(n):
        h, f = bytearray(hdr), bytearray(frame)
        lo, hi = struct.unpack_from("<HH", h, 6)
        struct.pack_into("<HH", h, 6, lo | 0x0040, hi & ~0x0004)   # AMPDU on, SVHT off
        cookie = struct.unpack_from("<H", h, 16)[0]
        struct.pack_into("<HH", h, 16, (cookie & 0xF000) | ((cookie + k) & 0x0FFF), (seq0 + k) << 4)
        struct.pack_into("<H", f, 22, ((seq0 + k) & 0xFFF) << 4 | (f[22] & 0xF))
        h[106] = h[107] = 32                                        # mpdu_max, primary and fallback
        struct.pack_into("<H", h, 108, 0x152 << 4 | 6)              # ampdu_dur, density 8 us
        h[110], h[111] = 63, 16                                     # ba_win, max_agg_len
        out.append(bytes(h) + bytes(f))
    return out, seq0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("fwdir")
    ap.add_argument("ucode")
    ap.add_argument("capture")
    ap.add_argument("--index", type=int, default=0)
    ap.add_argument("--ack", action="store_true")
    ap.add_argument("--us", type=int, default=2000)
    ap.add_argument("--rev", type=int, help="take the frames of this corerev from the capture "
                    "(default the chip's; b43-next's TX header is the same on the pre-AC cores)")
    ap.add_argument("--ampdu", type=int, metavar="N",
                    help="queue N aggregatable copies (corerev 40+, a QoS data frame)")
    ap.add_argument("--ba-bitmap", type=lambda x: int(x, 0), metavar="B",
                    help="the Block Ack's bitmap from the first MPDU on (default all N)")
    args = ap.parse_args()

    chip, corerev = rxframe.setup(args.fwdir, args.ucode)
    rev = args.rev if args.rev is not None else corerev
    frames = captured(args.capture, rev)
    if not frames:
        raise SystemExit(f"no corerev {rev} TX header in {args.capture}")
    ring, hdr, frame = frames[args.index]
    tx = []

    class Rec(rxframe.vchip.Recorder):
        def tx_condition(self, sel):
            tx.append((chip.clock, sel))

    chip.recorder, chip.recording = Rec(), True
    t0 = chip.clock
    counters = list(chip.m.shm[0x70:0xA0])
    if corerev >= 40:
        # the station's address is the TA of the frame it sends
        rxframe.amt_row(chip, 0x3F, frame[10:16], 0x8008)
    if args.ampdu:
        if corerev < 40:
            raise SystemExit("--ampdu: the AC TX header, corerev 40+")
        copies, seq0 = ampdu_copies(hdr, frame, args.ampdu)
        for data in copies:
            chip.txe.tx_frame(ring, data)
        bitmap = args.ba_bitmap if args.ba_bitmap is not None else (1 << args.ampdu) - 1
        reply = block_ack(frame[10:16], frame[4:10], frame[24] & 0xF, seq0, bitmap)
    else:
        chip.txe.tx_frame(ring, hdr + frame)
        reply = ack_frame(frame[10:16])
    if args.ack:
        def acked():
            # the frame went out: the FIFO engine handed it to the PHY, or
            # (corerev 40+) the chip's TX engine finished it; an A-MPDU once
            # no MPDU has finished for 5 us
            done = [c for c, sel in tx if sel == 0x22]
            gone = chip.txe.sent or (done and (not args.ampdu or chip.clock - done[-1] > 5 * chip.mhz))
            if gone and not chip.rxe.busy():
                chip.rxe.rx_frame(rxe.RxFrame(reply, ra_match=True, bss_match=True, plcp_us=10,
                                              air_us=44 if args.ampdu else 34),
                                  at=chip.clock + 16 * chip.mhz)
                return True
            return False
        end = chip.clock + args.us * chip.mhz
        while chip.clock < end and not acked():
            chip.wait(10, 300000)
        chip.wait(max(0, (end - chip.clock) / chip.mhz), 300000)
    else:
        chip.wait(args.us, 300000)
    us = lambda clock: (clock - t0) / chip.mhz
    print(f"{args.ucode}: FIFO {ring}, TX header {len(hdr)} bytes, frame {len(frame)} bytes, "
          f"fc {frame[0] | frame[1] << 8:#06x}")
    for clock, event in chip.txe.log + [(c, f"TX condition {s:#04x}") for c, s in tx] + \
            [(c, f"RX {e}") for c, e in chip.rxe.log]:
        print(f"  {us(clock):8.2f} us  {event}")
    for st in chip.txe.status:
        print("  TX status " + " ".join(f"{w:08x}" for w in st.words))
    print(f"  SHM counters: {rxframe.moved(chip, counters) or '-'}")
    print(f"  PSM {'stopped: ' + chip.stopped if chip.stopped else f'at {chip.m.pc:#06x}'}")


if __name__ == "__main__":
    main()
