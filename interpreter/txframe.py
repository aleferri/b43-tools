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
went out. Printed: the TX engine's events, the conditions it raised, the
TX status words, the SHM counters that moved.

    txframe.py FWDIR UCODE CAPTURE [--index I] [--ack] [--us 2000] [--rev R]
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
    chip.txe.tx_frame(ring, hdr + frame)
    if args.ack:
        def acked():
            if chip.txe.sent and not chip.rxe.busy():
                chip.rxe.rx_frame(rxe.RxFrame(ack_frame(frame[10:16]), ra_match=True,
                                              bss_match=True, plcp_us=10, air_us=34),
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
        print(f"  TX status {st.words[0]:08x} {st.words[1]:08x}")
    print(f"  SHM counters: {rxframe.moved(chip, counters) or '-'}")
    print(f"  PSM {'stopped: ' + chip.stopped if chip.stopped else f'at {chip.m.pc:#06x}'}")


if __name__ == "__main__":
    main()
