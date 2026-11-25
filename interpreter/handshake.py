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
"""A station joins the AP of a corerev 42 ucode, and stays or leaves.

Authentication, association and the four EAPOL-Key frames, as a host and a
station would exchange them: the station's frames reach the RX engine, the
host's are queued with b43-next's AC TX header (legacy 6 Mb/s), and each side
answers the other's unicast frames with an ACK. Then the station listens for
beacons. It hears only the primary 20 MHz channel, so a beacon on another
sub-band (vchip's chip.beacons) is lost to it; after --lost-ms without one it
leaves, as the laptop of b43-next#43 did a second after the association.
The frames' contents are not checked: no key is derived, the EAPOL bodies
are zeros of the usual lengths.

    handshake.py FWDIR UCODE [--primary N] [--word1 V] [--lost-ms 1000]

--primary is the primary's sub-band in the 80 MHz block, as it stands in the
channel word; --word1 is the beacon's PHY control word 1 (shared memory
0x00CE), by default as b43-next left it, the sub-band bits 0.
"""

import argparse
import struct
import sys
import zlib

import rxe
import rxframe

ME, PEER = rxframe.ME, rxframe.PEER
BSS_WORD1, CHAN_WORD = 0xCE >> 1, 0xA0 >> 1
STEPS = 400000


def plcp(length):
    return struct.pack("<I", 0xB | length << 5) + b"\0\0"


def mpdu(fc, a1, a2, a3, seq, body):
    return struct.pack("<HH", fc, 0x2C) + a1 + a2 + a3 + struct.pack("<H", seq << 4) + body


def with_fcs(m):
    return m + struct.pack("<I", zlib.crc32(m))


def tx_header(frame, cookie, primary, chanspec):
    """b43-next's b43_txhdr_ac for a frame at legacy 6 Mb/s, one attempt rate."""
    n = len(frame) + 4
    rate = struct.pack("<3H", 0x00C1, primary, 0x000B) + plcp(n) + struct.pack("<4H", 0, 12, 0x0020, 0)
    hdr = bytes([0x02, 0, 0x02, 0]) + struct.pack(
        "<4HBB5H", 0, 0x4000 | 0x0080 | 0x0100, 0x0002, chanspec, 24, 0, n, cookie, 0, 0, 0)
    hdr += rate + bytes(4 * 20 - len(rate))
    return hdr + bytes(128 - len(hdr))


class Join:
    def __init__(self, fwdir, ucode, primary, word1):
        self.chip, corerev = rxframe.setup(fwdir, ucode)
        if corerev < 40:
            raise SystemExit("handshake.py: the AC TX header, corerev 40+")
        self.primary = primary
        shm = self.chip.m.shm
        self.chanspec = 0xE02A & ~0x0700 | primary << 8
        shm[CHAN_WORD] = self.chanspec
        shm[BSS_WORD1] = word1
        self.seq, self.cookie, self.tx = 0, 0, []
        self.answer = False             # answer the next frame sent with an ACK

        class Rec(rxframe.vchip.Recorder):
            def tx_condition(_, sel):
                self.tx.append((self.chip.clock, sel))
                if sel == 0x22 and self.answer:
                    # the station's ACK, SIFS after the frame's end: queued
                    # here, as a ucode that busy-waits keeps chip.wait() in
                    self.answer = False
                    ack = with_fcs(struct.pack("<HH", 0x00D4, 0) + ME)
                    self.chip.rxe.rx_frame(rxe.RxFrame(plcp(len(ack)) + ack, ra_match=True, bss_match=True,
                                                       plcp_us=10, air_us=34), at=self.chip.clock + 16 * self.chip.mhz)

        self.chip.recorder, self.chip.recording = Rec(), True

    def from_station(self, what, fc, a1, a3, body):
        """A frame of the station; returns whether the host got it and an ACK
        was sent."""
        c = self.chip
        self.seq += 1
        n_push, n_tx = len(c.rxe.fifo), len(self.tx)
        m = with_fcs(mpdu(fc, a1, PEER, a3, self.seq, body))
        f = rxe.RxFrame(plcp(len(m)) + m, ra_match=True, bss_match=True, plcp_us=20)
        c.rxe.rx_frame(f, at=c.clock + 50 * c.mhz)
        c.wait(50 + f.air() + 200, STEPS)
        got = len(c.rxe.fifo) == n_push + 1
        acked = any(sel == 0x22 for _t, sel in self.tx[n_tx:])
        return self.step(what, "->", got and acked,
                         f"pushed {len(c.rxe.fifo) - n_push}, ACK {'sent' if acked else 'missing'}")

    def from_host(self, what, fc, a1, a3, body):
        """A frame of the host; the station answers it with an ACK. Returns
        whether the ucode reports it acknowledged."""
        c = self.chip
        self.seq += 1
        m = mpdu(fc, a1, ME, a3, self.seq, body)
        self.cookie += 1
        n_st = len(c.txe.status)
        c.txe.tx_frame(3, tx_header(m, 0x4000 | self.cookie, self.primary, self.chanspec) + m)
        end = c.clock + 6000 * c.mhz
        self.answer = True
        while c.clock < end and len(c.txe.status) == n_st:
            c.wait(10, STEPS)
        self.answer = False
        st = c.txe.status[n_st:]
        acked = bool(st) and bool(st[0].words[0] & 0x8000)
        return self.step(what, "<-", acked,
                         f"status {' '.join(f'{w:08x}' for w in st[0].words[:2]) if st else 'none'}")

    def step(self, what, way, ok, detail):
        print(f"  {way} {what:22s} {'ok' if ok else 'FAIL':5s} {detail}")
        self.ok = self.ok and ok
        return ok

    def join(self):
        self.ok = True
        def eapol(n):
            return bytes([0xAA, 0xAA, 3, 0, 0, 0, 0x88, 0x8E, 2, 3]) + bytes(n - 10)
        self.from_station("authentication", 0x00B0, ME, ME, bytes(6))
        self.from_host("authentication", 0x00B0, PEER, ME, bytes(6))
        self.from_station("association request", 0x0000, ME, ME, bytes(60))
        self.from_host("association response", 0x0010, PEER, ME, bytes(80))
        self.from_host("EAPOL-Key 1 of 4", 0x0208, PEER, ME, eapol(99 + 8))
        self.from_station("EAPOL-Key 2 of 4", 0x0108, ME, ME, eapol(121 + 8))
        self.from_host("EAPOL-Key 3 of 4", 0x0208, PEER, ME, eapol(155 + 8))
        self.from_station("EAPOL-Key 4 of 4", 0x0108, ME, ME, eapol(99 + 8))
        return self.ok

    def listen(self, lost_ms):
        """Beacons after the join until the station leaves or 3 windows passed."""
        c = self.chip
        t0 = last = c.clock
        heard = sent = 0
        while c.clock < t0 + 3 * lost_ms * 1000 * c.mhz:
            n = len(c.beacons)
            c.wait(102400, STEPS * 50)
            for _t, sb, pri in c.beacons[n:]:
                sent += 1
                if sb == pri:
                    heard, last = heard + 1, _t
            if c.clock - last > lost_ms * 1000 * c.mhz:
                return f"left after {(c.clock - t0) / c.mhz / 1000:.0f} ms, {heard} of {sent} beacons heard"
        return f"stayed, {heard} of {sent} beacons heard"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("fwdir")
    ap.add_argument("ucode")
    ap.add_argument("--primary", type=int, default=0)
    ap.add_argument("--word1", type=lambda x: int(x, 0))
    ap.add_argument("--lost-ms", type=int, default=1000)
    args = ap.parse_args()
    j = Join(args.fwdir, args.ucode, args.primary, args.word1 if args.word1 is not None else 0x0040)
    print(f"{args.ucode}: primary sub-band {args.primary}, 0x00CE {j.chip.m.shm[BSS_WORD1]:#06x}")
    ok = j.join()
    print(f"  4-way handshake: {'complete' if ok else 'INCOMPLETE'}")
    print(f"  station: {j.listen(args.lost_ms)}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
