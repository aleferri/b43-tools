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
"""Receive frames on the virtual chip (rxe.py) and show what the ucode did.

fwmatrix.py's beacon setup brings the ucode to its idle AP loop, then the
host side as b43-next leaves it: the station address in the AMT's SELF and
BSSID rows (corerev 40+, b43_amt_write()), SHM word 0x10
(B43_SHM_SH_MAXRXFRMLEN) 2352, word 0x50 (B43_SHM_SH_CHAN) the channel, and
the copy length 0xB4 where the initvals leave it out (corerev 11). A frame
from a peer to that address (or --ra) arrives, its BSSID (addr3) the
station's: an OFDM PLCP at 6 Mb/s, the MPDU and its FCS; --then adds more.
Printed: the RX engine's events in microseconds, the conditions the TX
engine raised, the frames pushed to the FIFO, the SHM counters that moved
(brcmsmac's struct macstat from word 0x70) and the PCs executed.

    rxframe.py FWDIR UCODE [--fc 0x0108] [--len 40] [--bad-fcs] [--ra ADDR]
               [--cond SEL ...] [--then RA ...] [--us 400]
               [--initvals FILE ... --phy-type T]
"""

import argparse
import os
import struct
import zlib

import core
import fwmatrix
import rxe
import vchip

# brcmsmac's struct macstat, from SHM word 0x70: the counters the ucode moves.
MACSTAT = ("txallfrm txrtsfrm txctsfrm txackfrm txdnlfrm txbcnfrm " + "txfunfl " * 8 +
           "txtplunfl txphyerr pktengrxducast pktengrxdmcast rxfrmtoolong rxfrmtooshrt "
           "rxinvmachdr rxbadfcs rxbadplcp rxcrsglitch rxstrt rxdfrmucastmbss "
           "rxmfrmucastmbss rxcfrmucast rxrtsucast rxctsucast rxackucast rxdfrmocast "
           "rxmfrmocast rxcfrmocast rxrtsocast rxctsocast rxdfrmmcast rxmfrmmcast "
           "rxcfrmmcast rxbeaconmbss rxdfrmucastobss rxbeaconobss rxrsptmout").split()

ME = bytes.fromhex("020000000001")
PEER = bytes.fromhex("020000000002")


def amt_row(chip, index, addr, flags):
    chip.obj_rcmta[2 * index] = int.from_bytes(addr[:4], "little")
    chip.obj_rcmta[2 * index + 1] = int.from_bytes(addr[4:], "little") | flags << 16


def frame(fc, length, ra, fcs_ok):
    mpdu = struct.pack("<HH", fc, 0x2C) + ra + PEER + ME + struct.pack("<H", 0x10) + bytes(length)
    fcs = zlib.crc32(mpdu) ^ (0 if fcs_ok else 1)
    mpdu += struct.pack("<I", fcs)
    plcp = struct.pack("<I", 0xB | len(mpdu) << 5) + b"\0\0"
    return plcp + mpdu


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("fwdir")
    ap.add_argument("ucode", help="e.g. ucode42")
    ap.add_argument("--fc", type=lambda x: int(x, 0), default=0x0108,
                    help="frame control (default 0x0108, data to the DS)")
    ap.add_argument("--len", type=int, default=40, help="body bytes")
    ap.add_argument("--bad-fcs", action="store_true")
    ap.add_argument("--ra", type=bytes.fromhex, default=ME, help="addr1, 12 hex digits")
    ap.add_argument("--cond", type=lambda x: int(x, 0), action="append", default=[],
                    help="a condition the frame holds from its PLCP on (repeatable)")
    ap.add_argument("--then", action="append", default=[], metavar="RA",
                    help="another frame after it, to this addr1 (repeatable)")
    ap.add_argument("--us", type=int, default=400, help="time to run after the frame")
    ap.add_argument("--initvals", action="append", default=[],
                    help="initvals and bsinitvals, where fwmatrix.py finds none")
    ap.add_argument("--phy-type", type=lambda x: int(x, 0), default=0x2,
                    help="PHY type with --initvals (default 0x2, G)")
    args = ap.parse_args()

    name = args.ucode + ".fw"
    found = [u for u in fwmatrix.ucodes(args.fwdir) if u[0] == name]
    if not found:
        raise SystemExit(f"{name}: not in {args.fwdir}")
    _, corerev, phy = found[0]
    hw = core.for_corerev(corerev)
    sets = list(fwmatrix.initvals(args.fwdir, corerev, phy))
    if args.initvals:
        sets = [(os.path.basename(args.initvals[0])[:-3], args.phy_type, args.initvals)]
    if not sets:
        raise SystemExit(f"no initvals for {name}: give --initvals and --phy-type")
    iv, phy_type, files = sets[0]
    ls = fwmatrix.run(hw.isa.load_program(os.path.join(args.fwdir, name)), hw, files,
                      phy_type, fwmatrix.BEACON, tx=(20, 10, 30), mhz=80)
    chip = ls.chip
    chip.m.shm[0x10] = 2352
    # B43_SHM_SH_CHAN as b43-next writes it: the chanspec on the AC-PHY
    # (channel 36, 80 MHz: 0xE02A), the channel number before it (1)
    chip.m.shm[0x50] = 0xE02A if corerev >= 40 else 1
    # The copy length (SPR_RXE_COPY_LEN) the later cores' initvals give
    # (0xB4) and corerev 11's do not; ucode11 also reloads it from SHM word
    # 0x0A on a wake-up (0x0C1F), a word no driver source names. Without it
    # nothing is copied and ucode11 reads an all-zero header.
    if not chip.m.spr.get(rxe.SPR_RXE_COPY_LEN):
        chip.m.spr[rxe.SPR_RXE_COPY_LEN] = chip.m.shm[0x0A] = 0xB4
    if corerev >= 40:
        amt_row(chip, 0x3F, ME, 0x8008)
        amt_row(chip, 0x3E, ME, 0x8002)

    pcs, tx = set(), []

    class Rec(vchip.Recorder):
        def step(self, pc, ins, ext_flags):
            pcs.add(pc)

        def tx_condition(self, sel):
            tx.append((chip.clock, sel))

    chip.recorder, chip.recording = Rec(), True
    t0 = chip.clock
    counters = list(chip.m.shm[0x70:0xA0])
    old = corerev < 40      # RA and BSS match as OpenFWWF's conditions
    f = rxe.RxFrame(frame(args.fc, args.len, args.ra, not args.bad_fcs),
                    fcs_ok=not args.bad_fcs, ra_match=old and args.ra == ME,
                    bss_match=old, conds=tuple(args.cond))
    chip.rxe.rx_frame(f)
    chip.wait(args.us, 300000)
    for ra in args.then:
        ra = bytes.fromhex(ra)
        chip.rxe.rx_frame(rxe.RxFrame(frame(args.fc, args.len, ra, True),
                                      ra_match=old and ra == ME, bss_match=old))
        chip.wait(args.us, 300000)
    us = lambda clock: (clock - t0) / chip.mhz
    print(f"{name} ({iv}): frame of {len(f.data)} bytes, fc {args.fc:#06x}")
    for clock, event in chip.rxe.log:
        print(f"  {us(clock):8.2f} us  {event}")
    for clock, sel in tx:
        print(f"  {us(clock):8.2f} us  TX condition {sel:#04x}")
    for p in chip.rxe.fifo:
        print(f"  FIFO: RX header {p.rxhdr.hex() or '-'}, pad {p.pad}, {len(p.data)} bytes")
    moved = [f"{0x70 + i:#05x}{'(' + MACSTAT[i] + ')' if i < len(MACSTAT) else ''}+{(chip.m.shm[0x70 + i] - v) & 0xFFFF}"
             for i, v in enumerate(counters) if chip.m.shm[0x70 + i] != v]
    print(f"  SHM counters: {' '.join(moved) or '-'}")
    print(f"  {len(pcs)} PCs executed, PSM "
          f"{'stopped: ' + chip.stopped if chip.stopped else f'at {chip.m.pc:#06x}'}")


if __name__ == "__main__":
    main()
