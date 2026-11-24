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
import math
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

# mac80211's default EDCA parameters (aifs, cw_min, cw_max, txop in 32 us),
# as b43-next uploads them per queue (b43_qos_params_upload()): BK, BE, VI, VO.
EDCA = ((7, 15, 1023, 0), (3, 15, 1023, 0), (2, 7, 15, 94), (2, 3, 7, 47))
SHM_EDCFQ, QOSPARAMS = 0x240 >> 1, 16
HOSTF1, HF_EDCF = 0x05E >> 1, 0x0100
SPR_IFS_CTL, IFSCTL_USE_EDCF = 0x144, 0x0004

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


def htsig_crc(bits):
    """HT-SIG's CRC-8 (x^8 + x^2 + x + 1, ones in, complemented out)."""
    c = [1] * 8
    for b in bits:
        fb = b ^ c[7]
        c = [fb, c[0] ^ fb, c[1] ^ fb] + c[2:7]
    return [1 - x for x in reversed(c)]


def htsig(mcs, length, sig2=0x06):
    """The six PLCP bytes of an HT frame as b43 sees them: HT-SIG1 (MCS,
    20 MHz, length), HT-SIG2's first byte (0x06, or 0x0E with the
    aggregation bit), its CRC over the 34 bits before it, the tail. Checked
    against the copies of the RT-AC56U's BCM43217."""
    b = [mcs & 0x7F, length & 0xFF, length >> 8 & 0xFF, sig2]
    bits = [(x >> i) & 1 for x in b for i in range(8)] + [0, 0]
    rest = [0, 0] + htsig_crc(bits) + [0] * 6
    return bytes(b + [sum(v << i for i, v in enumerate(rest[:8])),
                      sum(v << i for i, v in enumerate(rest[8:]))])


def ampdu(fc, length, ra, n, mcs=7):
    """An HT A-MPDU of n MPDUs like frame()'s, sequence numbers 1..n: the
    first with its PLCP (the PPDU's length: the MPDUs, their 4-byte
    delimiters, each padded to 4 bytes but the last), and the rest."""
    mpdus = []
    for k in range(n):
        m = frame(fc, length, ra, True)[6:]
        m = m[:22] + struct.pack("<H", (k + 1) << 4) + m[24:-4]
        mpdus.append(m + struct.pack("<I", zlib.crc32(m)))
    total = sum(4 + len(m) for m in mpdus) + sum(-len(m) % 4 for m in mpdus[:-1])
    return htsig(mcs, total, 0x0E) + mpdus[0], tuple(mpdus[1:])


def setup(fwdir, ucode, initvals=(), phy_type=0x2):
    """The chip in its idle AP loop with the host side set up; and its
    corerev."""
    name = ucode + ".fw"
    found = [u for u in fwmatrix.ucodes(fwdir) if u[0] == name]
    if not found:
        raise SystemExit(f"{name}: not in {fwdir}")
    _, corerev, phy = found[0]
    hw = core.for_corerev(corerev)
    sets = list(fwmatrix.initvals(fwdir, corerev, phy))
    if initvals:
        sets = [(os.path.basename(initvals[0])[:-3], phy_type, list(initvals))]
    if not sets:
        raise SystemExit(f"no initvals for {name}: give --initvals and --phy-type")
    _iv, phy_type, files = sets[0]
    ls = fwmatrix.run(hw.isa.load_program(os.path.join(fwdir, name)), hw, files,
                      phy_type, fwmatrix.BEACON, tx=(20, 10, 30), mhz=80)
    chip = ls.chip
    chip.m.shm[0x10] = 2352
    # B43_SHM_SH_MIMO_MAXSYM as b43-next writes it with HT on the N-PHY
    # (b43_chip_init) and on the AC-PHY (phy_ac.c): with 0 the mimo ucodes
    # and ucode42 (784.2) drop every HT frame at the mark.
    if phy == "mimo" or corerev >= 40:
        chip.m.shm[0x5D] = 0xFFFF
    # B43_SHM_SH_CHAN as b43-next writes it: the chanspec on the AC-PHY
    # (channel 36, 80 MHz: 0xE02A), the channel number before it (1)
    chip.m.shm[0x50] = 0xE02A if corerev >= 40 else 1
    # The copy length (SPR_RXE_COPY_LEN) the later cores' initvals give
    # (0xB4) and corerev 11's do not; ucode11 also reloads it from SHM word
    # 0x0A on a wake-up (0x0C1F), a word no driver source names. Without it
    # nothing is copied and ucode11 reads an all-zero header.
    if not chip.m.spr.get(rxe.SPR_RXE_COPY_LEN):
        chip.m.spr[rxe.SPR_RXE_COPY_LEN] = chip.m.shm[0x0A] = 0xB4
    # QoS as b43-next's b43_qos_init() leaves it: the EDCA parameters of each
    # queue (backoff slots 0 where b43 takes them from the RNG), HF_EDCF and
    # IFSCTL's USE_EDCF. Without HF_EDCF the ucode sends from FIFO 3 alone.
    shm = chip.m.shm
    for q, (aifs, cwmin, cwmax, txop) in enumerate(EDCA):
        base = SHM_EDCFQ + q * QOSPARAMS
        for i, v in enumerate((txop * 32, cwmin, cwmax, cwmin, aifs, 0, aifs)):
            shm[base + i] = v
        shm[base + 7] |= 0x100          # B43_QOSPARAM_STATUS: updated
    shm[HOSTF1] |= HF_EDCF
    chip.spr_write(SPR_IFS_CTL, chip.m.spr.get(SPR_IFS_CTL, 0) | IFSCTL_USE_EDCF)
    # Retry limits as b43_chip_init() writes them: short 7 and long 4 in the
    # scratch registers 6 and 7 (B43_SHM_SC_SRLIMIT/LRLIMIT), the fallback
    # limits in SHM 0x44/0x46 (B43_SHM_SH_SFFBLIM/LFFBLIM).
    chip.m.reg[6], chip.m.reg[7] = 7, 4
    shm[0x44 >> 1], shm[0x46 >> 1] = 3, 2
    # MACCMD's DFQ_VALID, which the initvals set (and b43-next on the AC);
    # fwmatrix's op stream writes MACCMD whole. The ucode
    # sends from its FIFOs only with it set (OpenFWWF's ready_for_header_copy).
    chip.spr_write(vchip.SPR_MAC_CMD, chip.m.spr.get(vchip.SPR_MAC_CMD, 0) | 0x4)
    if corerev >= 40:
        amt_row(chip, 0x3F, ME, 0x8008)
        amt_row(chip, 0x3E, ME, 0x8002)
    return chip, corerev


def moved(chip, counters):
    """The SHM counters that moved since `counters`, named."""
    return " ".join(
        f"{0x70 + i:#05x}{'(' + MACSTAT[i] + ')' if i < len(MACSTAT) else ''}"
        f"+{(chip.m.shm[0x70 + i] - v) & 0xFFFF}"
        for i, v in enumerate(counters) if chip.m.shm[0x70 + i] != v)


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
    ap.add_argument("--mbps", type=float, default=None,
                    help="the frame's rate in Mb/s, for its air time (default 6)")
    ap.add_argument("--rx-buffer", metavar="BYTES|chip", default=None,
                    help="cut a frame the ucode has not let stream at BYTES ('chip': the "
                    "RT-AC56U's size for the corerev, rxe.RX_BUFFER_CHIP)")
    ap.add_argument("--phy143-chip", action="store_true",
                    help="corerev 40+: PHY 0x143 at the PLCP as the BCM4352 reads (rxe.phy143_chip)")
    ap.add_argument("--restart40", action="store_true",
                    help="corerev 40+: FIFOCTL1 bit 6 mid-frame restarts the copy, as on the BCM4352 (rxe.restart40)")
    ap.add_argument("--next40-mark", action="store_true",
                    help="corerev 40+: a mark write in an A-MPDU member makes SPR 0x02B read 0x8000, as on the BCM4352 (rxe.next40_mark)")
    ap.add_argument("--hdr-offset-moves", action="store_true",
                    help="SPR_RXE_RXHDR_OFFSET moves past the header a push takes, as on the chip (rxe.hdr_offset_moves)")
    ap.add_argument("--mark40-equal", action="store_true",
                    help="corerev 40+: a mark written equal to the frame's count raises no COND_RX_COMPLETE, as on the BCM4352 (rxe.mark40_equal)")
    ap.add_argument("--ampdu", type=int, default=0, metavar="N",
                    help="an HT A-MPDU of N such MPDUs (MCS 7) instead of one frame")
    ap.add_argument("--phy-type", type=lambda x: int(x, 0), default=0x2,
                    help="PHY type with --initvals (default 0x2, G)")
    args = ap.parse_args()

    name = args.ucode + ".fw"
    chip, corerev = setup(args.fwdir, args.ucode, args.initvals, args.phy_type)
    if args.rx_buffer is not None:
        chip.rxe.rx_buffer = rxe.RX_BUFFER_CHIP.get(corerev) if args.rx_buffer == "chip" \
            else int(args.rx_buffer, 0)
    if args.phy143_chip:
        chip.rxe.phy143_chip = True
    if args.restart40:
        chip.rxe.restart40 = True
    if args.next40_mark:
        chip.rxe.next40_mark = True
    if args.hdr_offset_moves:
        chip.rxe.hdr_offset_moves = True
    if args.mark40_equal:
        chip.rxe.mark40_equal = True

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
    if args.ampdu:
        first, behind = ampdu(args.fc, args.len, args.ra, args.ampdu)
        # HT mixed format: 36 us of preamble to HT-SIG and the HT-LTF, then
        # MCS 7 at 20 MHz (65 Mb/s) for the delimiter and the MPDU.
        f = rxe.RxFrame(first, ra_match=old and args.ra == ME, bss_match=old,
                        conds=tuple(args.cond), ofdm=False, ftype=2, behind=behind,
                        plcp_us=36.0, air_us=36.0 + (len(first) - 6 + 4) * 8 / 65)
    else:
        data = frame(args.fc, args.len, args.ra, not args.bad_fcs)
        air = None if args.mbps is None else 20.0 + (len(data) - 6) * 8 / args.mbps
        f = rxe.RxFrame(data, air_us=air,
                        fcs_ok=not args.bad_fcs, ra_match=old and args.ra == ME,
                        bss_match=old, conds=tuple(args.cond))
    chip.rxe.rx_frame(f)
    chip.wait(math.ceil(f.air()) + args.us, 300000)
    for ra in args.then:
        ra = bytes.fromhex(ra)
        g = rxe.RxFrame(frame(args.fc, args.len, ra, True), ra_match=old and ra == ME, bss_match=old)
        chip.rxe.rx_frame(g)
        chip.wait(math.ceil(g.air()) + args.us, 300000)
    us = lambda clock: (clock - t0) / chip.mhz
    print(f"{name}: frame of {len(f.data)} bytes, fc {args.fc:#06x}"
          + (f", A-MPDU of {args.ampdu}" if args.ampdu else ""))
    for clock, event in chip.rxe.log:
        print(f"  {us(clock):8.2f} us  {event}")
    for clock, sel in tx:
        print(f"  {us(clock):8.2f} us  TX condition {sel:#04x}")
    for p in chip.rxe.fifo:
        print(f"  FIFO: RX header {p.rxhdr.hex() or '-'}, pad {p.pad}, {len(p.data)} bytes")
    print(f"  SHM counters: {moved(chip, counters) or '-'}")
    print(f"  {len(pcs)} PCs executed, PSM "
          f"{'stopped: ' + chip.stopped if chip.stopped else f'at {chip.m.pc:#06x}'}")


if __name__ == "__main__":
    main()
