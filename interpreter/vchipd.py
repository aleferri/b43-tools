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
"""Serve a virtual D11 chip (vchip.py) to a driver over stdin/stdout.

The driver side (a test harness running b43) forwards the D11 core's MMIO and
the passing of time; the chip answers the reads. The ucode is whatever the
driver uploads, the initvals whatever it writes. One request per line, one
reply per request, numbers in hex:

    W <off> <val> <bits>    an MMIO write            -> K <irq>
    R <off> <bits>          an MMIO read             -> V <val> <irq>
    T <us>                  time passes (udelay...)  -> K <irq>
    Q                       end                      -> K <irq>

bits is 8, 16 or 32. <irq> is GEN_IRQ_REASON & IRQ_MASK, so the driver side
knows when to run the interrupt handler. Before every access --access-us
microseconds pass, so that the ucode runs between them.

The board is what the options say: the core revision, the PHY version word
(MMIO PHY_VER, and PHY register 0 as the ucode reads it), and the radio's
registers that hold its identity (--radio 0=0x4 --radio 1=0x2069 for a radio
2069 rev 4 read the way b43 reads it on corerev 40/42).

    vchipd.py --corerev N --phy-ver V [--radio REG=VAL ...] [--psm-mhz F]
              [--access-us U] [--tx-engine IFS,START,DONE] [--report FILE]
"""

import argparse
import sys

import core
import vchip


def board_radio(pairs):
    regs = {}
    for pair in pairs:
        reg, val = pair.split("=")
        regs[int(reg, 0)] = int(val, 0)
    return regs


def irq_line(chip):
    return chip.mmio_read(vchip.IRQ_REASON, 32) & chip.mmio_read(vchip.IRQ_MASK, 32)


def serve(chip, access_us, rx, tx):
    for line in rx:
        req = line.split()
        if not req:
            continue
        op = req[0]
        if op == "W":
            off, val, bits = (int(x, 16) for x in req[1:4])
            chip.advance(access_us)
            chip.mmio_write(off, val, bits)
            tx.write(f"K {irq_line(chip):x}\n")
        elif op == "R":
            off, bits = (int(x, 16) for x in req[1:3])
            chip.advance(access_us)
            val = chip.mmio_read(off, bits) & ((1 << bits) - 1)
            tx.write(f"V {val:x} {irq_line(chip):x}\n")
        elif op == "T":
            chip.advance(int(req[1], 16))
            tx.write(f"K {irq_line(chip):x}\n")
        elif op == "Q":
            tx.write(f"K {irq_line(chip):x}\n")
            tx.flush()
            return
        else:
            raise SystemExit(f"vchipd: bad request {line!r}")
        tx.flush()


def report(chip, out):
    w = out.write
    w(f"corerev {chip.hw.corerev}, {chip.hw.isa.NAME}, {len(chip.prog)} instructions uploaded\n")
    w(f"PSM {'stopped: ' + chip.stopped if chip.stopped else 'running' if chip.running else 'halted'}, "
      f"pc {chip.m.pc:#06x}, {chip.steps} steps, TSF {chip.tsf()} us\n")
    w(f"UCODEREV {chip.m.shm[0]:#06x} PATCH {chip.m.shm[1]:#06x}\n")
    w("MMIO registers read without a model (offset: reads):\n")
    for off, n in sorted(chip.unmodelled.items()):
        w(f"  {off:#06x}: {n}\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--corerev", type=int, required=True)
    ap.add_argument("--phy-ver", type=lambda x: int(x, 0), required=True,
                    help="MMIO PHY_VER: analog << 12 | type << 8 | revision")
    ap.add_argument("--radio", action="append", default=[], metavar="REG=VAL",
                    help="a radio register as the chip starts with it (identity)")
    ap.add_argument("--psm-mhz", type=float, default=80.0,
                    help="PSM instructions per microsecond, a model (default 80)")
    ap.add_argument("--access-us", type=float, default=0.1,
                    help="time a bus access takes, in microseconds (default 0.1)")
    ap.add_argument("--tx-engine", metavar="IFS,START,DONE")
    ap.add_argument("--report")
    args = ap.parse_args()

    hw = core.for_corerev(args.corerev)
    tx = tuple(int(x, 0) for x in args.tx_engine.split(",")) if args.tx_engine else None
    devices = {"radio": vchip.RegisterFile("radio", board_radio(args.radio))}
    chip = vchip.Chip([], hw, devices, tx=tx, mhz=args.psm_mhz, phy_version=args.phy_ver)
    serve(chip, args.access_us, sys.stdin, sys.stdout)
    if args.report:
        with open(args.report, "w") as out:
            report(chip, out)


if __name__ == "__main__":
    main()
