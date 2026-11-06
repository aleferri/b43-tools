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
"""Find the external conditions that lead the ucode to a given address.

The host op stream brings the co-simulation (lockstep.py) to a state, usually
an idle main loop. From there, for each selector 0x00..0x7E, a copy of the
machine runs with that condition raised, together with any given with
--with, held until the ucode acknowledges it with an EOI as lockstep.py holds
its own events, and the selector is reported if the ucode reaches TARGET
within --steps instructions. --blocking reports instead the selectors that,
added to the --with ones, keep it from TARGET.

Conditions are held where real hardware may drop them on its own, so a
selector found here is a lead to read in the listing, not a semantic.

    condfuzz.py OPS BLOB TARGET [--with SEL ...] [--blocking] [--steps N]
                [--initvals FILE ...] [--tx-engine IFS,START,DONE]
                [--psm-mhz F] [--settle MAX] [--corerev N] [--phy-ver V]
                [--cond-inc FILE]
"""

import argparse
import copy

import core
import lockstep
import psmlib
import vchip


def reaches(chip, sels, target, steps):
    """Whether a copy of the chip, with sels raised, reaches target."""
    chip = copy.deepcopy(chip, {id(chip.prog): chip.prog, id(chip.hw): chip.hw,
                                id(chip.recorder): vchip.Recorder()})
    chip.events |= set(sels)
    for _ in range(steps):
        if chip.m.pc == target:
            return True
        if not chip.step():
            return False
    return chip.m.pc == target


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ops")
    ap.add_argument("blob")
    ap.add_argument("target", type=lambda x: int(x, 0))
    ap.add_argument("--with", dest="with_", action="append", default=[],
                    type=lambda x: int(x, 0), metavar="SEL")
    ap.add_argument("--blocking", action="store_true")
    ap.add_argument("--steps", type=int, default=20000)
    ap.add_argument("--settle", type=int, default=300000, metavar="MAX")
    ap.add_argument("--initvals", action="append", default=[])
    ap.add_argument("--tx-engine", metavar="IFS,START,DONE")
    ap.add_argument("--psm-mhz", type=float, metavar="F")
    ap.add_argument("--cond-inc")
    ap.add_argument("--corerev", type=int, default=42)
    ap.add_argument("--phy-ver", type=lambda x: int(x, 0), metavar="V")
    args = ap.parse_args()

    label = lambda sel: ""
    if args.cond_inc:
        from cond import CondMap
        label = CondMap.load(args.cond_inc).label

    inits = [e for p in args.initvals for e in vchip.load_inits(p)]
    tx = tuple(int(x, 0) for x in args.tx_engine.split(",")) if args.tx_engine else None
    hw = core.for_corerev(args.corerev)
    ls = lockstep.Lockstep(hw.isa.load_program(args.blob), 100, args.settle,
                           inits, tx, args.psm_mhz, hw, args.phy_ver)
    ls.feed(args.ops)
    chip = ls.chip
    if not chip.running:
        raise SystemExit(f"PSM not running after the op stream ({chip.stopped or 'halted'})")

    alone = reaches(chip, args.with_, args.target, args.steps)
    print(f"start pc {chip.m.pc:#06x}; with {[hex(s) for s in args.with_]} only: "
          f"{'reaches' if alone else 'does not reach'} {args.target:#06x}")
    for sel in range(0x7F):
        if sel in args.with_ or sel == psmlib.COND_TRUE:
            continue
        hit = reaches(chip, args.with_ + [sel], args.target, args.steps)
        if hit != args.blocking:
            print(f"  0x{sel:02X} {label(sel)}")

if __name__ == "__main__":
    main()
