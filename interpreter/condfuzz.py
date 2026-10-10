#!/usr/bin/env python3
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
                [--psm-mhz F] [--settle MAX] [--cond-inc FILE]
"""

import argparse
import copy

import lockstep
import psm


def reaches(base, sels, target, steps):
    ls = copy.deepcopy(base)
    ls.events |= set(sels)
    for _ in range(steps):
        if ls.m.pc == target:
            return True
        if not ls.step():
            return False
    return ls.m.pc == target


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
    args = ap.parse_args()

    label = lambda sel: ""
    if args.cond_inc:
        from cond import CondMap
        label = CondMap.load(args.cond_inc).label

    inits = [e for p in args.initvals for e in lockstep.load_inits(p)]
    tx = tuple(int(x, 0) for x in args.tx_engine.split(",")) if args.tx_engine else None
    base = lockstep.Lockstep(psm.load_program(args.blob), 100, args.settle,
                             inits, tx, args.psm_mhz)
    base.feed(args.ops)
    if not base.running:
        raise SystemExit(f"PSM not running after the op stream ({base.stopped or 'halted'})")

    alone = reaches(base, args.with_, args.target, args.steps)
    print(f"start pc {base.m.pc:#06x}; with {[hex(s) for s in args.with_]} only: "
          f"{'reaches' if alone else 'does not reach'} {args.target:#06x}")
    for sel in range(0x7F):
        if sel in args.with_ or sel == psm.COND_TRUE:
            continue
        hit = reaches(base, args.with_ + [sel], args.target, args.steps)
        if hit != args.blocking:
            print(f"  0x{sel:02X} {label(sel)}")


if __name__ == "__main__":
    main()
