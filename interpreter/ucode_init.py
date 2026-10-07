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
"""
Run the real D11 microcode through the arch15 interpreter and dump the shared
memory it writes.

This is the ucode-authored part of the core's state: the version-stamp prologue
the PSM runs right after PSM_RUN, which publishes UCODEREV/PATCH/DATE/TIME (and
the rest of that block) into shared memory. It executes the actual instructions
of the extracted blob, not a model of them, using the sibling psm.py.

The real entry point (PC 0) cannot be run unattended: it settles in the main
dispatch loop and idles, because nothing asserts an external condition. So
execution starts at the version-stamp block; its address is a parameter, 0xF76
for the D6220/TG789 corerev-42 builds (see the interpreter's validation table).

    ucode_init.py BLOB [--start 0xF76] [--steps 64] [--out FILE]
                       [--seed COND ...] [--cond-inc FILE]

Output lines: "SHM 0x<byte_off> 0x<val16>", one per non-zero word.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import psm  # noqa: E402  (sibling module)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("blob")
    ap.add_argument("--start", default="0xF76",
                    help="entry of the version-stamp block (default 0xF76)")
    ap.add_argument("--steps", type=int, default=64,
                    help="max instructions to run through the block")
    ap.add_argument("--out", help="snapshot file (default stdout)")
    ap.add_argument("--seed", action="append", default=[],
                    help="external condition to force before running, by name or "
                         "selector (TX.MACEN, COND_RX_COMPLETE, 0x3d=0); needs "
                         "cond.py + OpenFWWF cond.inc")
    ap.add_argument("--cond-inc", help="path to OpenFWWF cond.inc (else autodetect)")
    args = ap.parse_args()

    start = int(args.start, 0)
    prog = psm.load_program(args.blob)
    m = psm.PSM(prog)

    if args.seed:
        from cond import CondMap
        cm = CondMap.load(args.cond_inc)
        seeds = cm.seeds(args.seed)
        m.ext_flags = dict(seeds)
        print("[ucode_init] seeded ext conditions: " +
              ", ".join(f"{cm.label(i)}={'T' if v else 'F'}"
                        for i, v in seeds.items() if not (i & 0x80)),
              file=sys.stderr)

    stopped = "max_steps"
    try:
        m.run(start=start, max_steps=args.steps, trace=False)
    except psm.HardwareWaitLoop:
        stopped = "hardware-wait (block complete)"
    except Exception as e:
        stopped = type(e).__name__

    out = open(args.out, "w") if args.out else sys.stdout
    for widx, val in enumerate(m.shm):
        if val & 0xFFFF:
            out.write(f"SHM 0x{widx * 2:04x} 0x{val & 0xFFFF:04x}\n")
    if args.out:
        out.close()

    print(f"[ucode_init] blob={args.blob} start=0x{start:X} stop={stopped}",
          file=sys.stderr)
    print(f"[ucode_init] UCODEREV=0x{m.shm[0] & 0xFFFF:04X} "
          f"UCODEPATCH=0x{m.shm[1] & 0xFFFF:04X}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
