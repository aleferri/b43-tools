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
Scan a D11 ucode blob for jext/jnext and name the external conditions they
test, using the OpenFWWF selector map from cond.py.

A selector is (condreg << 4) | bit, bit 7 = EOI. The names come from OpenFWWF
(corerev 5, arch5); the layout carries over to the arch15 cores, but a given
bit is a lead for a newer core, not a guarantee.

    extcond_scan.py BLOB [--until 0x1480] [--cond-inc FILE]
"""
import argparse
import sys

import core
from cond import CondMap


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("blob")
    ap.add_argument("--until", default="0x1480", help="scan PC range [0, until)")
    ap.add_argument("--cond-inc", help="path to OpenFWWF cond.inc (else autodetect)")
    ap.add_argument("--corerev", type=int, default=42,
                    help="D11 core revision, for its instruction set (core.py)")
    args = ap.parse_args()

    cm = CondMap.load(args.cond_inc)
    prog = core.isa_of(args.corerev).load_program(args.blob)
    until = int(args.until, 0)

    seen = {}
    for pc, ins in enumerate(prog):
        if pc >= until:
            break
        if ins.mnem in ("jext", "jnext"):
            imm = ins.extra["imm"]
            e = seen.setdefault(imm, [0, []])
            e[0] += 1
            if len(e[1]) < 6:
                e[1].append(pc)

    print(f"jext/jnext external conditions in [0,0x{until:X}) of {args.blob}:")
    print(f"{'imm':>5}  {'name':24}  {'count':>5}  sample PCs")
    for imm in sorted(seen):
        cnt, pcs = seen[imm]
        pcstr = " ".join(f"0x{p:04X}" for p in pcs)
        print(f" 0x{imm:02X}  {cm.label(imm):24}  {cnt:5}  {pcstr}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
