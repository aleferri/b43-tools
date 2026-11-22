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
# watchlist.py MATRIX [--timer N] [--per K]: from a matrix.sh capture, the addresses whose value changes around the
# frame, without free-running timers (more than N changes in a capture, default 300), grouped K per line (default 6)
# with FRAMELEN 0x418 first: the GROUPS file for watch.sh
import argparse
ap = argparse.ArgumentParser()
ap.add_argument("matrix"); ap.add_argument("--timer", type=int, default=300); ap.add_argument("--per", type=int, default=6)
o = ap.parse_args()
keep = []
for b in open(o.matrix).read().split("addr ")[1:]:
    L = b.splitlines(); h = L[0].split()[0]
    vals = [f[3] for f in (x.split() for x in L[1:]) if len(f) >= 5]
    n = sum(1 for i in range(1, len(vals)) if vals[i] != vals[i - 1])
    if 0 < n <= o.timer and h != "0x418":
        keep.append(h)
for i in range(0, len(keep), o.per):
    print(" ".join(["0x418"] + keep[i:i + o.per]))
