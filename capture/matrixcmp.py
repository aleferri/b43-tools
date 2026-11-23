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
# matrixcmp.py A B: compare two matrix.sh captures (e.g. the vendor ucode and yours): per address, the value sequence before the mark (rows < 1500) and after it
# (rows >= 1500), as (TSF us from the mark, value) at each change; addresses whose sequences differ in values
import sys
def load(fn):
    out = {}
    for b in open(fn).read().split("addr ")[1:]:
        L = b.splitlines(); h, mx = L[0].split()[:2]
        rows = [(int(f[0]), int(f[1], 16), f[3]) for f in (x.split() for x in L[1:]) if len(f) >= 5]
        if not rows or int(mx) == 0: out[h] = None; continue
        t0 = next((t for i, t, v in rows if i >= 1500), rows[0][1])
        seq = []; prev = None
        for i, t, v in rows:
            if v != prev: seq.append((((t - t0 + 0x8000) & 0xFFFF) - 0x8000, v)); prev = v
        out[h] = seq
    return out
a, b = load(sys.argv[1]), load(sys.argv[2])
n = 0
for h in sorted(set(a) | set(b), key=lambda x: int(x, 16)):
    sa, sb = a.get(h), b.get(h)
    if sa is None or sb is None: continue
    va, vb = [v for _, v in sa], [v for _, v in sb]
    if va == vb: continue
    if len(set(va)) == 1 and len(set(vb)) == 1:
        tag = "const"
    else:
        tag = "moves"
    n += 1
    fa = " ".join(f"{t}:{v}" for t, v in sa[:7]); fb = " ".join(f"{t}:{v}" for t, v in sb[:7])
    print(f"{h} SPR {(int(h, 16) - 0x400) // 2:03x} [{tag}]\n   A {fa}\n   B {fb}")
print("differing addresses:", n, file=sys.stderr)
