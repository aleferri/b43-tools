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
# frames.py FILE GROUP [N]: from a watch.sh log, per group, the first N frames where FRAMELEN (0x418) rises past 0x200: every event from
# 40 us before the frame's FRAMELEN 6 to 40 us after FRAMELEN drops, times relative to FRAMELEN 6
import sys
fn, gi = sys.argv[1], int(sys.argv[2]); N = int(sys.argv[3]) if len(sys.argv) > 3 else 2
groups = open(fn).read().split("== group ")[1:]
g = groups[gi].splitlines(); addrs = g[0].split()
ev = [(int(f[0]), f[1], f[2]) for f in (l.split() for l in g[1:] if not l.startswith("regwatch")) if len(f) == 3]
fl = [(t, int(v, 16)) for t, a, v in ev if a == "418"]
shown = 0; i = 0
while i < len(fl) and shown < N:
    t, v = fl[i]
    if v == 6:
        j = i; mx = 0
        while j + 1 < len(fl) and fl[j + 1][1] >= fl[j][1] - 8: j += 1; mx = max(mx, fl[j][1])
        if mx > 0x200:
            tend = fl[j + 1][0] if j + 1 < len(fl) else fl[j][0]
            print(f"-- frame: FRAMELEN 6 at {t} us, peak {mx:#x}, drop at +{tend - t} us; addresses {' '.join(addrs)}")
            print("   " + " ".join(f"{tt - t:+d}:{a}={vv}" for tt, a, vv in ev if t - 40 <= tt <= tend + 40))
            shown += 1
        i = j + 1
    else:
        i += 1
