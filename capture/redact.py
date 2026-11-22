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
# redact.py MAC [MAC ...] < IN > OUT: replaces every 16-bit value that is a word of one of the MAC addresses (both
# byte orders) with xxxx, in matrix.sh rows (value in the 4th column) and watch.sh rows (3rd column), before a
# capture is published
import sys
words = set()
for m in sys.argv[1:]:
    b = bytes(int(x, 16) for x in m.split(":"))
    for i in (0, 2, 4):
        words |= {f"{b[i] | b[i + 1] << 8:04x}", f"{b[i] << 8 | b[i + 1]:04x}"}
for line in sys.stdin:
    f = line.split()
    if len(f) == 5 and f[3].lower() in words and not line.startswith(("addr", "trig")):
        f[3] = "xxxx"; line = f"{f[0]:>5} {f[1]} {f[2]} {f[3]} {f[4]}\n"
    elif len(f) == 3 and f[2].lower() in words and not line.startswith(("==", "regwatch")):
        line = f"{f[0]} {f[1]} xxxx\n"
    sys.stdout.write(line)
