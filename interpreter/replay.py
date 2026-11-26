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
"""A capture's reads, answered again: a driver's MMIO as a Linux mmiotrace
recorded it, by BAR0 offset.

Each register has the queue of the values read from it, in the capture's
order; the last one comes again once the queue runs out. A register is its
width and BAR0 offset, and for the D11 core's data ports the index last
written: PHY 0x3FC (0x3FE), radio 0x3F6 (0x3FA, 0x3F8) and 0x3D8 (0x3DA),
object window 0x160 (0x164, 0x166). The window is the BAR0 offset's, not the
backplane's: an mmiotrace has no configuration writes to move it.

The driver's accesses are followed against the capture's sequence: one that
is the capture's next, or among the next `lookahead`, moves the cursor past
it, any other is a miss; `misses` keeps the first ones.
"""

INDEX = {0x3FC: "phy", 0x3F6: "radio", 0x3D8: "radio", 0x160: "obj"}
DATA = {0x3FE: "phy", 0x3FA: "radio", 0x3F8: "radio", 0x3DA: "radio",
        0x164: "obj", 0x166: "obj"}
KEEP_MISSES = 50


def mmiotrace(path, bar_size):
    """(write, bits, BAR0 offset, value) of each access inside the first
    mapping of an mmiotrace."""
    base = None
    for line in open(path):
        f = line.split()
        if not f:
            continue
        if f[0] == "MAP" and base is None:
            base = int(f[3], 16)
        elif f[0] in ("R", "W") and base is not None and len(f) >= 6:
            off = int(f[4], 16) - base
            if 0 <= off < bar_size:
                yield f[0] == "W", 8 * int(f[1]), off, int(f[5], 16)


class Replay:
    def __init__(self, accesses, lookahead=256):
        self.acc = list(accesses)
        self.lookahead = lookahead
        self.queues = {}
        index = {}
        for write, bits, off, val in self.acc:
            if write:
                if off in INDEX:
                    index[INDEX[off]] = val
            else:
                key = (bits, off, index.get(DATA.get(off)))
                self.queues.setdefault(key, []).append(val)
        self.pos = dict.fromkeys(self.queues, 0)
        self.index = {}
        self.cursor = 0
        self.accesses = self.skipped = self.unknown = self.exhausted = 0
        self.misses = []
        self.nmisses = 0

    def read(self, off, bits):
        key = (bits, off, self.index.get(DATA.get(off)))
        q = self.queues.get(key)
        if q is None:
            self.unknown += 1
            val = 0
        elif self.pos[key] < len(q):
            val = q[self.pos[key]]
            self.pos[key] += 1
        else:
            self.exhausted += 1
            val = q[-1]
        self.follow(False, bits, off, val)
        return val

    def write(self, off, val, bits):
        if off in INDEX:
            self.index[INDEX[off]] = val
        self.follow(True, bits, off, val)

    def follow(self, write, bits, off, val):
        self.accesses += 1
        end = min(self.cursor + 1 + self.lookahead, len(self.acc))
        for j in range(self.cursor, end):
            w, b, o, v = self.acc[j]
            if w == write and b == bits and o == off and (not write or v == val):
                self.skipped += j - self.cursor
                self.cursor = j + 1
                return
        self.nmisses += 1
        if len(self.misses) < KEEP_MISSES:
            self.misses.append((self.accesses, write, bits, off, val, self.cursor))

    def report(self, out):
        w = out.write
        w(f"replay: {self.accesses} accesses, capture {self.cursor} of {len(self.acc)}, "
          f"misses {self.nmisses}, skipped {self.skipped}, reads of registers the "
          f"capture never read {self.unknown}, past their queue {self.exhausted}\n")
        for n, write, bits, off, val, cur in self.misses:
            t = self.acc[cur] if cur < len(self.acc) else None
            exp = (f"{'W' if t[0] else 'R'}{t[1]} {t[2]:#06x} {t[3]:#010x}" if t else "-")
            w(f"  miss at access {n}: {'W' if write else 'R'}{bits} {off:#06x} {val:#010x}, "
              f"capture #{cur}: {exp}\n")
