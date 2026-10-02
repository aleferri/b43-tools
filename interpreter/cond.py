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
Parse OpenFWWF's cond.inc into the D11 external-condition map, and build
ext_flags seeds for psm.py by name.

The PSM's jext/jnext test a hardware "external condition" indexed by the low
byte of the instruction. psm.py leaves that byte as an opaque stub; OpenFWWF
(github.com/fullstory/openfwwf, GPL-2) is the human-written microcode that
names those signals, as (condreg << 4) | bit with bit 7 = EOI, COND_TRUE=0x7F.
This module parses that file rather than copying it, so the names track it and
nothing of it is vendored here.

OpenFWWF is corerev 5 (arch5). The selector layout and COND_TRUE carry over to
the arch15 cores (b43-tools uses the same jext/jnext encoding across both), but
a particular FIXME bit is a lead for a newer core, not a guarantee.

As a library:
    from cond import CondMap
    cm = CondMap.load()                 # finds cond.inc, or pass a path
    imm = cm.by_name("COND_TX_DONE")    # -> 0x22
    seeds = cm.seeds(["TX.MACEN", "PSM.bit0=1", "0x3d=0"])  # {imm: bool, ...}
"""
import os
import re


class CondMap:
    def __init__(self, regs, names):
        self.regs = regs          # CONDREG_NAME -> number
        self.names = names        # COND_NAME -> imm (0..0x7f)
        self._by_imm = {}
        for n, imm in names.items():
            self._by_imm.setdefault(imm, n)

    @classmethod
    def load(cls, path=None):
        path = path or cls._find()
        text = open(path).read()
        regs = {}
        for m in re.finditer(r"#define\s+(CONDREG_\w+)\s+(\d+)", text):
            regs[m.group(1)] = int(m.group(2))
        names = {}
        for m in re.finditer(r"#define\s+(COND_\w+)\s+EXTCOND\(\s*(\w+)\s*,\s*(\d+)\s*\)",
                             text):
            name, reg, bit = m.group(1), m.group(2), int(m.group(3))
            if reg in regs:
                names[name] = (regs[reg] << 4) | bit
        return cls(regs, names)

    @staticmethod
    def _find():
        here = os.path.dirname(os.path.abspath(__file__))
        for d in filter(None, [os.environ.get("OPENFWWF"),
                               os.path.join(here, "..", "..", "openfwwf"),
                               os.path.join(here, "..", "openfwwf")]):
            p = os.path.join(d, "cond.inc")
            if os.path.isfile(p):
                return p
        raise FileNotFoundError("cond.inc not found; set OPENFWWF or pass a path "
                                "(clone github.com/fullstory/openfwwf)")

    def by_name(self, token):
        """Resolve COND_NAME, REG.BIT (e.g. TX.MACEN), or a 0xNN literal."""
        t = token.strip()
        if t.upper().startswith("COND_") and t.upper() in self.names:
            return self.names[t.upper()]
        if "." in t:  # REG.SHORTNAME, matched against the tail of COND_ names
            reg, short = t.split(".", 1)
            want = f"COND_{short}".upper()
            for n, imm in self.names.items():
                if n.upper() == want or n.upper().endswith("_" + short.upper()):
                    if self.regs.get(f"CONDREG_{reg.upper()}") == (imm >> 4):
                        return imm
        return int(t, 0)  # literal

    def label(self, imm):
        base = imm & 0x7f
        n = self._by_imm.get(base, f"reg{(base >> 4) & 7}.bit{base & 0xf}")
        return n + (" +EOI" if imm & 0x80 else "")

    def seeds(self, specs):
        """specs: ["NAME", "NAME=1", "NAME=0", "0x3d"] -> {imm: bool}, both the
        bare selector and its EOI alias set, since they test one signal."""
        out = {}
        for spec in specs:
            name, _, val = spec.partition("=")
            truth = (val.strip() != "0")
            imm = self.by_name(name) & 0x7f
            out[imm] = truth
            out[imm | 0x80] = truth
        return out


if __name__ == "__main__":
    import sys
    cm = CondMap.load(sys.argv[1] if len(sys.argv) > 1 else None)
    for n in sorted(cm.names, key=lambda k: cm.names[k]):
        print(f"0x{cm.names[n]:02x}  {n}")
