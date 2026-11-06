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
#
# The decoder is a port of the arch 5 cases of disassembler/main.c
# (Copyright (C) 2006-2010 Michael Buesch <m@bues.ch>, GPLv2).
"""
The arch5 D11 PSM instruction set, core revisions 5..14 (b43-asm's %arch 5,
fwcutter's EXT_UCODE_2), on top of psmlib.py.

CONFIRMED (from b43-dasm's source, direct port, not inferred):
  - fields: opcode = bit[47:36] (12 bit); operand0 = bit[35:24],
    operand1 = bit[23:12], operand2 = bit[11:0] (12 bit each).
  - Decoding of a "standard" 12-bit operand (see decode_operand()).
  - call lrX,l (0x002) names a link register in operand0 and the target in
    operand2; ret lrX,lrY (0x003) names two link registers in operand0 and
    operand2. calls/rets (0x004/0x005) do not exist.

Nothing of this architecture was measured on hardware: jand/jnand and orx
7,8 are BEST-EFFORT here (see psm15.py for what was measured there).

BEST-EFFORT:
  - lrN is SPR 0x068+N (SPR_PC0..3 in debug/include/spr.inc). call stores
    the address after it in lrX and jumps; ret jumps to lrY and stores the
    address after it in lrX, lrY read first ("ret lr0, lr0" is how
    OpenFWWF returns).
"""

import sys

import psmlib
from psmlib import Instr, Operand

NAME = "arch5"
TOP_BIT = 48        # bits 63:48 of a codeword are clear

# Link registers lr0..lr3.
SPR_LINK0 = 0x068


def decode_operand(raw12: int) -> Operand:
    """arch5, 12 bit. Direct port of disasm_std_operand's 'case 5'."""
    if not (raw12 & 0x800):
        return Operand('mem', raw12 & 0x7FF)
    if (raw12 & 0xC00) == 0xC00:
        mask = 0x3FF
        signmask = 0x200
        v = raw12 & mask
        if v & signmask:
            v = v | (~mask & 0xFFFF)
            v &= 0xFFFF
            if v >= 0x8000:
                v -= 0x10000
        return Operand('imm', v)
    if (raw12 & 0xFC0) == 0xBC0:
        return Operand('reg', raw12 & 0x3F)
    if (raw12 & 0xE00) == 0x800:
        return Operand('spr', raw12 & 0x1FF)
    if (raw12 & 0xE00) == 0xA00:
        offset = raw12 & 0x3F
        idxreg = (raw12 >> 6) & 0x7
        return Operand('imem', None, offset=offset, idxreg=idxreg)
    return Operand('raw', raw12)


def decode_instr(codeword: int, logical_addr: int) -> Instr:
    opcode = (codeword >> 36) & 0xFFF
    raw0 = (codeword >> 24) & 0xFFF
    raw1 = (codeword >> 12) & 0xFFF
    raw2 = codeword & 0xFFF

    if opcode == 0x002:
        return Instr(logical_addr, opcode, None, None, None, "call",
                     {"lr": raw0, "label": raw2})
    if opcode == 0x003:
        return Instr(logical_addr, opcode, None, None, None, "ret",
                     {"lr_store": raw0, "lr_load": raw2})
    return (psmlib.decode_common(opcode, raw0, raw1, raw2, logical_addr, decode_operand, 0x3FF)
            or psmlib.decode_unknown(opcode, raw0, raw1, raw2, logical_addr, decode_operand))


def load_program(path: str) -> list[Instr]:
    return [decode_instr(w, i) for i, w in enumerate(psmlib.read_codewords(path, TOP_BIT))]


class PSM(psmlib.PSM):
    def execute_isa(self, instr):
        if instr.mnem == "call":
            lr = instr.extra['lr']
            self.store(('spr', SPR_LINK0 + lr), self.pc + 1)
            return instr.extra['label'], "BEST-EFFORT", f"call lr{lr}: return address in SPR_PC{lr}"
        if instr.mnem == "ret":
            store, load = instr.extra['lr_store'], instr.extra['lr_load']
            target = self.spr.get(SPR_LINK0 + load, 0)
            self.store(('spr', SPR_LINK0 + store), self.pc + 1)
            return target, "BEST-EFFORT", f"ret lr{store}, lr{load}"
        return None


if __name__ == "__main__":
    psmlib.cli(sys.modules[__name__], "ucode5.fw 0 40")
