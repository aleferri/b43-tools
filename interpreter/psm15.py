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
# The decoder is a port of the arch 15 cases of disassembler/main.c
# (Copyright (C) 2006-2010 Michael Buesch <m@bues.ch>, GPLv2).
"""
The arch15 D11 PSM instruction set, core revisions 15 and later (b43-asm's
%arch 15, fwcutter's EXT_UCODE_3), on top of psmlib.py.

CONFIRMED (from b43-dasm's source, direct port, not inferred):
  - fields: opcode = bit[50:39] (12 bit); operand0 = bit[38:26] (13 bit);
    operand1 = bit[25:13] (13 bit); operand2 = bit[12:0] (13 bit).
  - Decoding of a "standard" 13-bit operand (direct SHM memory / immediate /
    general register / SPR / indexed memory), including the exact
    discriminating bits (see decode_operand()).

HIGH CONFIDENCE (measured on arch15 hardware):
  - orx 7,8 with two immediates (b43-asm's encoding of a 16-bit mov):
    dest = (op0<<8)|op1. Verified numerically: "orx 7,8,0x27,0x15,[0x1]"
    produces 0x2715, which matches bomminor exactly as read from a real MMIO
    trace of corerev 42. The general orx formula reduces to this at M=7,S=8.
  - jand/jnand A,B: jand (0x041) jumps if A & B is nonzero, jnand (0x040)
    if it is zero, the names swapped with respect to upstream b43-tools (see
    the top-level README). Measured on core revisions 22 (BCM6362) and 42
    with a test ucode run for the operand pairs (0,0), (0,1), (1,0), (1,1).

BEST-EFFORT:
  - calls/rets: implemented as a simple return-address stack (push on
    calls, pop on rets). Unlike arch5's call/ret, which name link registers,
    arch15's calls/rets carry none: b43-dasm only checks that their operands
    are the fixed r0,r0 pattern. The depth of the hardware stack is unknown.
  - Opcode 0x003, arch5's ret, is decoded as b43-dasm decodes it but not
    executed; opcode 0x002, arch5's call, has no arch15 meaning in b43-dasm
    and is decoded as unk_002.
"""

import sys

import psmlib
from psmlib import Instr, Operand, HaltExecution

NAME = "arch15"
TOP_BIT = 51        # bits 63:51 of a codeword are clear


def decode_operand(raw13: int) -> Operand:
    """arch15, 13 bit. Direct port of disasm_std_operand's 'case 15'."""
    if not (raw13 & 0x1000):
        return Operand('mem', raw13 & 0xFFF)
    if (raw13 & 0x1800) == 0x1800:
        mask = 0x7FF
        signmask = 0x400
        v = raw13 & mask
        if v & signmask:
            v = v | (~mask & 0xFFFF)
            v &= 0xFFFF
            if v >= 0x8000:
                v -= 0x10000
        return Operand('imm', v)
    if (raw13 & 0x1F80) == 0x1780:
        return Operand('reg', raw13 & 0x7F)
    if (raw13 & 0x1C00) == 0x1000:
        return Operand('spr', raw13 & 0x7FF)
    if (raw13 & 0x1C00) == 0x1400:
        offset = raw13 & 0x7F
        idxreg = (raw13 >> 7) & 0x7
        return Operand('imem', None, offset=offset, idxreg=idxreg)
    # Should not happen (exhaustive coverage, same as the original tool)
    return Operand('raw', raw13)


def decode_instr(codeword: int, logical_addr: int) -> Instr:
    opcode = (codeword >> 39) & 0xFFF
    raw0 = (codeword >> 26) & 0x1FFF
    raw1 = (codeword >> 13) & 0x1FFF
    raw2 = codeword & 0x1FFF

    if opcode == 0x003:
        return Instr(logical_addr, opcode, None, None, None, "ret",
                     {"lr_store": raw0, "lr_load": raw2})
    if opcode == 0x004:
        return Instr(logical_addr, opcode, None, None, None, "calls", {"label": raw2})
    if opcode == 0x005:
        return Instr(logical_addr, opcode, None, None, None, "rets", {})
    return (psmlib.decode_common(opcode, raw0, raw1, raw2, logical_addr, decode_operand, 0x7FF)
            or psmlib.decode_unknown(opcode, raw0, raw1, raw2, logical_addr, decode_operand))


def load_program(path: str) -> list[Instr]:
    return [decode_instr(w, i) for i, w in enumerate(psmlib.read_codewords(path, TOP_BIT))]


class PSM(psmlib.PSM):
    def execute_isa(self, instr):
        if instr.mnem == "calls":
            self.call_stack.append(self.pc + 1)
            return instr.extra['label'], "HIGH", ""
        if instr.mnem == "rets":
            if not self.call_stack:
                raise HaltExecution("rets with an empty call stack")
            return self.call_stack.pop(), "HIGH", ""
        return None

    def hw_checked(self, instr):
        if instr.mnem in ("jand", "jnand"):
            return f"{instr.mnem}: measured on hardware (jand: A & B != 0, jnand: == 0)"
        if (instr.mnem == "orx" and (instr.extra['a'], instr.extra['b']) == (7, 8)
                and instr.op0.kind == 'imm' and instr.op1.kind == 'imm'):
            return "orx 7,8 with two immediates: measured on hardware"
        return None


if __name__ == "__main__":
    psmlib.cli(sys.modules[__name__],
               "wlD6220.o_save.d11ucode42.bin 0xF76 20  (D6220's version-stamp block)")
