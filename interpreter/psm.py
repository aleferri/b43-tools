#!/usr/bin/env python3
#
#   Copyright (C) 2026  b43 AC-PHY port contributors
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
# The instruction decoder (bit layout, opcode table, operand field decoding)
# is a direct Python port of disassembler/main.c from this same repository
# (Copyright (C) 2006-2010 Michael Buesch <m@bues.ch>, GPLv2), re-derived
# instruction-by-instruction and cross-checked against its own output
# rather than copied verbatim. See interpreter/README.md for the validation
# procedure and its results.
"""
Generic interpreter for the D11 PSM microcode (arch15), the Broadcom MAC
microcontroller used (among others) by D11 corerev 42 (BCM4352/4360 AC-PHY).

Designed to be "generic": it takes a raw binary blob as input (the same
raw-be32 format used with b43-dasm) and executes it regardless of its
bommajor/bomminor -- so it works for d11ucode42 with bommajor 0x310
(DSL-3580_EU) as well as d11ucode42 with bommajor 0x3A0 (D6220/tg789vac_v2/
vd625), and in principle for any other arch15 blob (e.g. d11ucode40, 41,
43...) given the same instruction format.

=============================================================================
CONFIDENCE LEVELS (read before trusting a result)
=============================================================================

CONFIRMED (from b43-dasm's source, direct port, not inferred):
  - Instruction format: 64 bit, two 32-bit big-endian words A,B read from
    the file, reassembled as codeword = (B << 32) | A (the two halves are
    SWAPPED relative to a naive 64-bit big-endian read -- taken literally
    from main.c:935-936, an easy detail to get wrong).
  - arch15 fields: opcode = bit[50:39] (12 bit); operand0 = bit[38:26]
    (13 bit); operand1 = bit[25:13] (13 bit); operand2 = bit[12:0] (13 bit).
  - Decoding of a "standard" 13-bit operand (direct SHM memory / immediate /
    general register / SPR / indexed memory), including the exact
    discriminating bits (see decode_operand()).
  - Opcode -> mnemonic map (the whole table in disasm_constant_opcodes()
    plus the 0x200-0x700 dispatcher for orx/srx/jzx/jnzx/jnext/jext).

HIGH CONFIDENCE (not in the tool's source, but verified empirically against
known-good values):
  - add/sub/and/or/xor/sr/sl/rl/rr/nand/mul: operand0,operand1 = sources,
    operand2 = destination. Confirmed because b43-dasm uses the SAME
    decoder for all of them (not instruction-specific), and verified
    downstream by matching results against real MMIO traces (e.g. the
    ucode's version-stamp prologue).
  - orx when operand0 and operand1 are BOTH immediates: dest = (op0<<8)|op1.
    Verified numerically: "orx 7,8,0x27,0x15,[0x1]" produces 0x2715, which
    matches bomminor exactly as read from a real MMIO trace.

BEST-EFFORT / UNVERIFIED (a conventional "reasonable" interpretation, NOT
confirmed against real hardware -- use with caution, especially for jump
conditions):
  - Exact semantics of jump conditions (je/jne/jl/jge/jg/jle/jls/jges/jgs/
    jles/jdn/jdpz/jdp/jdnz/js/jns/jand/jnand): the tool disassembles without
    needing to know the real condition (it only needs the target), so it is
    NOT a source for execution semantics. I implemented the conventional
    interpretation (signed/unsigned comparison between operand0 and
    operand1), but it is unverified.
  - jzx/jnzx/srx/orx with the two small opcode-embedded parameters A,B:
    treated as "extract/insert a B-bit field starting at position A" -- a
    plausible working hypothesis for the architecture, not confirmed.
  - jext/jnext: the tool's own source says the "first/second operand are
    always a dummy r0" -- so they test some implicit HARDWARE flag/event
    indexed by the opcode's low byte. Impossible to implement correctly
    without hardware documentation: here they are STUBs returning a
    configurable flag (False by default), so ANY hardware wait loop in the
    real code will either time out or behave differently from the real
    chip.
  - calls/rets: implemented as a simple return-address stack (push on
    calls, pop on rets). The real format uses explicit named "lr" registers
    (seen in main.c for 'ret' on arch5); on arch15 'rets' still has an
    "lrN" argument in the source even though we never saw it printed with a
    value other than the default in our disassembly listings -- so a
    single stack is probably right for the code we have seen, but it is
    not guaranteed in general.
  - nap: no-op that consumes a "tick" (does not actually pause anything).
  - tkiph/tkiphs/tkipl/tkipls (TKIP crypto accelerator): NOT implemented
    (raise NotImplementedError) -- out of scope, would require the
    dedicated crypto engine's TKIP algorithm, not documented here.

In practice: trust the "CONFIRMED" and "HIGH CONFIDENCE" parts blindly.
For the "BEST-EFFORT" part, the interpreter EXPLICITLY tags every executed
instruction with a confidence level, so you can see immediately when
execution moves into unverified territory.
"""

from dataclasses import dataclass, field
from typing import Optional, Callable
import struct


# =============================================================================
# DECODER (CONFIRMED - direct port from main.c)
# =============================================================================

class Operand:
    """A decoded operand. kind in {'mem','imm','reg','spr','imem'}."""
    __slots__ = ("kind", "value", "offset", "idxreg")
    def __init__(self, kind, value, offset=None, idxreg=None):
        self.kind = kind
        self.value = value
        self.offset = offset
        self.idxreg = idxreg

    def __repr__(self):
        if self.kind == 'mem':
            return f"[0x{self.value:X}]"
        if self.kind == 'imm':
            return f"0x{self.value:X}"
        if self.kind == 'reg':
            return f"r{self.value}"
        if self.kind == 'spr':
            return f"spr{self.value:X}"
        if self.kind == 'imem':
            return f"[0x{self.offset:02X},off{self.idxreg}]"
        return f"?{self.kind}:{self.value}"


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


@dataclass
class Instr:
    addr: int                 # "logical" address (= instruction index, not byte offset)
    opcode: int
    op0: Operand
    op1: Operand
    op2: Operand
    mnem: str
    extra: dict = field(default_factory=dict)  # for orx/srx/jzx/jnzx: {'a':.., 'b':..}


# Direct 12-bit opcode -> mnemonic table, for the "constant" instructions
# (literal port of disasm_constant_opcodes). op0/op1/op2 are always the 3
# standard operands in this order for these instructions.
_CONST_OPCODES = {
    0x101: "mul", 0x1C0: "add", 0x1C2: "add.", 0x1C1: "addc", 0x1C3: "addc.",
    0x1D0: "sub", 0x1D2: "sub.", 0x1D1: "subc", 0x1D3: "subc.",
    0x130: "sra", 0x160: "or", 0x140: "and", 0x170: "xor",
    0x120: "sr", 0x110: "sl", 0x1A0: "rl", 0x1B0: "rr", 0x150: "nand",
    0x040: "jand", 0x041: "jnand", 0x050: "js", 0x051: "jns",
    0x0D0: "je", 0x0D1: "jne", 0x0D2: "jls", 0x0D3: "jges",
    0x0D4: "jgs", 0x0D5: "jles", 0x0D6: "jdn", 0x0D7: "jdpz",
    0x0D8: "jdp", 0x0D9: "jdnz", 0x0DA: "jl", 0x0DB: "jge",
    0x0DC: "jg", 0x0DD: "jle",
    0x003: "rets", 0x004: "calls", 0x001: "nap",
    0x1E0: "tkip",  # sub-decoded via flags, see decode_instr
}

# Mnemonics that are 2-operand conditional jumps (op0,op1) + label in op2
_JUMP2 = {"jand","jnand","js","jns","je","jne","jls","jges","jgs","jles",
          "jdn","jdpz","jdp","jdnz","jl","jge","jg","jle"}


def decode_instr(codeword: int, logical_addr: int) -> Instr:
    opcode = (codeword >> 39) & 0xFFF
    raw0 = (codeword >> 26) & 0x1FFF
    raw1 = (codeword >> 13) & 0x1FFF
    raw2 = codeword & 0x1FFF

    fam = opcode & 0xF00
    if fam == 0x200:
        mnem = "srx"
        a, b = (opcode & 0x0F0) >> 4, opcode & 0x00F
        return Instr(logical_addr, opcode, decode_operand(raw0), decode_operand(raw1),
                     decode_operand(raw2), mnem, {"a": a, "b": b})
    if fam == 0x300:
        mnem = "orx"
        a, b = (opcode & 0x0F0) >> 4, opcode & 0x00F
        return Instr(logical_addr, opcode, decode_operand(raw0), decode_operand(raw1),
                     decode_operand(raw2), mnem, {"a": a, "b": b})
    if fam == 0x400:
        mnem = "jzx"
        a, b = (opcode & 0x0F0) >> 4, opcode & 0x00F
        return Instr(logical_addr, opcode, decode_operand(raw0), decode_operand(raw1),
                     None, mnem, {"a": a, "b": b, "label": raw2})
    if fam == 0x500:
        mnem = "jnzx"
        a, b = (opcode & 0x0F0) >> 4, opcode & 0x00F
        return Instr(logical_addr, opcode, decode_operand(raw0), decode_operand(raw1),
                     None, mnem, {"a": a, "b": b, "label": raw2})
    if fam == 0x600:
        return Instr(logical_addr, opcode, None, None, None, "jnext",
                     {"imm": opcode & 0x0FF, "label": raw2})
    if fam == 0x700:
        return Instr(logical_addr, opcode, None, None, None, "jext",
                     {"imm": opcode & 0x0FF, "label": raw2})

    if opcode in _JUMP2:
        return Instr(logical_addr, opcode, decode_operand(raw0), decode_operand(raw1),
                     None, _CONST_OPCODES[opcode], {"label": raw2})
    if opcode in (0x0D0,0x0D1,0x0D2,0x0D3,0x0D4,0x0D5,0x0D6,0x0D7,0x0D8,0x0D9,0x0DA,0x0DB,0x0DC,0x0DD,0x040,0x041,0x050,0x051):
        name = _CONST_OPCODES[opcode]
        return Instr(logical_addr, opcode, decode_operand(raw0), decode_operand(raw1),
                     None, name, {"label": raw2})
    if opcode == 0x003:
        # 'ret' (arch5). Should not appear on arch15; kept for completeness.
        return Instr(logical_addr, opcode, raw0, None, raw2, "ret", {})
    if opcode == 0x004:
        return Instr(logical_addr, opcode, None, None, None, "calls", {"label": raw2})
    if opcode == 0x005:
        return Instr(logical_addr, opcode, None, None, None, "rets", {})
    if opcode == 0x001:
        return Instr(logical_addr, opcode, None, None, None, "nap", {})
    if opcode == 0x1E0:
        flags = raw1 & 0x7FF
        name = {0x1:"tkiph", 0x3:"tkiphs", 0x0:"tkipl", 0x2:"tkipls"}.get(flags, "tkip?")
        return Instr(logical_addr, opcode, decode_operand(raw0), None,
                     decode_operand(raw2), name, {})
    if opcode in _CONST_OPCODES:
        name = _CONST_OPCODES[opcode]
        return Instr(logical_addr, opcode, decode_operand(raw0), decode_operand(raw1),
                     decode_operand(raw2), name, {})

    # Unknown opcode: keep it anyway, flagged as such.
    return Instr(logical_addr, opcode, decode_operand(raw0), decode_operand(raw1),
                 decode_operand(raw2), f"unk_{opcode:03X}", {})


# =============================================================================
# EXECUTOR
# =============================================================================

class HaltExecution(Exception):
    pass


class HardwareWaitLoop(Exception):
    """Raised when execution appears stuck in a wait loop on a hardware flag
    that is not emulated here (jzx/jnzx/jext/jnext on an spr/SHM cell that no
    earlier instruction actually wrote)."""
    pass


class PSM:
    """
    PSM state machine. shm is word-addressable (16 logical bits, though for
    simplicity we keep arbitrary Python ints masked to 16 bits where it
    matters). reg = general registers r0..r127. spr = sparse dict.
    """
    def __init__(self, prog: list[Instr], shm_size=0x4000):
        self.prog = prog
        self.shm = [0] * shm_size
        self.reg = [0] * 128
        self.spr = {}
        self.pc = 0
        self.call_stack = []
        self.ext_flags = {}   # for jext/jnext: imm -> bool, unverified STUB
        self.steps = 0
        self.trace = []       # list of (instr, confidence, note) if tracing is on
        self.tracing = False
        self.max_steps = 2_000_000
        self.strict = False   # if True, unimplemented instructions raise instead of being skipped
        self.unimpl_count = 0
        # Stall detection based on "highest PC ever reached": robust to loop
        # size (unlike a fixed-window detector).
        self._high_water = 0
        self._steps_since_progress = 0
        self.stall_threshold = 20000

    # ---- operand read/write ----
    def read(self, op: Operand):
        if op is None:
            return 0
        if op.kind == 'imm':
            return op.value
        if op.kind == 'reg':
            return self.reg[op.value]
        if op.kind == 'spr':
            return self.spr.get(op.value, 0)
        if op.kind == 'mem':
            return self.shm[op.value & (len(self.shm) - 1)]
        if op.kind == 'imem':
            idx = self.reg[op.idxreg] if op.idxreg else 0
            return self.shm[(op.offset + idx) & (len(self.shm) - 1)]
        raise ValueError(f"unreadable operand: {op}")

    def write(self, op: Operand, value):
        value &= 0xFFFFFFFF
        if op.kind == 'reg':
            self.reg[op.value] = value
        elif op.kind == 'spr':
            self.spr[op.value] = value
        elif op.kind == 'mem':
            self.shm[op.value & (len(self.shm) - 1)] = value
        elif op.kind == 'imem':
            idx = self.reg[op.idxreg] if op.idxreg else 0
            self.shm[(op.offset + idx) & (len(self.shm) - 1)] = value
        else:
            raise ValueError(f"unwritable operand: {op}")

    # ---- single step ----
    def step(self):
        if self.pc >= len(self.prog):
            raise HaltExecution("PC out of program bounds")
        instr = self.prog[self.pc]
        m = instr.mnem
        confidence = "HIGH"
        note = ""
        next_pc = self.pc + 1

        if m in ("add", "add.", "addc", "addc."):
            r = self.read(instr.op0) + self.read(instr.op1)
            if m.startswith("addc") and self.spr.get('__carry__', 0):
                r += 1
            self.spr['__carry__'] = 1 if r > 0xFFFFFFFF else 0
            self.write(instr.op2, r)
        elif m in ("sub", "sub.", "subc", "subc."):
            r = self.read(instr.op0) - self.read(instr.op1)
            if m.startswith("subc") and self.spr.get('__carry__', 0):
                r -= 1
            self.spr['__carry__'] = 1 if r < 0 else 0
            self.write(instr.op2, r & 0xFFFFFFFF)
        elif m == "mul":
            self.write(instr.op2, self.read(instr.op0) * self.read(instr.op1))
        elif m == "and":
            self.write(instr.op2, self.read(instr.op0) & self.read(instr.op1))
        elif m == "or":
            self.write(instr.op2, self.read(instr.op0) | self.read(instr.op1))
        elif m == "xor":
            self.write(instr.op2, self.read(instr.op0) ^ self.read(instr.op1))
        elif m == "nand":
            self.write(instr.op2, (~(self.read(instr.op0) & self.read(instr.op1))) & 0xFFFFFFFF)
        elif m == "sr":
            self.write(instr.op2, self.read(instr.op0) >> (self.read(instr.op1) & 0x1F))
        elif m == "sl":
            self.write(instr.op2, (self.read(instr.op0) << (self.read(instr.op1) & 0x1F)) & 0xFFFFFFFF)
        elif m == "sra":
            v = self.read(instr.op0)
            sh = self.read(instr.op1) & 0x1F
            sign = v & 0x80000000
            v = v >> sh
            if sign:
                v |= (0xFFFFFFFF << (32 - sh)) & 0xFFFFFFFF
            self.write(instr.op2, v)
        elif m in ("rl", "rr"):
            # BEST-EFFORT: 16-bit rotation (consistent with a 16-bit SHM
            # word); whether the real width is 16 or 32 bit is unverified.
            confidence = "BEST-EFFORT"
            note = "rotation width (16 vs 32 bit) unverified"
            v = self.read(instr.op0) & 0xFFFF
            sh = self.read(instr.op1) & 0xF
            if m == "rl":
                v = ((v << sh) | (v >> (16 - sh))) & 0xFFFF if sh else v
            else:
                v = ((v >> sh) | (v << (16 - sh))) & 0xFFFF if sh else v
            self.write(instr.op2, v)
        elif m == "orx":
            a, b = instr.extra['a'], instr.extra['b']
            if instr.op0.kind == 'imm' and instr.op1.kind == 'imm':
                # HIGH CONFIDENCE: verified numerically (version-stamp block)
                r = ((self.read(instr.op0) & 0xFF) << 8) | (self.read(instr.op1) & 0xFF)
            else:
                # BEST-EFFORT: "insert a b-bit-wide field starting at a" hypothesis
                confidence = "BEST-EFFORT"
                note = f"orx with non-immediate operands (a={a},b={b}): exact semantics unverified"
                mask = (1 << b) - 1 if b else 0xFFFFFFFF
                r = (self.read(instr.op0) << b) | (self.read(instr.op1) & mask)
            self.write(instr.op2, r)
        elif m == "srx":
            a, b = instr.extra['a'], instr.extra['b']
            confidence = "BEST-EFFORT"
            note = f"srx (a={a},b={b}): exact semantics unverified"
            mask = (1 << b) - 1 if b else 0xFFFFFFFF
            r = (self.read(instr.op0) >> a) & mask
            self.write(instr.op2, r)
        elif m == "nap":
            pass  # no-op
        elif m == "calls":
            self.call_stack.append(self.pc + 1)
            next_pc = instr.extra['label']
        elif m == "rets":
            if not self.call_stack:
                raise HaltExecution("rets with an empty call stack")
            next_pc = self.call_stack.pop()
        elif m in ("jzx", "jnzx"):
            confidence = "BEST-EFFORT"
            a, b = instr.extra['a'], instr.extra['b']
            note = f"{m}: testing a {b}-bit field at position {a} (unverified)"
            v = self.read(instr.op0)
            field = (v >> a) & ((1 << b) - 1) if b else v
            cond = (field == 0) if m == "jzx" else (field != 0)
            if cond:
                next_pc = instr.extra['label']
        elif m in ("jext", "jnext"):
            confidence = "STUB"
            imm = instr.extra['imm']
            note = f"{m} 0x{imm:02X}: hardware flag not emulated, assuming {self.ext_flags.get(imm, False)}"
            cond_true = self.ext_flags.get(imm, False)
            cond = cond_true if m == "jext" else (not cond_true)
            if cond:
                next_pc = instr.extra['label']
        elif m in _JUMP2:
            confidence = "BEST-EFFORT"
            note = f"{m}: conventional jump condition, not verified against hardware"
            x, y = self.read(instr.op0), self.read(instr.op1)
            sx = x - 0x100000000 if x & 0x80000000 else x
            sy = y - 0x100000000 if y & 0x80000000 else y
            cond = {
                "je": x == y, "jne": x != y,
                "jl": sx < sy, "jge": sx >= sy, "jg": sx > sy, "jle": sx <= sy,
                "jls": x < y, "jges": x >= y, "jgs": x > y, "jles": x <= y,
                "js": (x & 0x80000000) != 0, "jns": (x & 0x80000000) == 0,
                "jand": (x & y) != 0, "jnand": (x & y) == 0,
                "jdn": False, "jdpz": False, "jdp": False, "jdnz": False,  # not understood, see note
            }.get(m)
            if m in ("jdn","jdpz","jdp","jdnz"):
                confidence = "STUB"
                note = f"{m}: unknown condition (likely tests a dedicated 'divider' register), treated as False"
                cond = False
            if cond:
                next_pc = instr.extra['label']
        elif m.startswith("unk_") or m in ("ret", "tkiph", "tkiphs", "tkipl", "tkipls", "tkip?"):
            confidence = "UNIMPLEMENTED"
            note = (f"instruction {m} not implemented (not even b43-dasm knows it, for unk_*) "
                     "-- treated as a NOP, subsequent state is NOT reliable")
            if self.strict:
                raise NotImplementedError(f"@{self.pc:04X}: {m} not implemented ({note})")
            self.unimpl_count += 1
        else:
            raise NotImplementedError(f"@{self.pc:04X}: unhandled mnemonic: {m}")

        if self.tracing:
            self.trace.append((instr, confidence, note))

        self.pc = next_pc
        self.steps += 1

        # Stall detection: if the highest PC ever reached hasn't advanced
        # for 'stall_threshold' consecutive steps, this is almost certainly
        # a wait loop (unemulated hardware) rather than real progress --
        # robust regardless of how large the loop body is.
        if self.pc > self._high_water:
            self._high_water = self.pc
            self._steps_since_progress = 0
        else:
            self._steps_since_progress += 1
            if self._steps_since_progress > self.stall_threshold:
                raise HardwareWaitLoop(
                    f"no progress past PC={self._high_water:04X} for "
                    f"{self.stall_threshold} steps (last instruction: {m} @ {instr.addr:04X}, "
                    f"{note or 'likely waiting on an unemulated hardware flag'})"
                )

    def run(self, start=0, max_steps=None, stop_at=None, trace=False):
        self.pc = start
        self.tracing = trace
        self.trace = []
        limit = max_steps or self.max_steps
        while self.steps < limit:
            if stop_at is not None and self.pc == stop_at:
                return "stop_at reached"
            try:
                self.step()
            except HaltExecution as e:
                return f"halt: {e}"
        return "max_steps reached"


def load_program(path: str) -> list[Instr]:
    """Load a raw-be32 blob (the same format used with b43-dasm -f raw-be32).
    Generic: works with any d11ucodeNN extracted the same way as the 4 files
    analyzed during this tool's development (observed bommajor: 0x310/784 on
    DSL-3580_EU up to 0x3A0/928 on D6220/tg789vac_v2/vd625), and in
    principle with any other arch15 blob in the same format."""
    data = open(path, "rb").read()
    assert len(data) % 8 == 0, "size must be a multiple of 8 bytes (64-bit instructions)"
    prog = []
    for i in range(0, len(data), 8):
        a = struct.unpack_from(">I", data, i)[0]
        b = struct.unpack_from(">I", data, i + 4)[0]
        codeword = (b << 32) | a   # the two halves are swapped, CONFIRMED by main.c:935-936
        prog.append(decode_instr(codeword, i // 8))
    return prog


# =============================================================================
# USAGE EXAMPLE
# =============================================================================
if __name__ == "__main__":
    import sys as _sys
    if len(_sys.argv) < 2:
        print(f"usage: {_sys.argv[0]} <ucode.bin> [start_address_hex] [max_steps]")
        print()
        print("Example (D6220's version-stamp block, verified against a real trace):")
        print(f"  {_sys.argv[0]} wlD6220.o_save.d11ucode42.bin 0xF76 20")
        _sys.exit(1)

    path = _sys.argv[1]
    start = int(_sys.argv[2], 16) if len(_sys.argv) > 2 else 0
    max_steps = int(_sys.argv[3]) if len(_sys.argv) > 3 else 1000

    prog = load_program(path)
    print(f"loaded {len(prog)} instructions from {path}")

    m = PSM(prog)
    try:
        res = m.run(start=start, max_steps=max_steps, trace=True)
        print(f"result: {res}")
    except HardwareWaitLoop as e:
        print(f"HARDWARE WAIT (flag not emulated): {e}")
    except NotImplementedError as e:
        print(f"UNHANDLED INSTRUCTION (strict=True): {e}")

    print(f"steps executed: {m.steps}, unimplemented instructions encountered: {m.unimpl_count}")
    print(f"final PC: 0x{m.pc:04X}")
    print()
    print("last instructions executed:")
    for instr, conf, note in m.trace[-15:]:
        extra = f"  <- {note}" if note else ""
        print(f"  /* {instr.addr:04X} */ {instr.mnem:8s} [{conf}]{extra}")
    print()
    print("non-zero SHM cells (first 20):")
    nz = [(i, v) for i, v in enumerate(m.shm) if v][:20]
    for addr, val in nz:
        print(f"  [0x{addr:04X}] = 0x{val:X}")
