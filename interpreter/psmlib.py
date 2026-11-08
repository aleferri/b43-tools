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
# The instruction decoder (opcode table, here and in psm5.py/psm15.py the bit
# layout and operand field decoding) is a direct Python port of
# disassembler/main.c from this same repository
# (Copyright (C) 2006-2010 Michael Buesch <m@bues.ch>, GPLv2), re-derived
# instruction-by-instruction and cross-checked against its own output
# rather than copied verbatim. See interpreter/README.md for the validation
# procedure and its results.
"""
What the two D11 PSM instruction sets have in common: the opcode table, the
operand kinds and the executor. The PSM is the Broadcom MAC microcontroller;
b43-dasm knows two instruction sets for it, and psm5.py (core revisions 5..14)
and psm15.py (15 and later) each add what is their own on top of this module:
the instruction word and operand layout, the call/return instructions, and
which instruction forms were measured on hardware of that architecture.

It takes a ucode blob as input (raw-be32 as used with b43-dasm, or a b43 .fw
image with its header) and executes it regardless of its bommajor/bomminor:
the decoder works on the bit-level encoding, not on anything file-specific.

=============================================================================
CONFIDENCE LEVELS (read before trusting a result)
=============================================================================

CONFIRMED (from b43-dasm's source, direct port, not inferred):
  - Instruction format: 64 bit, two 32-bit big-endian words A,B read from
    the file, reassembled as codeword = (B << 32) | A (the two halves are
    SWAPPED relative to a naive 64-bit big-endian read, as the
    FMT_RAW_BE32 case of main.c does -- an easy detail to get wrong).
  - Opcode -> mnemonic map (the whole table in disasm_constant_opcodes()
    plus the 0x200-0x700 dispatcher for orx/srx/jzx/jnzx/jnext/jext), the
    same for both architectures but for opcodes 0x002..0x005.

HIGH CONFIDENCE (not in the tool's source, but verified empirically against
known-good values):
  - add/sub/and/or/xor/sr/sl/rl/rr/nand/mul: operand0,operand1 = sources,
    operand2 = destination. Confirmed because b43-dasm uses the SAME
    decoder for all of them (not instruction-specific), and verified
    downstream by matching results against real MMIO traces (e.g. the
    ucode's version-stamp prologue).
  - jzx/jnzx M,S,A,B: jump if the M+1 bits of A at S are (non)zero. This is
    what b43-asm's emulate_jand_insn() relies on when it turns a wide
    "jand/jnand A,MASK" into jzx/jnzx; it always emits B as the immediate 0,
    and every jzx/jnzx of the corerev 42 blobs has B = 0 too, so whether the
    hardware would combine a nonzero B with A is never exercised.
  - Offset registers off0..off6 of [imm,offN] operands are SPRs 0x060..0x066
    (see SPR_OFFSET0).
  - Forms an instruction-set module reports through hw_checked(): measured
    on hardware of that architecture, see psm15.py. Elsewhere jand/jnand
    follow b43-asm's README as this fork names them, and orx follows the
    general formula below: BEST-EFFORT.

BEST-EFFORT / UNVERIFIED (a conventional "reasonable" interpretation, NOT
confirmed against real hardware -- use with caution, especially for jump
conditions):
  - The other two-operand jumps (je/jne/jl/jge/jg/jle/jls/jges/jgs/jles/
    jdn/jdpz/jdp/jdnz/js/jns): the disassembler only needs their target, so
    it is not a source for their semantics. They follow b43-asm's README;
    OpenFWWF's comments agree for je/jne, the unsigned jl/jge/jg and the
    signed jges/jgs. js/jns and jdn..jdnz have no such check.
  - orx M,S,A,B: A rotated left by S, its M+1 bits at S inserted over B.
    srx M,S,A,B: the M+1 bits at S of the 32-bit B:A. Plausible, not
    checked against hardware.
  - jext/jnext: their operands are a dummy r0 in b43-dasm, so they test an
    external hardware condition selected by the low 7 bits of the opcode,
    bit 7 asking for an EOI of the same signal. Only COND_TRUE (0x7F) is
    known; every other selector is a STUB read from ext_flags (False by
    default), so a loop waiting on the hardware does not end here unless
    the caller models that condition.
  - nap: no-op that consumes a "tick" (does not actually pause anything).
  - tkiph/tkiphs/tkipl/tkipls (TKIP crypto accelerator): NOT implemented.
    Executed as a no-op and counted in unimpl_count; with strict=True they
    raise NotImplementedError instead.

In practice: rely on the "CONFIRMED" and "HIGH CONFIDENCE" parts.
For the "BEST-EFFORT" part, the interpreter EXPLICITLY tags every executed
instruction with a confidence level, so you can see immediately when
execution moves into unverified territory.
"""

from dataclasses import dataclass, field
from typing import Optional, Callable
import struct


# =============================================================================
# DECODER, the part both architectures share (CONFIRMED - port from main.c)
# =============================================================================

# The jext/jnext selector that is always true (OpenFWWF's cond.inc).
COND_TRUE = 0x7F

# Offset registers off0..off6 of [imm,offN] operands are SPRs 0x060..0x066:
# debug/include/spr.inc names them SPR_BASE0..6, and the corerev 42 blobs
# set spr060..066 while never touching spr0C0..0C5.
SPR_OFFSET0 = 0x060


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
            return f"0x{self.value & 0xFFFF:X}"
        if self.kind == 'reg':
            return f"r{self.value}"
        if self.kind == 'spr':
            return f"spr{self.value:X}"
        if self.kind == 'imem':
            return f"[0x{self.offset:02X},off{self.idxreg}]"
        return f"?{self.kind}:{self.value}"


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
    0x040: "jnand", 0x041: "jand", 0x050: "js", 0x051: "jns",
    0x0D0: "je", 0x0D1: "jne", 0x0D2: "jls", 0x0D3: "jges",
    0x0D4: "jgs", 0x0D5: "jles", 0x0D6: "jdn", 0x0D7: "jdpz",
    0x0D8: "jdp", 0x0D9: "jdnz", 0x0DA: "jl", 0x0DB: "jge",
    0x0DC: "jg", 0x0DD: "jle",
    0x1E0: "tkip",  # sub-decoded via flags, see decode_common
}

# Mnemonics that are 2-operand conditional jumps (op0,op1) + label in op2
_JUMP2 = {"jand","jnand","js","jns","je","jne","jls","jges","jgs","jles",
          "jdn","jdpz","jdp","jdnz","jl","jge","jg","jle"}


def decode_common(opcode, raw0, raw1, raw2, addr, operand, tkip_flags):
    """The instructions both architectures encode alike, given the operand
    decoder and TKIP flag mask of one; None for any other opcode."""
    fam = opcode & 0xF00
    if fam in (0x200, 0x300):
        mnem = "srx" if fam == 0x200 else "orx"
        a, b = (opcode & 0x0F0) >> 4, opcode & 0x00F
        return Instr(addr, opcode, operand(raw0), operand(raw1), operand(raw2), mnem,
                     {"a": a, "b": b})
    if fam in (0x400, 0x500):
        mnem = "jzx" if fam == 0x400 else "jnzx"
        a, b = (opcode & 0x0F0) >> 4, opcode & 0x00F
        return Instr(addr, opcode, operand(raw0), operand(raw1), None, mnem,
                     {"a": a, "b": b, "label": raw2})
    if fam in (0x600, 0x700):
        mnem = "jnext" if fam == 0x600 else "jext"
        return Instr(addr, opcode, None, None, None, mnem,
                     {"imm": opcode & 0x0FF, "label": raw2})

    if _CONST_OPCODES.get(opcode) in _JUMP2:
        return Instr(addr, opcode, operand(raw0), operand(raw1), None, _CONST_OPCODES[opcode],
                     {"label": raw2})
    if opcode == 0x001:
        return Instr(addr, opcode, None, None, None, "nap", {})
    if opcode == 0x1E0:
        name = {0x1:"tkiph", 0x3:"tkiphs", 0x0:"tkipl", 0x2:"tkipls"}.get(raw1 & tkip_flags, "tkip?")
        return Instr(addr, opcode, operand(raw0), None, operand(raw2), name, {})
    if opcode in _CONST_OPCODES:
        return Instr(addr, opcode, operand(raw0), operand(raw1), operand(raw2),
                     _CONST_OPCODES[opcode], {})
    return None


def decode_unknown(opcode, raw0, raw1, raw2, addr, operand):
    """Keep an opcode b43-dasm has no name for, flagged as such."""
    return Instr(addr, opcode, operand(raw0), operand(raw1), operand(raw2),
                 f"unk_{opcode:03X}", {})


# b43 .fw header (fwcutter/fwcutter.h): type, version, 2 pad bytes, be32 size.
FW_HEADER = struct.Struct(">BBxxI")
FW_HDR_VER = 1


def strip_fw_header(data: bytes, fw_type: str):
    """The payload of a b43 .fw image of the given type ('u' ucode, 'i'
    initvals), or None if data does not start with such a header."""
    if len(data) < FW_HEADER.size:
        return None
    t, ver, _size = FW_HEADER.unpack_from(data)
    if t != ord(fw_type) or ver != FW_HDR_VER:
        return None
    return data[FW_HEADER.size:]


def unpack_words8(data: bytes) -> list[int]:
    """Two big-endian 32-bit words per instruction, the low one first: what
    b43 writes to the ucode memory and b43-dasm reads."""
    words = []
    for i in range(0, len(data), 8):
        a, b = struct.unpack_from(">II", data, i)
        words.append((b << 32) | a)   # the two halves are swapped, as in b43-dasm
    return words


def unpack_words7(data: bytes) -> list[int]:
    """Seven bytes per instruction, the codeword big-endian: how the vendor
    driver stores some ucode (the sslpn and lcn ones of DSL-3580_EU), which
    wlc_ucode_write_byte writes as bytes 3..6 and then 0..2."""
    return [int.from_bytes(data[i:i + 7], "big") for i in range(0, len(data), 7)]


def read_codewords(path: str, top: int) -> list[int]:
    """The 64-bit codewords of a ucode blob, raw or in a b43 .fw image, the
    same bytes after a header; in 8-byte words or packed in 7 bytes. The
    packing is the one whose length fits and that, like b43-dasm, leaves no
    codeword with bits set from `top` up."""
    data = open(path, "rb").read()
    payload = strip_fw_header(data, 'u')
    data = data if payload is None else payload
    fits = [unpack(data) for size, unpack in ((8, unpack_words8), (7, unpack_words7))
            if len(data) % size == 0]
    fits = [words for words in fits if not any(w >> top for w in words)]
    if len(fits) != 1:
        raise ValueError(f"{path}: " + ("not ucode of this architecture, in 8-byte words or "
                                        "7-byte packed instructions" if not fits else
                                        "fits both 8-byte words and 7-byte packing"))
    return fits[0]


# =============================================================================
# EXECUTOR
# =============================================================================

class HaltExecution(Exception):
    pass


class HardwareWaitLoop(Exception):
    """Raised when the PC has not passed its highest value for
    stall_threshold steps: almost always a loop waiting on hardware that is
    not modelled here."""
    pass


class PSM:
    """
    PSM state machine. shm is word-addressable (16 logical bits, though for
    simplicity we keep arbitrary Python ints masked to 16 bits where it
    matters). reg = general registers r0..r127. spr = sparse dict.

    The instruction-set modules subclass it: execute_isa() runs the
    instructions of their own (call/return), hw_checked() names the forms
    measured on their hardware.
    """
    def __init__(self, prog: list[Instr], shm_size=0x4000):
        self.prog = prog
        self.shm = [0] * shm_size
        self.reg = [0] * 128
        self.spr = {}
        self.pc = 0
        self.call_stack = []
        self.carry = 0
        self.ext_flags = {}   # for jext/jnext: selector & 0x7F -> bool, unverified STUB
        self.steps = 0
        self.trace = []       # list of (instr, confidence, note) if tracing is on
        self.tracing = False
        self.max_steps = 2_000_000
        self.strict = False   # if True, unimplemented instructions raise instead of being skipped
        self.unimpl_count = 0
        # Called as observer(loc, old, new) on every write to a register, SPR
        # or shared memory, loc being ('reg'|'spr'|'shm', index); within
        # step(), pc is still the writer's.
        self.observer = None
        # Called as on_eoi(selector & 0x7F) after a jext/jnext with bit 7 has
        # tested the condition, for a hardware model to acknowledge the event.
        self.on_eoi = None
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
            idx = self.spr.get(SPR_OFFSET0 + op.idxreg, 0)
            return self.shm[(op.offset + idx) & (len(self.shm) - 1)]
        raise ValueError(f"unreadable operand: {op}")

    def write(self, op: Operand, value):
        if op.kind == 'imm':
            # "or sprN, 0, 0": the ucode reads a register for its side effect
            # and drops the result
            return
        self.store(self.location(op), value & 0xFFFF)

    def location(self, op: Operand):
        """The state cell a writable operand names: ('reg'|'spr'|'shm', index)."""
        if op.kind == 'reg':
            return ('reg', op.value)
        if op.kind == 'spr':
            return ('spr', op.value)
        if op.kind == 'mem':
            return ('shm', op.value & (len(self.shm) - 1))
        if op.kind == 'imem':
            idx = self.spr.get(SPR_OFFSET0 + op.idxreg, 0)
            return ('shm', (op.offset + idx) & (len(self.shm) - 1))
        raise ValueError(f"unwritable operand: {op}")

    def peek(self, loc):
        kind, n = loc
        if kind == 'reg':
            return self.reg[n]
        if kind == 'spr':
            return self.spr.get(n, 0)
        return self.shm[n]

    def store(self, loc, value):
        """Every register, SPR and shared-memory write goes through here, so an
        observer sees them all."""
        old = self.peek(loc)
        kind, n = loc
        if kind == 'reg':
            self.reg[n] = value
        elif kind == 'spr':
            self.spr[n] = value
        else:
            self.shm[n] = value
        if self.observer:
            self.observer(loc, old, value)

    # ---- what an instruction-set module adds ----
    def execute_isa(self, instr):
        """Execute an instruction of this architecture's own: return
        (next_pc, confidence, note), or None to leave it to step()."""
        return None

    def hw_checked(self, instr):
        """A note if this form of instr was measured on hardware of this
        architecture, else None."""
        return None

    # ---- single step ----
    # Mnemonics an instruction-set module executes in execute_isa().
    ISA_MNEMS = frozenset()

    def step(self):
        if self.pc >= len(self.prog):
            raise HaltExecution("PC out of program bounds")
        instr = self.prog[self.pc]
        m = instr.mnem
        confidence = note = None
        isa = self.execute_isa(instr) if m in self.ISA_MNEMS else None
        if isa is not None:
            next_pc, confidence, note = isa
        else:
            run = _EXEC.get(m) or (_EXEC_UNIMPL if m.startswith("unk_") or m in _UNIMPL else None)
            if run is None:
                raise NotImplementedError(f"@{self.pc:04X}: unhandled mnemonic: {m}")
            next_pc = run(self, instr)
            if next_pc is None:
                next_pc = self.pc + 1

        if self.tracing:
            if confidence is None:
                confidence, note = self.describe(instr)
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
                if confidence is None:
                    confidence, note = self.describe(instr)
                raise HardwareWaitLoop(
                    f"no progress past PC={self._high_water:04X} for "
                    f"{self.stall_threshold} steps (last instruction: {m} @ {instr.addr:04X}, "
                    f"{note or 'likely waiting on an unemulated hardware flag'})"
                )

    def describe(self, instr):
        """(confidence, note) of an executed instruction."""
        m = instr.mnem
        if m in ("rl", "rr"):
            # BEST-EFFORT: 16-bit rotation (consistent with a 16-bit SHM
            # word); whether the real width is 16 or 32 bit is unverified.
            return "BEST-EFFORT", "rotation width (16 vs 32 bit) unverified"
        if m == "orx":
            a, b = instr.extra['a'], instr.extra['b']
            note = self.hw_checked(instr)
            if note:
                return "HIGH", note
            return "BEST-EFFORT", f"orx M={a},S={b}: not checked against hardware in this form"
        if m == "srx":
            a, b = instr.extra['a'], instr.extra['b']
            return "BEST-EFFORT", f"srx M={a},S={b}: shift right over two registers, not checked on hardware"
        if m in ("jzx", "jnzx"):
            a, b = instr.extra['a'], instr.extra['b']
            # jzx/jnzx M,S,SRC,Y,LABEL: per assembler/main.c's emulate_jand_insn()
            # -- the actual generator of "jand REG,0xMASK,LABEL" once 0xMASK is
            # too wide for a plain JAND immediate. first_bit/last_bit come from
            # the contiguous-bitmask scan; M = last_bit-first_bit (width-1),
            # S = first_bit-1 (0-indexed position). SRC is oper[2] -- whichever
            # of the original two JAND operands was NOT the immediate mask.
            # Y (-> op1 here) is always a dummy immediate (em_imm.imm = 0):
            #     em_ol.oper[3] = &em_op_y;
            # it is never read or combined with SRC. So this tests (M+1) bits
            # of op0 ALONE, starting at bit S -- not a 32-bit op0:op1 pair.
            return "HIGH", f"{m} width={a+1},pos={b}: testing bits [{b+a}:{b}] of op0 alone"
        if m in ("jext", "jnext"):
            imm = instr.extra['imm']
            if (imm & 0x7F) == COND_TRUE:
                return "HIGH CONFIDENCE", f"{m} 0x{imm:02X}: COND_TRUE, always true"
            cond_true = self.ext_flags.get(imm & 0x7F, False)
            return "STUB", f"{m} 0x{imm:02X}: hardware flag not emulated, assuming {cond_true}"
        if m in _JUMP2:
            checked = self.hw_checked(instr)
            if checked:
                return "HIGH CONFIDENCE", checked
            if m in ("jand", "jnand"):
                return "BEST-EFFORT", f"{m}: this fork's naming (jand: A & B != 0), not measured on this architecture"
            return "BEST-EFFORT", f"{m}: b43-asm's definition, not verified against hardware"
        if m.startswith("unk_") or m in _UNIMPL:
            return "UNIMPLEMENTED", (f"instruction {m} not implemented (not even b43-dasm knows it, for unk_*) "
                                     "-- treated as a NOP, subsequent state is NOT reliable")
        return "HIGH", ""

    def goto(self, pc):
        """Continue at pc, restarting the stall detection from there."""
        self.pc = pc
        self._high_water = pc
        self._steps_since_progress = 0

    def run(self, start=0, max_steps=None, stop_at=None, trace=False):
        self.goto(start)
        self.tracing = trace
        self.trace = []
        limit = self.steps + (max_steps or self.max_steps)
        while self.steps < limit:
            if stop_at is not None and self.pc == stop_at:
                return "stop_at reached"
            try:
                self.step()
            except HaltExecution as e:
                return f"halt: {e}"
        return "max_steps reached"


def _add(psm, i):
    m = i.mnem
    # b43-asm: the "c" forms add the carry, the "." forms set it
    r = (psm.read(i.op0) & 0xFFFF) + (psm.read(i.op1) & 0xFFFF)
    if m[3:4] == "c":
        r += psm.carry
    if m[-1] == ".":
        psm.carry = int(r > 0xFFFF)
    psm.write(i.op2, r)


def _sub(psm, i):
    m = i.mnem
    r = (psm.read(i.op0) & 0xFFFF) - (psm.read(i.op1) & 0xFFFF)
    if m[3:4] == "c":
        r -= psm.carry
    if m[-1] == ".":
        psm.carry = int(r < 0)
    psm.write(i.op2, r)


def _mul(psm, i):
    # b43-asm: spr06d = low half of the product, rD = high half
    r = (psm.read(i.op0) & 0xFFFF) * (psm.read(i.op1) & 0xFFFF)
    psm.store(('spr', 0x06D), r & 0xFFFF)
    psm.write(i.op2, r >> 16)


def _sra(psm, i):
    v = psm.read(i.op0) & 0xFFFF
    sh = psm.read(i.op1) & 0x1F
    if v & 0x8000:
        v -= 0x10000
    psm.write(i.op2, v >> sh)


def _rotate(psm, i):
    v = psm.read(i.op0) & 0xFFFF
    sh = psm.read(i.op1) & 0xF
    if i.mnem == "rl":
        v = ((v << sh) | (v >> (16 - sh))) & 0xFFFF if sh else v
    else:
        v = ((v >> sh) | (v << (16 - sh))) & 0xFFFF if sh else v
    psm.write(i.op2, v)


def _orx(psm, i):
    # orx M,S,A,B: A rotated left by S, M+1 bits of it at S over B
    a, b = i.extra['a'], i.extra['b']
    x = psm.read(i.op0) & 0xFFFF
    rot = ((x << b) | (x >> (16 - b))) & 0xFFFF if b else x
    mask = (((1 << (a + 1)) - 1) << b) & 0xFFFF
    psm.write(i.op2, (rot & mask) | (psm.read(i.op1) & ~mask & 0xFFFF))


def _srx(psm, i):
    # srx M,S,A,B: M+1 bits at S of B:A (B the high word)
    a, b = i.extra['a'], i.extra['b']
    v = ((psm.read(i.op1) & 0xFFFF) << 16) | (psm.read(i.op0) & 0xFFFF)
    psm.write(i.op2, (v >> b) & ((1 << (a + 1)) - 1))


def _jzx(psm, i):
    a, b = i.extra['a'], i.extra['b']
    field = (psm.read(i.op0) >> b) & ((1 << (a + 1)) - 1)
    if (field == 0) == (i.mnem == "jzx"):
        return i.extra['label']
    return None


def _jext(psm, i):
    imm = i.extra['imm']
    if (imm & 0x7F) == COND_TRUE:
        cond_true = True
    else:
        # bit 7 asks for an EOI of the same signal once it is tested
        cond_true = psm.ext_flags.get(imm & 0x7F, False)
        if imm & 0x80 and psm.on_eoi:
            psm.on_eoi(imm & 0x7F)
    if cond_true == (i.mnem == "jext"):
        return i.extra['label']
    return None


def _signed(v):
    return v - 0x10000 if v & 0x8000 else v


# b43-asm: the "s" comparisons are the signed ones; jdn..jdnz test the
# difference without the carry
_COMPARE = {
    "je": lambda x, y: x == y, "jne": lambda x, y: x != y,
    "jls": lambda x, y: _signed(x) < _signed(y), "jges": lambda x, y: _signed(x) >= _signed(y),
    "jgs": lambda x, y: _signed(x) > _signed(y), "jles": lambda x, y: _signed(x) <= _signed(y),
    "jl": lambda x, y: x < y, "jge": lambda x, y: x >= y,
    "jg": lambda x, y: x > y, "jle": lambda x, y: x <= y,
    "js": lambda x, y: (x & y) == x, "jns": lambda x, y: (x & y) != x,
    "jand": lambda x, y: (x & y) != 0, "jnand": lambda x, y: (x & y) == 0,
    "jdn": lambda x, y: _signed((x - y) & 0xFFFF) < 0,
    "jdpz": lambda x, y: _signed((x - y) & 0xFFFF) >= 0,
    "jdp": lambda x, y: _signed((x - y) & 0xFFFF) > 0,
    "jdnz": lambda x, y: _signed((x - y) & 0xFFFF) <= 0,
}


def _jump2(psm, i):
    compare = _COMPARE.get(i.mnem)
    if compare and compare(psm.read(i.op0) & 0xFFFF, psm.read(i.op1) & 0xFFFF):
        return i.extra['label']
    return None


def _unimpl(psm, i):
    if psm.strict:
        _, note = psm.describe(i)
        raise NotImplementedError(f"@{psm.pc:04X}: {i.mnem} not implemented ({note})")
    psm.unimpl_count += 1


_UNIMPL = frozenset(("ret", "tkiph", "tkiphs", "tkipl", "tkipls", "tkip?"))
_EXEC_UNIMPL = _unimpl
_EXEC = {
    "add": _add, "add.": _add, "addc": _add, "addc.": _add,
    "sub": _sub, "sub.": _sub, "subc": _sub, "subc.": _sub,
    "mul": _mul,
    "and": lambda psm, i: psm.write(i.op2, psm.read(i.op0) & psm.read(i.op1)),
    "or": lambda psm, i: psm.write(i.op2, psm.read(i.op0) | psm.read(i.op1)),
    "xor": lambda psm, i: psm.write(i.op2, psm.read(i.op0) ^ psm.read(i.op1)),
    # b43-asm: clear the bits of B in A
    "nand": lambda psm, i: psm.write(i.op2, psm.read(i.op0) & ~psm.read(i.op1)),
    "sr": lambda psm, i: psm.write(i.op2, (psm.read(i.op0) & 0xFFFF) >> (psm.read(i.op1) & 0x1F)),
    "sl": lambda psm, i: psm.write(i.op2, psm.read(i.op0) << (psm.read(i.op1) & 0x1F)),
    "sra": _sra,
    "rl": _rotate, "rr": _rotate,
    "orx": _orx, "srx": _srx,
    "nap": lambda psm, i: None,
    "jzx": _jzx, "jnzx": _jzx,
    "jext": _jext, "jnext": _jext,
}
_EXEC.update((m, _jump2) for m in _JUMP2)


def cli(isa, example):
    """The command line of an instruction-set module: run a blob from an
    address and show the trace tail and the shared memory written."""
    import sys
    if len(sys.argv) < 2:
        print(f"usage: {sys.argv[0]} <ucode.bin> [start_address_hex] [max_steps]")
        print()
        print(f"Example: {sys.argv[0]} {example}")
        sys.exit(1)

    path = sys.argv[1]
    start = int(sys.argv[2], 16) if len(sys.argv) > 2 else 0
    max_steps = int(sys.argv[3]) if len(sys.argv) > 3 else 1000

    prog = isa.load_program(path)
    print(f"loaded {len(prog)} instructions from {path}")

    m = isa.PSM(prog)
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
