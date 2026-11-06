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
"""Run the host op stream and the real D11 microcode in lockstep.

Unlike cosim.py, which runs the ucode only at hand-offs, the PSM keeps
running on the chip of vchip.py: each host operation is applied to the chip,
then the PSM executes --cycles instructions or, with --settle, runs until
idle.

The ops map onto the chip's host side:

  OBJ.WR    to shared memory (sel routing 1) lands in the PSM's memory.
  REG.WR    an MMIO write (IHR/SPR, MAC registers, TSF and beacon timing).
  MAC.MCTRL with a mask (the vendor captures, brcms_b_mctrl) updates only the
            masked bits; without one it is the whole register.
  MAC.MCMD  MACCMD.
  MAC.MHF   idx/val/mask updates host-flag word idx (HOSTF1..5).
  PHY.WR    and PHY.RD with a value set the PHY register.
  REG.RD    of MMIO PHY_VER with a value seeds PHY register 0.
  WAIT      us=0x... lets that many microseconds of TSF pass.

The captures do not carry the initvals: --initvals gives them to the chip.
--corerev picks the profile (default 42), --tx-engine IFS,START,DONE and
--psm-mhz F turn on the TX engine model and the running TSF.

The host's reads come from its own trace (the oracle the trace was made
with), so the ucode does not feed back into the host here: what this measures
is what the ucode does with the state the host builds.

Input lines are `cpuN CLASS key=val ...` (test/integration) or
`<ts> #<n> cpuN CLASS key=val ...` (vendor captures, mmio2ops.py).

With --settle the report adds a timetable in PSM instructions: how long the
ucode works after each host operation, when it raises IRQ bits, and which
shared-memory words it wrote that the host reads later. Bus transactions
complete at once here, so the counts of code that waits on them are lower
bounds.

    lockstep.py OPS BLOB [--cycles 100 | --settle MAX] [--initvals FILE ...]
                [--tx-engine IFS,START,DONE] [--psm-mhz F]
                [--corerev N] [--phy-ver V]
                [--report FILE] [--cond-inc FILE]
"""

import argparse
import collections
import re
import sys

import core
import psmlib
import vchip

OP = re.compile(r"^\s*(?:\S+\s+#(\d+)\s+)?cpu\d+\s+(\S+)\s*(.*)$")
FIELD = re.compile(r"(\w+)=0x([0-9a-fA-F]+)")


def fields(rest):
    """Fields of an op line, with the hex width of each value."""
    return {k: (int(v, 16), len(v)) for k, v in FIELD.findall(rest)}


class Recorder(vchip.Recorder):
    """The chip's events, attributed to the host op that preceded them."""

    def __init__(self):
        self.chip = None
        self.op_id, self.op_desc, self.op_start = None, "", 0
        self.pcs = collections.Counter()
        self.conds = collections.Counter()      # (pc, mnem, sel, value)
        self.hw_reads = collections.Counter()   # (spr, pc) never written
        self.unk = collections.Counter()        # (pc, mnem)
        self.bus_access = collections.Counter() # (bus, rw, addr, value, source)
        self.tx_log = []                # (op_id, offset, condition)
        self.tbtt_log = []              # (op_id, TSF) of each TBTT raised
        self.irqs = []                  # (op_id, desc, offset, spr, bits)
        self.shm_src = {}               # word -> (op_id, offset, pc) of the last ucode write
        self.pc = 0

    def offset(self):
        return self.chip.steps - self.op_start

    def step(self, pc, ins, ext_flags):
        self.pc = pc
        self.pcs[pc] += 1
        if ins.mnem in ("jext", "jnext"):
            sel = ins.extra["imm"]
            val = True if (sel & 0x7F) == psmlib.COND_TRUE else ext_flags.get(sel & 0x7F, False)
            self.conds[(pc, ins.mnem, sel, val)] += 1
        elif ins.mnem.startswith("unk_"):
            self.unk[(pc, ins.mnem)] += 1

    def unset_spr_read(self, spr, pc):
        self.hw_reads[(spr, pc)] += 1

    def store(self, loc, old, new):
        kind, n = loc
        if kind == "spr" and n in (vchip.SPR_IRQ_LO, vchip.SPR_IRQ_HI):
            self.irqs.append((self.op_id, self.op_desc, self.offset(), n, new))
        if new != old and kind == "shm":
            self.shm_src[n] = (self.op_id, self.offset(), self.pc)

    def bus(self, name, kind, addr, value, source):
        self.bus_access[(name, kind, addr, value, source)] += 1

    def tbtt(self, tsf):
        self.tbtt_log.append((self.op_id, tsf))

    def tx_condition(self, sel):
        self.tx_log.append((self.op_id, self.offset(), sel))


class Lockstep:
    def __init__(self, prog, cycles, settle=None, inits=None, tx=None, mhz=None,
                 hw=core.for_corerev(42), phy_ver=None):
        self.rec = Recorder()
        devices = None
        if phy_ver is not None:
            devices = {"phy": vchip.RegisterFile("phy", {0: phy_ver & 0xFFFF})}
        self.chip = vchip.Chip(prog, hw, devices, inits, tx, mhz, self.rec)
        self.rec.chip = self.chip
        self.prog = prog
        self.cycles = cycles
        self.settle_max = settle
        self.ops = 0
        self.reactions = []             # (op_id, desc, steps, idle pc or None)
        self.handoffs = []              # (op_id, offset, pc, reader op_id, word, value, traced)

    def apply(self, line, lineno=0):
        mm = OP.match(line)
        if not mm:
            return False
        chip, rec = self.chip, self.rec
        cls, rest = mm.group(2), mm.group(3)
        f = fields(rest)
        rec.op_id = int(mm.group(1)) if mm.group(1) else lineno
        rec.op_desc = f"{cls} {rest}".strip()
        if cls == "OBJ.WR" and "sel" in f:
            if (f["sel"][0] >> 16) & 0xFF == 1:
                val, width = f["val"]
                chip.host_shm_write(f["addr"][0] >> 1, val, 2 if width == 8 else 1)
        elif cls == "OBJ.RD" and "sel" in f and (f["sel"][0] >> 16) & 0xFF == 1:
            w = (f["addr"][0] >> 1) & (len(chip.m.shm) - 1)
            if w in rec.shm_src:
                traced = f["val"][0] & 0xFFFF if "val" in f else None
                self.handoffs.append((*rec.shm_src[w], rec.op_id, w, chip.host_shm_read(w), traced))
        elif cls == "REG.WR":
            val, width = f["val"]
            chip.host_reg_write(f["off"][0], val, 32 if width == 8 else 16)
        elif cls == "MAC.MCMD":
            chip.host_maccmd_write(f["val"][0])
        elif cls == "MAC.MCTRL":
            chip.host_macctl_write(f["val"][0], f["mask"][0] if "mask" in f else None)
        elif cls == "MAC.MHF" and "addr" in f:
            chip.host_hostflags(f["addr"][0], f["val"][0], f["mask"][0])
        elif cls == "PHY.WR" or (cls == "PHY.RD" and "val" in f):
            chip.host_phy_write(f["addr"][0], f["val"][0])
        elif cls == "REG.RD" and "off" in f and "val" in f:
            chip.host_reg_observed(f["off"][0], f["val"][0])
        return True

    def feed(self, path):
        chip, rec = self.chip, self.rec
        idle = False
        with open(path) as f:
            for lineno, line in enumerate(f, 1):
                if idle:
                    before = chip.snapshot()
                if not self.apply(line, lineno):
                    continue
                self.ops += 1
                rec.op_start = chip.steps
                if rec.op_desc.startswith("WAIT"):
                    if not chip.mhz or self.settle_max is None:
                        raise SystemExit("WAIT needs --psm-mhz and --settle")
                    us = fields(rec.op_desc).get("us", (0,))[0]
                    for tsf, steps, pc in chip.wait(us, self.settle_max):
                        self.reactions.append((rec.op_id, f"WAIT, TSF {tsf}", steps, pc))
                    idle = False
                elif not chip.running:
                    idle = False
                elif self.settle_max is None:
                    chip.run(self.cycles)
                elif not idle or before != chip.snapshot():
                    steps, pc = chip.settle(self.settle_max)
                    if steps:
                        self.reactions.append((rec.op_id, rec.op_desc, steps, pc))
                    idle = pc is not None

    # ---- report ----
    def describe(self, pc):
        ins = self.prog[pc]
        if ins.mnem in ("jext", "jnext"):
            return f"{ins.mnem} 0x{ins.extra['imm']:02X}"
        if "label" in ins.extra:
            return f"{ins.mnem} {ins.op0},{ins.op1}"
        return ins.mnem

    def report(self, out, label):
        chip, rec = self.chip, self.rec
        m = chip.m
        w = out.write
        w(f"host ops {self.ops}, ucode steps {chip.steps}, PSM "
          f"{'stopped: ' + chip.stopped if chip.stopped else 'running' if chip.running else 'halted by the host'}, "
          f"final PC {m.pc:#06x}\n")
        w(f"UCODEREV {m.shm[0]:#06x} PATCH {m.shm[1]:#06x}\n")
        hw = chip.hw
        w(f"core: corerev {hw.corerev}, {hw.isa.NAME}, hardware profile read from the ucode "
          f"of corerev {', '.join(map(str, hw.read_from))}"
          f"{', assumed for this revision' if hw.assumed else ''}\n")
        if chip.inits_state != "none":
            w(f"initvals: {'written' if chip.inits_state == 'done' else 'never written (no MAC_SUSPENDED after PSM_RUN)'}\n")
        w(f"distinct PCs executed: {len(rec.pcs)} of {len(self.prog)}\n")
        w("hottest PCs: " + ", ".join(f"{pc:#05x} x{n}" for pc, n in rec.pcs.most_common(12)) + "\n")
        w("external conditions (pc, insn, selector, value: times):\n")
        for (pc, mn, sel, v), n in sorted(rec.conds.items()):
            w(f"  {pc:#06x} {mn:5s} 0x{sel:02X} {label(sel):32s} {str(v):5s} x{n}\n")
        w("SPRs read before the host or the ucode wrote them (read as 0):\n")
        by = collections.defaultdict(list)
        for (s, pc) in rec.hw_reads:
            by[s].append(pc)
        for s in sorted(by):
            w(f"  spr{s:03X} (MMIO {vchip.IHR_BASE + 2 * s:#05x}) at " +
              ", ".join(f"{pc:#05x}" for pc in sorted(by[s])[:8]) + "\n")
        w("indirect register accesses of the ucode (bus, kind, register, value, source: times):\n")
        for (bus, rw, a, v, src), n in sorted(rec.bus_access.items(),
                                              key=lambda x: (x[0][0], x[0][2], x[0][1])):
            w(f"  {bus:7s} {rw:5s} {a:#06x} = {v:#06x} ({src}) x{n}\n")
        w("unknown or stub instructions executed:\n")
        for (pc, mn), n in sorted(rec.unk.items()):
            w(f"  {pc:#06x} {mn} x{n}\n")
        if self.settle_max is None:
            return
        w("timetable, in PSM instructions after the host op:\n")
        w(" ucode work until idle (op, steps, idle at):\n")
        for op_id, desc, steps, pc in self.reactions:
            idle = f"{pc:#06x} {self.describe(pc)}" if pc is not None else "not idle"
            w(f"  #{op_id:<8d} {steps:>7d}  {idle:24s} {desc[:60]}\n")
        if chip.tx:
            w(" TX engine conditions raised (op, +steps, condition):\n")
            for op_id, off, sel in rec.tx_log:
                w(f"  #{op_id:<8d} +{off:<7d} 0x{sel:02X} {label(sel)}\n")
        if rec.tbtt_log:
            w(f" TBTTs raised (op, TSF): {len(rec.tbtt_log)}\n")
            for op_id, t in rec.tbtt_log[:20]:
                w(f"  #{op_id:<8d} {t}\n")
        w(" IRQ bits the ucode raised (op, +steps, register, bits):\n")
        for op_id, desc, off, n, bits in rec.irqs:
            w(f"  #{op_id:<8d} +{off:<7d} spr{n:03X} {bits:#06x}  {desc[:50]}\n")
        w(" shared memory the ucode wrote and the host read (word: written at, first read):\n")
        first = {}
        for op_id, off, pc, reader, word, val, traced in self.handoffs:
            k = (op_id, word)
            if k in first:
                first[k][-1] += 1
                continue
            first[k] = [op_id, off, pc, reader, word, val, traced, 1]
        for op_id, off, pc, reader, word, val, traced, n in first.values():
            agree = "" if traced is None else ("= trace" if traced == val else f"trace {traced:#06x}")
            w(f"  {word:#05x} (byte {2 * word:#06x}) #{op_id} +{off} by {pc:#06x}, "
              f"read #{reader} x{n}: {val:#06x} {agree}\n")



def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ops")
    ap.add_argument("blob")
    ap.add_argument("--cycles", type=int, default=100,
                    help="ucode instructions after each host operation")
    ap.add_argument("--settle", type=int, metavar="MAX",
                    help="run the ucode until idle after each host operation, "
                         "at most MAX instructions, and report the timetable")
    ap.add_argument("--initvals", action="append", default=[],
                    help="d11init blob, written after the boot self-suspend "
                         "(repeatable: initvals, then bsinitvals)")
    ap.add_argument("--tx-engine", metavar="IFS,START,DONE",
                    help="model the TX engine with these delays, in PSM instructions")
    ap.add_argument("--psm-mhz", type=float, metavar="F",
                    help="advance the TSF by 1 us every F PSM instructions")
    ap.add_argument("--corerev", type=int, default=42,
                    help="D11 core revision: instruction set and hardware model (core.py)")
    ap.add_argument("--phy-ver", type=lambda x: int(x, 0), metavar="V",
                    help="PHY register 0, as MMIO PHY_VER reads (analog << 12 | "
                         "type << 8 | revision), instead of the trace's")
    ap.add_argument("--report")
    ap.add_argument("--cond-inc", help="OpenFWWF cond.inc, to name the selectors")
    args = ap.parse_args()

    label = lambda sel: ""
    if args.cond_inc:
        from cond import CondMap
        cm = CondMap.load(args.cond_inc)
        label = cm.label

    inits = [e for p in args.initvals for e in vchip.load_inits(p)]
    tx = tuple(int(x, 0) for x in args.tx_engine.split(",")) if args.tx_engine else None
    hw = core.for_corerev(args.corerev)
    ls = Lockstep(hw.isa.load_program(args.blob), args.cycles, args.settle, inits, tx,
                  args.psm_mhz, hw, args.phy_ver)
    ls.feed(args.ops)
    out = open(args.report, "w") if args.report else sys.stdout
    ls.report(out, label)
    if args.report:
        out.close()


if __name__ == "__main__":
    main()
