#!/usr/bin/env python3
"""Run the host op stream and the real D11 microcode in lockstep.

Unlike cosim.py, which runs the ucode only at hand-offs and restarts it each
time, here the PSM is one machine that keeps running: it starts at PC 0 when
the host sets MACCONTROL.PSM_RUN, restarts at 0 on PSM_JMP_0, stops while
PSM_RUN is clear, and after every host operation executes either --cycles
instructions or, with --settle, as many as it takes to go idle. Host and
ucode share one state:

  SHM       OBJ.WR to shared memory lands in the PSM's memory (32-bit writes
            as two words).
  IHR/SPR   a host MMIO write at 0x400 + 2n is SPR n (b43's TSF_0 0x632 is
            OpenFWWF's SPR_TSF_WORD0 0x119, RNG 0x65A SPR_TSF_Random 0x12D,
            IFSSTAT 0x690 SPR_IFS_STAT 0x148, PSM_PHY_HDR 0x492 SPR 0x049);
            32-bit writes set SPR n and n+1.
  MAC regs  MACCMD (0x124) is SPR_MAC_CMD 0x047; GEN_IRQ_REASON (0x128) is SPR
            0x042/0x043 and a host write clears the bits it sets; the IRQ mask
            (0x12C) is SPR 0x044/0x045; MACCONTROL's high half is SPR 0x041.
  MACCTL    MAC.MCTRL with a mask (the vendor captures, brcms_b_mctrl) updates
            only the masked bits; without one it is the whole register.
  HOSTF     MAC.MHF idx/val/mask updates host-flag word idx (HOSTF1..5).
  initvals  not part of the captures: with --initvals they are written the
            first time the ucode raises MAC_SUSPENDED (IRQ bit 0) after
            PSM_RUN, as brcms_b_coreinit writes them after waiting for the
            self-suspend.

PHY      the ucode reaches PHY registers through SPR 0x018/0x019 (OpenFWWF's
            SPR_Ext_IHR_Address/Data): it writes 0x018 with bit 14 set and
            waits for the bit to clear, with bit 13 set for a read (the data
            then in 0x019) and clear for a write of 0x019; both blobs use
            only these two commands. A transaction completes at once, on a
            PHY register file that holds what the host wrote and read.
            Register 0 reads as MMIO PHY_VER (0x3E0) when the host never set
            it: the ucode splits it as b43 splits PHY_VER (version in bits
            7:0 to SHM 0x28, type in 11:8 to SHM 0x29), and OpenFWWF reads
            PHY register 0 for its SHM_PHYVER/SHM_PHYTYPE.

CORECTL  SPR 0x078 (MMIO 0x4F0, brcmsmac's psm_corectlsts): both blobs set
            bit 5 and then wait for bit 13, the only bit of it they wait on;
            bit 13 follows bit 5. A model of the handshake, not a known
            semantic.

External conditions: COND_TRUE (0x7F) is true, COND_MACEN (0x24) follows
MACCONTROL.ENABLED, and condition register 4 (0x40..0x4F) is SPR_BRC bit by
bit, as OpenFWWF documents it ("the NEED_RESPONSEFR bit is set in SPR_BRC.
This will trigger the condition COND_NEED_RESPONSEFR"). With --tx-engine
the TX engine raises COND_TX_NOW, COND_TX_POWER and COND_TX_DONE (below).
Every other jext/jnext selector is False and reported. The selector names
are OpenFWWF's corerev-5 ones, not verified on corerev 42.

TX engine (--tx-engine IFS,START,DONE, in PSM instructions): a model of
OpenFWWF's transmit sequence, not a known semantic. Setting bit 0 of
SPR_TXE0_CTL (0x080) raises COND_TX_NOW after IFS instructions; a write of
SPR 0x320 with bits 15 and 0 set, which the 0x3A0 builds make once a frame
is set up, raises COND_TX_POWER after START and COND_TX_DONE after DONE.
Each stays raised until the ucode acknowledges it with an EOI jext/jnext.
Without it no frame ever completes, and neither does a MAC suspend that
waits for one.

The host's reads come from its own trace (the oracle the trace was made
with), so the ucode does not feed back into the host here: what this measures
is what the ucode does with the state the host builds.

Input lines are `cpuN CLASS key=val ...` (test/integration) or
`<ts> #<n> cpuN CLASS key=val ...` (vendor captures, mmio2ops.py).

With --settle the PSM runs after each host operation until it is back at a
(pc, call stack, carry) it already reached with no net change in between:
from there only the host can change its state. The report then adds a
timetable in PSM instructions: how long the ucode works after each host
operation, when it raises IRQ bits, and which shared-memory words it wrote
that the host reads later. PHY transactions complete at once here, so the
counts of code that waits on them are lower bounds.

    lockstep.py OPS BLOB [--cycles 100 | --settle MAX] [--initvals FILE ...]
                [--report FILE] [--cond-inc FILE]
"""

import argparse
import collections
import re
import struct
import sys

import psm

MACCTL, MACCMD, IRQ_REASON, IRQ_MASK = 0x120, 0x124, 0x128, 0x12C
MACCTL_ENABLED, MACCTL_PSM_RUN, MACCTL_PSM_JMP0 = 0x1, 0x2, 0x4
IHR_BASE = 0x400
SPR_MAC_CTLHI, SPR_IRQ_LO, SPR_IRQ_HI = 0x041, 0x042, 0x043
SPR_IRQMASK_LO, SPR_IRQMASK_HI, SPR_MAC_CMD = 0x044, 0x045, 0x047
SPR_EXT_IHR_ADDR, SPR_EXT_IHR_DATA = 0x018, 0x019
EXT_IHR_GO, EXT_IHR_READ = 0x4000, 0x2000
SPR_CORECTL, CORECTL_REQ, CORECTL_ACK = 0x078, 0x0020, 0x2000
MMIO_PHY_VER = 0x3E0
COND_MACEN = 0x24
COND_TX_NOW, COND_TX_POWER, COND_TX_DONE = 0x20, 0x21, 0x22
CONDREG_BRC = 0x40              # condition register 4 mirrors SPR_BRC
SPR_BRC, SPR_TXE0_CTL, SPR_TXE_CMD = 0x048, 0x080, 0x320
TXE_CMD_GO = 0x8001
IRQ_MAC_SUSPENDED = 0x0001
HOSTF_WORDS = (0x05E >> 1, 0x060 >> 1, 0x062 >> 1, 0x078 >> 1, 0x0D4 >> 1)
OBJADDR, OBJDATA, OBJDATA_HI = 0x160, 0x164, 0x166
OBJADDR_SEL_MASK, OBJADDR_SHM_SEL, OBJADDR_WINC = 0x000F0000, 0x00010000, 0x01000000

OP = re.compile(r"^\s*(?:\S+\s+#(\d+)\s+)?cpu\d+\s+(\S+)\s*(.*)$")
FIELD = re.compile(r"(\w+)=0x([0-9a-fA-F]+)")


def fields(rest):
    """Fields of an op line, with the hex width of each value."""
    return {k: (int(v, 16), len(v)) for k, v in FIELD.findall(rest)}


def load_inits(path):
    """A d11init blob: big-endian {u16 addr, u16 size, u32 value} records
    up to the 0xFFFF terminator (brcms_c_write_inits)."""
    data = open(path, "rb").read()
    inits = []
    for off in range(0, len(data) - 7, 8):
        addr, size, value = struct.unpack_from(">HHI", data, off)
        if addr == 0xFFFF:
            break
        inits.append((addr, size, value))
    return inits


class Lockstep:
    def __init__(self, prog, cycles, settle=None, inits=None, tx=None):
        self.prog = prog
        self.m = psm.PSM(prog)
        self.m.stall_threshold = float("inf")
        self.m.observer = self.observe
        self.m.on_eoi = self.eoi
        self.tx = tx                    # (ifs, start, done) or None
        self.events = set()             # raised TX conditions
        self.pending = []               # (step, condition) still to raise
        self.tx_log = []                # (op_id, offset, condition)
        self.cycles = cycles
        self.settle_max = settle
        self.inits = inits or []
        self.inits_state = "wait" if self.inits else "none"   # wait, due, done
        self.running = False
        self.macctl = 0
        self.objaddr = 0
        self.host_spr = set()           # SPRs the host has written
        self.host_epoch = 0             # bumped by host writes made during a run
        self.stopped = None             # exception that stopped the PSM
        self.steps = 0
        self.pcs = collections.Counter()
        self.conds = collections.Counter()      # (pc, mnem, sel, value)
        self.hw_reads = collections.Counter()   # (spr, pc) never written
        self.unk = collections.Counter()        # (pc, mnem)
        self.phy = {}                           # PHY register file
        self.phy_access = collections.Counter() # (rw, addr, value, source)
        self.ops = 0
        # timetable (--settle)
        self.op_id, self.op_desc, self.op_start = None, "", 0
        self.changes = []               # (loc, old) of state changes in this settle
        self.reactions = []             # (op_id, desc, steps, idle pc or None)
        self.irqs = []                  # (op_id, desc, offset, spr, bits)
        self.shm_src = {}               # word -> (op_id, offset, pc) of the last ucode write
        self.handoffs = []              # (op_id, offset, pc, reader op_id, word, value, traced)

    # ---- host side ----
    def spr_write(self, n, val):
        self.m.spr[n] = val & 0xFFFF
        self.host_spr.add(n)

    def reg_write(self, off, val, width):
        if off >= IHR_BASE:
            n = (off - IHR_BASE) >> 1
            self.spr_write(n, val)
            if width == 8:
                self.spr_write(n + 1, val >> 16)
        elif off == MACCMD:
            self.spr_write(SPR_MAC_CMD, val)
        elif off == IRQ_REASON:
            for n, part in ((SPR_IRQ_LO, val), (SPR_IRQ_HI, val >> 16)):
                self.m.spr[n] = self.m.spr.get(n, 0) & ~part & 0xFFFF
                self.host_spr.add(n)
        elif off == IRQ_MASK:
            self.spr_write(SPR_IRQMASK_LO, val)
            self.spr_write(SPR_IRQMASK_HI, val >> 16)

    def macctl_write(self, val):
        self.macctl = val
        self.spr_write(SPR_MAC_CTLHI, val >> 16)
        if val & MACCTL_PSM_JMP0:
            self.m.pc = 0
            self.m.call_stack.clear()
        self.running = bool(val & MACCTL_PSM_RUN) and self.stopped is None

    def write_inits(self):
        m = self.m
        for addr, size, value in self.inits:
            if addr == OBJADDR:
                self.objaddr = value
            elif addr in (OBJDATA, OBJDATA_HI):
                if self.objaddr & OBJADDR_SEL_MASK != OBJADDR_SHM_SEL:
                    continue
                w = ((self.objaddr & 0xFFFF) << 1) & (len(m.shm) - 1)
                if size == 4:
                    m.shm[w], m.shm[w + 1] = value & 0xFFFF, value >> 16
                else:
                    m.shm[w + (addr == OBJDATA_HI)] = value & 0xFFFF
                if self.objaddr & OBJADDR_WINC and (size == 4 or addr == OBJDATA_HI):
                    self.objaddr = (self.objaddr & ~0xFFFF) | ((self.objaddr + 1) & 0xFFFF)
            elif addr >= IHR_BASE:
                self.reg_write(addr, value, 8 if size == 4 else 4)
        self.inits_state = "done"
        self.host_epoch += 1

    def apply(self, line, lineno=0):
        mm = OP.match(line)
        if not mm:
            return False
        cls, rest = mm.group(2), mm.group(3)
        f = fields(rest)
        self.op_id = int(mm.group(1)) if mm.group(1) else lineno
        self.op_desc = f"{cls} {rest}".strip()
        if cls == "OBJ.WR" and "sel" in f:
            routing = (f["sel"][0] >> 16) & 0xFF
            if routing == 1:
                w = f["addr"][0] >> 1
                val, width = f["val"]
                self.m.shm[w & (len(self.m.shm) - 1)] = val & 0xFFFF
                if width == 8:
                    self.m.shm[(w + 1) & (len(self.m.shm) - 1)] = val >> 16
        elif cls == "OBJ.RD" and "sel" in f and (f["sel"][0] >> 16) & 0xFF == 1:
            w = (f["addr"][0] >> 1) & (len(self.m.shm) - 1)
            if w in self.shm_src:
                traced = f["val"][0] & 0xFFFF if "val" in f else None
                self.handoffs.append((*self.shm_src[w], self.op_id, w, self.m.shm[w], traced))
        elif cls == "REG.WR":
            self.reg_write(f["off"][0], *f["val"])
        elif cls == "MAC.MCMD":
            self.spr_write(SPR_MAC_CMD, f["val"][0])
        elif cls == "MAC.MCTRL":
            val = f["val"][0]
            if "mask" in f:
                val = (self.macctl & ~f["mask"][0]) | val
            self.macctl_write(val)
        elif cls == "MAC.MHF" and "addr" in f:
            w = HOSTF_WORDS[f["addr"][0]]
            val, mask = f["val"][0], f["mask"][0]
            self.m.shm[w] = (self.m.shm[w] & ~mask) | (val & mask)
        elif cls == "PHY.WR" or (cls == "PHY.RD" and "val" in f):
            self.phy[f["addr"][0]] = f["val"][0] & 0xFFFF
        elif cls == "REG.RD" and f.get("off", (None,))[0] == MMIO_PHY_VER and "val" in f:
            self.phy.setdefault(0, f["val"][0] & 0xFFFF)
        return True

    # ---- ucode side ----
    def eoi(self, sel):
        self.events.discard(sel)

    def tx_schedule(self, loc, old, new):
        ifs, start, done = self.tx
        if loc == ("spr", SPR_TXE0_CTL) and new & 1 and not old & 1:
            self.pending.append((self.steps + ifs, COND_TX_NOW))
        elif loc == ("spr", SPR_TXE_CMD) and new & TXE_CMD_GO == TXE_CMD_GO:
            self.pending.append((self.steps + start, COND_TX_POWER))
            self.pending.append((self.steps + done, COND_TX_DONE))

    def conditions(self):
        m = self.m
        flags = {COND_MACEN: bool(self.macctl & MACCTL_ENABLED)}
        brc = m.spr.get(SPR_BRC, 0)
        for bit in range(16):
            flags[CONDREG_BRC | bit] = bool(brc >> bit & 1)
        if self.pending:
            due = [p for p in self.pending if p[0] <= self.steps]
            for p in due:
                self.pending.remove(p)
                self.events.add(p[1])
                self.tx_log.append((self.op_id, self.steps - self.op_start, p[1]))
        for sel in self.events:
            flags[sel] = True
        m.ext_flags = flags

    def observe(self, loc, old, new):
        offset = self.steps - self.op_start
        kind, n = loc
        if self.tx and kind == "spr":
            self.tx_schedule(loc, old, new)
        if kind == "spr" and n in (SPR_IRQ_LO, SPR_IRQ_HI):
            self.irqs.append((self.op_id, self.op_desc, offset, n, new))
            if n == SPR_IRQ_LO and new & IRQ_MAC_SUSPENDED and self.inits_state == "wait":
                self.inits_state = "due"
        if new != old:
            self.changes.append((loc, old))
            if kind == "shm":
                self.shm_src[n] = (self.op_id, offset, self.m.pc)

    def step(self):
        """One ucode instruction plus the hardware the model answers for it.
        False once the PSM has stopped."""
        m, prog = self.m, self.prog
        pc = m.pc
        if pc >= len(prog):
            self.stopped = f"PC {pc:#x} out of the program"
            self.running = False
            return False
        ins = prog[pc]
        self.conditions()
        self.pcs[pc] += 1
        for op in (ins.op0, ins.op1):
            if op is not None and op.kind == "spr" and \
                    op.value not in m.spr and op.value not in self.host_spr:
                self.hw_reads[(op.value, pc)] += 1
        if ins.mnem in ("jext", "jnext"):
            sel = ins.extra["imm"]
            val = True if (sel & 0x7F) == psm.COND_TRUE else m.ext_flags.get(sel & 0x7F, False)
            self.conds[(pc, ins.mnem, sel, val)] += 1
        elif ins.mnem.startswith("unk_"):
            self.unk[(pc, ins.mnem)] += 1
        try:
            m.step()
        except Exception as e:      # noqa: BLE001 -- reported, not hidden
            self.stopped = f"{type(e).__name__} at {pc:#x}: {e}"
            self.running = False
            return False
        self.steps += 1
        cmd = m.spr.get(SPR_EXT_IHR_ADDR, 0)
        if cmd & EXT_IHR_GO:
            self.ext_ihr(cmd)
        ctl = m.spr.get(SPR_CORECTL, 0)
        ack = CORECTL_ACK if ctl & CORECTL_REQ else 0
        if (ctl & CORECTL_ACK) != ack:
            m.store(("spr", SPR_CORECTL), (ctl & ~CORECTL_ACK) | ack)
        if self.inits_state == "due":
            self.write_inits()
        return True

    def run(self, n):
        for _ in range(n):
            if not self.step():
                return

    def settle(self):
        """Run until idle; return (steps until the idle cycle began, its pc),
        the pc being None if --settle ran out first."""
        m = self.m
        start, self.changes, seen, epoch = self.steps, [], {}, self.host_epoch
        while self.running and self.steps - start < self.settle_max:
            if epoch != self.host_epoch:
                seen, epoch = {}, self.host_epoch
            key = (m.pc, tuple(m.call_stack), m.spr.get("__carry__", 0),
                   tuple(sorted(self.events)))
            prev = seen.get(key)
            if prev is not None and not self.pending and self.unchanged_since(prev[1]):
                return prev[0] - start, m.pc
            seen[key] = (self.steps, len(self.changes))
            if not self.step():
                break
        return self.steps - start, None

    def unchanged_since(self, i):
        first_old = {}
        for loc, old in self.changes[i:]:
            first_old.setdefault(loc, old)
        return all(self.m.peek(loc) == old for loc, old in first_old.items())

    def ext_ihr(self, cmd):
        m = self.m
        addr = cmd & 0x1FFF
        if cmd & EXT_IHR_READ:
            src = "host" if addr in self.phy else "never set"
            val = self.phy.get(addr, 0)
            m.store(("spr", SPR_EXT_IHR_DATA), val)
            self.phy_access[("read", addr, val, src)] += 1
        else:
            val = m.spr.get(SPR_EXT_IHR_DATA, 0)
            self.phy[addr] = val
            self.phy_access[("write", addr, val, "ucode")] += 1
        m.store(("spr", SPR_EXT_IHR_ADDR), cmd & ~(EXT_IHR_GO | EXT_IHR_READ))

    def feed(self, path):
        idle = False
        with open(path) as f:
            for lineno, line in enumerate(f, 1):
                if idle:
                    before = (self.macctl, self.m.pc, self.host_epoch,
                              list(self.m.shm), dict(self.m.spr), dict(self.phy))
                if not self.apply(line, lineno):
                    continue
                self.ops += 1
                self.op_start = self.steps
                if not self.running:
                    idle = False
                elif self.settle_max is None:
                    self.run(self.cycles)
                elif not idle or before != (self.macctl, self.m.pc, self.host_epoch,
                                            self.m.shm, self.m.spr, self.phy):
                    steps, pc = self.settle()
                    if steps:
                        self.reactions.append((self.op_id, self.op_desc, steps, pc))
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
        m = self.m
        w = out.write
        w(f"host ops {self.ops}, ucode steps {self.steps}, PSM "
          f"{'stopped: ' + self.stopped if self.stopped else 'running' if self.running else 'halted by the host'}, "
          f"final PC {m.pc:#06x}\n")
        w(f"UCODEREV {m.shm[0]:#06x} PATCH {m.shm[1]:#06x}\n")
        if self.inits_state != "none":
            w(f"initvals: {'written' if self.inits_state == 'done' else 'never written (no MAC_SUSPENDED after PSM_RUN)'}\n")
        w(f"distinct PCs executed: {len(self.pcs)} of {len(self.prog)}\n")
        w("hottest PCs: " + ", ".join(f"{pc:#05x} x{n}" for pc, n in self.pcs.most_common(12)) + "\n")
        w("external conditions (pc, insn, selector, value: times):\n")
        for (pc, mn, sel, v), n in sorted(self.conds.items()):
            w(f"  {pc:#06x} {mn:5s} 0x{sel:02X} {label(sel):32s} {str(v):5s} x{n}\n")
        w("SPRs read before the host or the ucode wrote them (read as 0):\n")
        by = collections.defaultdict(list)
        for (s, pc) in self.hw_reads:
            by[s].append(pc)
        for s in sorted(by):
            w(f"  spr{s:03X} (MMIO {IHR_BASE + 2 * s:#05x}) at " +
              ", ".join(f"{pc:#05x}" for pc in sorted(by[s])[:8]) + "\n")
        w("PHY accesses of the ucode (kind, register, value, source: times):\n")
        for (rw, a, v, src), n in sorted(self.phy_access.items(), key=lambda x: (x[0][1], x[0][0])):
            w(f"  {rw:5s} {a:#06x} = {v:#06x} ({src}) x{n}\n")
        w("unknown or stub instructions executed:\n")
        for (pc, mn), n in sorted(self.unk.items()):
            w(f"  {pc:#06x} {mn} x{n}\n")
        if self.settle_max is None:
            return
        w("timetable, in PSM instructions after the host op:\n")
        w(" ucode work until idle (op, steps, idle at):\n")
        for op_id, desc, steps, pc in self.reactions:
            idle = f"{pc:#06x} {self.describe(pc)}" if pc is not None else "not idle"
            w(f"  #{op_id:<8d} {steps:>7d}  {idle:24s} {desc[:60]}\n")
        if self.tx:
            w(" TX engine conditions raised (op, +steps, condition):\n")
            for op_id, off, sel in self.tx_log:
                w(f"  #{op_id:<8d} +{off:<7d} 0x{sel:02X} {label(sel)}\n")
        w(" IRQ bits the ucode raised (op, +steps, register, bits):\n")
        for op_id, desc, off, n, bits in self.irqs:
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
    ap.add_argument("--report")
    ap.add_argument("--cond-inc", help="OpenFWWF cond.inc, to name the selectors")
    args = ap.parse_args()

    label = lambda sel: ""
    if args.cond_inc:
        from cond import CondMap
        cm = CondMap.load(args.cond_inc)
        label = cm.label

    inits = [e for p in args.initvals for e in load_inits(p)]
    tx = tuple(int(x, 0) for x in args.tx_engine.split(",")) if args.tx_engine else None
    ls = Lockstep(psm.load_program(args.blob), args.cycles, args.settle, inits, tx)
    ls.feed(args.ops)
    out = open(args.report, "w") if args.report else sys.stdout
    ls.report(out, label)
    if args.report:
        out.close()


if __name__ == "__main__":
    main()
