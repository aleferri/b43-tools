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
Co-simulate a host MMIO trace against the real D11 microcode and snapshot the
complete D11 state.

The host op stream and the real ucode share one state. Host writes land in it
(shared memory, scratch, RCMTA, template RAM, the MMIO/IHR register file); at
the host->ucode hand-offs the real microcode is run on that same shared memory
through psm.py, so it reads what the host set and writes its results back. The
final snapshot covers every cell that interacts with D11:

  SHM     shared memory (16-bit words)         -- host + ucode, provenance marked
  SCR     objmem scratch                       -- host
  RCMTA   receive-match TA table               -- host
  TPL     template RAM (via RAM_CONTROL/DATA)  -- host
  REG     D11 MMIO / IHR register file          -- host
  GPR     PSM general registers r0..r127        -- ucode
  SPR     PSM special registers (sparse)        -- ucode

Input is a decoded MMIO op stream in the vocabulary of b43-ac-wip's
reverse-tools/mmio2ops.py (and test/integration/trace_out.c): lines carrying
OBJ.WR addr=.. val=.. sel=.., REG.WR off=.. val=.., MAC.MCMD val=.., MAC.MHF
addr=.. val=.. mask=.. and MAC.MCTRL val=.. (mask=.. in vendor captures, where
only the masked bits change). Everything else (PHY/RAD/reads/wrapper) is faked
or ignored.

Hand-offs that run the ucode:
  - PSM boot (MACCONTROL PSM_RUN): the version-stamp prologue (--start, default
    0xF76), which publishes the revinfo words into shared memory.
  - a host hostflag/command write to shared memory, or a MAC command: the main
    dispatch loop is run on the shared memory so the ucode reacts to it.

Honest limits: psm.py models SHM + GPR + SPR, not IHR/MMIO *reads*, so a command
the ucode would pick up by reading the MAC command register does not dispatch
here; it is recorded, and whatever the host routed through shared memory for it
is still applied. Executor confidence is high for the version-stamp and
arithmetic, best-effort for jumps (see psm.py); seeded conditions use OpenFWWF
(arch5) names via cond.py.

    cosim.py OPS BLOB [--start 0xF76] [--out STATE] [--max-steps N]
                 [--seed COND ...] [--cond-inc FILE]
"""
import argparse
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import psm  # noqa: E402  (sibling module)

# D11 MMIO offsets that are commands/channels, not plain cells.
MACCTL, MACCMD = 0x120, 0x124
RAM_CONTROL, RAM_DATA = 0x130, 0x134
MACCTL_PSM_RUN = 0x2

HOSTFLAG_WORDS = {0x2f, 0x30, 0x31}  # SHM words the main loop polls (0x5e/60/62)
HOSTF_WORDS = (0x2f, 0x30, 0x31, 0x3c, 0x6a)  # MAC.MHF idx -> HOSTF1..5 word
MAIN_LOOP = 0x0002


def field(line, key):
    m = re.search(re.escape(key) + r"(0x[0-9a-fA-F]+|\d+)", line)
    return int(m.group(1), 0) if m else None


class Cosim:
    def __init__(self, prog, start, max_steps, seeds=None):
        self.m = psm.PSM(prog)          # m.shm is the shared memory
        self.start = start
        self.max_steps = max_steps
        self.seeds = seeds or {}
        self.scr, self.rcmta, self.tpl, self.reg = {}, {}, {}, {}
        self.tpl_ptr = 0
        self.booted = False
        self.ucode_words_written = set()
        self.ucode_runs = 0
        self.commands = []

    def run_ucode(self, start, seeds=None):
        m = self.m
        before = list(m.shm)
        m.ext_flags = dict(seeds) if seeds else {}
        m.pc = start
        m._high_water = start
        seen_top = 0
        for _ in range(self.max_steps):
            try:
                m.step()
            except Exception:
                break
            if m.pc == MAIN_LOOP:
                seen_top += 1
                if seen_top >= 2:
                    break
        for w in range(len(m.shm)):
            if m.shm[w] != before[w]:
                self.ucode_words_written.add(w)
        self.ucode_runs += 1

    def apply(self, line):
        if "OBJ.WR" in line:
            addr, val, sel = field(line, "addr="), field(line, "val="), field(line, "sel=")
            routing = (sel >> 16) & 0xff
            if routing == 1:
                w = addr >> 1
                if w < len(self.m.shm):
                    self.m.shm[w] = val & 0xffff
                if w in HOSTFLAG_WORDS and self.booted and val:
                    self.run_ucode(MAIN_LOOP, self.seeds)
            elif routing == 2:
                self.scr[addr] = val
            elif routing == 4:
                self.rcmta[addr] = val
            return
        if "REG.WR" in line:
            off, val = field(line, "off="), field(line, "val=")
            if off == RAM_CONTROL:
                self.tpl_ptr = val & 0xffff
            elif off == RAM_DATA:
                self.tpl[self.tpl_ptr] = val
                self.tpl_ptr += 4
            elif off == MACCTL:
                self.reg[off] = val
                if (val & MACCTL_PSM_RUN) and not self.booted:
                    self.booted = True
                    self.run_ucode(self.start)
            elif off == MACCMD:
                self.reg[off] = val
                self.commands.append(("MACCMD", val))
                if self.booted:
                    self.run_ucode(MAIN_LOOP, self.seeds)
            else:
                self.reg[off] = val
            return
        if " MAC.MCMD " in line:
            val = field(line, "val=")
            self.reg[MACCMD] = val
            self.commands.append(("MCMD", val))
            if self.booted:
                self.run_ucode(MAIN_LOOP, self.seeds)
            return
        if " MAC.MCTRL " in line:
            val, mask = field(line, "val="), field(line, "mask=")
            if mask is not None:
                val = (self.reg.get(MACCTL, 0) & ~mask) | val
            self.reg[MACCTL] = val
            if (val & MACCTL_PSM_RUN) and not self.booted:
                self.booted = True
                self.run_ucode(self.start)
            return
        if " MAC.MHF " in line:
            w = HOSTF_WORDS[field(line, "addr=")]
            val, mask = field(line, "val="), field(line, "mask=")
            new = (self.m.shm[w] & ~mask) | (val & mask)
            changed = new != self.m.shm[w]
            self.m.shm[w] = new
            if w in HOSTFLAG_WORDS and self.booted and changed:
                self.run_ucode(MAIN_LOOP, self.seeds)

    def run(self, ops_path):
        with open(ops_path) as f:
            for line in f:
                if any(k in line for k in ("OBJ.WR", "REG.WR", "MAC.MCMD", "MAC.MCTRL", "MAC.MHF")):
                    self.apply(line)

    def snapshot(self, out):
        m = self.m
        for w, v in enumerate(m.shm):
            if v & 0xffff:
                tag = "ucode" if w in self.ucode_words_written else "host"
                out.write(f"SHM 0x{w*2:04x} 0x{v & 0xffff:04x} {tag}\n")
        for a in sorted(self.scr):
            if self.scr[a]:
                out.write(f"SCR 0x{a:04x} 0x{self.scr[a]:08x} host\n")
        for a in sorted(self.rcmta):
            if self.rcmta[a]:
                out.write(f"RCMTA 0x{a:04x} 0x{self.rcmta[a]:08x} host\n")
        for a in sorted(self.tpl):
            if self.tpl[a]:
                out.write(f"TPL 0x{a:04x} 0x{self.tpl[a]:08x} host\n")
        for off in sorted(self.reg):
            if self.reg[off]:
                out.write(f"REG 0x{off:04x} 0x{self.reg[off]:08x} host\n")
        for i, v in enumerate(m.reg):
            if v:
                out.write(f"GPR 0x{i:02x} 0x{v & 0xffffffff:08x} ucode\n")
        for k in sorted(k for k in m.spr if isinstance(k, int)):
            if m.spr[k]:
                out.write(f"SPR 0x{k:04x} 0x{m.spr[k] & 0xffffffff:08x} ucode\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ops")
    ap.add_argument("blob")
    ap.add_argument("--start", default="0xF76")
    ap.add_argument("--out")
    ap.add_argument("--max-steps", type=int, default=4000)
    ap.add_argument("--seed", action="append", default=[],
                    help="external condition to force in the command/hostflag "
                         "runs, by name or selector (needs cond.py + cond.inc)")
    ap.add_argument("--cond-inc")
    args = ap.parse_args()

    seeds = {}
    if args.seed:
        from cond import CondMap
        seeds = CondMap.load(args.cond_inc).seeds(args.seed)

    prog = psm.load_program(args.blob)
    cs = Cosim(prog, int(args.start, 0), args.max_steps, seeds)
    cs.run(args.ops)

    out = open(args.out, "w") if args.out else sys.stdout
    cs.snapshot(out)
    if args.out:
        out.close()

    shm_host = sum(1 for w, v in enumerate(cs.m.shm)
                   if v & 0xffff and w not in cs.ucode_words_written)
    shm_uc = sum(1 for w in cs.ucode_words_written if cs.m.shm[w] & 0xffff)
    gpr = sum(1 for v in cs.m.reg if v)
    spr = sum(1 for k, v in cs.m.spr.items() if isinstance(k, int) and v)
    cmds = {}
    for kind, val in cs.commands:
        cmds[(kind, val)] = cmds.get((kind, val), 0) + 1

    print(f"[cosim] booted={cs.booted}  ucode runs={cs.ucode_runs}  "
          f"UCODEREV={cs.m.shm[0] & 0xffff:#06x} UCODEPATCH={cs.m.shm[1] & 0xffff:#06x}",
          file=sys.stderr)
    print(f"[cosim] SHM: host={shm_host} ucode={shm_uc} | "
          f"SCR={sum(1 for a in cs.scr if cs.scr[a])} "
          f"RCMTA={sum(1 for a in cs.rcmta if cs.rcmta[a])} "
          f"TPL={sum(1 for a in cs.tpl if cs.tpl[a])} "
          f"REG={sum(1 for o in cs.reg if cs.reg[o])} GPR={gpr} SPR={spr}", file=sys.stderr)
    if cmds:
        print("[cosim] host commands: " +
              ", ".join(f"{k}:{v:#x}x{n}" for (k, v), n in sorted(cmds.items())),
              file=sys.stderr)


if __name__ == "__main__":
    sys.exit(main())
