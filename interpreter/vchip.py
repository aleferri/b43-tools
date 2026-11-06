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
"""A virtual D11 chip: the PSM running the real ucode and the hardware around
it, shared by host and ucode. It knows a core revision only through its
core.py profile, and PHY and radio only as devices (RegisterFile) with what
the board gives them: the PHY version word, the radio's identity registers.

Host MMIO, as b43 uses it:
  0x160/0x164/0x166  object window. Control word routing << 16 | offset in
                     32-bit cells; 32-bit data access moves a cell, 16-bit
                     ones its low half (0x164) or high half (0x166); AUTOINC
                     steps after an access that ends a cell. Spaces: UCODE
                     (decoded when the PSM starts), SHARED, SCRATCH (r0..r63),
                     HW, RCMTA.
  0x3FC/0x3FE        PHY address/data: the device the ucode reaches.
  0x3F6/0x3FA/0x3F8  radio address/data/data high; 0x3D8/0x3DA from corerev 24.
  0x3E0              PHY_VER, the board's word.
  0x130/0x134        template RAM address/data, 4 bytes a write.
  0x120              MACCONTROL: PSM_RUN runs the PSM, PSM_JMP_0 restarts it
                     at 0; its high half is SPR 0x041.
  0x124/0x128/0x12C  MACCMD, GEN_IRQ_REASON (write 1 to clear), IRQ mask:
                     SPR 0x047, 0x042/0x043, 0x044/0x045.
  0x180/0x184        TSF; 0x188/0x18C beacon interval (us << 6) and first TBTT.
  0x400 + 2n         SPR n; the profile's host_regs act on host accesses.
Any other register reads back what was written; reads of it are counted.

Initvals go through the MMIO when the ucode first raises MAC_SUSPENDED after
PSM_RUN, as brcms_b_coreinit writes them.

The ucode reaches the profile's buses, a transaction completing at once.
Conditions: COND_TRUE, COND_MACEN from MACCONTROL.ENABLED, condition register
4 (0x40..0x4F) from SPR_BRC; with a TX engine (IFS, START, DONE in
instructions) COND_TX_NOW after bit 0 of SPR_TXE0_CTL, COND_TX_POWER and
COND_TX_DONE after the profile's tx_go; with the TSF running
COND_TX_TBTTEXPIRE at each TBTT. Each is held until the ucode's EOI; every
other condition is false. The TX engine is a model, not a known semantic.

With mhz F the TSF advances one microsecond every F instructions (a model).
settle() runs the PSM until idle: back at a (pc, call stack, carry, raised
conditions) with no net change and no TSF read since. wait() and advance()
let time pass, the clock jumping to the next TBTT while the ucode is idle;
advance() does not run an idle PSM again until the host writes.
"""

import struct

import psmlib

MACCTL, MACCMD, IRQ_REASON, IRQ_MASK = 0x120, 0x124, 0x128, 0x12C
MACCTL_ENABLED, MACCTL_PSM_RUN, MACCTL_PSM_JMP0 = 0x1, 0x2, 0x4
IHR_BASE = 0x400
SPR_MAC_CTLHI, SPR_IRQ_LO, SPR_IRQ_HI = 0x041, 0x042, 0x043
SPR_IRQMASK_LO, SPR_IRQMASK_HI, SPR_MAC_CMD = 0x044, 0x045, 0x047
MMIO_PHY_VER = 0x3E0
COND_MACEN = 0x24
COND_TX_NOW, COND_TX_POWER, COND_TX_DONE = 0x20, 0x21, 0x22
CONDREG_BRC = 0x40              # condition register 4 mirrors SPR_BRC
SPR_BRC, SPR_TXE0_CTL = 0x048, 0x080
SPR_TSF_WORD0 = 0x119           # SPR_TSF_WORD0..3, MMIO 0x632..0x638
TSF_SPRS = range(SPR_TSF_WORD0, SPR_TSF_WORD0 + 4)
TSF_TIMERLOW, TSF_TIMERHIGH = 0x180, 0x184
TSF_CFPREP, TSF_CFPSTART, CFPREP_CBI_SHIFT = 0x188, 0x18C, 6
COND_TBTT = 0x2C                # OpenFWWF's COND_TX_TBTTEXPIRE
IRQ_MAC_SUSPENDED = 0x0001
HOSTF_WORDS = (0x05E >> 1, 0x060 >> 1, 0x062 >> 1, 0x078 >> 1, 0x0D4 >> 1)
OBJADDR, OBJDATA, OBJDATA_HI = 0x160, 0x164, 0x166
OBJ_UCODE, OBJ_SHARED, OBJ_SCRATCH, OBJ_HW, OBJ_RCMTA = range(5)
OBJ_AUTOINC_W, OBJ_AUTOINC_R = 0x01000000, 0x02000000
RAM_CONTROL, RAM_DATA = 0x130, 0x134
RADIO24_CONTROL, RADIO24_DATA = 0x3D8, 0x3DA
RADIO_CONTROL, RADIO_DATA_HIGH, RADIO_DATA_LOW = 0x3F6, 0x3F8, 0x3FA
PHY_CONTROL, PHY_DATA = 0x3FC, 0x3FE
SCRATCH_REGS = 64

B43_IV_OFFSET_MASK, B43_IV_32BIT = 0x7FFF, 0x8000


def load_inits(path):
    """A d11init blob: big-endian {u16 addr, u16 size, u32 value} records
    up to the 0xFFFF terminator (brcms_c_write_inits); or a b43 initvals .fw,
    big-endian {u16 offset|32BIT, u16 or u32 value} records after the header
    (b43_write_initvals). Both as (addr, size, value)."""
    data = open(path, "rb").read()
    iv = psmlib.strip_fw_header(data, 'i')
    if iv is not None:
        return load_b43_inits(iv)
    inits = []
    for off in range(0, len(data) - 7, 8):
        addr, size, value = struct.unpack_from(">HHI", data, off)
        if addr == 0xFFFF:
            break
        inits.append((addr, size, value))
    return inits


def load_b43_inits(data):
    inits, off = [], 0
    while off + 4 <= len(data):
        offset_size = struct.unpack_from(">H", data, off)[0]
        addr = offset_size & B43_IV_OFFSET_MASK
        if offset_size & B43_IV_32BIT:
            inits.append((addr, 4, struct.unpack_from(">I", data, off + 2)[0]))
            off += 6
        else:
            inits.append((addr, 2, struct.unpack_from(">H", data, off + 2)[0]))
            off += 4
    return inits


class RegisterFile:
    """A device behind a bus that holds what was written and reads 0 where
    nothing was. The interface the chip expects of every device:

      read(addr) / write(addr, value)   a transaction, from host or ucode
      known(addr)                       whether anyone set the register
      seed(addr, value)                 set it unless someone already did
      snapshot()                        a comparable copy of the state

    A model of a PHY or a radio extends it with what its registers do."""

    def __init__(self, name, initial=None):
        self.name = name
        self.regs = dict(initial or {})

    def read(self, addr):
        return self.regs.get(addr, 0)

    def write(self, addr, value):
        self.regs[addr] = value & 0xFFFF

    def known(self, addr):
        return addr in self.regs

    def seed(self, addr, value):
        self.regs.setdefault(addr, value & 0xFFFF)

    def snapshot(self):
        return dict(self.regs)


class Recorder:
    """Events of the chip, for an observer."""

    def step(self, pc, ins, ext_flags):
        pass

    def unset_spr_read(self, spr, pc):
        pass

    def store(self, loc, old, new):
        pass

    def bus(self, name, kind, addr, value, source):
        pass

    def tbtt(self, tsf):
        pass

    def tx_condition(self, sel):
        pass


class Chip:
    def __init__(self, prog, hw, devices=None, inits=None, tx=None, mhz=None,
                 recorder=None, phy_version=None):
        self.prog = prog
        self.hw = hw
        self.devices = {name: RegisterFile(name) for name in
                        ["phy", "radio"] + [bus.name for bus in hw.buses]}
        self.devices.update(devices or {})
        self.phy = self.devices["phy"]
        self.radio = self.devices["radio"]
        self.phy_version = phy_version
        if phy_version is not None:
            self.phy.seed(0, phy_version)
        self.recorder = recorder or Recorder()
        self.m = hw.isa.PSM(prog)
        self.m.stall_threshold = float("inf")
        self.m.observer = self.observe
        self.m.on_eoi = self.eoi
        self.tx = tx                    # (ifs, start, done) or None
        self.events = set()             # raised conditions held until EOI
        self.pending = []               # (step, condition) still to raise
        self.mhz = mhz                  # instructions per TSF microsecond
        self.clock = 0                  # time, in PSM instructions
        self.tsf_base, self.tsf_clock = 0, 0
        self.tbtt_us, self.next_tbtt = 0, None
        self.tsf_read_at = -1           # step of the last ucode read of the TSF
        self.inits = inits or []
        self.inits_state = "wait" if self.inits else "none"   # wait, due, done
        self.running = False
        self.macctl = 0
        self.objaddr = 0                # SHM_CONTROL
        self.ucode = []                 # program memory, 32-bit cells
        self.ucode_dirty = False
        self.obj_hw, self.obj_rcmta = {}, {}
        self.template = {}              # template RAM, 32-bit cells by byte offset
        self.ram_control = 0
        self.phy_addr = self.radio_addr = 0
        self.mmio = {}                  # registers without a meaning here
        self.unmodelled = {}            # offset -> reads of them
        self.quiet = False              # idle, and no host write since
        self.host_regs = {hr.spr: hr for hr in hw.host_regs}
        self.host_spr = set()           # SPRs the host has written
        self.host_epoch = 0             # bumped by host writes made during a run
        self.stopped = None             # exception that stopped the PSM
        self.steps = 0
        self.changes = []               # (loc, old) of state changes in this settle

    def spr_write(self, n, val):
        self.m.spr[n] = val & 0xFFFF
        self.host_spr.add(n)

    def host_reg_write(self, off, val, bits):
        """The effect of a host MMIO write on the MAC registers, the TSF and
        the IHR; bits is 16 or 32."""
        if self.mhz and off == TSF_CFPREP:
            self.tbtt_us = (val & 0xFFFFFFFF) >> CFPREP_CBI_SHIFT
            self.tbtt_align()
        elif self.mhz and off == TSF_CFPSTART:
            self.next_tbtt = val & 0xFFFFFFFF
            self.tbtt_align()
        elif self.mhz and off in (TSF_TIMERLOW, TSF_TIMERHIGH):
            shift = 0 if off == TSF_TIMERLOW else 32
            self.tsf_set(self.tsf() & ~(0xFFFFFFFF << shift) | (val & 0xFFFFFFFF) << shift)
        elif off >= IHR_BASE:
            n = (off - IHR_BASE) >> 1
            for k, part in ((n, val), (n + 1, val >> 16))[:2 if bits == 32 else 1]:
                if self.mhz and k in TSF_SPRS:
                    self.tsf_set_word(k - SPR_TSF_WORD0, part)
                    self.host_spr.add(k)
                else:
                    hr = self.host_regs.get(k)
                    self.spr_write(k, hr.after_write(part) if hr else part)
        elif off == MACCMD:
            self.spr_write(SPR_MAC_CMD, val)
        elif off == IRQ_REASON:
            for n, part in ((SPR_IRQ_LO, val), (SPR_IRQ_HI, val >> 16)):
                self.m.spr[n] = self.m.spr.get(n, 0) & ~part & 0xFFFF
                self.host_spr.add(n)
        elif off == IRQ_MASK:
            self.spr_write(SPR_IRQMASK_LO, val)
            self.spr_write(SPR_IRQMASK_HI, val >> 16)

    def host_reg_observed(self, off, val):
        """A value the host read from MMIO in a capture."""
        if off == MMIO_PHY_VER:
            if self.phy_version is None:
                self.phy_version = val & 0xFFFF
            self.phy.seed(0, val)

    def host_macctl_write(self, val, mask=None):
        if mask is not None:
            val = (self.macctl & ~mask) | val
        self.macctl = val
        self.spr_write(SPR_MAC_CTLHI, val >> 16)
        if self.ucode_dirty and val & (MACCTL_PSM_RUN | MACCTL_PSM_JMP0):
            self.load_ucode()
        if val & MACCTL_PSM_JMP0:
            self.m.pc = 0
            self.m.call_stack.clear()
        self.running = bool(val & MACCTL_PSM_RUN) and self.stopped is None

    def host_maccmd_write(self, val):
        self.spr_write(SPR_MAC_CMD, val)

    def host_shm_write(self, word, val, words=1):
        shm = self.m.shm
        shm[word & (len(shm) - 1)] = val & 0xFFFF
        if words == 2:
            shm[(word + 1) & (len(shm) - 1)] = val >> 16

    def host_shm_read(self, word):
        return self.m.shm[word & (len(self.m.shm) - 1)]

    def host_hostflags(self, index, val, mask):
        w = HOSTF_WORDS[index]
        self.m.shm[w] = (self.m.shm[w] & ~mask) | (val & mask)

    def host_phy_write(self, addr, val):
        self.phy.write(addr, val)

    def write_inits(self):
        for addr, size, value in self.inits:
            self.mmio_write(addr, value, 32 if size == 4 else 16)
        self.inits_state = "done"
        self.host_epoch += 1

    def mmio_write(self, off, val, bits):
        """A host write to the D11 core's registers."""
        self.quiet = False
        if off == OBJADDR:
            self.objaddr = val & 0xFFFFFFFF
        elif off in (OBJDATA, OBJDATA_HI):
            self.obj_access(off, bits, val)
        elif off == MACCTL:
            self.host_macctl_write(val)
        elif off == RAM_CONTROL:
            self.ram_control = val & 0xFFFFFFFF
        elif off == RAM_DATA:
            self.template[self.ram_control & ~3] = val & 0xFFFFFFFF
            self.ram_control = (self.ram_control + 4) & 0xFFFFFFFF
        elif off == PHY_CONTROL:
            self.phy_addr = val & 0xFFFF
        elif off == PHY_DATA:
            self.phy.write(self.phy_addr, val)
        elif off in (RADIO_CONTROL, RADIO24_CONTROL):
            self.radio_addr = val & 0xFFFF
        elif off in (RADIO_DATA_LOW, RADIO24_DATA):
            self.radio.write(self.radio_addr, val)
        elif off in (MACCMD, IRQ_REASON, IRQ_MASK, TSF_CFPREP, TSF_CFPSTART,
                     TSF_TIMERLOW, TSF_TIMERHIGH) or off >= IHR_BASE:
            self.host_reg_write(off, val, bits)
        else:
            self.mmio[off] = val & (0xFFFFFFFF if bits == 32 else 0xFFFF)

    def mmio_read(self, off, bits):
        """A host read of the D11 core's registers."""
        spr = self.m.spr
        if off == OBJADDR:
            return self.objaddr
        if off in (OBJDATA, OBJDATA_HI):
            return self.obj_access(off, bits)
        if off == MACCTL:
            return self.macctl
        if off == MACCMD:
            return spr.get(SPR_MAC_CMD, 0)
        if off == IRQ_REASON:
            return spr.get(SPR_IRQ_LO, 0) | spr.get(SPR_IRQ_HI, 0) << 16
        if off == IRQ_MASK:
            return spr.get(SPR_IRQMASK_LO, 0) | spr.get(SPR_IRQMASK_HI, 0) << 16
        if off == RAM_CONTROL:
            return self.ram_control
        if off == MMIO_PHY_VER:
            return self.phy_version or 0
        if off == PHY_CONTROL:
            return self.phy_addr
        if off == PHY_DATA:
            return self.phy.read(self.phy_addr)
        if off in (RADIO_CONTROL, RADIO24_CONTROL):
            return self.radio_addr
        if off in (RADIO_DATA_LOW, RADIO24_DATA):
            return self.radio.read(self.radio_addr) & 0xFFFF
        if off == RADIO_DATA_HIGH:
            return self.radio.read(self.radio_addr) >> 16 & 0xFFFF
        if self.mhz and off in (TSF_TIMERLOW, TSF_TIMERHIGH):
            return self.tsf() >> (0 if off == TSF_TIMERLOW else 32) & 0xFFFFFFFF
        if off >= IHR_BASE:
            n = (off - IHR_BASE) >> 1
            if self.mhz and (n in TSF_SPRS or n + 1 in TSF_SPRS):
                self.tsf_publish()
            val = spr.get(n, 0)
            if bits == 32:
                val |= spr.get(n + 1, 0) << 16
            for k in (n, n + 1)[:2 if bits == 32 else 1]:
                hr = self.host_regs.get(k)
                if hr:
                    spr[k] = hr.after_read(spr.get(k, 0))
            return val
        self.unmodelled[off] = self.unmodelled.get(off, 0) + 1
        return self.mmio.get(off, 0)

    def obj_access(self, port, bits, val=None):
        """One data access through the object window: a write if val is
        given, else a read whose value is returned."""
        ctl = self.objaddr
        space, cell = ctl >> 16 & 0xFF, ctl & 0xFFFF
        if bits == 32:
            lo = hi = True
        else:
            lo, hi = port == OBJDATA, port == OBJDATA_HI
        if val is not None:
            if lo:
                self.obj_half(space, cell, 0, val & 0xFFFF)
            if hi:
                self.obj_half(space, cell, 1, (val >> 16 if bits == 32 else val) & 0xFFFF)
            step = ctl & OBJ_AUTOINC_W
        else:
            val = 0
            if lo:
                val = self.obj_half(space, cell, 0)
            if hi:
                h = self.obj_half(space, cell, 1)
                val = val | h << 16 if bits == 32 else h
            step = ctl & OBJ_AUTOINC_R
        if step and hi:
            self.objaddr = (ctl & ~0xFFFF) | ((cell + 1) & 0xFFFF)
        return val

    def obj_half(self, space, cell, half, val=None):
        """Read one 16-bit half of a cell, or write it if val is given."""
        if space == OBJ_SHARED:
            shm = self.m.shm
            w = (2 * cell + half) & (len(shm) - 1)
            if val is None:
                return shm[w]
            shm[w] = val
            return None
        if space == OBJ_SCRATCH:
            if half:
                return 0 if val is None else None
            if val is None:
                return self.m.reg[cell % SCRATCH_REGS]
            self.m.reg[cell % SCRATCH_REGS] = val
            return None
        if space == OBJ_UCODE:
            store = self.ucode
            if cell >= len(store):
                store.extend([0] * (cell + 1 - len(store)))
            old = store[cell]
        elif space in (OBJ_HW, OBJ_RCMTA):
            store = self.obj_hw if space == OBJ_HW else self.obj_rcmta
            old = store.get(cell, 0)
        else:
            return 0 if val is None else None
        if val is None:
            return old >> 16 * half & 0xFFFF
        store[cell] = old & ~(0xFFFF << 16 * half) | val << 16 * half
        if space == OBJ_UCODE:
            self.ucode_dirty = True
        return None

    def load_ucode(self):
        """Decode the program memory with the profile's instruction set."""
        isa, words = self.hw.isa, self.ucode
        prog = []
        for i in range(len(words) // 2):
            codeword = words[2 * i + 1] << 32 | words[2 * i]
            if codeword >> isa.TOP_BIT:
                self.stopped = f"ucode word {i:#x} is not {isa.NAME}: {codeword:#018x}"
                break
            prog.append(isa.decode_instr(codeword, i))
        self.prog = self.m.prog = prog
        self.ucode_dirty = False

    def snapshot(self):
        """What the host could change or the ucode could react to, as a
        comparable copy."""
        return (self.macctl, self.m.pc, self.host_epoch, list(self.m.shm), dict(self.m.spr),
                {name: d.snapshot() for name, d in self.devices.items()})

    def tsf(self):
        return self.tsf_base + int((self.clock - self.tsf_clock) / self.mhz)

    def tsf_set(self, value):
        self.tsf_base, self.tsf_clock = value & (1 << 64) - 1, self.clock
        self.tbtt_align()

    def tbtt_align(self):
        if self.next_tbtt is not None and self.tbtt_us:
            t = self.tsf()
            while self.next_tbtt <= t:
                self.next_tbtt += self.tbtt_us

    def tbtt_due_clock(self):
        """The clock at which the TSF reaches the next TBTT, or None."""
        if self.next_tbtt is None:
            return None
        return self.tsf_clock + (self.next_tbtt - self.tsf_base) * self.mhz

    def tsf_set_word(self, i, val):
        shift = 16 * i
        self.tsf_set(self.tsf() & ~(0xFFFF << shift) | (val & 0xFFFF) << shift)

    def tsf_publish(self):
        t = self.tsf()
        for i in range(4):
            self.m.spr[SPR_TSF_WORD0 + i] = t >> 16 * i & 0xFFFF

    def eoi(self, sel):
        self.events.discard(sel)

    def tx_schedule(self, loc, old, new):
        ifs, start, done = self.tx
        if loc == ("spr", SPR_TXE0_CTL) and new & 1 and not old & 1:
            self.pending.append((self.steps + ifs, COND_TX_NOW))
        elif self.hw.tx_go and loc == ("spr", self.hw.tx_go.spr) and \
                new & self.hw.tx_go.bits == self.hw.tx_go.bits:
            self.pending.append((self.steps + start, COND_TX_POWER))
            self.pending.append((self.steps + done, COND_TX_DONE))

    def conditions(self):
        m = self.m
        flags = {COND_MACEN: bool(self.macctl & MACCTL_ENABLED)}
        brc = m.spr.get(SPR_BRC, 0)
        for bit in range(16):
            flags[CONDREG_BRC | bit] = bool(brc >> bit & 1)
        if self.mhz and self.next_tbtt is not None and self.tsf() >= self.next_tbtt:
            self.events.add(COND_TBTT)
            self.recorder.tbtt(self.next_tbtt)
            if self.tbtt_us:
                self.tbtt_align()
            else:
                self.next_tbtt = None
        if self.pending:
            due = [p for p in self.pending if p[0] <= self.steps]
            for p in due:
                self.pending.remove(p)
                self.events.add(p[1])
                self.recorder.tx_condition(p[1])
        for sel in self.events:
            flags[sel] = True
        m.ext_flags = flags

    def observe(self, loc, old, new):
        kind, n = loc
        if self.tx and kind == "spr":
            self.tx_schedule(loc, old, new)
        if self.mhz and kind == "spr" and n in TSF_SPRS:
            self.tsf_set_word(n - SPR_TSF_WORD0, new)
        if kind == "spr" and n == SPR_IRQ_LO and new & IRQ_MAC_SUSPENDED \
                and self.inits_state == "wait":
            self.inits_state = "due"
        self.recorder.store(loc, old, new)
        if new != old:
            self.changes.append((loc, old))

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
        if self.mhz:
            self.tsf_publish()
        self.conditions()
        for op in (ins.op0, ins.op1):
            if op is None or op.kind != "spr":
                continue
            if self.mhz and op.value in TSF_SPRS:
                self.tsf_read_at = self.steps
            elif op.value not in m.spr and op.value not in self.host_spr:
                self.recorder.unset_spr_read(op.value, pc)
        self.recorder.step(pc, ins, m.ext_flags)
        try:
            m.step()
        except Exception as e:      # noqa: BLE001
            self.stopped = f"{type(e).__name__} at {pc:#x}: {e}"
            self.running = False
            return False
        self.steps += 1
        self.clock += 1
        for bus in self.hw.buses:
            cmd = m.spr.get(bus.addr_spr, 0)
            kind = bus.command.command(cmd)
            if kind:
                self.transfer(bus, cmd, kind)
        for hs in self.hw.handshakes:
            value = m.spr.get(hs.spr, 0)
            answer = hs.respond(value)
            if answer != value:
                m.store(("spr", hs.spr), answer)
        if self.inits_state == "due":
            self.write_inits()
        return True

    def transfer(self, bus, cmd, kind):
        m = self.m
        dev = self.devices[bus.name]
        addr = cmd & bus.command.addr_mask
        if kind == "read":
            src = "host" if dev.known(addr) else "never set"
            val = dev.read(addr)
            m.store(("spr", bus.data_spr), val)
        else:
            src = "ucode"
            val = m.spr.get(bus.data_spr, 0)
            dev.write(addr, val)
        self.recorder.bus(bus.name, kind, addr, val, src)
        m.store(("spr", bus.addr_spr), bus.command.complete(cmd))

    def run(self, n):
        for _ in range(n):
            if not self.step():
                return

    def settle(self, max_steps):
        """Run until idle; return (steps until the idle cycle began, its pc),
        the pc being None if max_steps ran out first."""
        m = self.m
        start, self.changes, seen, epoch = self.steps, [], {}, self.host_epoch
        while self.running and self.steps - start < max_steps:
            if epoch != self.host_epoch:
                seen, epoch = {}, self.host_epoch
            key = (m.pc, tuple(m.call_stack), m.carry,
                   tuple(sorted(self.events)))
            prev = seen.get(key)
            if prev is not None and not self.pending and \
                    self.tsf_read_at < prev[0] and self.unchanged_since(prev[1]):
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

    def advance(self, us):
        """Let `us` microseconds pass for a host that drives the chip in time:
        the PSM runs for them, or until it is idle; idle and with nothing
        written by the host since, it does not run again, and the clock
        jumps to the next TBTT or to the end."""
        end = self.clock + int(us * self.mhz)
        while self.clock < end:
            if not self.running:
                break
            if not self.quiet:
                steps, pc = self.settle(end - self.clock)
                if pc is None:
                    continue
                self.quiet = True
            due = self.tbtt_due_clock()
            if due is None or due >= end:
                break
            self.clock = max(self.clock, int(due) + 1)
            self.quiet = False
        self.clock = max(self.clock, end)

    def wait(self, us, max_steps):
        """Let `us` microseconds of TSF pass. The ucode runs; while it is idle
        nothing it could observe changes but the TSF, so the clock jumps to
        the next TBTT or to the end of the wait. Returns the (TSF, steps, idle
        pc or None) of each stretch the ucode worked."""
        worked = []
        end = self.clock + us * self.mhz
        while self.clock < end:
            if not self.running:
                self.clock = end
                break
            steps, pc = self.settle(max_steps)
            if steps:
                worked.append((self.tsf(), steps, pc))
            if pc is None:
                continue
            due = self.tbtt_due_clock()
            if due is None or due >= end:
                self.clock = end
                break
            self.clock = max(self.clock, int(due) + 1)
        return worked
