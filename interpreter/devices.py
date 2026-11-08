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
"""PHY and radio models: a register file plus behaviours, each owning some
registers. What a PHY or radio type does is the list of its behaviours;
devcheck.py measures each against captures."""

import vchip


class Device(vchip.RegisterFile):
    """now() is the time in microseconds, set by whoever runs the device."""

    def __init__(self, name, behaviours=(), initial=None):
        super().__init__(name, initial)
        self.now = lambda: 0
        self.owner = {}
        for b in behaviours:
            b.device = self
            for reg in b.registers:
                self.owner[reg] = b

    def read(self, addr):
        b = self.owner.get(addr)
        return b.read(addr) if b else super().read(addr)

    def write(self, addr, value):
        b = self.owner.get(addr)
        if b:
            b.write(addr, value & 0xFFFF)
        else:
            super().write(addr, value)

    def known(self, addr):
        b = self.owner.get(addr)
        return b.known(addr) if b else super().known(addr)

    def observe(self, addr, value):
        b = self.owner.get(addr)
        if b:
            b.observe(addr, value & 0xFFFF)
        else:
            super().observe(addr, value)

    def waiting(self):
        return any(b.waiting() for b in dict.fromkeys(self.owner.values()))

    def next_change(self):
        times = [b.next_change() for b in dict.fromkeys(self.owner.values())]
        times = [t for t in times if t is not None]
        return min(times) if times else None

    def snapshot(self):
        return (super().snapshot(),
                tuple(b.snapshot() for b in dict.fromkeys(self.owner.values())))


class TablePortSplit:
    """Tables reached through an id register, an offset register and data
    ports: a 32-bit cell is written HI then LO, LO storing the cell and
    stepping the offset; a read of LO fetches the cell, steps the offset and
    latches the high half for HI. The alternate port moves 16 bits a step."""

    def __init__(self, id_reg, offset_reg, lo, hi, alt):
        self.id_reg, self.offset_reg, self.lo, self.hi, self.alt = id_reg, offset_reg, lo, hi, alt
        self.registers = (id_reg, offset_reg, lo, hi, alt)
        self.tables = {}
        self.table_id = self.offset = 0
        self.hi_in = self.hi_out = 0

    def cell(self):
        return self.tables.setdefault(self.table_id, {})

    def step(self):
        self.offset = (self.offset + 1) & 0xFFFF

    def write(self, addr, value):
        if addr == self.id_reg:
            self.table_id = value
        elif addr == self.offset_reg:
            self.offset = value
        elif addr == self.hi:
            self.hi_in = value
        elif addr == self.lo:
            self.cell()[self.offset] = self.hi_in << 16 | value
            self.hi_in = 0
            self.step()
        else:
            self.cell()[self.offset] = value
            self.step()

    def read(self, addr):
        if addr == self.id_reg:
            return self.table_id
        if addr == self.offset_reg:
            return self.offset
        if addr == self.hi:
            return self.hi_out
        value = self.cell().get(self.offset, 0)
        self.step()
        if addr == self.lo:
            self.hi_out = value >> 16
        return value & 0xFFFF

    def known(self, addr):
        if addr in (self.id_reg, self.offset_reg, self.hi):
            return True
        return self.offset in self.cell()

    def observe(self, addr, value):
        """A captured read: the cell read holds that value (the read stepped
        the offset already)."""
        if addr in (self.lo, self.alt):
            off = (self.offset - 1) & 0xFFFF
            old = self.cell().get(off, 0)
            self.cell()[off] = old & ~0xFFFF | value if addr == self.lo else value
        elif addr == self.hi:
            self.hi_out = value

    def waiting(self):
        return False

    def next_change(self):
        return None

    def snapshot(self):
        return (self.table_id, self.offset, {t: dict(c) for t, c in self.tables.items()})


class BusyBits:
    """A command register whose bits stay set for `us` microseconds after a
    write that sets them, then clear."""

    def __init__(self, reg, bits, us):
        self.reg, self.bits, self.us = reg, bits, us
        self.registers = (reg,)
        self.value = None
        self.since = None

    def write(self, addr, value):
        self.value = value
        self.since = self.device.now() if value & self.bits else None

    def read(self, addr):
        if self.since is not None and self.device.now() - self.since >= self.us:
            self.value &= ~self.bits
            self.since = None
        return self.value or 0

    def known(self, addr):
        return self.value is not None

    def observe(self, addr, value):
        self.value = value
        if not value & self.bits:
            self.since = None

    def waiting(self):
        return self.since is not None and self.device.now() - self.since < self.us

    def next_change(self):
        return self.since + self.us if self.waiting() else None

    def snapshot(self):
        return (self.value, self.since)


class StatusBits:
    """A register that reads `status` on top of what was written while the
    `trigger` bits are written set."""

    def __init__(self, reg, trigger, status):
        self.reg, self.trigger, self.status = reg, trigger, status
        self.registers = (reg,)
        self.value = None

    def write(self, addr, value):
        self.value = value

    def read(self, addr):
        value = self.value or 0
        return value | self.status if value & self.trigger else value

    def known(self, addr):
        return self.value is not None

    def observe(self, addr, value):
        self.value = value

    def waiting(self):
        return False

    def next_change(self):
        return None

    def snapshot(self):
        return self.value


# PHY types as in PHY_VER bits 11:8 (b43.h's B43_PHYTYPE_*).
PHY_BEHAVIOURS = {
    0x0B: lambda: [                                                       # AC
        TablePortSplit(0x00D, 0x00E, 0x00F, 0x010, 0x011),
        BusyBits(0x380, 0x8000, us=200),    # RX AFE calibration: 40 us..1 ms captured
        BusyBits(0x270, 0x0001, us=200),    # RX IQ estimator: up to ~40 busy polls captured
    ],
}


# Radio ids as b43 reports them.
RADIO_BEHAVIOURS = {
    0x2069: lambda: [
        StatusBits(0x40B, 0x0001, 0x0168),  # power-on: 0x0001 written, 0x0169 read
    ],
}


def phy_device(phy_type, initial=None):
    make = PHY_BEHAVIOURS.get(phy_type)
    return Device("phy", make() if make else (), initial)


def radio_device(radio_id, initial=None):
    make = RADIO_BEHAVIOURS.get(radio_id)
    return Device("radio", make() if make else (), initial)
