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
"""The protocols through which the ucode reaches the PHY and the radio, as
core.py's profiles describe them, over the devices of devices.py:

  SprBus      a command SPR (register, read or write, busy) and a data SPR:
              the PHY on every core, the radio from corerev 24
  FourWireBus the radio behind two PHY registers, up to corerev 23, for
              ucode and host alike

A transaction on an SprBus takes `latency` PSM instructions, during which
its busy bit reads as set. Nothing measured gives it: with 0, the default,
it completes right after the instruction that issued it.
"""


class SprBus:
    def __init__(self, spec, device, latency=0):
        self.spec = spec                # core.IndirectBus
        self.device = device
        self.latency = latency
        self.pending = None             # [kind, command, write value, instructions left]

    def tick(self, m):
        """After a PSM instruction: the transaction it completes, as (kind,
        addr, value, source, timed), timed if a read met a device whose
        registers change in time; else None."""
        spec = self.spec
        if self.pending is None:
            cmd = m.spr.get(spec.addr_spr, 0)
            kind = spec.command.command(cmd)
            if kind is None:
                return None
            value = m.spr.get(spec.data_spr, 0) if kind == "write" else None
            self.pending = [kind, cmd, value, self.latency]
            if self.latency:
                m.store(("spr", spec.addr_spr), cmd | spec.command.busy)
        if self.pending[3]:
            self.pending[3] -= 1
            return None
        kind, cmd, value, _ = self.pending
        self.pending = None
        addr = cmd & spec.command.addr_mask
        timed = False
        if kind == "read":
            source = "host" if self.device.known(addr) else "never set"
            timed = self.device.waiting()
            value = self.device.read(addr)
            m.store(("spr", spec.data_spr), value)
        else:
            source = "ucode"
            self.device.write(addr, value)
        m.store(("spr", spec.addr_spr), spec.command.complete(cmd))
        return kind, addr, value, source, timed


class FourWireBus:
    """The PHY as its bus and the host see it: the register in `spec.addr`
    names a radio register, `spec.data` reaches it; every other access is
    the PHY's."""

    def __init__(self, spec, phy, radio):
        self.spec = spec                # core.FourWire
        self.phy = phy
        self.radio = radio
        self.name = phy.name

    def radio_addr(self, addr):
        return self.phy.read(self.spec.addr) if addr == self.spec.data else None

    def read(self, addr):
        radio = self.radio_addr(addr)
        return self.phy.read(addr) if radio is None else self.radio.read(radio)

    def write(self, addr, value):
        radio = self.radio_addr(addr)
        if radio is None:
            self.phy.write(addr, value)
        else:
            self.radio.write(radio, value)

    def known(self, addr):
        radio = self.radio_addr(addr)
        return self.phy.known(addr) if radio is None else self.radio.known(radio)

    def seed(self, addr, value):
        self.phy.seed(addr, value)

    def observe(self, addr, value):
        radio = self.radio_addr(addr)
        if radio is None:
            self.phy.observe(addr, value)
        else:
            self.radio.observe(radio, value)

    def latch(self, addr, bits=0):
        return self.phy.latch(addr, bits)

    def waiting(self):
        return self.phy.waiting() or self.radio.waiting()

    def next_change(self):
        times = [t for t in (self.phy.next_change(), self.radio.next_change()) if t is not None]
        return min(times) if times else None
