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
"""A board as the host sees it through its PCI function: BAR0 and the vendor
part of the configuration space, routed to the chip's cores by backplane
address.

BAR0 of a 16 KiB host interface, pcicfg.h's PCI_16KB0_*:
  0x0000-0x0FFF  window on the backplane at PCI_BAR0_WIN (config 0x80)
  0x1000-0x1FFF  second window at PCI_BAR0_WIN2 (0xAC; 0x70 on PCIe gen2)
  0x2000-0x2FFF  the host core, PCI or PCIe
  0x3000-0x3FFF  ChipCommon, at the backplane's first core
The vendor configuration registers (0x40-0xFF) read back what was written.

A core is a model of its registers at a backplane address: mmio_read(off,
bits) and mmio_write(off, val, bits) with core-relative offsets, and
optionally advance(us) and irq(). An access no core models goes to the
fallback by BAR0 offset, which sees every write, and is counted by backplane
address: what of the board something else than a model still answers. While
the host core's address is not known, its region counts as "host".
"""

import collections

CC_BASE = 0x18000000
BAR0_WIN, BAR0_WIN2 = 0x80, 0xAC
PCIE2_BAR0_WIN2 = 0x70


class Board:
    def __init__(self, fallback, host_core=None, pcie_gen=1, cores=None):
        self.fallback = fallback
        self.host_core = host_core
        self.win2_reg = PCIE2_BAR0_WIN2 if pcie_gen == 2 else BAR0_WIN2
        self.cores = cores or {}
        self.config = bytearray(0x100)
        self.unmodelled = collections.Counter()

    def config_word(self, off):
        return int.from_bytes(self.config[off:off + 4], "little")

    def backplane(self, off):
        """The backplane address BAR0 offset off reaches."""
        region, rel = off & ~0xFFF, off & 0xFFF
        if region == 0x0000:
            return self.config_word(BAR0_WIN) + rel
        if region == 0x1000:
            return self.config_word(self.win2_reg) + rel
        if region == 0x2000:
            return self.host_core + rel if self.host_core is not None else None
        return CC_BASE + rel

    def core(self, addr):
        if addr is None:
            return None, None
        base = addr & ~0xFFF
        return self.cores.get(base), addr - base

    def mmio_read(self, off, bits):
        addr = self.backplane(off)
        core, rel = self.core(addr)
        if core is None:
            self.unmodelled[addr if addr is not None else f"host+{off & 0xFFF:#05x}"] += 1
            return self.fallback.read(off, bits)
        val = core.mmio_read(rel, bits)
        self.fallback.follow(False, bits, off, val)
        return val

    def mmio_write(self, off, val, bits):
        core, rel = self.core(self.backplane(off))
        if core is not None:
            core.mmio_write(rel, val, bits)
        self.fallback.write(off, val, bits)

    def config_read(self, off, bits):
        return int.from_bytes(self.config[off:off + bits // 8], "little")

    def config_write(self, off, val, bits):
        self.config[off:off + bits // 8] = val.to_bytes(bits // 8, "little")

    def advance(self, us):
        for core in self.cores.values():
            if hasattr(core, "advance"):
                core.advance(us)

    def irq(self):
        return any(core.irq() for core in self.cores.values() if hasattr(core, "irq"))

    def report(self, out, top=40):
        n = sum(self.unmodelled.values())
        out.write(f"board: {n} reads of {len(self.unmodelled)} registers no core models, "
                  f"by backplane address:\n")
        for addr, count in self.unmodelled.most_common(top):
            out.write(f"  {addr:#010x}: {count}\n" if isinstance(addr, int) else f"  {addr}: {count}\n")
