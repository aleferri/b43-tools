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
"""The hardware around the PSM, per core revision, as read from the ucode of
each generation: OpenFWWF for corerev 5, the DSL-3580_EU driver's ucode for
the others (STATUS.md).

  isa         psm5 up to corerev 14, psm15 from 15 (b43-asm's %arch mapping)
  buses       address/command SPR and data SPR of each indirect register
              interface, named after the device behind it
  handshakes  SPR bits the ucode sets and waits on, answered at once
  tx_go       the SPR write that launches a prepared frame
  host_regs   IHR registers that answer host accesses on their own

A revision without a ucode of its own gets the profile of its generation,
marked as assumed.
"""

from dataclasses import dataclass, replace
import psm5
import psm15


@dataclass(frozen=True)
class BusCommand:
    """A command with all `read` bits set reads the register into the data
    SPR; one with all `write` bits set (and not a read) writes the data SPR
    to it."""
    read: int
    write: int
    addr_mask: int

    def command(self, cmd):
        if cmd & self.read == self.read:
            return "read"
        if cmd & self.write == self.write:
            return "write"
        return None

    def complete(self, cmd):
        return cmd & ~(self.read | self.write)


@dataclass(frozen=True)
class IndirectBus:
    name: str
    addr_spr: int
    data_spr: int
    command: BusCommand


@dataclass(frozen=True)
class SprFollow:
    """`ack` reads as set while `request` is set."""
    spr: int
    request: int
    ack: int

    def respond(self, value):
        return (value & ~self.ack) | (self.ack if value & self.request else 0)


@dataclass(frozen=True)
class SprSelfClear:
    """The hardware clears `bits` once the ucode has set them."""
    spr: int
    bits: int

    def respond(self, value):
        return value & ~self.bits


@dataclass(frozen=True)
class HostTaken:
    """Bits of a host write the hardware takes at once: never read back."""
    spr: int
    bits: int

    def after_write(self, value):
        return value & ~self.bits

    def after_read(self, value):
        return value


@dataclass(frozen=True)
class HostBusy:
    """Bits of a host write still set at the first read after it, then clear."""
    spr: int
    bits: int

    def after_write(self, value):
        return value

    def after_read(self, value):
        return value & ~self.bits


@dataclass(frozen=True)
class SprTrigger:
    spr: int
    bits: int


@dataclass(frozen=True)
class Core:
    corerev: int
    isa: object             # psm5 or psm15
    buses: tuple            # IndirectBus, the PHY's named "phy"
    handshakes: tuple
    tx_go: SprTrigger
    read_from: tuple        # corerevs whose ucode this profile was read from
    host_regs: tuple = ()
    assumed: bool = False


# OpenFWWF's sel_phy_reg/write_phy_reg: bit 12 reads, bit 13 writes.
CMD_D11 = BusCommand(read=0x1000, write=0x2000, addr_mask=0x0FFF)
# Bit 14 starts a transaction, bit 13 makes it a read.
CMD_GO = BusCommand(read=0x6000, write=0x4000, addr_mask=0x1FFF)

PHY_D11 = IndirectBus("phy", 0x018, 0x019, CMD_D11)
PHY_GO = IndirectBus("phy", 0x018, 0x019, CMD_GO)
# From corerev 24. That the radio is behind it is inferred (STATUS.md).
RADIO = IndirectBus("radio", 0x030, 0x031, CMD_D11)

# RXE_FIFOCTL1_SUSPEND, set before a MAC suspend.
RXE_SUSPEND = SprSelfClear(spr=0x004, bits=0x0004)
# brcmsmac's psm_corectlsts: the arch15 ucode sets bit 5, waits for bit 13.
CORECTL = SprFollow(spr=0x078, request=0x0020, ack=0x2000)

# Up to corerev 30: TX_COUNT bytes of template RAM to the serializer.
TX_TEMPLATE = SprTrigger(spr=0x0B4, bits=0x0826)
# From corerev 40: 0x8001 for a queued frame, 0x8000 for the beacon.
TX_SERIAL = SprTrigger(spr=0x320, bits=0x8000)

# TX FIFO setup on corerev 42, as the stock driver's bus capture shows it.
XMTFIFOCMD_BUSY = HostBusy(spr=0x0A0, bits=0x0001)
AGGFIFODATA0_GO = HostTaken(spr=0x098, bits=0x8000)

_PROFILES = (
    ((5, 14), Core(corerev=5, isa=psm5, buses=(PHY_D11,),
                   handshakes=(RXE_SUSPEND,), tx_go=TX_TEMPLATE,
                   read_from=(5, 11))),
    ((15, 39), Core(corerev=22, isa=psm15, buses=(PHY_D11, RADIO),
                    handshakes=(RXE_SUSPEND, CORECTL), tx_go=TX_TEMPLATE,
                    read_from=(16, 22, 24, 26, 29, 30))),
    ((40, None), Core(corerev=42, isa=psm15, buses=(PHY_GO, RADIO),
                      handshakes=(RXE_SUSPEND, CORECTL), tx_go=TX_SERIAL,
                      read_from=(40, 42),
                      host_regs=(XMTFIFOCMD_BUSY, AGGFIFODATA0_GO))),
)


def for_corerev(corerev):
    for (first, last), profile in _PROFILES:
        if first <= corerev and (last is None or corerev <= last):
            return replace(profile, corerev=corerev,
                           assumed=corerev not in profile.read_from)
    raise ValueError(f"corerev {corerev}: no profile")


def isa_of(corerev):
    return for_corerev(corerev).isa
