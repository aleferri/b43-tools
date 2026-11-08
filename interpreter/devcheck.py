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
Replay the PHY and radio accesses of a capture on the chip's device models
and check each read against the captured value.

A read is a match, a miss (the model predicts another value) or unknown (the
model has no value for the register). Misses are listed by register, worst
first. Time does not pass in the replay: the captures' millisecond stamps
cannot resolve what devices do in microseconds.

    devcheck.py CAPTURE [--phy-type T] [--radio-id I] [--device phy|radio] [--top N]
"""

import argparse
import collections
import re

import devices
import vchip

OP = re.compile(r"\b(PHY|RAD)\.(RD|WR)\s+addr=0x([0-9a-fA-F]+)\s+val=0x([0-9a-fA-F]+)")
DEVICE = {"PHY": "phy", "RAD": "radio"}


def replay(path, devices):
    """Per device: Counter of (addr, outcome), and the misses as
    (addr, predicted, captured) in order."""
    outcome = {name: collections.Counter() for name in devices}
    misses = {name: [] for name in devices}
    for line in open(path):
        m = OP.search(line)
        if not m:
            continue
        name = DEVICE[m.group(1)]
        dev = devices[name]
        addr, val = int(m.group(3), 16), int(m.group(4), 16) & 0xFFFF
        if m.group(2) == "WR":
            dev.write(addr, val)
            continue
        if not dev.known(addr):
            outcome[name][(addr, "unknown")] += 1
        else:
            got = dev.read(addr)
            if got == val:
                outcome[name][(addr, "match")] += 1
            else:
                outcome[name][(addr, "miss")] += 1
                misses[name].append((addr, got, val))
        dev.observe(addr, val)
    return outcome, misses


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture")
    ap.add_argument("--phy-type", type=lambda x: int(x, 0),
                    help="PHY type for the PHY model (PHY_VER bits 11:8); none: a register file")
    ap.add_argument("--radio-id", type=lambda x: int(x, 0),
                    help="radio id for the radio model; none: a register file")
    ap.add_argument("--device", choices=("phy", "radio"))
    ap.add_argument("--top", type=int, default=15)
    args = ap.parse_args()

    phy = devices.phy_device(args.phy_type) if args.phy_type is not None else vchip.RegisterFile("phy")
    radio = devices.radio_device(args.radio_id) if args.radio_id is not None else vchip.RegisterFile("radio")
    devs = {"phy": phy, "radio": radio}
    outcome, misses = replay(args.capture, devs)
    for name in devs:
        if args.device and name != args.device:
            continue
        total = collections.Counter()
        per_reg = collections.Counter()
        for (addr, kind), n in outcome[name].items():
            total[kind] += n
            if kind == "miss":
                per_reg[addr] += n
        reads = sum(total.values())
        print(f"{name}: {reads} reads, {total['match']} match, {total['miss']} miss, "
              f"{total['unknown']} unknown")
        for addr, n in per_reg.most_common(args.top):
            ex = [(p, c) for a, p, c in misses[name] if a == addr][:3]
            print(f"  {addr:#06x}: {n} miss  " +
                  " ".join(f"{p:#06x}!={c:#06x}" for p, c in ex))


if __name__ == "__main__":
    main()
