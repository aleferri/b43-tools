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
Run every ucode of a b43-fwcutter output directory through the interpreter.

For each ucodeNN[_phy].fw (placeholders of a few bytes skipped):

  decoder   with --dasm, the instructions against b43-dasm's listing:
            mnemonic, operands and jump targets. A blob packed in 7 bytes
            per instruction is handed to b43-dasm as 8-byte words.

  boot      lockstep.py with core.py's profile of corerev NN, the initvals
            of that corerev whose name prefix belongs to the PHY of the
            ucode's name, and PHY register 0 reading as that PHY type
            (revision 0): PSM_RUN, enable, then valid beacon templates.

  beacon    the same, then a beacon interval of 102400 us, the TSF running
            and the TX engine model on, for 393 ms.

Usage:
    fwmatrix.py FWDIR [--dasm PATH] [--only NAME ...]
"""

import argparse
import os
import re
import struct
import subprocess
import tempfile

import core
import lockstep
import psmlib
import vchip

# Initvals name prefix -> PHY type (b43.h's B43_PHYTYPE_*), and the prefixes
# that go with the PHY named in a ucode file name.
PHY_TYPE = {"n": 0x04, "lp": 0x05, "sslpn": 0x06, "ht": 0x07, "lcn": 0x08, "ac": 0x0B}
PREFIXES = {"mimo": ("n", "ht"), "lp": ("lp",), "sslpn": ("sslpn",), "lcn": ("lcn",),
            "": ("n", "ac")}

BOOT = """\
cpu0 MAC.MCTRL val=0x00000404
cpu0 MAC.MCTRL val=0x00000402
cpu0 MAC.MCTRL val=0x00000403
cpu0 MAC.MCMD val=0x00000003
"""
BEACON = """\
cpu0 MAC.MCTRL val=0x00000404
cpu0 MAC.MCTRL val=0x00000402
cpu0 MAC.MCTRL val=0x04000404
cpu0 MAC.MCTRL val=0x04020402
cpu0 REG.WR off=0x0188 val=0x00640000
cpu0 REG.WR off=0x018c val=0x00019000
cpu0 MAC.MCTRL val=0x04020403
cpu0 MAC.MCMD val=0x00000003
cpu0 WAIT us=0x00060000
"""


def ucodes(fwdir):
    for name in sorted(os.listdir(fwdir)):
        m = re.fullmatch(r"ucode(\d+)(?:_([a-z]+?))?(?:_nobt)?\.fw", name)
        if m and os.path.getsize(os.path.join(fwdir, name)) > psmlib.FW_HEADER.size + 8:
            yield name, int(m.group(1)), m.group(2) or ""


def initvals(fwdir, corerev, phy):
    """(PHY type, [initvals, bsinitvals]) for each set of this corerev."""
    for name in sorted(os.listdir(fwdir)):
        m = re.fullmatch(r"([a-z]+?)(\d+)initvals(\d+)\.fw", name)
        if m and int(m.group(3)) == corerev and m.group(1) in PREFIXES[phy]:
            bs = os.path.join(fwdir, name.replace("initvals", "bsinitvals"))
            sets = [os.path.join(fwdir, name)] + ([bs] if os.path.exists(bs) else [])
            yield name[:-3], PHY_TYPE[m.group(1)], sets


def dasm_mismatches(dasm, path, prog, isa):
    """Instructions where b43-dasm's listing and the decoder disagree."""
    words = psmlib.read_codewords(path, isa.TOP_BIT)
    with tempfile.TemporaryDirectory() as tmp:
        blob, listing = os.path.join(tmp, "u.fw"), os.path.join(tmp, "u.asm")
        with open(blob, "wb") as f:
            f.write(psmlib.FW_HEADER.pack(ord('u'), psmlib.FW_HDR_VER, 8 * len(words)))
            for w in words:
                f.write(struct.pack(">II", w & 0xFFFFFFFF, w >> 32))
        arch = isa.NAME.removeprefix("arch")
        subprocess.run([dasm, blob, listing, "-a", arch, "-f", "b43", "--paddr"], check=True)
        text = open(listing).read()

    labels, insns, pending = {}, [], []
    for line in text.splitlines():
        m = re.match(r"^(L\d+):", line)
        if m:
            pending.append(m.group(1))
            continue
        m = re.match(r"^/\* ([0-9A-F]{4}) \*/\t(\S+)\t?(.*)$", line)
        if m:
            addr = int(m.group(1), 16)
            labels.update((l, addr) for l in pending)
            pending = []
            ops = [o.strip() for o in re.split(r",(?![^\[]*\])", m.group(3))] if m.group(3) else []
            insns.append((addr, m.group(2), ops))
    bad = 0
    for addr, mnem, ops in insns:
        if not mnem.startswith("@") and (mnem, [labels.get(o, o) for o in ops]) != \
                (prog[addr].mnem, operands(prog[addr])):
            bad += 1
    return bad + abs(len(insns) - len(prog))


def operands(ins):
    """An instruction's operands as b43-dasm prints them."""
    e, lab = ins.extra, ins.extra.get("label")
    if ins.mnem in ("srx", "orx"):
        return [str(e["a"]), str(e["b"]), repr(ins.op0), repr(ins.op1), repr(ins.op2)]
    if ins.mnem in ("jzx", "jnzx"):
        return [str(e["a"]), str(e["b"]), repr(ins.op0), repr(ins.op1), lab]
    if ins.mnem in ("jext", "jnext"):
        return [f"0x{e['imm']:02X}", lab]
    if ins.mnem == "call":
        return [f"lr{e['lr']}", lab]
    if ins.mnem == "ret":
        return [f"lr{e['lr_store']}", f"lr{e['lr_load']}"]
    if ins.mnem == "calls":
        return [lab]
    if ins.mnem in ("rets", "nap"):
        return []
    if ins.mnem.startswith("tkip"):
        return [repr(ins.op0), repr(ins.op2)]
    if lab is not None:
        return [repr(ins.op0), repr(ins.op1), lab]
    return [repr(ins.op0), repr(ins.op1), repr(ins.op2)]


def run(prog, hw, sets, phy_type, ops, **kw):
    inits = [e for p in sets for e in vchip.load_inits(p)]
    ls = lockstep.Lockstep(prog, 100, 300000, inits, hw=hw, phy_ver=phy_type << 8, **kw)
    with tempfile.NamedTemporaryFile("w", suffix=".ops") as f:
        f.write(ops)
        f.flush()
        ls.feed(f.name)
    return ls


def boot_summary(ls):
    chip, rec = ls.chip, ls.rec
    m = chip.m
    irqs = " ".join(f"#{op}:{bits:#x}" for op, _d, _o, _spr, bits in rec.irqs)
    buses = " ".join(f"{name}:{sum(n for (b, *_), n in rec.bus_access.items() if b == name)}"
                     for name in [bus.name for bus in chip.hw.buses])
    idle = " ".join(f"#{op}:{'-' if pc is None else f'{pc:#06x}'}"
                    for op, _d, _s, pc in ls.reactions)
    return (f"UCODEREV {m.shm[0]:#06x}.{m.shm[1]}  initvals {chip.inits_state}  "
            f"bus accesses {buses}  steps {chip.steps}  idle at [{idle}]  IRQs [{irqs}]"
            + (f"  stopped: {chip.stopped}" if chip.stopped else ""))


def beacon_summary(ls):
    chip, rec = ls.chip, ls.rec
    return (f"TBTTs {len(rec.tbtt_log)}  TX conditions {len(rec.tx_log)}  "
            f"IRQ bits {sorted({bits for *_x, bits in rec.irqs})}  final pc {chip.m.pc:#06x}"
            + (f"  stopped: {chip.stopped}" if chip.stopped else ""))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("fwdir")
    ap.add_argument("--dasm", help="b43-dasm, to check the decoder against")
    ap.add_argument("--only", nargs="*")
    args = ap.parse_args()

    for name, corerev, phy in ucodes(args.fwdir):
        if args.only and name[:-3] not in args.only:
            continue
        path = os.path.join(args.fwdir, name)
        hw = core.for_corerev(corerev)
        prog = hw.isa.load_program(path)
        payload = os.path.getsize(path) - psmlib.FW_HEADER.size
        packing = 8 if payload == 8 * len(prog) else 7
        print(f"{name}: corerev {corerev}, {hw.isa.NAME}, {len(prog)} instructions, "
              f"{packing}-byte packing, profile from corerev "
              f"{','.join(map(str, hw.read_from))}{' (assumed)' if hw.assumed else ''}")
        if args.dasm:
            print(f"  decoder vs b43-dasm: {dasm_mismatches(args.dasm, path, prog, hw.isa)} mismatches")
        sets = list(initvals(args.fwdir, corerev, phy))
        if not sets:
            print("  no initvals for this ucode")
        for iv, phy_type, files in sets:
            ls = run(prog, hw, files, phy_type, BOOT, mhz=80)
            print(f"  {iv} (PHY type {phy_type:#x}) boot:   {boot_summary(ls)}")
            ls = run(prog, hw, files, phy_type, BEACON, tx=(20, 10, 30), mhz=80)
            print(f"  {iv} (PHY type {phy_type:#x}) beacon: {beacon_summary(ls)}")


if __name__ == "__main__":
    main()
