#!/usr/bin/env python3
"""Emits the same random program for b43-asm and b43-masm.

Usage: diffgen.py ARCH SEED COUNT OUT_B43ASM OUT_MASM
"""
import random
import sys

arch, seed, count = int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3])
rnd = random.Random(seed)
imm_bits = 10 if arch == 5 else 11
spr_max = 0x1FF if arch == 5 else 0x3FF
mem_max = 0x7FF if arch == 5 else 0xFFF
off_max = 0x3F if arch == 5 else 0x7F


def imm():
    lim = 1 << (imm_bits - 1)
    v = rnd.randrange(-lim, lim)
    return str(v) if rnd.random() < 0.5 or v < 0 else hex(v)


def reg():
    return "r%d" % rnd.randrange(64)


def spr():
    return "spr%03x" % rnd.randrange(spr_max + 1)


def mem():
    if rnd.random() < 0.5:
        return "[0x%x]" % rnd.randrange(mem_max + 1)
    return "[0x%x,off%d]" % (rnd.randrange(off_max + 1), rnd.randrange(6))


def src(allow_spr=True, allow_mem=True):
    kinds = [reg, imm] + ([spr] if allow_spr else []) + ([mem] if allow_mem else [])
    return rnd.choice(kinds)()


def srcs():
    a = src()
    b = src(allow_spr=not a.startswith("spr"), allow_mem=not a.startswith("["))
    return a, b


def dst():
    return rnd.choice([reg, spr, mem])()


labels = ["L%d" % i for i in range(max(4, count // 20))]
alu = ["add", "add.", "addc", "addc.", "sub", "sub.", "subc", "subc.", "sra",
       "or", "and", "xor", "sr", "sl", "rl", "rr", "nand", "mul"]
jcmp = ["js", "jns", "je", "jne", "jls", "jges", "jgs", "jles", "jdn", "jdpz",
        "jdp", "jdnz", "jl", "jge", "jg", "jle"]


def insn():
    k = rnd.random()
    tgt = rnd.choice(labels)
    if k < 0.30:
        a, b = srcs()
        return "%s %s, %s, %s" % (rnd.choice(alu), a, b, dst())
    if k < 0.38:
        a, b = srcs()
        return "%s %d, %d, %s, %s, %s" % (rnd.choice(["srx", "orx"]), rnd.randrange(16),
                                         rnd.randrange(16), a, b, dst())
    if k < 0.50:
        v = rnd.choice([imm(), hex(rnd.randrange(0x10000)), reg(), spr(), mem()])
        return "mov %s, %s" % (v, dst())
    if k < 0.62:
        a, b = srcs()
        return "%s %s, %s, %s" % (rnd.choice(jcmp), a, b, tgt)
    if k < 0.70:
        if rnd.random() < 0.5:
            lo, w = rnd.randrange(16), rnd.randrange(1, 17)
            w = min(w, 16 - lo)
            m = hex(((1 << w) - 1) << lo)
            return "%s %s, %s, %s" % (rnd.choice(["jand", "jnand"]), reg(), m, tgt)
        a, b = srcs()
        return "%s %s, %s, %s" % (rnd.choice(["jand", "jnand"]), a, b, tgt)
    if k < 0.78:
        a, b = srcs()
        return "%s %d, %d, %s, %s, %s" % (rnd.choice(["jzx", "jnzx"]), rnd.randrange(16),
                                         rnd.randrange(16), a, b, tgt)
    if k < 0.84:
        return "%s 0x%x, %s" % (rnd.choice(["jext", "jnext"]), rnd.randrange(256), tgt)
    if k < 0.87:
        return "jmp %s" % tgt
    if k < 0.91:
        if arch == 5:
            return rnd.choice(["call lr%d, %s" % (rnd.randrange(4), tgt),
                               "ret lr%d, lr%d" % (rnd.randrange(4), rnd.randrange(4))])
        return rnd.choice(["calls %s" % tgt, "rets"])
    if k < 0.95:
        return "%s %s, %s" % (rnd.choice(["tkiph", "tkiphs", "tkipl", "tkipls"]),
                             src(), dst())
    if k < 0.97:
        return "nap"
    return "RAW %x %x %x %s" % (rnd.randrange(0x1000), rnd.randrange(1 << (12 if arch == 5 else 13)),
                               rnd.randrange(1 << (12 if arch == 5 else 13)), reg())


lines = []
prev_jump = False
for i in range(count):
    if i % 20 == 0:
        lines.append("%s:" % labels[i // 20])
    s = insn()
    if prev_jump and (s.startswith("ret") or s == "rets"):
        s = "nap"
    prev_jump = s.split()[0].startswith("j")
    lines.append(s)
while len(lines) and labels[-1] + ":" not in lines:
    labels.pop()

with open(sys.argv[4], "w") as old, open(sys.argv[5], "w") as new:
    old.write("%%arch %d\n" % arch)
    new.write(".arch %d\n" % arch)
    for l in lines:
        if l.startswith("RAW "):
            _, op, a, b, d = l.split()
            old.write("\t@%s @%s,@%s,%s\n" % (op, a, b, d))
            new.write("\tINSN 0x%s, RAW(0x%s), RAW(0x%s), %s\n" % (op, a, b, d))
        else:
            old.write("\t%s\n" % l if not l.endswith(":") else l + "\n")
            new.write("\t%s\n" % l if not l.endswith(":") else l + "\n")
