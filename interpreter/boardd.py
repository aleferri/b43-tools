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
"""Serve a board (board.py) to QEMU's bcm-board device over a UNIX socket.

The device forwards BAR0, the vendor part of the configuration space and the
passing of time; one request per line, one reply per request, numbers in
hex, every reply with the level of the board's interrupt line:

    W <off> <val> <bits>    BAR0 write              -> K <irq>
    R <off> <bits>          BAR0 read               -> V <val> <irq>
    P <off> <val> <bits>    configuration write     -> K <irq>
    p <off> <bits>          configuration read      -> V <val> <irq>
    T <us>                  time passes             -> K <irq>
    Q                       end                     -> K <irq>

No core is modelled yet: the reads are a capture's, replayed (replay.py).
--host-core is the backplane address of the host core behind BAR0 0x2000.

    boardd.py --socket PATH --trace MMIOTRACE [--host-core ADDR]
              [--bar-size N] [--pcie-gen 1|2] [--report FILE]
"""

import argparse
import os
import socket
import sys

import board
import replay


def serve(b, rx, tx):
    for line in rx:
        req = line.split()
        if not req:
            continue
        op, args = req[0], [int(x, 16) for x in req[1:]]
        if op == "R":
            val = b.mmio_read(*args)
        elif op == "W":
            b.mmio_write(*args)
        elif op == "p":
            val = b.config_read(*args)
        elif op == "P":
            b.config_write(*args)
        elif op == "T":
            b.advance(*args)
        elif op != "Q":
            raise SystemExit(f"boardd: bad request {line!r}")
        irq = int(b.irq())
        tx.write(f"V {val:x} {irq:x}\n" if op in "Rp" else f"K {irq:x}\n")
        tx.flush()
        if op == "Q":
            return


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--socket", required=True)
    ap.add_argument("--trace", required=True, help="the board's mmiotrace")
    ap.add_argument("--host-core", type=lambda x: int(x, 0))
    ap.add_argument("--bar-size", type=lambda x: int(x, 0), default=0x4000)
    ap.add_argument("--pcie-gen", type=int, choices=(1, 2), default=1)
    ap.add_argument("--report")
    args = ap.parse_args()

    fallback = replay.Replay(replay.mmiotrace(args.trace, args.bar_size))
    b = board.Board(fallback, args.host_core, args.pcie_gen)

    if os.path.exists(args.socket):
        os.unlink(args.socket)
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(args.socket)
    srv.listen(1)
    conn, _ = srv.accept()
    with conn.makefile("r") as rx, conn.makefile("w") as tx:
        serve(b, rx, tx)
    out = open(args.report, "w") if args.report else sys.stdout
    fallback.report(out)
    b.report(out)


if __name__ == "__main__":
    main()
