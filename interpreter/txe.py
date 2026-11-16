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
"""The transmit FIFOs of the virtual chip: frames the host queues, the TX
engine's moves of their bytes, and the TX status the ucode reports.

A model of the FIFO interface of the cores before corerev 40, read from
OpenFWWF and spr.inc (SPR_TXE0_FIFO_CMD, _FIFO_RDY, _SELECT, _TX_COUNT,
_TX_SHM_ADDR, SPR_TX_STATUS0..3); the corerev 40+ ucode drives another
engine and has none here. A queued frame is what the host's DMA puts in a
FIFO: the TX header in the firmware's format, then the frame.

  tx_frame()     appends a frame to FIFO n and sets bit n of
                 SPR_TXE0_FIFO_RDY while the FIFO is not empty, and bit 0 of
                 SPR_TXE0_STATUS while any is: the ucode picks a FIFO to send
                 from only with it set (ucode30_mimo 0x01CC, ucode11 0x017B),
                 a hypothesis.
  FIFO_CMD       bit 14 (COPY) makes FIFO (bits 8..10) the one the engine
                 reads, from the start of its first frame; bit 13 (TXDONE)
                 takes that frame off the FIFO.
  SELECT         source in bits 8..12 (FIFO n, 0x800 template RAM),
                 destination in bits 0..1: 1 copies TX_COUNT bytes (bit 2)
                 from the read position to shared memory at byte address
                 TX_SHM_ADDR, 3 skips them, 2 sends to the PHY (the chip's TX
                 engine conditions follow, as after its tx_go), none rewinds
                 to the frame's start (ucode30_mimo 0x019E, before it skips
                 the TX header up to the PLCP). COND_TX_BUSY while a copy
                 lasts.
  TX_STATUS0     a write with bit 0 set queues SPR_TX_STATUS0..3 as the two
                 words b43 reads from XMITSTAT_0/1: STATUS1 << 16 | STATUS0,
                 STATUS3 << 16 | STATUS2 (`status`).

With b43-next's host side (txframe.py: DFQ_VALID in MACCMD, EDCF and its
queue parameters) every 784.2 ucode before corerev 40 sends the frames
b43-next queued on a BCM43217 (corerev 30), captured with their TX headers,
and reports for each the first status word the router reported: all 39 on
ucode30_mimo, three of them alike on ucode11, 16_lp and 16..29 mimo. The
second word, the sequence number, comes from the ucode's own counter.

That COPY rewinds the read position and the copy timing are modelling
choices.
"""

from dataclasses import dataclass

SPR_TX_STATUS0, SPR_TXE0_STATUS = 0x08C, 0x087
TXE0_STATUS_FRAME = 0x0001
SPR_TXE0_FIFO_CMD, SPR_TXE0_FIFO_RDY = 0x0A0, 0x0A3
SPR_TXE0_SELECT, SPR_TXE0_TX_COUNT, SPR_TXE0_TX_SHM_ADDR = 0x0B4, 0x0B5, 0x0B6
FIFO_CMD_TXDONE, FIFO_CMD_COPY = 0x2000, 0x4000
SELECT_DST, SELECT_DST_SHM, SELECT_DST_PHY, SELECT_DST_DISCARD = 0x3, 0x1, 0x2, 0x3
SELECT_USE_TXCNT, SELECT_SRC_TRAM = 0x4, 0x0800
WATCHED = (SPR_TXE0_FIFO_CMD, SPR_TXE0_SELECT, SPR_TX_STATUS0)
COND_TX_BUSY = 0x2F             # OpenFWWF's COND_TX_BUSY
COPY_BYTES_PER_US = 400         # FIFO to shared memory, a model
FIFOS = 8


@dataclass
class TxStatus:
    words: tuple                # (XMITSTAT_0, XMITSTAT_1)
    clock: int


class TxEngine:
    def __init__(self, chip):
        self.chip = chip
        self.fifos = [[] for _ in range(FIFOS)]   # queued frames, bytes
        self.cur = None             # FIFO the engine reads
        self.pos = 0                # read position in its first frame
        self.busy_until = None      # clock at which a copy ends
        self.sent = []              # (clock, fifo, bytes) handed to the PHY
        self.status = []            # TxStatus
        self.log = []               # (clock, event)
        self.writes = []            # (spr, value) of the ucode, to serve

    # ---- the host side ----
    def tx_frame(self, fifo, data):
        self.fifos[fifo].append(bytes(data))
        self.publish()
        self.chip.napping = False
        self.log.append((self.chip.clock, f"queued {len(data)} bytes in FIFO {fifo}"))

    def pop_status(self):
        return self.status.pop(0) if self.status else None

    def publish(self):
        spr = self.chip.m.spr
        rdy = 0
        for n, q in enumerate(self.fifos):
            if q:
                rdy |= 1 << n
        spr[SPR_TXE0_FIFO_RDY] = spr.get(SPR_TXE0_FIFO_RDY, 0) & ~0xFF | rdy
        spr[SPR_TXE0_STATUS] = spr.get(SPR_TXE0_STATUS, 0) & ~TXE0_STATUS_FRAME | \
            (TXE0_STATUS_FRAME if rdy else 0)

    # ---- time ----
    def busy(self):
        return self.busy_until is not None

    def next_clock(self):
        return self.busy_until

    def tick(self):
        if self.busy_until is not None and self.chip.clock >= self.busy_until:
            self.busy_until = None

    def held(self):
        return {COND_TX_BUSY} if self.busy_until is not None else set()

    # ---- the ucode side ----
    def service_all(self):
        while self.writes:
            self.service(*self.writes.pop(0))

    def service(self, n, value):
        """A ucode write of a watched SPR."""
        c = self.chip
        if n == SPR_TXE0_FIFO_CMD:
            fifo = value >> 8 & 0x7
            if value & FIFO_CMD_COPY:
                self.cur, self.pos = fifo, 0
            if value & FIFO_CMD_TXDONE and self.fifos[fifo]:
                self.fifos[fifo].pop(0)
                if self.cur == fifo:
                    self.pos = 0
                self.publish()
                self.log.append((c.clock, f"FIFO {fifo} frame done"))
        elif n == SPR_TXE0_SELECT:
            self.select(value)
        elif n == SPR_TX_STATUS0 and value & 1:
            spr = c.m.spr
            w0 = (spr.get(SPR_TX_STATUS0 + 1, 0) << 16) | (value & 0xFFFF)
            w1 = (spr.get(SPR_TX_STATUS0 + 3, 0) << 16) | spr.get(SPR_TX_STATUS0 + 2, 0)
            self.status.append(TxStatus((w0, w1), c.clock))
            self.log.append((c.clock, f"TX status {w0:08x} {w1:08x}"))

    def select(self, value):
        c = self.chip
        dst = value & SELECT_DST
        if value & SELECT_SRC_TRAM:
            return
        fifo = value >> 8 & 0x7
        if not dst:
            # Source alone: the read position back to the frame's start
            # (ucode30_mimo writes it before skipping the TX header up to the
            # PLCP and copying PLCP and MAC header, 0x019E..0x01A5).
            self.cur, self.pos = fifo, 0
            return
        q = self.fifos[fifo]
        if not q:
            return
        if fifo != self.cur:
            self.cur, self.pos = fifo, 0
        frame = q[0]
        spr = c.m.spr
        count = spr.get(SPR_TXE0_TX_COUNT, 0) if value & SELECT_USE_TXCNT else len(frame) - self.pos
        chunk = frame[self.pos:self.pos + count]
        if dst == SELECT_DST_SHM:
            shm = c.m.shm
            addr = spr.get(SPR_TXE0_TX_SHM_ADDR, 0)
            for i in range(0, len(chunk), 2):
                w = chunk[i] | (chunk[i + 1] << 8 if i + 1 < len(chunk) else 0)
                shm[((addr + i) >> 1) & (len(shm) - 1)] = w
            self.busy_until = c.clock + c.rxe._clock(1 + len(chunk) / COPY_BYTES_PER_US)
            self.log.append((c.clock, f"FIFO {fifo} -> SHM {addr:#06x}, {len(chunk)} bytes"))
        elif dst == SELECT_DST_PHY:
            self.sent.append((c.clock, fifo, bytes(chunk)))
            if c.tx:
                _ifs, start, done = c.tx
                c.pending.append((c.steps + start, 0x21))   # COND_TX_POWER
                c.pending.append((c.steps + done, 0x22))    # COND_TX_DONE
            self.log.append((c.clock, f"FIFO {fifo} -> PHY, {len(chunk)} bytes"))
        else:
            self.log.append((c.clock, f"FIFO {fifo} skip {len(chunk)} bytes"))
        self.pos += len(chunk)
