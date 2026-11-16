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
    words: tuple                # (XMITSTAT_0, XMITSTAT_1); corerev 40+: XMITSTAT_0..3 twice
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


# ---- corerev 40 and later ----
# Names from Broadcom's d11.h (the corerev 40+ register block, as published
# in its GPL source drops): SPR n is MMIO 0x400 + 2n.
SPR_TXE40_FIFO = 0x0A4          # BMCCmd
BMC_UPDATE_BA = 0x0080          # BMCCmd_UpdateBA
BMC_CONSUME = 0x0100            # BMCCmd_Consume
BMC_AGGREGATE = 0x0200          # BMCCmd_Aggregate: the next frame handed over
BMC_GO = 0x8000                 # with UpdateBA/Consume (no d11.h name)
SPR_TXE40_MPDU_GO = 0x0B8       # PSM-only (a gap in the host map): 0x000a per MPDU
SPR_TXE40_CMD, SPR_TXE40_OFFSET, SPR_TXE40_COUNT, SPR_TXE40_SHM = 0x093, 0x094, 0x095, 0x097
TXE40_GO, TXE40_FROM_OFFSET, TXE40_SENDING = 0x8000, 0x0080, 0x0400
SPR_TXE40_DESCS = 0x0BC         # txe_status1; ucode42 sends only with it at 2 or more (0x0229)
# the aggregation queue manager
SPR_AQM_RCVDBA = 0x203          # AQMRcvdBA0..3: a block ack bitmap, 64 bits
SPR_AQM_BASSN, SPR_AQM_REFSN = 0x207, 0x208
SPR_AQM_UPDBA = 0x216           # AQMUpdBA0..3
SPR_AQM_ACKCNT, SPR_AQM_CONSCNT = 0x21A, 0x21B
SPR_TXE40_FIFO_RDY = 0x21C      # AQMFifoReady
SPR_TXE40_SEND, TXE40_SEND_FRAME = 0x320, 0x8001   # a queued frame (0x8000 alone: the beacon)
SPR_TXSTAT40_CMD, SPR_TXSTAT40_DATA = 0x08C, 0x08D
STAT40_NEXT = 0x0080            # the next package of a status entry
TOE_LEN = 4                     # b43_txhdr_ac's offload header
AC_TXH_LEN = 128                # b43_txhdr_ac, the offload header included
AIFS_US = 16 + 2 * 9            # SIFS and AIFSN 2 short slots; no backoff modelled
ACK_TIMEOUT_US = 16 + 9 + 25    # SIFS, a slot and the PHY's RX start delay
COND_TX_NOW, COND_RX_RSPTMOUT = 0x20, 0x23
WATCHED40 = (SPR_TXE40_FIFO, SPR_TXE40_CMD, SPR_TXSTAT40_CMD, SPR_TXSTAT40_DATA, SPR_TXE40_SEND,
             SPR_TXE40_MPDU_GO)


class TxEngine40(TxEngine):
    """The corerev 40+ engine as far as ucode42 drives it, a model of what it
    reads and writes for single frames and A-MPDUs:

    - BMCCmd (SPR 0x0A4) picks the FIFO in bits 3:0; Aggregate hands the next
      frame over to the aggregation queue manager. A write of SPR 0x093 with
      bit 15 copies SPR 0x095 bytes of a frame, from offset 0 or with bit 7
      from the byte offset in SPR 0x094, to shared memory at byte address
      SPR 0x097: of the next frame to hand over, or with bit 10 of the one
      handed over last. COND_TX_BUSY while it lasts.
    - AIFS after a hand-over COND_TX_NOW; the ucode starts each MPDU with
      SPR 0x320 = 0x8001 (vchip's TX_SERIAL raises TX_POWER and TX_DONE),
      for a VHT PPDU or an A-MPDU after a write of SPR 0x0B8, which clears
      when done (answered at once). A unicast frame with no PLCP received
      within the ACK timeout after its end: COND_RX_RSPTMOUT (0x23, counted in
      macstat rxrsptmout). The hardware's own retries and rate fallback are
      not modelled.
    - The response goes to the aggregation queue manager (aqm_update()), then
      the ucode writes the TX status to SPR 0x08C/0x08D (close_entry())."""

    def __init__(self, chip):
        super().__init__(chip)
        self.fetched = [0] * FIFOS  # frames of each FIFO handed over
        self.package = None         # the status package being written, 1 or 2
        self.slot = 0               # its next 16-bit slot
        self.halves = [None] * 16   # the status entry, two packages of 8 halves
        self.ack_due = None         # clock by which the sent frame's ACK must start
        self.ack_rx = 0             # RX engine log entries before that send
        self.marked = [set() for _ in range(FIFOS)]  # in-flight MPDUs a response acknowledged
        self.acked = 0              # the MPDUs the last block ack update acknowledged, a bitmap
        self.tried = 0              # the MPDUs in flight at the last consume

    def publish(self):
        spr = self.chip.m.spr
        rdy, waiting = 0, 0
        for n, q in enumerate(self.fifos):
            left = len(q) - self.fetched[n]
            if left > 0:
                rdy |= 1 << n
                waiting += left
        # the frames not yet handed over: after one is, the ucode's scheduler
        # would otherwise take the same frame again
        spr[SPR_TXE40_FIFO_RDY] = spr.get(SPR_TXE40_FIFO_RDY, 0) & ~0xFF | rdy
        # a hypothesis: the DMA descriptors waiting, two a frame (header, body)
        spr[SPR_TXE40_DESCS] = 2 * waiting

    def next_clock(self):
        due = [t for t in (self.busy_until, self.ack_due) if t is not None]
        return min(due) if due else None

    def tick(self):
        super().tick()
        c = self.chip
        if self.ack_due is not None and c.clock >= self.ack_due:
            self.ack_due = None
            if not any(e == "plcp" for _, e in c.rxe.log[self.ack_rx:]):
                c.events.add(COND_RX_RSPTMOUT)
                c.napping = False
                self.log.append((c.clock, "no ACK: response timeout"))

    def service(self, n, value):
        c = self.chip
        spr = c.m.spr
        if n == SPR_TXE40_SEND:
            q = self.fifos[self.cur] if self.cur is not None else None
            # from the end of the last MPDU of the PPDU
            if value == TXE40_SEND_FRAME and q \
                    and len(q[0]) > AC_TXH_LEN + 4 \
                    and not q[0][AC_TXH_LEN + 4] & 1 and c.mhz:
                done = c.tx[2] if c.tx else 0
                self.ack_due = c.clock + done + ACK_TIMEOUT_US * c.mhz
                self.ack_rx = len(c.rxe.log)
            return
        if n == SPR_TXE40_MPDU_GO:
            if value:
                spr[SPR_TXE40_MPDU_GO] = 0
            return
        if n == SPR_TXE40_FIFO:
            self.cur = value & 0xF
            if value & (BMC_UPDATE_BA | BMC_CONSUME):
                self.aqm_update(self.cur, value & BMC_CONSUME)
                spr[SPR_TXE40_FIFO] = value & ~(BMC_GO | BMC_UPDATE_BA | BMC_CONSUME)
            if value & BMC_AGGREGATE:
                spr[SPR_TXE40_FIFO] = spr.get(SPR_TXE40_FIFO, value) & ~BMC_AGGREGATE
                if self.fetched[self.cur] < len(self.fifos[self.cur]):
                    self.fetched[self.cur] += 1
                self.publish()
                self.log.append((c.clock, f"FIFO {self.cur} frame handed over"))
                if c.tx and c.mhz:
                    c.pending.append((c.steps + AIFS_US * c.mhz, COND_TX_NOW))
            return
        if n == SPR_TXSTAT40_CMD:
            # bit 7: the next package, written from 16-bit slot (bits 2:0)
            # down; without it slot 0 of the same package
            if value & STAT40_NEXT:
                self.package = (self.package or 0) + 1
            self.slot = value & 7
            return
        if n == SPR_TXSTAT40_DATA:
            if not self.package or self.package > 2 or self.slot < 0:
                return
            self.halves[8 * (self.package - 1) + self.slot] = value & 0xFFFF
            self.slot -= 1
            if self.package == 2 and self.slot < 0:
                self.close_entry()
            return
        if n != SPR_TXE40_CMD or not value & TXE40_GO:
            return
        q = self.fifos[self.cur] if self.cur is not None and self.cur < FIFOS else []
        if not q:
            return
        start = spr.get(SPR_TXE40_OFFSET, 0) if value & TXE40_FROM_OFFSET else 0
        count = spr.get(SPR_TXE40_COUNT, 0)
        k = self.fetched[self.cur] - 1 if value & TXE40_SENDING else self.fetched[self.cur]
        # the offload header is not in what the ucode copies: it reads the
        # chanspec 6 bytes in (0x02B3)
        frame = q[max(0, min(k, len(q) - 1))][TOE_LEN:]
        chunk = frame[start:start + count]
        shm = c.m.shm
        addr = spr.get(SPR_TXE40_SHM, 0)
        for i in range(0, len(chunk), 2):
            w = chunk[i] | (chunk[i + 1] << 8 if i + 1 < len(chunk) else 0)
            shm[((addr + i) >> 1) & (len(shm) - 1)] = w
        self.busy_until = c.clock + c.rxe._clock(1 + len(chunk) / COPY_BYTES_PER_US)
        self.log.append((c.clock, f"FIFO {self.cur} [{start}:+{count}] -> SHM {addr:#06x}, "
                                  f"command {value:#06x}"))

    def aqm_update(self, fifo, consume):
        """BMCCmd UpdateBA: mark the in-flight MPDUs the block ack bitmap in
        AQMRcvdBA0..3 acknowledges, bit i for sequence AQMBaSSN + i and MPDU k
        of the FIFO at AQMRefSN + k (ucode42 writes the bitmap of a block ack
        with its SSN and the first MPDU's, and for an ACK bit 0 with both 0);
        AQMAckCnt and AQMUpdBA0..3 report them. With Consume the marked MPDUs
        leave the FIFO and AQMConsCnt counts them (after a block ack or none,
        ucode42 consumes with every bit set). Only the MPDUs a response
        acknowledged go; without one, all in flight go."""
        c = self.chip
        spr = c.m.spr
        bitmap = 0
        for i in range(4):
            bitmap |= spr.get(SPR_AQM_RCVDBA + i, 0) << 16 * i
        ssn, ref = spr.get(SPR_AQM_BASSN, 0), spr.get(SPR_AQM_REFSN, 0)
        hit = {k for k in range(self.fetched[fifo])
               if (ref + k - ssn) & 0xFFF < 64 and bitmap >> ((ref + k - ssn) & 0xFFF) & 1}
        if not consume:
            self.marked[fifo] |= hit
            upd = sum(1 << k for k in hit)
            self.acked = upd
            spr[SPR_AQM_ACKCNT] = len(hit)
            for i in range(4):
                spr[SPR_AQM_UPDBA + i] = upd >> 16 * i & 0xFFFF
            return
        # after a response the MPDUs it acknowledged go, the rest stay for a
        # retry (the real statuses count only those in AQMConsCnt); with none
        # every MPDU in flight goes, as after the hardware's last retry
        gone = self.marked[fifo] or hit
        self.tried = self.fetched[fifo]
        n = 0
        for k in sorted(gone, reverse=True):
            if k < len(self.fifos[fifo]):
                self.fifos[fifo].pop(k)
                n += 1
        # the ones left are offered again, to go out in the next A-MPDU
        self.fetched[fifo] = 0
        self.marked[fifo] = set()
        spr[SPR_AQM_CONSCNT] = n
        self.publish()
        self.log.append((c.clock, f"FIFO {fifo}: {n} MPDU(s) consumed"))

    def close_entry(self):
        """A TX status entry: the two packages b43-next reads from
        XMITSTAT_0..3 (b43_txstatus_read_ac()). The ucode writes word 0
        (frame ID << 16 | status: the MPDU count from SPR 0x21B in bits 14:8,
        ACKED bit 15), word 1 (the sequence number) and the second package's
        last word; the attempt and acknowledged counts of word 2, the second
        package's first word and the block ack bitmap are the hardware's, here
        for one attempt with the acknowledged count of the last block ack
        update. 269 of the 297 real corerev 42 statuses (BCM4352) are of that
        form; the rest are the hardware's retries and fallback."""
        c = self.chip
        h = [x or 0 for x in self.halves]
        self.package, self.halves = None, [None] * 16
        w = [h[2 * i] | h[2 * i + 1] << 16 for i in range(8)]
        mpdus = w[0] >> 8 & 0x7F
        bitmap = self.acked if w[0] & 0x8000 else 0
        self.acked = 0
        w[2] = bin(bitmap).count("1") << 8 | max(self.tried, mpdus)
        self.tried = 0
        w[3] = 0
        w[4], w[5], w[6] = 1, bitmap & 0xFFFFFFFF, bitmap >> 32
        self.status.append(TxStatus(tuple(w), c.clock))
        self.log.append((c.clock, "TX status " + " ".join(f"{x:08x}" for x in w)))
