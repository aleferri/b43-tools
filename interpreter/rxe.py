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
"""The receive engine (RXE) of the virtual chip: a frame from the air into
shared memory, the conditions the ucode waits on, and the push of RX header
and frame into the host's FIFO (`fifo`).

The base is OpenFWWF's receive path (corerev 5; ucode5.asm's notes on
SPR_RXE_FIFOCTL1 and SPR_RXE_0x1a, here SPR_RXE_STATUS). Where the ucode of
the 6.30.163.46 driver (784.2) needed more, the addition is a model and its
reason is given with the PC that showed it; register numbers are spr.inc's.
Checked against brcmsmac's macstat counters (SHM from word 0x70) on every
ucode of that driver with initvals (11, 16_lp, 16..30 mimo, 40, 42) and on
OpenFWWF, and against real RX headers of corerev 30 and 42 (rxframe.py).

A frame is the bytes the PHY hands over, PLCP first (6 bytes, as b43's
b43_plcp_hdr6), then the MPDU with its FCS. From rx_frame(), in chip time:

  + plcp_us   COND_RX_PLCP, held until the EOI.
              SPR_RXE_STATUS bit 12; without it the ucode counts the frame in
              SHM 0x0AE and flushes it (ucode30_mimo 0x0795, ucode42 0x0806).
              Bit 13 until the last byte is in the RX buffer, COPY_LAG_US
              after the frame's end: the end condition comes with it still
              set, as on the RT-AC56U (spr.inc's SPR_RXE_0x1a). The LP ucode
              aborts a reception with bit 12 and not bit 13 after 14 us
              (ucode16_lp 0x0ED1), a hypothesis.
              SPR_RXE_ENCODING bit 13 for OFDM, and on arch15 bits 10..12 the
              frame format the ucode tells 0..3 apart by (ucode29_mimo
              0x07E5; 0 makes it CCK): 1 for OFDM.
              The PHY's six RX status words: SPR_RXE_PHYRXSTAT0..3 and SPR
              0x022/0x023 (spr.inc's SPR_RXE_0x44/0x46), which ucode42 copies
              to RX header words 2..7 and ucode30_mimo, 2..5.
              SPR 0x11F (spr.inc's SPR_TSF_0x3e) the TSF's low word, the RX
              header's mac_time (word 10).
              COND_RX_RAMATCH and COND_RX_BSSMATCH as given, while the frame
              lasts (before corerev 40).
  then        the bytes arrive at an even rate and reach the RX buffer
              COPY_LAG_US later; SPR_RXE_FRAMELEN counts the ones there (on
              corerev 30 up to 2 short of the frame: the RX header length
              the ucode takes from it at the push is 2 short of the PLCP's on
              the RT-AC56U; that the last 2 bytes are never counted is a
              hypothesis), and
              the first SPR_RXE_COPY_LEN of them are in shared memory from
              word SPR_RXE_COPY_OFFSET, from the next word with the HT and AC
              PHY (copy_lead()). The lag: the HT ucode clears the first two
              PLCP words of the copy at COND_RX_PLCP (ucode29_mimo 0x07F1)
              and reads the length from them once six bytes are there;
              ucode11 reads the header once FRAMELEN reaches 0x26 (0x064C).
              SPR 0x02A (spr.inc's SPR_RXE_0x54), if not zero:
              COND_RX_COMPLETE, held until its EOI, once FRAMELEN reaches it
              (ucode30_mimo 0x079D..0x07A8, 0x07CD; ucode42 0x080A..0x080E,
              0x0832). A hypothesis.
              SPR_RXE_STATUS bit 14 once all of the copy is there and the
              frame has ended.
  + air_us    COND_RX_COMPLETE, raised while the frame is in the RX buffer and
              latched until its EOI besides. COND_RX_FCS_GOOD, held until the
              EOI, if the FCS is good.

The PLCP and the end of a frame wake a napping PSM even when their
condition was still raised from the frame before; the conditions a frame
raised stay until their EOI after it. SPR 0x02B (spr.inc's SPR_RXE_0x56)
reads 0x8000: the ucode takes bit 15 clear as another frame behind the one
it finished (bits 0..11 its length, ucode30_mimo 0x07C4) and serves it
without a PLCP. Frames do not queue here.

The ucode's writes of SPR_RXE_FIFOCTL1:

  bit 0       push: the RX header, SPR_RXE_RXHDR_LEN words of shared memory
              from word SPR_RXE_RXHDR_OFFSET (OpenFWWF 10, ucode30_mimo 12,
              ucode42 14), two bytes of padding if bit 5 is set, then the
              frame, go to the FIFO and the frame ends; made before the
              frame's end, at its end. COND_RX_FIFOBUSY while the copy lasts:
              every ucode waits for it to rise and fall.
  bit 4       drop: the RX buffer is flushed and the frame ends (written
              after a frame not taken, and after a push).

From corerev 40 the ucode needs more:

  - SPR 0x241: the frame control, from which ucode42 takes type and
    subtype (0x0842).
  - command 1 in SPR 0x260, then a wait for bit 0 of SPR 0x271 (0x0848): an
    address lookup in b43-next's 64-row AMT (RCMTA space, b43_amt_write()),
    answered at once; condition 0x61 when the RA is the SELF row, 0x62 when
    the TA is a BSSID row, 0x63 when addr3 is one or the broadcast address.
    Hypotheses: ucode42 never tests 0x60 or 0x66 but 0x61..0x64, 0x67 and
    0x68; it keeps a group-addressed probe request only on 0x63 (0x09D8),
    as the routers' headers show it does with the wildcard BSSID, and tests
    0x62 for frames from the DS.
  - PHY register 0x143 bits 12..15, read at the PLCP (0x07FA): the 20 MHz
    sub-bands the frame came on, all four here (`subbands`). A hypothesis:
    ucode42 keeps a probe request only if they include the chanspec's
    control sub-band (SHM 0x50, 0x0A84).
  - SPR 0x263 bit 12 from the frame's end to the next PLCP: ucode42 waits
    for it before it takes COND_RX_COMPLETE (0x095F), and after the push.
  - SPR 0x290 bit 0: without it a push reaches no RX DMA ring (RT-AC56U);
    ucode42 sets it (0x0D03).
  - SPR 0x293 counts the PLCPs: ucode42 puts its bits 0..5 in bits 10..15
    of RX header word 9 (0x0A3F), bits that grow from frame to frame on the
    routers.

Not modelled: which FIFOCTL1 bits clear themselves, the timing, overflow,
bad PLCPs, frames back to back, the BSSID match before corerev 40
(COND_RX_BSSMATCH is held for every frame), conditions 0x64, 0x67 and
0x68, SPR 0x240/0x244/0x274 beyond the above, and the crypto engine (its
GO bit is in core.py).
"""

from dataclasses import dataclass, field

COND_RX_FCS_GOOD, COND_RX_BADPLCP, COND_RX_COMPLETE = 0x03, 0x05, 0x06
COND_RX_PLCP, COND_RX_FIFOBUSY = 0x0E, 0x0B
COND_RX_RAMATCH, COND_RX_BSSMATCH = 0x60, 0x66

SPR_RXE_COPY_OFFSET, SPR_RXE_COPY_LEN = 0x001, 0x002
SPR_RXE_FIFOCTL1 = 0x004
SPR_RXE_RXHDR_OFFSET, SPR_RXE_RXHDR_LEN = 0x007, 0x008
SPR_RXE_PHYRXSTAT0, SPR_RXE_PHYRXSTAT1 = 0x009, 0x00A
SPR_RXE_FRAMELEN, SPR_RXE_STATUS, SPR_RXE_ENCODING = 0x00C, 0x00D, 0x00E
SPR_RXE_PHYRXSTAT2, SPR_RXE_PHYRXSTAT3 = 0x01A, 0x01B
SPR_RXE_PHYRXSTAT4, SPR_RXE_PHYRXSTAT5 = 0x022, 0x023   # spr.inc's SPR_RXE_0x44/0x46
SPR_RXE_FC40 = 0x241
SPR_RXE_MARK = 0x02A            # spr.inc's SPR_RXE_0x54
SPR_RXE_NEXT = 0x02B            # spr.inc's SPR_RXE_0x56
NEXT_NONE = 0x8000
SPR_RXE_STAT40 = 0x263
SPR_RXE_COUNT40 = 0x293         # bits 10..15 of the RX header's word 9 on corerev 42
SPR_RXE_TSF = 0x11F             # spr.inc's SPR_TSF_0x3e: the RX header's mac_time
SPR_RXE_LOOKUP40, SPR_RXE_LOOKUP40_DONE, SPR_RXE_LOOKUP40_IDX = 0x260, 0x271, 0x274
AMT_ROWS, AMT_F_VALID, AMT_F_SELF, AMT_F_BSSID = 64, 0x8000, 0x0008, 0x0002
COND_RX_RAMATCH40, COND_RX_TABSS40, COND_RX_A3BSS40 = 0x61, 0x62, 0x63
PHY_RXSUBBANDS40 = 0x143        # AC-PHY register, bits 12..15
STAT40_DONE = 0x1000

FIFOCTL1_PUSH, FIFOCTL1_DROP, FIFOCTL1_PAD = 0x0001, 0x0010, 0x0020
SPR_RXE_DMA40, DMA40_ON = 0x290, 0x0001
FRAMELEN_SHORT = {30: 2}        # corerev: bytes SPR_RXE_FRAMELEN never counts
STATUS_COPIED, STATUS_PLCP, STATUS_DATA = 0x4000, 0x1000, 0x2000
ENCODING_OFDM = 0x2000
PLCP_LEN = 6
PHYTYPE_HT, PHYTYPE_AC = 0x7, 0xB   # PHY register 0 bits 8..11, b43.h's B43_PHYTYPE_*
FIFO_BYTES_PER_US = 400       # the push's copy to the FIFO, a model
COPY_LAG_US = 2                 # bytes reach shared memory this long after the PHY, a model


@dataclass
class RxFrame:
    data: bytes                 # PLCP, MPDU, FCS
    fcs_ok: bool = True
    ofdm: bool = True
    ra_match: bool = False
    bss_match: bool = False
    phyrxstat: tuple = (0, 0, 0, 0, 0, 0)
    plcp_us: float = 20.0
    air_us: float = None        # default: 6 Mb/s for the bytes after the PLCP
    ftype: int = None           # corerev 40+: SPR_RXE_ENCODING bits 10..12
    rcm: tuple = None           # corerev 40+: the RCM conditions, else from the AMT
    conds: tuple = ()           # more conditions the frame holds from its PLCP on
    subbands: int = 0xF         # corerev 40+: the 20 MHz sub-bands it came on


@dataclass
class Pushed:
    rxhdr: bytes
    pad: bool
    data: bytes
    clock: int
    frame: RxFrame = field(repr=False, default=None)


class RxEngine:
    def __init__(self, chip):
        self.chip = chip
        # Nothing behind the frame in the buffer: frames come one at a time.
        chip.m.spr[SPR_RXE_NEXT] = NEXT_NONE
        self.frame = None
        self.queue = []             # (clock, RxFrame) still to start
        self.t_plcp = self.t_end = None
        self.received = 0           # bytes the PHY handed over
        self.held = 0               # bytes in the RX buffer
        self.copied = 0             # bytes copied to shared memory
        self.levels = set()         # conditions held while the frame lasts
        self.complete = False
        self.marked = 0             # the SPR_RXE_MARK the frame's bytes reached last
        self.push_due = None        # (rxhdr, pad) of a push made before the end
        self.lookup_due = False      # a write of command 1 to SPR 0x260
        self.fifo_busy_until = None # clock at which the push's copy ends
        self.fifo = []              # Pushed
        self.dropped = []           # RxFrame
        self.log = []               # (clock, event)

    # ---- the air side ----
    def rx_frame(self, frame, at=None):
        """Queue a frame whose first bit reaches the antenna at clock `at`
        (now if None)."""
        self.queue.append((self.chip.clock if at is None else at, frame))
        self.queue.sort(key=lambda q: q[0])

    def receiving(self):
        """A frame is on the air: what the ucode polls may still change."""
        return self.frame is not None and \
            (not self.complete or self.held < len(self.frame.data) or not self.copy_done())

    def busy(self):
        return bool(self.queue) or self.frame is not None or self.fifo_busy_until is not None

    def next_clock(self):
        """The clock of the next change the air brings, or None."""
        c = self.chip
        due = [self.fifo_busy_until] if self.fifo_busy_until is not None else []
        if self.frame is not None:
            if self.held < len(self.frame.data) or not self.copy_done():
                return c.clock + 1
        elif self.queue:
            due.append(self.queue[0][0])
        return min(due) if due else None

    def _clock(self, us):
        return int(us * (self.chip.mhz or 80))

    def tick(self):
        """Bring the engine to the chip's clock."""
        c = self.chip
        if self.fifo_busy_until is not None and c.clock >= self.fifo_busy_until:
            self.fifo_busy_until = None
            self.levels.discard(COND_RX_FIFOBUSY)
        if self.frame is None and not self.queue:
            return
        if self.frame is None and self.queue and self.queue[0][0] <= c.clock:
            start, f = self.queue.pop(0)
            air = f.air_us if f.air_us is not None else \
                f.plcp_us + 4 * ((len(f.data) - PLCP_LEN) * 8 + 22 + 23) // 24
            self.frame, self.received, self.held, self.copied = f, 0, 0, 0
            self.t_plcp = start + self._clock(f.plcp_us)
            self.t_end = start + self._clock(air)
            self.levels = self.levels & {COND_RX_FIFOBUSY}
            self.complete, self.marked, self.push_due = False, 0, None
        f = self.frame
        if f is None or c.clock < self.t_plcp:
            return
        spr = c.m.spr
        if self.received == 0:
            spr[SPR_RXE_ENCODING] = (spr.get(SPR_RXE_ENCODING, 0) & ~ENCODING_OFDM) | \
                (ENCODING_OFDM if f.ofdm else 0)
            for n, v in zip((SPR_RXE_PHYRXSTAT0, SPR_RXE_PHYRXSTAT1,
                             SPR_RXE_PHYRXSTAT2, SPR_RXE_PHYRXSTAT3,
                             SPR_RXE_PHYRXSTAT4, SPR_RXE_PHYRXSTAT5), f.phyrxstat):
                spr[n] = v & 0xFFFF
            spr[SPR_RXE_STATUS] = spr.get(SPR_RXE_STATUS, 0) & ~STATUS_COPIED | \
                STATUS_PLCP | STATUS_DATA
            if SPR_RXE_STAT40 in spr:
                spr[SPR_RXE_STAT40] &= ~STAT40_DONE
            if f.ra_match:
                self.levels.add(COND_RX_RAMATCH)
            if f.bss_match:
                self.levels.add(COND_RX_BSSMATCH)
            self.levels |= set(f.conds)
            if c.hw.corerev >= 15:
                # Bits 10..12 the frame format, which the arch15 ucode
                # tells 0..3 apart by; 1 (OFDM) for OFDM here, a model.
                spr[SPR_RXE_ENCODING] = spr[SPR_RXE_ENCODING] & ~0x1C00 | \
                    ((f.ftype if f.ftype is not None else int(f.ofdm)) << 10)
            if c.hw.corerev >= 40:
                v = c.phy.read(PHY_RXSUBBANDS40)
                c.phy.observe(PHY_RXSUBBANDS40, v & 0x0FFF | (f.subbands & 0xF) << 12)
            if len(f.data) >= PLCP_LEN + 2 and c.hw.corerev >= 40:
                # The frame control word, where the corerev 42 ucode reads
                # type and subtype (bits 2..7), a model.
                spr[SPR_RXE_FC40] = f.data[PLCP_LEN] | f.data[PLCP_LEN + 1] << 8
            if c.mhz:
                spr[SPR_RXE_TSF] = c.tsf() & 0xFFFF
            if c.hw.corerev >= 40:
                spr[SPR_RXE_COUNT40] = (spr.get(SPR_RXE_COUNT40, 0) + 1) & 0xFFFF
            c.events.add(COND_RX_PLCP)
            c.napping = False       # a new frame wakes the PSM, raised before or not
            self.log.append((c.clock, "plcp"))
            self.received = PLCP_LEN
        want = self.arrived(c.clock)
        if want > self.received:
            self.received = want
        n = len(f.data)
        self.held = held = self.arrived(c.clock - self._clock(COPY_LAG_US))
        framelen = min(held, n - FRAMELEN_SHORT.get(c.hw.corerev, 0))
        if framelen != spr.get(SPR_RXE_FRAMELEN):
            spr[SPR_RXE_FRAMELEN] = framelen
        self.copy(held)
        if held == n and spr.get(SPR_RXE_STATUS, 0) & STATUS_DATA:
            spr[SPR_RXE_STATUS] &= ~STATUS_DATA
        mark = spr.get(SPR_RXE_MARK, 0)
        if mark != self.marked and framelen >= mark and not self.complete:
            self.marked = mark
            if mark:
                c.events.add(COND_RX_COMPLETE)
                c.napping = False
                self.log.append((c.clock, f"{mark} bytes"))
        if self.copy_done() and not spr.get(SPR_RXE_STATUS, 0) & STATUS_COPIED \
                and self.complete:
            spr[SPR_RXE_STATUS] = spr.get(SPR_RXE_STATUS, 0) | STATUS_COPIED
        if c.clock >= self.t_end and not self.complete:
            self.complete = True
            self.levels.add(COND_RX_COMPLETE)
            c.events.add(COND_RX_COMPLETE)
            c.napping = False
            if f.fcs_ok:
                c.events.add(COND_RX_FCS_GOOD)
            if self.copy_done():
                spr[SPR_RXE_STATUS] = spr.get(SPR_RXE_STATUS, 0) | STATUS_COPIED
            if c.hw.corerev >= 40:
                # The corerev 42 ucode waits for it before it takes the
                # frame's end (EOI of COND_RX_COMPLETE); when the hardware
                # sets it is not known: here, at the end.
                spr[SPR_RXE_STAT40] = spr.get(SPR_RXE_STAT40, 0) | STAT40_DONE
            self.log.append((c.clock, "complete"))
            if self.push_due:
                self.deliver(*self.push_due)

    def copy_lead(self):
        """Whether the frame starts one word into the copy: with the HT and
        the AC PHY the ucode reads the PLCP at SPR_RXE_COPY_OFFSET + 1 (its
        off1: corerev 26 0x5F5, 29 0x5FD, 40/42 0x78D), with the N PHY at
        SPR_RXE_COPY_OFFSET (corerev 16..25, 30: 0x70C on corerev 30)."""
        return (self.chip.phy.read(0) >> 8) & 0xF in (PHYTYPE_HT, PHYTYPE_AC)

    def arrived(self, clock):
        """The bytes of the frame the PHY has handed over by clock."""
        n = len(self.frame.data)
        if clock < self.t_plcp:
            return 0
        if clock >= self.t_end:
            return n
        span = max(1, self.t_end - self.t_plcp)
        return PLCP_LEN + (n - PLCP_LEN) * (clock - self.t_plcp) // span

    def copy_done(self):
        f = self.frame
        return f is not None and \
            self.copied >= min(len(f.data), self.chip.m.spr.get(SPR_RXE_COPY_LEN, 0))

    def copy(self, upto):
        """The bytes inside the copy window that arrived COPY_LAG_US ago, to
        shared memory."""
        c = self.chip
        spr, shm = c.m.spr, c.m.shm
        limit = min(upto, len(self.frame.data), spr.get(SPR_RXE_COPY_LEN, 0))
        base = spr.get(SPR_RXE_COPY_OFFSET, 0) + (1 if self.copy_lead() else 0)
        data = self.frame.data
        while self.copied < limit:
            i = self.copied & ~1
            end = min(i + 2, limit)
            shm[(base + i // 2) & (len(shm) - 1)] = int.from_bytes(data[i:end], "little")
            self.copied = end

    # ---- the ucode side ----
    def amt(self):
        """The address match table, rows (address, flags), as b43-next's
        b43_amt_write() fills the RCMTA object space."""
        rows = []
        cells = self.chip.obj_rcmta
        for i in range(AMT_ROWS):
            lo, hi = cells.get(2 * i, 0), cells.get(2 * i + 1, 0)
            rows.append(((lo | (hi & 0xFFFF) << 32).to_bytes(6, "little"), hi >> 16))
        return rows

    def lookup(self):
        """Command 1 in SPR 0x260 (corerev 40+): the ucode then waits for bit 0
        of SPR 0x271 and tests conditions 0x61..0x68. A hypothesis: an address
        lookup in the AMT, 0x61 the RA matching the SELF row (64-row table,
        6-bit indices as the ucode extracts from SPR 0x274). Answered at once."""
        c = self.chip
        spr = c.m.spr
        spr[SPR_RXE_LOOKUP40_DONE] = spr.get(SPR_RXE_LOOKUP40_DONE, 0) | 1
        f = self.frame
        if f is None:
            return
        if f.rcm is not None:
            self.levels |= set(f.rcm)
            return
        hdr = f.data[PLCP_LEN:]
        ra, ta, a3 = hdr[4:10], hdr[10:16], hdr[16:22]
        rows = self.amt()
        for i, (addr, flags) in enumerate(rows):
            if flags & AMT_F_VALID and flags & AMT_F_SELF and addr == ra:
                self.levels.add(COND_RX_RAMATCH40)
                spr[SPR_RXE_LOOKUP40_IDX] = spr.get(SPR_RXE_LOOKUP40_IDX, 0) & 0xFF | i << 8
                self.log.append((c.clock, f"RA in AMT row {i:#x}"))
                break
        bssids = {addr for addr, flags in rows
                  if flags & AMT_F_VALID and flags & AMT_F_BSSID == AMT_F_BSSID}
        if ta in bssids:
            self.levels.add(COND_RX_TABSS40)
        if len(a3) == 6 and (a3 in bssids or a3 == b"\xff" * 6):
            self.levels.add(COND_RX_A3BSS40)

    def service(self):
        """Act on the ucode's writes of SPR_RXE_FIFOCTL1 and of the lookup
        command."""
        c = self.chip
        spr = c.m.spr
        if c.hw.corerev >= 40:
            if self.lookup_due:
                self.lookup_due = False
                self.lookup()
        ctl = spr.get(SPR_RXE_FIFOCTL1, 0)
        if not ctl & (FIFOCTL1_PUSH | FIFOCTL1_DROP):
            return
        f = self.frame
        if ctl & FIFOCTL1_PUSH and f is not None:
            shm = c.m.shm
            base, length = spr.get(SPR_RXE_RXHDR_OFFSET, 0), spr.get(SPR_RXE_RXHDR_LEN, 0)
            words = [shm[(base + k) & (len(shm) - 1)] for k in range(length)]
            hdr = b"".join(w.to_bytes(2, "little") for w in words)
            if self.complete:
                self.deliver(hdr, bool(ctl & FIFOCTL1_PAD))
            else:
                self.push_due = (hdr, bool(ctl & FIFOCTL1_PAD))
                self.log.append((c.clock, "push before the end"))
        elif ctl & FIFOCTL1_DROP:
            if f is not None:
                self.dropped.append(f)
                self.log.append((c.clock, "drop"))
                self.end()
        c.m.store(("spr", SPR_RXE_FIFOCTL1), ctl & ~(FIFOCTL1_PUSH | FIFOCTL1_DROP))

    def deliver(self, hdr, pad):
        c = self.chip
        size = len(hdr) + len(self.frame.data)
        if c.hw.corerev >= 40 and not c.m.spr.get(SPR_RXE_DMA40, 0) & DMA40_ON:
            self.log.append((c.clock, "push, no RX DMA: SPR 0x290 bit 0 clear"))
        else:
            self.fifo.append(Pushed(hdr, pad, self.frame.data, c.clock, self.frame))
            self.log.append((c.clock, "push"))
        self.end()
        self.levels.add(COND_RX_FIFOBUSY)
        self.fifo_busy_until = c.clock + self._clock(1 + size / FIFO_BYTES_PER_US)

    def end(self):
        spr = self.chip.m.spr
        spr[SPR_RXE_STATUS] = spr.get(SPR_RXE_STATUS, 0) & \
            ~(STATUS_COPIED | STATUS_PLCP | STATUS_DATA)
        self.frame = None
        self.levels &= {COND_RX_FIFOBUSY}
