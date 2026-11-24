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
              Bits 12 and 13 until the last byte is in the RX buffer,
              COPY_LAG_US after the frame's end: the end condition comes with
              them still set, as on the RT-AC56U (spr.inc's SPR_RXE_0x1a),
              where both clear together in all 31 frame ends traced. The LP
              ucode aborts a reception with bit 12 and not bit 13 after 14 us
              (ucode16_lp 0x0ED1, ucode11 0x0E12), a hypothesis; frames do
              not pass through that state at their end.
              SPR_RXE_ENCODING bit 13 for OFDM, and on arch15 bits 10..12 the
              frame format the ucode tells 0..3 apart by (ucode29_mimo
              0x07E5; 0 makes it CCK): 1 for OFDM.
              The PHY's six RX status words: SPR_RXE_PHYRXSTAT0..3 and SPR
              0x022/0x023 (spr.inc's SPR_RXE_0x44/0x46), which ucode42 copies
              to RX header words 2..7 and ucode30_mimo, 2..5. On corerev 40+
              an A-MPDU's come at the end of its last MPDU; until then the
              words are the reception's before (the RT-AC56U's BCM4352: every
              member but the last had the header words 2..7 of a failed
              reception before it, under the open ucode, and 784.2 marks
              only the last one's RX status valid, MAC status bit 24).
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
              frame's last byte is in the RX buffer: on the RT-AC56U
              (corerev 42) FRAMELEN reads the whole frame wherever bit 14 is
              first seen set. Single frames wait for it before the push,
              where ucode42 puts FRAMELEN in the RX header (0x0968, 0x0D08).
              It stays past the push until the next PLCP, as there.
  + air_us    COND_RX_COMPLETE, raised while the frame is in the RX buffer and
              latched until its EOI besides. COND_RX_FCS_GOOD, held until the
              EOI, if the FCS is good.

The PLCP and the end of a frame wake a napping PSM even when their
condition was still raised from the frame before; the conditions a frame
raised stay until their EOI after it. SPR 0x02B (spr.inc's SPR_RXE_0x56)
reads 0x8000: the ucode takes bit 15 clear as another frame behind the one
it finished (bits 0..11 its length, ucode30_mimo 0x07C4) and serves it
without a PLCP. Single frames do not queue here.

An A-MPDU is a frame with MPDUs `behind` it (opt-in). SPR 0x02B then holds
the length of the MPDU the RXE parsed from the delimiter last: from a
member's start its own (MPDU and FCS), which ucode30_mimo makes the mark
(SPR 0x02A) of each member on the RT-AC56U, and from its end the next
member's, or 0x8000 after the last. The mark counts the member's MPDU
bytes, past its PLCP, and a member with others behind it ends when its
last byte is in the RX buffer, with no end on the air: before reading
FRAMELEN for such a member's RX header ucode30_mimo waits only for
COND_RX_COMPLETE (0x0919, 0x0924, 0x0A04), as ucode42 does (0x0967), so
on the chip neither can come before; a hypothesis. The next member streams
once the ucode has pushed or dropped the one before, with no PLCP event
and six zero bytes in place of the PLCP, as ucode30_mimo pushes members
on the chip. On the chip it streams while the one before is still being
pushed: the order here is a model.

The ucode's writes of SPR_RXE_FIFOCTL1:

  bit 0       push: the RX header, SPR_RXE_RXHDR_LEN words of shared memory
              from word SPR_RXE_RXHDR_OFFSET (OpenFWWF 10, ucode30_mimo 12,
              ucode42 14), two bytes of padding if bit 5 is set, then the
              frame, go to the FIFO and the frame ends; made before the
              frame's end, at its end. COND_RX_FIFOBUSY while the copy lasts:
              every ucode waits for it to rise and fall.
  bit 4       drop: the RX buffer is flushed and the frame ends (written
              after a frame not taken, and after a push). FRAMELEN goes on
              counting it to its last byte, past the RX buffer's cut, until
              a PLCP: on the RT-AC56U one dropped at 0x86 bytes reaches
              0x48C. Not an A-MPDU member with others behind it, after which
              the next starts at once (below).

A push needs bit 0 set by the write, as it reads back latched from corerev
40 on. What a ucode write leaves in SPR_RXE_FIFOCTL1, before corerev 40 the
write without bits 0 and 4; from corerev 40, as the RT-AC56U (corerev 42)
reads it back, at idle 1 us after a write of each bit and while frames
arrive:

  bit 11            always set
  bits 4, 6, 7, 13  never read back
  bits 2, 10        once written, until the end on the air of the frame
                    being received, dropped or not, whatever is written
                    meanwhile (0x0904 after 0x0110, 0x0804 and 0x0C22 to its
                    last byte; clear at once at idle); bit 2 is the RX
                    suspend every ucode sets before a MAC suspend and waits on
  the others        as written: bit 0 after a push until a write clears it
                    (0x0801 up to the next PLCP); bits 3, 14 and 15 are not
                    measured

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
  - SPR 0x240, cleared at the PLCP, once the MAC header of a data or
    management frame is in the RX buffer: bit 0, bits 1..6 the header's
    length in bytes, bit 8 if it is 2 bytes short of a multiple of 4.
    ucode42 waits for COND_RX_COMPLETE or bit 0 (0x083A), takes the crypto
    offset from bits 1..6 plus the PLCP (0x083D) and the RX padding from
    bit 7 XOR bit 8 (0x0844..0x0847). On the RT-AC56U what follows bit 0
    (FIFOCTL1 bits 1 and 5, SPR_BRC 0x0142, WEP_CTL) is there ~38 bytes
    into a frame, QoS data is pushed padded and never counted in SHM 0x98,
    where ucode42 counts unicast frames with bit 7 (0x09FB): bit 7 stays
    clear here. Bits 1..6 and 8 are a hypothesis; for control frames
    nothing is set.
  - SPR 0x263 bit 12, set throughout on the RT-AC56U, during frames and
    between them: ucode42 waits for it before it takes COND_RX_COMPLETE
    (0x095F), and after the push.
  - PHY 0x143 on the RT-AC56U's BCM4352 (5 GHz ch 36, HT20) reads 0x2C26 at
    an HT frame's PLCP and 0x2C25 at an OFDM frame's (a test ucode reading
    it at each PLCP): with `phy143_chip` the model gives those values.
  - FIFOCTL1 bit 6 written while a frame streams: on the BCM4352 the RXE
    drops what it holds past the PLCP and restarts the copy there. With the
    open ucode's second step (0x20C2 at FRAMELEN 38) a dropped frame's SHM
    copy began at the LLC ethertype, FRAMELEN ended 32 bytes short and every
    such frame failed the FCS check; with 0x2082 (bit 6 clear) they are
    received; a preceding 0x20C0 does not prevent it. With `restart40` the
    model drops those bytes from the frame, FRAMELEN and the SHM copy, and
    clears the FCS verdict.
  - The mark (SPR 0x02A) written equal to FRAMELEN on the RT-AC56U's
    BCM4352: no COND_RX_COMPLETE follows. A frame the RXE holds at 6 bytes
    when the ucode writes the mark 6 at its PLCP stays held (pdtrace: 3.6 ms
    at FRAMELEN 6 until the next PLCP), and the open ucode left a third of a
    station's 6 Mb/s frames unanswered that way until it went on to the
    mark's work whenever FRAMELEN had reached the mark already. With
    `mark40_equal` the model raises it only for a count that passes the mark
    after the write.
  - A streamed frame pushed after the response's RX suspend pulse (FIFOCTL1
    0x0004, 0), while the response is on the air: on the BCM4352 (RT-AC56U)
    the next frame is then held. Of the full-size 6 Mb/s frames after the
    open ucode's ACK 30 of 36 (WPA2) and 26 of 31 (open) stayed at FRAMELEN
    2 for 1.6-1.7 ms of their 2.0, SPR 0x00D at 0xFA80 (a good frame 0x3A80)
    and 0x00E at 0x4400 (0x0400), came up to 506 or 522 bytes at once, failed
    the FCS and got no answer. 784.2, which pushes before its pulse, had 16
    of 17 received; a frame after a drop (FIFOCTL1 0x0014, 0x0010, 0) is
    received, and that reset at the ACK's TX_DONE left no hold (five runs).
    With `push_hold` the model holds the next frame until a drop's reset. The
    trigger is inferred from the order of the two writes, not separated from
    the push overlapping the transmission; the hold's end and the bytes kept
    are those seen for 1536-byte frames; the status words are not modelled.
  - SPR_RXE_RXHDR_OFFSET after a push: the chip takes the RX header out of
    shared memory and the offset moves on by the words taken (784.2's
    ucode30_mimo on the BCM43217: 0x0766 -> 0x0772 after every push); a
    ucode that waits for it (the open ucode, bounded) waited out its bound
    here. With `hdr_offset_moves` the push moves it at once.
  - SPR 0x02B in an A-MPDU on the RT-AC56U's BCM4352: an MPDU's length
    shows at the PPDU's mark (FRAMELEN 6) and at each member boundary, and
    two reads back to back give the same value; once the ucode has written
    the mark it reads 0x8000 to the member's end (784.2 sets the mark from
    it and the host then samples 0x8000 inside every member; a ucode that
    wrote mark 0 and FIFOCTL1 0x20C0 after reading it read 0x8000 at FRAMELEN
    38); a mark written at the PLCP, before any of the MPDU's bytes, leaves
    it (our gate read the length after the PLCP's mark 6). Which write
    clears it, the mark's or FIFOCTL1's, is not separated; with
    `next40_mark` a mark written once FRAMELEN is past the PLCP makes the
    model read 0x8000.
  - A VHT PPDU is always an A-MPDU, also with one MPDU, and SPR 0x02B holds
    its length at the gate. On the RT-AC56U's BCM4352, of 11039 such frames
    of a phone (VHT80, upload) 8262 read the length alone and 2777 with bit
    14 set, e.g. 0x4065; none read 0x8000. About as many frames were repeated
    by the phone to its retry limit while the open ucode answered them with a
    Block Ack (2881 duplicates dropped). Bit 14 is taken for the delimiter's
    EOF, an S-MPDU, which wants an ACK. `RxFrame.eof` False and True give
    those two; None keeps the model's 0x8000 of a frame that is not an A-MPDU.
  - SPR 0x290 bit 0: without it a push reaches no RX DMA ring (RT-AC56U);
    ucode42 sets it (0x0D03).
  - SPR 0x293 counts the PLCPs: ucode42 puts its bits 0..5 in bits 10..15
    of RX header word 9 (0x0A3F), bits that grow from frame to frame on the
    routers.

The RX buffer's size (opt-in, `rx_buffer`): a frame stays in the RX buffer
until the ucode lets it stream, with a write of SPR_RXE_FIFOCTL1 that has bit
7 set while the frame arrives (0x20C0, 0x2082). One that outgrows the buffer
before that is cut there: SPR_RXE_FRAMELEN stops at the size, its other bytes
are lost, COND_RX_FCS_GOOD does not come and a push delivers what was kept.
On the RT-AC56U the BCM43217 (corerev 30) cut at 383 bytes without the
release, the BCM4352 (corerev 42) at 1020 (RX_BUFFER_CHIP); which bit is the
release is a hypothesis.

A frame starts no earlier than the end on the air of the one before: one
given to rx_frame() for an earlier time arrives then.

Not modelled: which FIFOCTL1 bits clear themselves before corerev 40;
the timing, overflow, bad PLCPs, frames back to back, the BSSID match
before corerev 40 (COND_RX_BSSMATCH is held for every frame), conditions
0x64, 0x67 and 0x68, SPR 0x240/0x244/0x274 beyond the above, and the
crypto engine (its GO bit is in core.py). Nor condition 0x08,
after which ucode42 counts rxcrsglitch (SHM 0x87) unless SPR_RXE_STATUS
bit 7 is set (0x00B6, 0x07E3), nor the RX status bits other than 12..14:
on the RT-AC56U bits 1, 5, 7, 9 and 11 are mostly set, the low ones
differing between frames, and bit 15, which ucode42 takes as an abort
(0x0927), once in 2068 samples, while FRAMELEN stood still for 510 us.
"""

from dataclasses import dataclass, field, replace

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
NEXT_EOF = 0x4000               # the delimiter's EOF (corerev 40+, RxFrame.eof)
SPR_RXE_STAT40 = 0x263
SPR_RXE_HDR40 = 0x240
HDR40_IN, HDR40_PAD = 0x0001, 0x0100
SPR_RXE_COUNT40 = 0x293         # bits 10..15 of the RX header's word 9 on corerev 42
SPR_RXE_TSF = 0x11F             # spr.inc's SPR_TSF_0x3e: the RX header's mac_time
SPR_RXE_LOOKUP40, SPR_RXE_LOOKUP40_DONE, SPR_RXE_LOOKUP40_IDX = 0x260, 0x271, 0x274
AMT_ROWS, AMT_F_VALID, AMT_F_SELF, AMT_F_BSSID = 64, 0x8000, 0x0008, 0x0002
COND_RX_RAMATCH40, COND_RX_TABSS40, COND_RX_A3BSS40 = 0x61, 0x62, 0x63
PHY_RXSUBBANDS40 = 0x143        # AC-PHY register, bits 12..15
PHY143_CHIP = {1: 0x2C25, 2: 0x2C26}    # read at the PLCP on the RT-AC56U's BCM4352 (OFDM, HT)
STAT40_B12 = 0x1000

FIFOCTL1_PUSH, FIFOCTL1_DROP, FIFOCTL1_PAD = 0x0001, 0x0010, 0x0020
FIFOCTL1_SUSPEND = 0x0004       # the response's RX suspend pulse (0x0004, 0), push_hold
PUSH_HOLD_BYTES, PUSH_HOLD_KEPT = 1232, 506    # push_hold: bytes until the hold ends, bytes then kept
MEMBER_HELD = 4                 # bytes of an A-MPDU member before it can be dropped, a model
FIFOCTL1_STREAM = 0x0080        # the frame leaves the RX buffer as it arrives (rx_buffer)
FIFOCTL1_RESTART = 0x0040       # corerev 40+, written while the frame streams: the copy restarts (restart40)
FIFOCTL1_SET40, FIFOCTL1_TAKEN40, FIFOCTL1_HELD40 = 0x0800, 0x20D0, 0x0404
RX_BUFFER_CHIP = {30: 383, 42: 1020}   # corerev: bytes a held frame is cut at on the RT-AC56U
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
    behind: tuple = ()          # A-MPDU: the MPDUs after this one (MPDU and FCS each)
    member: bool = False        # an A-MPDU member after the first: no PLCP event
    eof: bool = None            # corerev 40+: a VHT PPDU of one MPDU: its delimiter's EOF (True: an S-MPDU), None: as a frame alone
    behind_ok: tuple = ()       # A-MPDU: the FCS verdicts of the MPDUs `behind`, good where it is shorter

    def air(self):
        """From the first bit to the last, in microseconds."""
        return self.air_us if self.air_us is not None else \
            self.plcp_us + 4 * ((len(self.data) - PLCP_LEN) * 8 + 22 + 23) // 24


@dataclass
class Pushed:
    rxhdr: bytes
    pad: bool
    data: bytes
    clock: int
    frame: RxFrame = field(repr=False, default=None)


def mac_header_len(mpdu):
    """The length of the MAC header of a data or management frame (802.11
    with HT control); None for others."""
    if len(mpdu) < 2:
        return None
    fc = mpdu[0] | mpdu[1] << 8
    ftype = fc >> 2 & 3
    if ftype not in (0, 2):
        return None
    qos = ftype == 2 and bool(fc & 0x0080)
    a4 = ftype == 2 and fc >> 8 & 3 == 3
    htc = bool(fc & 0x8000) and (qos or ftype == 0)
    return 24 + 6 * a4 + 2 * qos + 4 * htc


class RxEngine:
    def __init__(self, chip):
        self.chip = chip
        # Nothing behind the frame in the buffer: frames come one at a time.
        chip.m.spr[SPR_RXE_NEXT] = NEXT_NONE
        if chip.hw.corerev >= 40:
            chip.m.spr[SPR_RXE_STAT40] = STAT40_B12
        self.frame = None
        self.queue = []             # (clock, RxFrame) still to start
        self.t_plcp = self.t_end = None
        self.received = 0           # bytes the PHY handed over
        self.held = 0               # bytes in the RX buffer
        self.copied = 0             # bytes copied to shared memory
        self.levels = set()         # conditions held while the frame lasts
        self.complete = False
        self.marked = 0             # the SPR_RXE_MARK the frame's bytes reached last
        self.counted = 0            # the count the mark was last compared with
        self.push_due = None        # (rxhdr, pad) of a push made before the end
        self.lookup_due = False      # a write of command 1 to SPR 0x260
        self.fifo_busy_until = None # clock at which the push's copy ends
        self.fifo = []              # Pushed
        self.dropped = []           # RxFrame
        self.log = []               # (clock, event)
        self.rx_buffer = None       # bytes a frame not streaming is cut at; None: no limit
        self.phy143_chip = False    # corerev 40+: PHY 0x143 as the BCM4352 reads at the PLCP; False: sub-bands only
        self.restart40 = False      # corerev 40+: FIFOCTL1 bit 6 written mid-frame restarts the copy (the chip)
        self.next40_mark = False    # corerev 40+: a mark write in an A-MPDU member makes SPR 0x02B read 0x8000
        self.hdr_offset_moves = False  # SPR_RXE_RXHDR_OFFSET moves past the header a push takes (the chip)
        self.mark40_equal = False   # corerev 40+: a mark written equal to the frame's count raises no event (the chip)
        self.push_hold = False      # corerev 42: a push after the RX suspend pulse holds the next frame (the chip)
        self.pulsed = False         # push_hold: FIFOCTL1 bit 2 written with the frame received and not pushed
        self.stuck = False          # push_hold: the next frame is held
        self.hold_until = None      # push_hold: the clock the held frame's bytes come at
        self.lost = 0               # bytes a restarted copy dropped from this frame
        self.streaming = False      # the ucode let the frame stream (FIFOCTL1 bit 7)
        self.cut = None             # the bytes a cut frame kept
        self.hdr40 = None           # corerev 40+: the MAC header's length, until SPR 0x240 has it
        self.air_end = 0            # clock at which the last frame started ends on the air
        self.dropped_on_air = None  # a frame dropped before its last byte, which FRAMELEN still counts
        self.ctl_write = None       # the ucode's FIFOCTL1 writes since service(): (before, written)
        if chip.hw.corerev >= 40:
            chip.m.spr[SPR_RXE_FIFOCTL1] = FIFOCTL1_SET40

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

    def mark_written(self):
        """SPR_RXE_MARK written (corerev 40+). `next40_mark`: inside an
        A-MPDU's MPDU, SPR 0x02B then reads 0x8000 until the MPDU's end.
        `mark40_equal`: a mark equal to the frame's count when written
        raises no COND_RX_COMPLETE."""
        c, f = self.chip, self.frame
        if self.mark40_equal and c.hw.corerev >= 40 and f is not None and not self.complete:
            mark = c.m.spr.get(SPR_RXE_MARK, 0)
            if mark and self.counted == mark:
                self.marked = mark
        if self.next40_mark and c.hw.corerev >= 40 and f is not None and (f.behind or f.member) \
                and not self.complete and c.m.spr.get(SPR_RXE_FRAMELEN, 0) > PLCP_LEN:
            c.m.spr[SPR_RXE_NEXT] = NEXT_NONE

    def fifoctl1_written(self, old, new):
        """A ucode write of SPR_RXE_FIFOCTL1, `old` what it read before."""
        self.ctl_write = (old if self.ctl_write is None else self.ctl_write[0], new)

    def on_air(self):
        return self.chip.clock < self.air_end

    def busy(self):
        return bool(self.queue) or self.frame is not None or self.fifo_busy_until is not None

    def next_clock(self):
        """The clock of the next change the air brings, or None."""
        c = self.chip
        due = [self.fifo_busy_until] if self.fifo_busy_until is not None else []
        if self.frame is not None:
            if self.held < len(self.frame.data) or not self.copy_done():
                return c.clock + 1
        elif self.dropped_on_air is not None:
            return c.clock + 1
        elif self.queue:
            due.append(max(self.queue[0][0], self.air_end))
        if self.on_air() and c.m.spr.get(SPR_RXE_FIFOCTL1, 0) & FIFOCTL1_HELD40 \
                and c.hw.corerev >= 40:
            due.append(self.air_end)
        return min(due) if due else None

    def _clock(self, us):
        return int(us * (self.chip.mhz or 80))

    def start(self, start, f):
        self.frame, self.received, self.held, self.copied = f, 0, 0, 0
        self.dropped_on_air = None
        self.t_plcp = start + self._clock(f.plcp_us)
        self.t_end = self.air_end = start + self._clock(f.air())
        self.levels = self.levels & {COND_RX_FIFOBUSY}
        self.complete, self.marked, self.counted, self.push_due = False, 0, 0, None
        self.streaming, self.cut = False, None
        self.lost = 0
        self.pulsed, self.hold_until = False, None
        if self.stuck:
            self.hold_until = min(self.t_plcp + self._clock(8 * PUSH_HOLD_BYTES / 6), self.t_end)
            self.cut = min(PUSH_HOLD_KEPT, len(f.data))
            self.log.append((self.chip.clock, "held: a push came after the RX suspend pulse"))
        if self.chip.hw.corerev >= 40:
            self.hdr40 = mac_header_len(f.data[PLCP_LEN:])

    def tick(self):
        """Bring the engine to the chip's clock."""
        c = self.chip
        if self.fifo_busy_until is not None and c.clock >= self.fifo_busy_until:
            self.fifo_busy_until = None
            self.levels.discard(COND_RX_FIFOBUSY)
        if c.hw.corerev >= 40 and not self.on_air() and \
                c.m.spr.get(SPR_RXE_FIFOCTL1, 0) & FIFOCTL1_HELD40:
            self.read_back(c.m.spr[SPR_RXE_FIFOCTL1])
        if self.dropped_on_air is not None:
            d = self.dropped_on_air
            n = len(d.data)
            held = self.arrived(c.clock - self._clock(COPY_LAG_US), d)
            c.m.spr[SPR_RXE_FRAMELEN] = min(held, n - FRAMELEN_SHORT.get(c.hw.corerev, 0)) - self.lost
            if held == n:
                self.dropped_on_air = None
        if self.frame is None and not self.queue:
            return
        if self.frame is None and self.queue and self.queue[0][0] <= c.clock and not self.on_air():
            at, f = self.queue.pop(0)
            self.start(max(at, self.air_end), f)
        f = self.frame
        if f is None or c.clock < self.t_plcp:
            return
        spr = c.m.spr
        if self.received == 0:
            spr[SPR_RXE_ENCODING] = (spr.get(SPR_RXE_ENCODING, 0) & ~ENCODING_OFDM) | \
                (ENCODING_OFDM if f.ofdm else 0)
            if not (c.hw.corerev >= 40 and (f.behind or f.member)):
                self.phy_status(f)
            spr[SPR_RXE_STATUS] = spr.get(SPR_RXE_STATUS, 0) & ~STATUS_COPIED | \
                STATUS_PLCP | STATUS_DATA
            if c.hw.corerev >= 40:
                spr[SPR_RXE_HDR40] = 0
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
                v = c.phy.read(PHY_RXSUBBANDS40) & 0x0FFF | (f.subbands & 0xF) << 12
                if self.phy143_chip:
                    # BCM4352 under 784.2, 5 GHz ch 36 HT20: 0x2C26 for an HT frame, 0x2C25 for OFDM
                    v = PHY143_CHIP.get(f.ftype if f.ftype is not None else int(f.ofdm), v)
                c.phy.observe(PHY_RXSUBBANDS40, v)
            if len(f.data) >= PLCP_LEN + 2 and c.hw.corerev >= 40:
                # The frame control word, where the corerev 42 ucode reads
                # type and subtype (bits 2..7), a model.
                spr[SPR_RXE_FC40] = f.data[PLCP_LEN] | f.data[PLCP_LEN + 1] << 8
            if c.mhz:
                spr[SPR_RXE_TSF] = c.tsf() & 0xFFFF
            if c.hw.corerev >= 40:
                spr[SPR_RXE_COUNT40] = (spr.get(SPR_RXE_COUNT40, 0) + 1) & 0xFFFF
            if f.behind or f.member or f.eof is not None:
                spr[SPR_RXE_NEXT] = (len(f.data) - PLCP_LEN) & 0x0FFF | (NEXT_EOF if f.eof else 0)
            if f.member:
                self.log.append((c.clock, "member"))
            else:
                c.events.add(COND_RX_PLCP)
                c.napping = False   # a new frame wakes the PSM, raised before or not
                self.log.append((c.clock, "plcp"))
            self.received = PLCP_LEN
        want = self.arrived(c.clock)
        if want > self.received:
            self.received = want
        n = len(f.data)
        held = self.arrived(c.clock - self._clock(COPY_LAG_US))
        if self.rx_buffer is not None and not self.streaming and self.received > self.rx_buffer \
                and self.cut is None:
            self.cut = self.rx_buffer
            self.log.append((c.clock, f"cut at {self.rx_buffer} bytes: the RX buffer full"))
        if self.cut is not None:
            held = self.cut if c.clock < self.t_end else n
        if self.hold_until is not None and c.clock < self.hold_until:
            held = 2
        self.held = held
        framelen = min(held if self.cut is None or self.hold_until is not None and c.clock < self.hold_until
                       else self.cut, n - FRAMELEN_SHORT.get(c.hw.corerev, 0)) - self.lost
        if framelen != spr.get(SPR_RXE_FRAMELEN):
            spr[SPR_RXE_FRAMELEN] = framelen
        self.copy(held - self.lost)
        if self.hdr40 is not None and held >= PLCP_LEN + self.hdr40:
            length, self.hdr40 = self.hdr40, None
            spr[SPR_RXE_HDR40] = HDR40_IN | length << 1 | (HDR40_PAD if length % 4 == 2 else 0)
            self.log.append((c.clock, f"MAC header in, {length} bytes"))
        if held == n and spr.get(SPR_RXE_STATUS, 0) & STATUS_DATA:
            spr[SPR_RXE_STATUS] &= ~(STATUS_PLCP | STATUS_DATA)
        mark = spr.get(SPR_RXE_MARK, 0)
        kept = held - self.lost
        count = self.counted = kept - PLCP_LEN if (f.behind or f.member) and kept > PLCP_LEN else framelen
        if mark != self.marked and count >= mark and not self.complete:
            self.marked = mark
            if mark:
                c.events.add(COND_RX_COMPLETE)
                c.napping = False
                self.log.append((c.clock, f"{mark} bytes"))
        if self.copy_done() and not spr.get(SPR_RXE_STATUS, 0) & STATUS_COPIED \
                and self.complete and held == n:
            spr[SPR_RXE_STATUS] = spr.get(SPR_RXE_STATUS, 0) | STATUS_COPIED
        if not self.complete and (held == n if f.behind else c.clock >= self.t_end):
            self.complete = True
            if c.hw.corerev >= 40 and f.member and not f.behind:
                self.phy_status(f)
            self.levels.add(COND_RX_COMPLETE)
            c.events.add(COND_RX_COMPLETE)
            c.napping = False
            if f.fcs_ok and self.cut is None and not self.lost:
                c.events.add(COND_RX_FCS_GOOD)
            if self.copy_done() and held == n:
                spr[SPR_RXE_STATUS] = spr.get(SPR_RXE_STATUS, 0) | STATUS_COPIED
            if f.behind or f.member or f.eof is not None:
                spr[SPR_RXE_NEXT] = (len(f.behind[0]) & 0x0FFF) if f.behind else NEXT_NONE
            self.log.append((c.clock, "complete"))
            if self.push_due:
                self.deliver(*self.push_due)

    def phy_status(self, f):
        """The PHY's six RX status words, as the ucode reads them."""
        for n, v in zip((SPR_RXE_PHYRXSTAT0, SPR_RXE_PHYRXSTAT1,
                         SPR_RXE_PHYRXSTAT2, SPR_RXE_PHYRXSTAT3,
                         SPR_RXE_PHYRXSTAT4, SPR_RXE_PHYRXSTAT5), f.phyrxstat):
            self.chip.m.spr[n] = v & 0xFFFF

    def copy_lead(self):
        """Whether the frame starts one word into the copy: with the HT and
        the AC PHY the ucode reads the PLCP at SPR_RXE_COPY_OFFSET + 1 (its
        off1: corerev 26 0x5F5, 29 0x5FD, 40/42 0x78D), with the N PHY at
        SPR_RXE_COPY_OFFSET (corerev 16..25, 30: 0x70C on corerev 30)."""
        return (self.chip.phy.read(0) >> 8) & 0xF in (PHYTYPE_HT, PHYTYPE_AC)

    def arrived(self, clock, f=None):
        """The bytes of the frame (or `f`, the last started) the PHY has
        handed over by clock."""
        n = len((f or self.frame).data)
        if clock < self.t_plcp:
            return 0
        if clock >= self.t_end:
            return n
        span = max(1, self.t_end - self.t_plcp)
        return PLCP_LEN + (n - PLCP_LEN) * (clock - self.t_plcp) // span

    def kept(self):
        """The frame's bytes without those a restarted copy dropped."""
        data = self.frame.data
        return data[:PLCP_LEN] + data[PLCP_LEN + self.lost:] if self.lost else data

    def copy_done(self):
        f = self.frame
        return f is not None and \
            self.copied >= min(len(f.data) - self.lost, self.chip.m.spr.get(SPR_RXE_COPY_LEN, 0))

    def copy(self, upto):
        """The bytes inside the copy window that arrived COPY_LAG_US ago, to
        shared memory."""
        c = self.chip
        spr, shm = c.m.spr, c.m.shm
        data = self.frame.data
        limit = min(upto, len(data) - self.lost, spr.get(SPR_RXE_COPY_LEN, 0))
        base = spr.get(SPR_RXE_COPY_OFFSET, 0) + (1 if self.copy_lead() else 0)
        while self.copied < limit:
            i = self.copied & ~1
            end = min(i + 2, limit)
            j = i if i < PLCP_LEN else i + self.lost
            shm[(base + i // 2) & (len(shm) - 1)] = int.from_bytes(data[j:j + end - i], "little")
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
        if self.ctl_write is None:
            return
        old, ctl = self.ctl_write
        self.ctl_write = None
        f = self.frame
        if self.restart40 and c.hw.corerev >= 40 and f is not None and self.streaming \
                and ctl & FIFOCTL1_RESTART and not self.complete:
            # the BCM4352: bit 6 written once the frame streams drops what the RXE holds past the PLCP and
            # restarts the copy there (a dropped frame's SHM copy began at the LLC ethertype, 32 bytes on)
            n = max(0, spr.get(SPR_RXE_FRAMELEN, 0) - PLCP_LEN)
            if n:
                self.lost += n
                self.copied = min(self.copied, PLCP_LEN)
                self.log.append((c.clock, f"copy restarted: {n} bytes lost"))
        if self.push_hold and c.hw.corerev >= 40:
            if ctl & FIFOCTL1_DROP:
                self.stuck = False      # the drop's reset: 0x0014, 0x0010, 0
            elif ctl & FIFOCTL1_SUSPEND and f is not None and self.complete and self.streaming:
                self.pulsed = True
            if ctl & FIFOCTL1_PUSH and not old & FIFOCTL1_PUSH and f is not None and self.pulsed:
                self.stuck = True
                self.log.append((c.clock, "push after the RX suspend pulse"))
        if ctl & FIFOCTL1_STREAM and f is not None and not self.streaming and self.cut is None:
            self.streaming = True
            self.log.append((c.clock, "streams"))
        if ctl & FIFOCTL1_PUSH and not old & FIFOCTL1_PUSH and f is not None:
            shm = c.m.shm
            base, length = spr.get(SPR_RXE_RXHDR_OFFSET, 0), spr.get(SPR_RXE_RXHDR_LEN, 0)
            words = [shm[(base + k) & (len(shm) - 1)] for k in range(length)]
            hdr = b"".join(w.to_bytes(2, "little") for w in words)
            if self.hdr_offset_moves:
                # the chip: the push takes the header out of shared memory and the offset moves past it
                spr[SPR_RXE_RXHDR_OFFSET] = (base + length) & 0xFFFF
            if self.complete:
                self.deliver(hdr, bool(ctl & FIFOCTL1_PAD))
            else:
                self.push_due = (hdr, bool(ctl & FIFOCTL1_PAD))
                self.log.append((c.clock, "push before the end"))
        elif ctl & FIFOCTL1_DROP and f is not None and \
                not (f.member and self.held < PLCP_LEN + MEMBER_HELD):
            # a member whose bytes have not come yet is not there to be dropped: 784.2 writes
            # a bad member's drop twice within 1 us, and its Block Acks have holes
            self.dropped.append(f)
            self.log.append((c.clock, "drop"))
            if self.held < len(f.data) and not f.behind:
                self.dropped_on_air = f
            self.end()
        self.read_back(ctl, old)

    def read_back(self, ctl, before=0):
        """SPR_RXE_FIFOCTL1 as the hardware leaves it after `ctl`, written
        over `before`."""
        c = self.chip
        if c.hw.corerev < 40:
            value = ctl & ~(FIFOCTL1_PUSH | FIFOCTL1_DROP)
        elif self.on_air():
            value = (ctl | before & FIFOCTL1_HELD40) & ~FIFOCTL1_TAKEN40 | FIFOCTL1_SET40
        else:
            value = ctl & ~(FIFOCTL1_TAKEN40 | FIFOCTL1_HELD40) | FIFOCTL1_SET40
        if value != ctl:
            c.m.store(("spr", SPR_RXE_FIFOCTL1), value)
            self.ctl_write = None

    def deliver(self, hdr, pad):
        c = self.chip
        size = len(hdr) + len(self.frame.data)
        if c.hw.corerev >= 40 and not c.m.spr.get(SPR_RXE_DMA40, 0) & DMA40_ON:
            self.log.append((c.clock, "push, no RX DMA: SPR 0x290 bit 0 clear"))
        else:
            data = self.kept() if self.cut is None else self.frame.data[:self.cut]
            self.fifo.append(Pushed(hdr, pad, data, c.clock, self.frame))
            self.log.append((c.clock, "push"))
        self.end()
        self.levels.add(COND_RX_FIFOBUSY)
        self.fifo_busy_until = c.clock + self._clock(1 + size / FIFO_BYTES_PER_US)

    def end(self):
        spr, f = self.chip.m.spr, self.frame
        spr[SPR_RXE_STATUS] = spr.get(SPR_RXE_STATUS, 0) & ~(STATUS_PLCP | STATUS_DATA)
        self.frame = None
        self.levels &= {COND_RX_FIFOBUSY}
        if f.behind:
            # The next A-MPDU member, at the rate of the one before.
            # its FCS verdict is its own: the one before's falls (the BCM4352: bad
            # members after good ones are told apart).
            self.chip.events.discard(COND_RX_FCS_GOOD)
            mpdu = f.behind[0]
            air = None if f.air_us is None else \
                (f.air_us - f.plcp_us) * len(mpdu) / max(1, len(f.data) - PLCP_LEN)
            self.start(self.chip.clock, replace(f, data=bytes(PLCP_LEN) + mpdu, behind=f.behind[1:],
                                                fcs_ok=f.behind_ok[0] if f.behind_ok else True, behind_ok=f.behind_ok[1:],
                                                member=True, plcp_us=0.0, air_us=air))
