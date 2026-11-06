Where the interpreter stands
============================

The vendor ucode of every core revision in the DSL-3580_EU driver boots from
PC 0 to its main loop and sends beacons, in both instruction sets.


Blobs
-----

`bcmdrivers/broadcom/net/wl/impl14_v07/wlDSL-3580_EU.o_save` of the DSL-3580_EU
GPL release (the `gpl-release` asset of github.com/aleferri/dsl-3580l-sources;
md5 74fc13626ed4a592a94b235787a9f6aa, ucode 784.2). `b43-fwcutter -w fw`
extracts 14 ucode with code in them and their initvals; all report UCODEREV
0x0310, PATCH 2. `ucode16_sslpn`, `ucode16_sslpn_nobt`, `ucode22_sslpn` and
`ucode24_lcn` are packed in 7 bytes per instruction (the driver's
`wlc_ucode_write_byte` writes bytes 3..6, then 0..2); fwcutter writes them out
as they are.

OpenFWWF, built with this fork's `b43-asm` (`jand`/`jnand` swapped in the
source), is the arch5 ucode for corerev 5.


Results
-------

`fwmatrix.py fw/b43 --dasm ../disassembler/b43-dasm`, TSF at 80 instructions
per microsecond, PHY register 0 as the PHY type of the ucode's name
(revision 0):

  boot    PSM_RUN, MAC enable, valid beacon templates: the ucode suspends
          (IRQ 0x1), gets its initvals and answers with IRQ 0x2.
  beacon  AP mode, beacon interval 102400 us, 393 ms of TSF, TX engine
          model on: each beacon is a TBTT, COND_TX_NOW, the launch,
          COND_TX_POWER, COND_TX_DONE.

    ucode               corerev  isa    bytes  instr  vs dasm  initvals          boot  beacons
    ucode11             11       arch5  8      3843   0        n0initvals11      yes   3 of 3
    ucode16_lp          16       arch15 8      4221   0        lp0initvals16     yes*  3 of 3
    ucode16_mimo        16       arch15 8      4920   0        n0initvals16      yes   3 of 3
    ucode16_sslpn       16       arch15 7      4949   0        sslpn0initvals16  yes   3 of 3
    ucode16_sslpn_nobt  16       arch15 7      4100   0        sslpn0initvals16  yes   3 of 3
    ucode22_mimo        22       arch15 8      4793   0        n0initvals22      yes   3 of 3
    ucode22_sslpn       22       arch15 7      4754   0        sslpn4initvals22  yes   3 of 3
    ucode24_lcn         24       arch15 7      5014   0        lcn0..2initvals24 yes*  3 of 3
    ucode24_mimo        24       arch15 8      4996   0        n0initvals24      yes   3 of 3
    ucode26_mimo        26       arch15 8      4962   0        ht0initvals26     yes   3 of 3
    ucode29_mimo        29       arch15 8      5402   0        ht0initvals29     yes   3 of 3
    ucode30_mimo        30       arch15 8      5009   0        n16initvals30     yes   3 of 3
    ucode40             40       arch15 8      5072   0        ac0initvals40     yes   3 of 3
    ucode42             42       arch15 8      5002   0        ac1initvals42     yes   3 of 3

    * The main loop of ucode16_lp and ucode24_lcn reads the TSF on every
      pass, so with the TSF running `lockstep.py --settle` never finds it
      idle and runs each host operation to its limit; the ucode still answers
      as the others do.

OpenFWWF: 0 mismatches against `b43-dasm` on 1190 instructions; boots to its
idle `nap`; `condfuzz.py` finds COND_RX_COMPLETE as the condition that
reaches `rx_complete`.


Hardware read from the ucode
----------------------------

- PHY at SPR 0x018/0x019: bit 12 reads, bit 13 writes up to corerev 30
  (OpenFWWF's `sel_phy_reg`/`write_phy_reg`, same sequence in the vendor
  ucode); bit 14 with bit 13 for a read on 40 and 42. `ucode22_mimo` with the
  corerev 42 commands hangs on its first PHY read. `ucode16_mimo`, `_lp` and
  `_sslpn` access the PHY alike.
- Radio at SPR 0x030/0x031 from corerev 24, inferred: up to 22 the radio
  routines go through the N-PHY's four-wire registers 0x0B/0x0D, register in
  r0, value in r1, skipped while MACCONTROL.RADIOLOCK is set; from 24 the
  routines at SPR 0x030 take the same r0/r1. The HT ucode writes 0x184,
  0x584, 0x984 (b43's R2059_C1..C3 core offsets); the LCN one reaches 0x033,
  0x112, 0x113 (registers of brcmsmac's radio 2064).
- Handshakes: every ucode sets bit 2 of SPR_RXE_FIFOCTL1 before a MAC
  suspend and waits for it to clear; the arch15 ones set bit 5 of SPR 0x078
  and wait for bit 13.
- TX launch: 0x0826 in SPR_TXE0_SELECT after the template pointer and
  TX_COUNT up to corerev 30 (OpenFWWF: "select TX_Count bytes from template
  ram, put into serializer and generate EOF"); SPR 0x320 with bit 15 on 40
  and 42. Each generation writes only its own.


Not modelled, or not known
--------------------------

- The radio's registers read as written, else 0; the four-wire access
  through the PHY is not connected to the radio device.
- PHY registers the hardware updates (e.g. 0x05B0 read by the sslpn ucode,
  0x0235 by `ucode30_mimo`) read as written, else 0.
- The PHY revision: `fwmatrix.py` gives the type only.
- Other values in SPR_TXE0_SELECT (low bits 0x05, 0x07, 0x43) do not launch
  a frame.
- External conditions other than COND_TRUE, COND_MACEN, the TX engine's,
  the TBTT and condition register 4 stay false; their names are OpenFWWF's.
- arch5: nothing measured on hardware.
- Revisions other than 5, 11, 16, 22, 24, 26, 29, 30, 40, 42: profile of
  their generation, reported as assumed. Corerev 4 and earlier: no
  instruction set.
