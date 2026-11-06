PSM interpreter
================

`psm15.py` and `psm5.py` interpret the D11 PSM microcode in the two
instruction sets `disassembler/` knows: `arch15` (core revisions 15 and
later) and `arch5` (5..14). What they share, the opcode table and the
executor, is `psmlib.py`. They load a `d11ucodeNN` blob, raw (`raw-be32`, as
used with `b43-dasm`) or a b43 `.fw` image as `b43-fwcutter` writes it, in
8-byte words or packed in 7 bytes per instruction, as the vendor driver stores
its sslpn and lcn ucode (`wlc_ucode_write_byte`), and execute it instruction
by instruction, tracking GPR/SPR/SHM state.

They do not target one specific ucode build: the decoder works on the
bit-level instruction encoding, not on anything file-specific, and has been
run unmodified against four different corerev 42 blobs spanning two
distinct ucode families (bommajor 0x310 and 0x3A0), the arch15 ucode of
seven other core revisions and OpenFWWF (see Validation).


About this software
--------------------

The D11 PSM is the small in-chip microcontroller that runs the vendor's
closed-source MAC microcode (`d11ucodeNN.fw`, extracted by `b43-fwcutter`
and loaded by the driver at runtime; this project does not, and does not
aim to, replace it). `interpreter/` is a dynamic-analysis complement to
`disassembler/`: instead of only producing a static listing, it lets you
single-step execution from an arbitrary address with a chosen initial
state and inspect exactly what each instruction read and wrote.

It is split into two parts with very different confidence levels, both
documented at the top of `psmlib.py`, `psm15.py` and `psm5.py`:

- **Decoder** (bytes -> instructions): a from-scratch Python re-derivation
  of `disassembler/main.c`'s bit-level logic (instruction word layout,
  opcode table, operand field decoding). Validated instruction-by-
  instruction against `b43-dasm`'s own output -- see Validation below.
- **Executor** (instruction -> effect on machine state): confidence varies
  per instruction group. Plain arithmetic/logic (`add`/`sub`/`and`/`or`/
  `xor`/`sr`/`sl`/`rl`/`rr`/`nand`/`mul`), `orx 7,8` with two immediates
  (b43-asm's 16-bit `mov`), `jzx`/`jnzx` and `jand`/`jnand` are
  high-confidence: the source operand convention is structural in the
  original decoder, the `orx 7,8` concatenation is confirmed against a real
  MMIO capture (see Validation), `jzx`/`jnzx` follow b43-asm's own `jand`
  emulation, and `jand` jumps on a nonzero AND, `jnand` on a zero one, as
  measured on core revisions 22 and 42. The other jump conditions,
  `orx`/`srx` with other parameters, and `jext`/`jnext` (which test an
  implicit hardware flag the disassembler itself never needed to resolve)
  are best-effort or outright stubs. Every instruction the interpreter
  executes is tagged with its confidence level in the trace, so unverified
  territory is visible rather than silently assumed correct.


Usage
-----

    psm15.py <ucode.bin> [start_address_hex] [max_steps]
    psm5.py <ucode.bin> [start_address_hex] [max_steps]

Example (D6220's version-stamp block, see Validation):

    ./psm15.py d11ucode42.bin 0xF76 20

As a library:

```python
from psm15 import load_program, PSM

prog = load_program("d11ucode42.bin")
m = PSM(prog)
m.run(start=0xF76, max_steps=20, trace=True)
for instr, confidence, note in m.trace:
    print(instr.addr, instr.mnem, confidence, note)
print(hex(m.shm[0]))   # UCODEREV
```

`PSM.run()` raises `HardwareWaitLoop` when execution stalls without
advancing past its highest-reached PC for `stall_threshold` steps in a
row -- the expected outcome of running from the real entry point (PC 0),
where the boot waits on hardware the interpreter does not model (see Known
limitations). This is reported explicitly rather than spinning silently or
producing a misleading result.


Validation
----------

Decoder, checked against `b43-dasm -a 15 -f raw-be32`'s own output,
instruction by instruction, on D6220's 5976-instruction `d11ucode42`:

    jump-target address mismatches:  0 / 5976
    mnemonic mismatches:             0 / 5976, beyond the opcodes that
                                      b43-dasm has no mnemonic for either
                                      (0x002, 0x070, 0x071 and the 0x000
                                      end-of-code marker) -- a shared gap,
                                      not a disagreement between the two

Executor, checked against the version-stamp prologue (the block that
writes `UCODEREV`/`UCODEPATCH`/... to SHM), whose values are confirmed
independently by real MMIO captures. Running from that block's entry on
all four available corerev 42 blobs:

    board          start   SHM[0] (UCODEREV)   SHM[1] (UCODEPATCH)
    D6220          0xF76   0x3A0  (expected)    0x2715 (expected)
    TG789vac v2    0xF76   0x3A0  (expected)    0x05DE (expected)
    vd625          0xF77   0x3A0  (expected)    0x04B1 (expected)
    DSL-3580_EU    0xCED   0x310  (expected)    0x0002 (expected)

4/4 exact matches, including DSL-3580_EU's unrelated 0x310 ucode family --
not a value tuned to pass, the interpreter has no per-file special-casing.

The other instruction set and the other core revisions, decoder against
`b43-dasm` as above (mnemonics, operands and jump targets):

    blob                                         arch   instructions  mismatches
    OpenFWWF ucode5 (this fork's b43-asm)        5      1190          0
    DSL-3580_EU ucode16_mimo, ucode16_lp,        15     4221..5402    0
      ucode22_mimo, ucode24_mimo, ucode26_mimo,
      ucode29_mimo, ucode30_mimo, ucode40, ucode42

The DSL-3580_EU blobs are those `b43-fwcutter` extracts from
`impl14_v07/wlDSL-3580_EU.o_save` of the router's GPL release.


Known limitations
------------------

On its own, the interpreter run from the real entry point (PC 0) stops at the
first wait on the hardware: the PHY register interface (SPR 0x018/0x019) and
the SPR handshakes of the boot, then the main loop's external conditions.
`lockstep.py` runs the whole program against a host op stream, with the PHY
interface and the handshakes modelled as `core.py` describes them for the core
revision, and COND_MACEN taken from MACCONTROL; from PC 0 it boots both the
0x3A0 and the 0x310 builds of corerev 42, the corerev 22 `ucode22_mimo` and
OpenFWWF to their main loop. Every other `jext`/`jnext` condition stays false.

A MAC disable suspends the ucode only when nothing is pending. If the host
has just written a nonzero word to SHM byte 0x00B8 (0x7148, once in most
D6220 cold-sweep segments), the 0x3A0 builds copy it to SPR 0x0E7, start the
TX engine (SPR_TXE0_CTL = 0x4001), set bit 1 of SPR_BRC and go back to the
main loop to send a frame before suspending. That suspend completes only
with `lockstep.py --tx-engine`, a model of OpenFWWF's transmit sequence
(COND_TX_NOW, then COND_TX_POWER and COND_TX_DONE, each held until the
ucode acknowledges it) rather than a known semantic of the corerev 42 TX
engine; on the D6220 captures its outcome does not depend on the delays
given to it.

The 0x310 build (DSL-3580_EU) also waits on the TSF before that suspend: at
0x066C it goes back to the main loop until 8 microseconds have passed since
the value it saved in SHM word 0x81A. Without a running TSF it never gets
there; with `--psm-mhz` it sends its frame and suspends.

On arch15 `unk_002` (opcode 2 of the control group, beside `nap` 1, `calls`
4 and `rets` 5; arch5's `call`) has no operands and no known meaning; it is
executed as a no-op.

TKIP acceleration (`tkiph`/`tkiphs`/`tkipl`/`tkipls`) is not implemented.


Companion tools
---------------

The co-simulation tools read the host side as an op stream, one MMIO-level
operation per line, as the b43 AC-PHY port's trace and capture tools
(`test/integration`, `reverse-tools/mmio2ops.py`) write it:

    [<timestamp> #<n>] cpu<N> <CLASS> key=0x<hex> ...

with classes such as

    REG.WR     off= val=
    OBJ.WR     addr= val= sel=
    MAC.MCTRL  val= [mask=]     (vendor captures: brcms_b_mctrl's mask/val)
    MAC.MCMD   val=
    MAC.MHF    addr= val= mask=

A tool skips the classes it does not use.

`vchip.py` is the virtual chip: the PSM running the real ucode and the
hardware around it, built from the `core.py` profile of a core revision and
from PHY and radio devices. Its docstring describes what it models.

`vchipd.py` serves the chip to a driver over stdin/stdout: the driver
forwards the D11 core's MMIO and the passing of time, the chip answers the
reads and says when an interrupt is pending. The board is given by options
(core revision, PHY version word, the radio's identity registers):

    ./vchipd.py --corerev 42 --phy-ver 0x0b01 --radio 0=0x4 --radio 1=0x2069

    W <off> <val> <bits>    an MMIO write            -> K <irq>
    R <off> <bits>          an MMIO read             -> V <val> <irq>
    T <us>                  time passes              -> K <irq>
    Q                       end                      -> K <irq>

`lockstep.py` co-simulates a host op stream with the ucode as one machine:
the PSM starts at PC 0 on PSM_RUN and runs `--cycles` instructions after
every host operation, over shared memory and an IHR register file the
host's MMIO writes reach (SPR n is MMIO 0x400 + 2n). Its report lists every
assumption it had to make: the external conditions with the value used, the
SPRs read before anyone wrote them, the PHY registers the ucode read and where
their value came from.

    ./lockstep.py b43.trace d11ucode42.bin --cond-inc cond.inc

The captures do not contain the initvals: `--initvals` writes them (initvals,
then bsinitvals) when the ucode first reports MAC_SUSPENDED after PSM_RUN,
where brcms_b_coreinit writes them. A fixed `--cycles` is too short for the
boot, which clears shared memory before suspending (about 9700 instructions on
the 0x3A0 builds): host writes made meanwhile are lost. `--settle MAX` runs
the ucode until it is idle after each host operation instead, and adds a
timetable to the report: the instructions the ucode spends on each operation,
when it raises IRQ bits, and the shared-memory words it writes that the host
reads later.

    ./lockstep.py cold01-ch36-bw20.txt d11ucode42.bin --settle 200000 \
        --initvals d11ac1initvals42.bin --initvals d11ac1bsinitvals42.bin

The initvals can be brcmsmac's d11init blobs or b43's `.fw` images.

`--corerev N` (default 42) picks the instruction set and the `core.py`
profile, per generation:

    corerev  PHY (SPR 0x018/0x019)        radio (0x030)    handshakes           read from
    5..14    bit 12 read, bit 13 write    -                RXE_FIFOCTL1         5 (OpenFWWF), 11
    15..39   bit 12 read, bit 13 write    bit 12/13        RXE_FIFOCTL1, 0x078  16, 22, 24, 26, 29, 30
    40..     bit 14 start, bit 13 read    bit 12/13        RXE_FIFOCTL1, 0x078  40, 42

That the radio is behind SPR 0x030/0x031 is inferred (STATUS.md); up to
corerev 22 the ucode reaches the radio through the PHY's four-wire
registers, not modelled. A revision outside a profile's "read from" list is
reported as assumed. PHY register 0 reads as the trace's MMIO PHY_VER, or as
`--phy-ver`.

Condition register 4 follows SPR_BRC bit by bit, as OpenFWWF documents it.
`--tx-engine IFS,START,DONE` adds the TX engine model described below, with
its delays in PSM instructions.

On corerev 40 and 42, the TX engine model starts a frame when the ucode writes
SPR 0x320 (SPR_TX_Serial_Control) with bit 15 set: 0x8001 when it sends a
queued frame, 0x8000 when it sends the beacon (0x0462 on the 0x310 build).
Up to corerev 30 it starts one on 0x0826 in SPR_TXE0_SELECT, TX_Count bytes
of template RAM; other values written there are not modelled.

`--psm-mhz F` makes the TSF run: SPR_TSF_WORD0..3 (0x119..0x11C) advance by
one microsecond every F instructions, as on a PSM running one instruction per
cycle at F MHz. Neither the clock nor the cycles per instruction of the
corerev 42 PSM are known, so F is a model parameter, like the TX engine
delays. Host writes to MMIO 0x632..0x638 or to the 32-bit TSF registers at
0x180/0x184 set the TSF, and so do ucode writes to those SPRs. A loop that
reads the TSF is not idle for `--settle`. On D6220's cold01 capture the
report is the same at 80 and at 200.

With the TSF running, the beacon interval the host programs raises
COND_TX_TBTTEXPIRE (0x2C) at every TBTT: MMIO 0x188 (tsf_cfprep) holds the
interval in microseconds shifted left by 6, MMIO 0x18C (tsf_cfpstart) the
first TBTT, as brcmsmac and b43 program them. An op line `cpuN WAIT
us=0x...` lets that much time pass; while the ucode is idle the clock jumps
to the next TBTT. Raising one condition at a time on an idle, enabled ucode
(`condfuzz.py`) finds 0x2C as the only one that reaches the beacon block on
both families, through bit 12 of SPR_BRC.

    cpu0 MAC.MCTRL val=0x04000404
    cpu0 MAC.MCTRL val=0x04020402
    cpu0 REG.WR off=0x0188 val=0x00640000
    cpu0 REG.WR off=0x018c val=0x00019000
    cpu0 MAC.MCTRL val=0x04020403
    cpu0 MAC.MCMD val=0x00000003
    cpu0 WAIT us=0x00060000

With `--settle 300000 --tx-engine 20,10,30 --psm-mhz 80` and the initvals,
this sends a beacon at each of the three TBTTs on DSL-3580_EU and on D6220,
raising TBTT_INDI and BEACON_TX_OK each time.

`cond.py` parses OpenFWWF's `cond.inc` (github.com/fullstory/openfwwf) into the
external-condition map, so the jext/jnext signals the executor stubs can be
referred to by name and turned into `ext_flags` seeds:

    ./cond.py                       # list COND_* -> selector
    python3 -c "from cond import CondMap; print(hex(CondMap.load().by_name('TX.MACEN')))"

Seeding the matching condition steers the real dispatcher: from the main loop,
with `COND_RX_COMPLETE` forced the ucode enters its RX handler where unseeded it
idles. The names are OpenFWWF's corerev-5 (arch5) names; the selector layout and
`COND_TRUE=0x7F` carry over to the arch15 cores, but a given bit is a lead for
a newer core, not a guarantee.

`condfuzz.py` brings the co-simulation to a state with an op stream, then
raises each condition in turn (on top of any given with `--with`) and lists
those that lead the ucode to a target address, or with `--blocking` those
that keep it away. Conditions stay raised until the ucode acknowledges them,
where hardware may drop them sooner: a hit is a lead to read in the listing.

    ./condfuzz.py idle.ops d11ucode42.bin 0x016B --psm-mhz 80 \
        --tx-engine 20,10,30 --initvals d11ac1initvals42.bin \
        --initvals d11ac1bsinitvals42.bin

`fwmatrix.py` runs every ucode of a `b43-fwcutter` output directory: the
decoder against `b43-dasm`, then a boot and a beacon interval in
`lockstep.py` with the profile of the ucode's corerev and the initvals of its
PHY. STATUS.md has its results on the DSL-3580_EU driver.

    ./fwmatrix.py fw/b43 --dasm ../disassembler/b43-dasm

`ucode_init.py` runs just the version-stamp prologue and dumps the shared
memory the ucode writes (UCODEREV/PATCH/...); `extcond_scan.py` lists, named via
`cond.py`, the jext/jnext conditions a blob tests:

    ./ucode_init.py d11ucode42.bin --out ucode.shm
    ./extcond_scan.py d11ucode42.bin

`cosim.py` runs the real ucode against a host MMIO trace, sharing one state so
the microcode reads what the host wrote and writes back, then snapshots the
complete D11 state (shared memory, scratch, RCMTA, template RAM, the MMIO/IHR
register file, and the PSM's own GPR/SPR):

    ./cosim.py host.ops d11ucode42.bin --out d11.state

The boot hand-off runs the validated version-stamp; per-command hand-offs
run the main loop, but a command the ucode picks up by reading the MAC
command register dispatches through IHR reads this interpreter does not
model, so those handlers do not run (the command is recorded, and whatever
the host routed through shared memory for it is still applied).
