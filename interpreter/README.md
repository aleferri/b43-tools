PSM interpreter
================

`psm.py` is a generic interpreter for the D11 PSM microcode (`arch15`
instruction set, as disassembled by `disassembler/`). It loads a raw
`d11ucodeNN` blob (same `raw-be32` format used with `b43-dasm`) and
executes it instruction by instruction, tracking GPR/SPR/SHM state.

It does not target one specific ucode build: the decoder works on the
bit-level instruction encoding, not on anything file-specific, and has
been run unmodified against four different corerev 42 blobs spanning two
distinct ucode families (bommajor 0x310 and 0x3A0).


About this software
--------------------

The D11 PSM is the small in-chip microcontroller that runs the vendor's
closed-source MAC microcode (`d11ucodeNN.fw`, loaded by `fwcutter`/the
stock driver at runtime; this project does not, and does not aim to,
replace it). `interpreter/` is a dynamic-analysis complement to
`disassembler/`: instead of only producing a static listing, it lets you
single-step execution from an arbitrary address with a chosen initial
state and inspect exactly what each instruction read and wrote.

It is split into two parts with very different confidence levels, both
documented at the top of `psm.py`:

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
  emulation, and `jand` jumps on a zero AND, `jnand` on a nonzero one, as
  OpenFWWF (which runs on the hardware) uses them. The other jump
  conditions, `orx`/`srx` with other parameters, and `jext`/`jnext` (which
  test an implicit hardware flag the disassembler itself never needed to
  resolve) are best-effort or outright stubs. Every instruction the interpreter executes is tagged with its
  confidence level in the trace, so unverified territory is visible rather
  than silently assumed correct.


Usage
-----

    psm.py <ucode.bin> [start_address_hex] [max_steps]

Example (D6220's version-stamp block, see Validation):

    ./psm.py d11ucode42.bin 0xF76 20

As a library:

```python
from psm import load_program, PSM

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
since early hardware-ready polling loops (`jext`/`jnext`, unimplemented
here for lack of a public signal map) never resolve. This is reported
explicitly rather than spinning silently or producing a misleading result;
see Validation and Known limitations below for how to work around it.


Validation
----------

Decoder, checked against `b43-dasm -a 15 -f raw-be32`'s own output,
instruction by instruction, on D6220's 5976-instruction `d11ucode42`:

    jump-target address mismatches:  0 / 5976
    mnemonic mismatches:             0 / 5976, beyond 5 opcodes (0x002,
                                      0x070, 0x071, the 0x000 end-of-code
                                      marker) that b43-dasm has no
                                      mnemonic for either -- a shared gap,
                                      not a disagreement between the two

Executor, checked against the known version-stamp prologue (the block
that writes `UCODEREV`/`UCODEPATCH`/... to SHM, independently confirmed
against a real MMIO capture earlier in this port's reverse-engineering
work -- see `../docs/` in the AC-PHY port repository this tool supports).
Running from that block's entry on all four available corerev 42 blobs:

    board          start   SHM[0] (UCODEREV)   SHM[1] (UCODEPATCH)
    D6220          0xF76   0x3A0  (expected)    0x2715 (expected)
    TG789vac v2    0xF76   0x3A0  (expected)    0x05DE (expected)
    vd625          0xF77   0x3A0  (expected)    0x04B1 (expected)
    DSL-3580_EU    0xCED   0x310  (expected)    0x0002 (expected)

4/4 exact matches, including DSL-3580_EU's unrelated 0x310 ucode family --
not a value tuned to pass, the interpreter has no per-file special-casing.


Known limitations
------------------

On its own, `psm.py` run from the real entry point (PC 0) stops at the first
wait on the hardware: the PHY register interface (SPR 0x018/0x019) and the
SPR 0x078 handshake of the boot, then the main loop's external conditions.
`lockstep.py` runs the whole program against a host op stream, with those two
handshakes modelled and COND_MACEN taken from MACCONTROL; from PC 0 it boots
both the 0x3A0 and the 0x310 builds to their main loop. Every other
`jext`/`jnext` condition stays false.

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

`unk_002` (opcode 2 of the control group, beside `nap` 1, `calls` 4 and
`rets` 5) has no operands and no known meaning; it is executed as a no-op.

TKIP acceleration (`tkiph`/`tkiphs`/`tkipl`/`tkipls`) is not implemented.


Companion tools
---------------

`lockstep.py` co-simulates a host op stream (`test/integration`'s trace, a
vendor capture, or `reverse-tools/mmio2ops.py` output) with the ucode as one
machine: the PSM starts at PC 0 on PSM_RUN and runs `--cycles` instructions
after every host operation, over shared memory and an IHR register file the
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

Condition register 4 follows SPR_BRC bit by bit, as OpenFWWF documents it.
`--tx-engine IFS,START,DONE` adds the TX engine model described below, with
its delays in PSM instructions.

The TX engine model starts a frame when the ucode writes SPR 0x320
(SPR_TX_Serial_Control) with bit 15 set: 0x8001 when it sends a queued frame,
0x8000 when it sends the beacon (0x0462 on the 0x310 build).

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
to the next TBTT.

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
`COND_TRUE=0x7F` carry over to the arch15 cores, but a given FIXME bit is a lead
for a newer core, not a guarantee.

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

The input op stream is the decoded-MMIO vocabulary of the b43 AC-PHY port's
`reverse-tools/mmio2ops.py`. The boot hand-off runs the validated version-stamp;
per-command hand-offs run the main loop, but a command the ucode picks up by
reading the MAC command register dispatches through IHR reads this interpreter
does not model, so those handlers do not run (the command is recorded, and
whatever the host routed through shared memory for it is still applied).
