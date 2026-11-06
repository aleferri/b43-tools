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
