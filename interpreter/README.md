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
  `xor`/`sr`/`sl`/`rl`/`rr`/`nand`/`mul`) and `orx` with immediate operands
  are high-confidence (the source operand convention is structural in the
  original decoder, not guessed from a single example, and the `orx`
  immediate-concatenation case is confirmed against a real MMIO capture --
  see Validation). Jump conditions, the extended `orx`/`srx`/`jzx`/`jnzx`
  mode bits, and `jext`/`jnext` (which test an implicit hardware flag the
  disassembler itself never needed to resolve) are best-effort or outright
  stubs. Every instruction the interpreter executes is tagged with its
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

Running from the real entry point (PC 0) does not reach the version-stamp
block on its own: it stalls after roughly 20000 steps, PC stuck below
0x147C, in an early hardware-ready wait loop built on `jext`/`jnext`
(`HardwareWaitLoop` is raised rather than spinning or silently returning a
wrong state). This is not a bug being papered over -- it is the direct
consequence of `jext`/`jnext` having no known signal mapping (see the
confidence-level notes in `psm.py`), and is unlikely to be fixable without
either hardware documentation or a register trace covering those exact
flags.

Practical workaround: start execution at the address of interest directly,
or pre-seed `PSM.ext_flags[imm]` / `PSM.spr[n]` with known-good values
before running through an earlier wait loop. There is no way to make the
whole program run unattended from reset without that information.

TKIP acceleration (`tkiph`/`tkiphs`/`tkipl`/`tkipls`) is not implemented.
