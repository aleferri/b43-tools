b43-masm
========

A MASM-style macro assembler for the Broadcom D11 PSM (b43 microcode), for
both instruction set generations: arch 5 (core revisions 5-14) and arch 15
(core revision 15 and later). It is plain C99 with no dependencies.

Instruction encoding is identical to `b43-asm`. On top of it come symbols with
scopes, STRUC/UNION/RECORD layouts, segments that lay out code, shared memory
and registers, INCLUDE, MASM macros and conditional assembly, and procedures
with a register contract the assembler checks.

    b43-masm INPUT OUTPUT [OPTIONS]

    -a|--arch N          Target architecture (5 or 15)
    -r|--corerev N       Target core revision (sets the architecture)
    -f|--format FMT      b43 (default), raw-le32, raw-be32
    -e|--ivext EXT       Initvals file name extension (default .initvals)
    -I DIR               Include directory
    -D NAME[=VALUE]      Define a symbol
    -m|--map FILE        Segment, procedure and symbol map
    -l|--listing FILE    Listing with addresses and encoded words
    -W|--werror          Treat warnings as errors
    -s|--psize           Print output sizes

The target can also be set in the source with `.arch 5|15` or
`.corerev N`; `@Arch` and `@CoreRev` hold it. The same source assembles for
both architectures when it uses `CALL`, `RET` and `INVOKE` instead of
`call`/`calls`/`ret`/`rets`.

Initvals files are written next to OUTPUT, one per initvals section.

`make check` runs the tests: error cases, a byte-for-byte comparison with
`b43-asm` on random programs for both architectures (when `../assembler` is
built), and programs executed on the PSM interpreter in `../interpreter`.


Lexical conventions
-------------------

Comments start with `;` or `//`, or are enclosed in `/* */`. A line ending in
`\` continues on the next. Keywords, directives, mnemonics and register
names are case-insensitive; user symbols are case-sensitive.

Numbers: `42`, `0x2A`, `2Ah`, `0b101010`, `101010b`. Character constants:
`'A'`, `'AB'`. Registers: `r0`..`r63`, `spr000`..`spr3FF`, `off0`..`off6`,
`lr0`..`lr3`, or computed with `GPR(n)`, `SPR(n)`, `OFFR(n)`.


Expressions
-----------

C precedence, 64-bit arithmetic, comparisons yield 0 or 1.

    || && | ^ & == != < <= > >= << >> + - * / % ! ~ unary-
    OR XOR AND EQ NE LT LE GT GE SHL SHR MOD NOT      (MASM spellings)

    $                 current address
    @B, @F            previous / next @@: label
    SIZEOF x          size in words (also SIZEOF segment)
    LENGTHOF x        number of elements
    TYPE x            size of one element
    WIDTH f, MASK f   record field width and in-place mask
    OFFSET x          address of a variable as a plain number
    LOW, HIGH         bits 0-7 / 8-15
    LOWWORD, HIGHWORD bits 0-15 / 16-31
    DEFINED(name)     1 if name is defined at this point

Registers are values: `cnt EQU r12` works, and a register plus or minus a
number is another register. A symbol that names shared memory data is a
memory operand, as in MASM: `mov counter, r1` reads the word, `OFFSET
counter` is its address.


Operands
--------

    r5  spr06c  off2        registers
    42                      immediate (10 bits sign-extended on arch 5, 11 on arch 15)
    [0x120], counter        direct SHM word (address up to 0x7FF / 0xFFF)
    [off1 + Hdr.len]        indirect: offset register plus 0..0x3F / 0..0x7F
    [3, off1]               the same in b43-asm syntax
    RAW(0xC11)              a raw operand field

Out-of-range values are errors; nothing is truncated. `mov` of a 16-bit
value, including a code address, uses `orx` when needed, and `jand`/`jnand`
with a long contiguous mask become `jnzx`/`jzx`, exactly as in `b43-asm`.
`INSN opcode, A, B, D` emits an arbitrary instruction.


Symbols and scopes
------------------

    name:              label in the current segment
    name::             label that stays global inside a PROC
    @@:                anonymous label, referenced with @B and @F
    name EQU expr      constant (or text, if it is not an expression)
    name EQU <text>    text macro
    name = expr        redefinable constant
    name TEXTEQU <t>   text macro (also %expr or another text macro)

Inside a PROC every label, EQU, `=`, TEXTEQU, parameter and LOCAL is local to
it unless declared with `::`. Forward references to labels and constants work
anywhere; text macros, macros and types must be defined before use.


Layouts
-------

Sizes and offsets are in 16-bit words, the unit of the PSM's shared memory.

    TxCtl   RECORD prio:3, ack:1=1, rsvd:4, rate:8      ; first field is the MSB

    TxHdr   STRUC
    ctl     TxCtl <>
    len     DW ?
    ra      DW 3 DUP (?)
            UNION
    seq     DW ?
    frag    DW ?
            ENDS
    fcs     DD 0                                         ; low word first
    TxHdr   ENDS

`TxHdr.len` is the field offset, `tx.len` the address of the field of a
variable, `TxCtl.rate` the field's shift. Field defaults come from the
declaration; instances override them positionally:
`tx TxHdr <<2, 0, 0, 0x1B>, 24>`. Initializers are `?`, expressions,
`n DUP (...)` and `<...>` for records and structures.

    srx   WIDTH TxCtl.rate - 1, TxCtl.rate, tx.ctl, 0, r6
    mov   [off1 + TxHdr.fcs], r4


Memory organization
-------------------

    name SEGMENT CODE|SHM|GPR [AT addr] [ALIGN n] [INIT ivname]
    ...
    name ENDS
    .code [name]       switch the base code segment (default _TEXT)
    ORG offset         offset inside the current segment
    ALIGN n, EVEN

Segments of a space without AT are placed one after another in order of
first appearance; AT fixes the base. Reopening a segment continues it.
Overlaps and the size of each space are checked; `.memory CODE|SHM|GPR, n`
sets a limit (defaults: the jump range for code, 64 registers).

A GPR segment allocates registers by name: `cnt DW ?` makes `cnt` a
register, and a STRUC instance there is a group of named registers.

Initialized data is only allowed in SHM segments with `INIT name`; it ends up
in that initvals section as host SHM writes (two aligned words become one
32-bit write). Explicit writes go in an INITVALS block, with the argument
order of `b43-asm`:

    b0g0initvals5 INITVALS
            mmio16  value, offset
            mmio32  value, offset
            phy     value, offset
            radio   value, offset
            shm16   value, routing, offset
            shm32   value, routing, offset
            tram    value, offset
    b0g0initvals5 ENDS

Explicit writes come first in the file, then segment data by address.


Procedures and contracts
------------------------

    name PROC [LINK lrN] [USES regs | USES ALL] [, [IN|OUT|INOUT] p:reg ...]
            LOCAL t:reg [, ...]
            ...
            RET
    name ENDP

    CALL name
    INVOKE name, arg, ...

The contract of a PROC is the set of GPRs and offset registers it may write:
its parameters, its LOCALs and USES (`r10..r15` ranges allowed). The
assembler checks that

- every instruction in the body writes only registers in the contract;
- every callee's contract is contained in the caller's;
- on arch 5, a call does not reuse a link register that holds the caller's
  return address anywhere down the call chain, and a tail jump goes to a
  PROC with the same link register;
- there is no recursion;
- on arch 15, the deepest call chain fits `.stackdepth n` if given.

`USES ALL` opts out of the contract: such a PROC is unchecked and cannot be
called from a PROC that has one. Writes to SPRs and memory are not tracked.

`LINK lrN` (default lr0) only matters on arch 5. `CALL` becomes
`call lrN, name` or `calls name`, `RET` becomes `ret lrN, lrN` or `rets`.

`INVOKE` loads IN/INOUT arguments into the parameter registers, calls, then
copies OUT/INOUT results into the given destinations, skipping moves to the
same register; it reports an argument that reads a register an earlier
argument already overwrote. Missing trailing OUT arguments are allowed.


Macros
------

    name MACRO p1, p2:REQ, p3:=<default>, rest:VARARG
            LOCAL l1, l2
            ...
            EXITM [<text> | %expr]
            ENDM
    REPT n ... ENDM
    FOR p, <a, b, c> ... ENDM        (IRP)
    FORC p, <text> ... ENDM          (IRPC)
    WHILE expr ... ENDM
    PURGE name

A macro used as `name(args)` inside a line is a macro function and is
replaced by its EXITM text. Arguments in `<...>` are taken literally, `%expr`
passes the value. In the body `&` joins a parameter to adjacent text.

Conditional assembly: `IF`, `IFE`, `IFDEF`, `IFNDEF`, `IFB`, `IFNB`,
`IFIDN[I]`, `IFDIF[I]`, their `ELSEIF...` forms, `ELSE`, `ENDIF`.
Diagnostics: `ECHO`, `.ERR`, `.ERRE`, `.ERRNZ`, `.ERRDEF`, `.ERRNDEF`.
`INCLUDE file` searches the including file's directory, then `-I`.
`END [entry]` stops assembly and checks that the entry point is at 0.


Assembly passes
---------------

The source is assembled repeatedly until labels, segment sizes and placement
and procedure contracts stop changing (usually two passes); only the
diagnostics of the last pass are printed.


Differences from b43-asm
------------------------

There is no C preprocessor: use EQU, macros and IF. `;` starts a comment
instead of separating statements. `%arch` and `%start` become `.arch` and
code placed at address 0; `%assert` becomes `.ERRE`. Raw instructions are
written `INSN 0x1C0, RAW(0xC11), RAW(0xC22), r3`.
