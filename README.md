# Tools for the Broadcom 43xx series WLAN chip

> **Warning: `jand` and `jnand` are swapped with respect to upstream b43-tools.**
> In this fork `jand` is opcode 0x041 and jumps if `A & B` is nonzero, `jnand`
> is opcode 0x040 and jumps if it is zero, as measured on the hardware
> (core revisions 22 and 42). Upstream names them the other way round, so
> sources written for upstream `b43-asm`, such as OpenFWWF, must have every
> `jand` and `jnand` swapped before being assembled with this fork, and
> `b43-dasm` output differs in the same way.
