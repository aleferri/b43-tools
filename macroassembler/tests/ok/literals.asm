SHIFTED MACRO reg, bit
	EXITM <((bit) | ((reg) << 4))>
	ENDM
COND_A	EQU SHIFTED(7, 15)
	.ERRNZ COND_A - 0x7F, <shift inside a text literal>
COND_A	EQU 0x7F
R	RECORD hi:8, lo:8
S	STRUC
r	R <>
w	DW 2 DUP (?)
S	ENDS
d	SEGMENT SHM AT 0x10 INIT iv
x	S <<1, 2>, <3, 4>>
d	ENDS
	mov	x.w + 1, r1
