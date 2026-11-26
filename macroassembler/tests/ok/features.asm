	INCLUDE regs.inc
	INCLUDE "regs.inc"

vars	SEGMENT GPR AT 32
cnt	DW ?
tmp	DW ?
pair	DD ?
vars	ENDS

state	SEGMENT SHM AT 0x100 INIT b0g0initvals5
tx	TxHdr <<2, 0, 0, 0x1B>, 24>
magic	DW 0xBEEF
table	DW 1, 2, 3 DUP (7), ?
state	ENDS

LOADW	MACRO dst:REQ, val:=<0>
	LOCAL again
again:	mov val, dst
	jne dst, val, again
	ENDM

	jmp	main

wr	PROC LINK lr1 USES r40 off1, IN addr:r33, IN val:r34, OUT old:r35
	mov	addr, off1
	mov	[0, off1], old
	or	val, 0, r40
	RET
wr	ENDP

top2	PROC USES r33..r35 r40 off1 tmp
	LOCAL acc:r41
	INVOKE	wr, cnt, 5, tmp
	mov	tmp, acc
	@@:	sub	acc, 1, acc
	jne	acc, 0, @B
	or	acc, 0, acc
	RET
top2	ENDP

main:
	LOADW	cnt, SIZEOF TxHdr
	LOADW	r3
	mov	OFFSET tx.len, r1
	mov	tx.len, r2
	mov	[TxHdr.fcs, off1], r4
	mov	[off2 + TxHdr.ra + 1], r5
	srx	WIDTH TxCtl.rate - 1, TxCtl.rate, tx.ctl, 0, r6
	mov	MASK TxCtl.ack, r7
	CALL	top2
	IF @Arch EQ 5
	mov	1, r8
	ELSE
	mov	2, r8
	ENDIF
	REPT 2
	add	r9, 1, r9
	ENDM
	FOR reg, <r10, r11>
	mov	0, reg
	ENDM
	jmp	main
	END
