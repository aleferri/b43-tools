BIT	MACRO n
	EXITM <(1 SHL (n))>
	ENDM
SUM	MACRO first, rest:VARARG
	LOCAL acc
	acc = first
	FOR x, <rest>
	acc = acc + x
	ENDM
	EXITM %acc
	ENDM
CLR	MACRO regs:VARARG
	FOR r, <regs>
	IFIDNI <r>, <r0>
	.ERR <r0 is reserved>
	ENDIF
	mov 0, r
	ENDM
	ENDM
NAME	TEXTEQU <r7>
	mov BIT(3), NAME
	mov SUM(1, 2, 3, 4), r8
i = 0
	WHILE i LT 3
	add r9, i, r9
i = i + 1
	ENDM
	FORC c, <ab>
	mov '&c', r10
	ENDM
	CLR r1, r2
