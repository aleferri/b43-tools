; arch: 5
; expect: argument 2 reads a register already loaded
q	PROC, IN a:r1, IN b:r2
	RET
q	ENDP
	INVOKE q, r2, r1
