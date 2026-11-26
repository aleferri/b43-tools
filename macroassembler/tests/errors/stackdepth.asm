; arch: 15
; expect: call chains need 2 return stack entries, more than 1
	.stackdepth 1
a	PROC
	CALL b
	RET
a	ENDP
b	PROC
	nap
	RET
b	ENDP
	CALL a
