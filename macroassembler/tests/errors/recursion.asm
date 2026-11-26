; arch: 15
; expect: recursive call chain: a -> b -> a
a	PROC
	CALL b
	RET
a	ENDP
b	PROC
	CALL a
	RET
b	ENDP
