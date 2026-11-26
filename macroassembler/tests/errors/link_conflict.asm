; arch: 5
; expect: overwrites lr0, which holds the return address of outer
	jmp m
inner	PROC LINK lr0
	RET
inner	ENDP
outer	PROC
	CALL inner
	RET
outer	ENDP
m:	CALL outer
