; arch: 15
; expect: calling q clobbers r7, not in the contract of p
q	PROC USES r7
	mov 1, r7
	RET
q	ENDP
p	PROC
	CALL q
	RET
p	ENDP
