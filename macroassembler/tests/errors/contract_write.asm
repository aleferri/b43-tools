; arch: 5
; expect: PROC p writes r2, which its contract does not declare
p	PROC USES r1
	mov 1, r2
	RET
p	ENDP
