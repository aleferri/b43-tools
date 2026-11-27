; arch: 5
; expect: a macro's LOCAL must come before any other line of its body
M	MACRO
IF 1
	LOCAL l
l:	nap
ENDIF
	ENDM
	M
