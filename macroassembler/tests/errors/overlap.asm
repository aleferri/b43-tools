; arch: 5
; expect: segment s2 overlaps s1
s1	SEGMENT SHM AT 0x10
	DW 4 DUP (?)
s1	ENDS
s2	SEGMENT SHM AT 0x12
	DW ?
s2	ENDS
