; arch: 5
; expect: does not fit the 3-bit field prio
R	RECORD prio:3, rest:13
s	SEGMENT SHM AT 0 INIT iv
x	R <9>
s	ENDS
