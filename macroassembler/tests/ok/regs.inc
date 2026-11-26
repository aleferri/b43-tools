IFNDEF REGS_INC
REGS_INC EQU 1
SPR_PSM_COND	EQU spr06c
TxCtl	RECORD	prio:3, ack:1=1, rsvd:4, rate:8
TxHdr	STRUC
	ctl	TxCtl <>
	len	DW ?
	ra	DW 3 DUP (?)
	UNION
		seq	DW ?
		frag	DW ?
	ENDS
	fcs	DD 0
TxHdr	ENDS
ENDIF
