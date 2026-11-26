; expect-shm: 0x40=0x11 0x41=0x78 0x42=0xbeef
res	SEGMENT SHM AT 0x40
out	DW 4 DUP (?)
res	ENDS

	jmp	main

add3	PROC LINK lr1 USES r20, IN a:r10, IN b:r11, OUT s:r12
	add	a, b, r20
	add	r20, 3, s
	RET
add3	ENDP

twice	PROC USES r10..r12 r20, IN x:r21, OUT y:r13
	INVOKE	add3, x, x, y
	RET
twice	ENDP

main:
	INVOKE	twice, 7, r1
	mov	r1, out
	INVOKE	add3, r1, 100, r2
	mov	r2, out + 1
	mov	0xBEEF, [OFFSET out + 2]
done:	jmp	done
