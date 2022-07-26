
;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Wait xms ~(x*4*250)  (Different entry points)	
;
; No assumptions
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
wait1ms:	
	mov	Temp2, #1
	jmp	waitxms_o

wait3ms:	
	mov	Temp2, #3
	jmp	waitxms_o

wait10ms:	
	mov	Temp2, #10
	jmp	waitxms_o

wait30ms:	
	mov	Temp2, #30
	jmp	waitxms_o

wait100ms:	
	mov	Temp2, #100
	jmp	waitxms_o

wait200ms:	
	mov	Temp2, #200
	jmp	waitxms_o

waitxms_o:	; Outer loop
	mov	Temp1, #23
waitxms_m:	; Middle loop
	clr	A
 	djnz	ACC, $	; Inner loop (42.7us - 1024 cycles)
	djnz	Temp1, waitxms_m
	djnz	Temp2, waitxms_o
	ret
    