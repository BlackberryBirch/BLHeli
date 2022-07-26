
;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Find throttle gains
;
; No assumptions
;
; Finds throttle gains for both directions in bidirectional mode
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
find_throttle_gains:
	; Check if full range is chosen
	jnb	Flags2.RCP_FULL_RANGE, find_throttle_gains_normal

	mov	Temp3, #0		; Min throttle
	mov	Temp4, #0
	mov	Temp5, #255	; Max throttle
	mov	Temp6, #0
	mov	Temp7, #0		; Deadband
	call	find_throttle_gain
	mov	Throttle_Gain_M, Temp4
	mov	Throttle_Gain, Temp3
	ret

find_throttle_gains_normal:
	; Check if bidirectional operation
	jnb	Flags3.PGM_BIDIR, find_throttle_gains_bidir_done

	mov	Temp1, #Pgm_Min_Throttle
	mov	A, @Temp1
	mov	Temp3, A
	mov	Temp4, #0
	mov	Temp1, #Pgm_Center_Throttle
	mov	A, @Temp1
	mov	Temp5, A
	mov	Temp6, #0
	clr	C
	mov	A, Temp3			; Scale gains in bidirectional
	rlc	A
	mov	Temp3, A
	mov	A, Temp4
	rlc	A
	mov	Temp4, A
	clr	C
	mov	A, Temp5
	rlc	A
	mov	Temp5, A
	mov	A, Temp6
	rlc	A
	mov	Temp6, A
	mov	Temp7, #10		; Compensate for deadband in bidirectional
	call	find_throttle_gain
	mov	Throttle_Gain_BD_Rev_M, Temp4
	mov	Throttle_Gain_BD_Rev, Temp3

find_throttle_gains_bidir_done:
	mov	Temp1, #Pgm_Min_Throttle
	jnb	Flags3.PGM_BIDIR, ($+5)

	mov	Temp1, #Pgm_Center_Throttle

	mov	A, @Temp1
	mov	Temp3, A
	mov	Temp4, #0
	mov	Temp1, #Pgm_Max_Throttle
	mov	A, @Temp1
	mov	Temp5, A
	mov	Temp6, #0
	mov	Temp7, #0			; No deadband
	jnb	Flags3.PGM_BIDIR, find_throttle_gain_fwd

	clr	C
	mov	A, Temp3			; Scale gains in bidirectional
	rlc	A
	mov	Temp3, A
	mov	A, Temp4
	rlc	A
	mov	Temp4, A
	clr	C
	mov	A, Temp5
	rlc	A
	mov	Temp5, A
	mov	A, Temp6
	rlc	A
	mov	Temp6, A
	mov	Temp7, #10		; Compensate for deadband in bidirectional

find_throttle_gain_fwd:
	call	find_throttle_gain
	mov	Throttle_Gain_M, Temp4
	mov	Throttle_Gain, Temp3
	ret


;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Find throttle gain
;
; The difference between max and min throttle must be more than 140us (a Pgm_xxx_Throttle difference of 35)
; Temp4/3 holds min throttle, Temp6/5 holds max throttle, Temp7 holds deadband, Temp4/Temp3 gives resulting gain
;
; Finds throttle gain from throttle calibration values
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
find_throttle_gain:
	; Subtract deadband from max
	clr	C
	mov	A, Temp5
	subb	A, Temp7
	mov	Temp5, A
	mov	A, Temp6
	subb	A, #0
	mov	Temp6, A
	; Calculate difference
	clr	C
	mov	A, Temp5
	subb	A, Temp3
	mov	Temp5, A
	mov	A, Temp6
	subb	A, Temp4
	mov	Temp6, A
	; Check that difference is minimum 35
	clr	C
	mov	A, Temp5
	subb	A, #35
	mov	A, Temp6
	subb	A, #0
	jnc	($+6)

	mov	Temp5, #35
	mov	Temp6, #0

	; Check that difference is maximum 511
	clr	C
	mov	A, Temp5
	subb	A, #255
	mov	A, Temp6
	subb	A, #1
	jc	($+6)

	mov	Temp5, #255
	mov	Temp6, #1

	; Find gain
	mov	Temp4, #0FFh
find_throttle_gain_loop:
	inc	Temp4
	mov	Temp3, #0
test_throttle_gain:
	inc	Temp3
	mov	A, Temp3
	jnz	test_throttle_gain_mult

	clr	C
	mov	A, Temp5			; Set multiplier x2 and range /2
	rlc	A
	mov	Temp5, A
	mov	A, Temp6
	rlc	A
	mov	Temp6, A
	ajmp	find_throttle_gain_loop

test_throttle_gain_mult:
	mov	A, Temp5			; A has difference, B has gain
	mov	B, Temp3
	mul	AB
	mov	Temp7, B
	mov	A, Temp6
	mov	B, Temp3
	mul	AB
	add	A, Temp7
	subb	A, #124
	jc	test_throttle_gain

	mov	A, Temp3
	cpl	A
	jz	find_throttle_gain_loop

	ret


;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Average throttle 
;
; Outputs result in Temp8
;
; Averages throttle calibration readings
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
average_throttle:
	setb	Flags2.RCP_FULL_RANGE	; Set range to 1000-2020us
	call	find_throttle_gains	; Set throttle gains
	call wait30ms		
	call wait30ms
	mov	Temp3, #0
	mov	Temp4, #0
	mov	Temp5, #16		; Average 16 measurments
average_throttle_meas:
	call	wait3ms			; Wait for new RC pulse value
	mov	A, New_Rcp		; Get new RC pulse value
	add	A, Temp3
	mov	Temp3, A
	mov	A, #0
	addc A, Temp4
	mov	Temp4, A
	djnz	Temp5, average_throttle_meas

	mov	Temp5, #4			; Shift 4 times
average_throttle_div:
	clr	C
	mov	A, Temp4   		; Shift right 
	rrc	A      
	mov	Temp4, A   
	mov	A, Temp3   
	rrc	A      
	mov	Temp3, A   
	djnz	Temp5, average_throttle_div

	mov	Temp8, A   		; Copy to Temp8
	mov	A, Temp4
	jz	($+4)

	mov	Temp8, #0FFh

	clr	Flags2.RCP_FULL_RANGE
	call	find_throttle_gains	; Set throttle gains
	ret
