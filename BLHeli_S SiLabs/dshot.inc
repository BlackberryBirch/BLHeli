
;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Timer 1 interrupt routine
;
; No assumptions
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
t1_int:
	clr 	IE_EA
	clr	IE_EX0			; Disable int0 interrupts
	anl	EIE1, #0EFh		; Disable pca interrupts
	clr	TCON_TR1			; Stop timer 1
	mov	TL1, DShot_Timer_Preset	; Reset sync timer
	push	PSW
	setb	PSW.3			; Select register bank 1 for this interrupt
	push	ACC
	push	B				; Will be pop'ed by int0 exit
	clr	TMR2CN0_TR2		; Timer 2 disabled
	mov	Temp1, TMR2L		; Read timer value
	mov	Temp2, TMR2H
	setb	TMR2CN0_TR2		; Timer 2 enabled
	setb	IE_EA
	; Reset timer 0
	mov	TL0, #0
	; Check frame time length
	clr	C
	mov	A, Temp1
	subb	A, DShot_Frame_Start_L
	mov	Temp1, A
	mov	A, Temp2
	subb	A, DShot_Frame_Start_H
	mov	Temp2, A
	; Divide by 2 (or 4 for 48MHz). Unit is then us
	clr	C
	mov	A, Temp2
	rrc	A
	mov	Temp2, A
	mov	A, Temp1
	rrc	A
	mov	Temp1, A
	mov	A, Clock_Set_At_48MHz
	jz	t1_int_frame_time_scaled

	clr	C
	mov	A, Temp2
	rrc	A
	mov	Temp2, A
	mov	A, Temp1
	rrc	A
	mov	Temp1, A

t1_int_frame_time_scaled:
	mov	A, Temp2
	jnz	t1_int_msb_fail	; Frame too long
	mov	A, Temp1
	subb	A, DShot_Frame_Length_Thr
	jc	t1_int_msb_fail	; Frame too short
	subb	A, DShot_Frame_Length_Thr
	jnc	t1_int_msb_fail	; Frame too long

	; Check that correct number of pulses is received
	mov	A, DPL			; Read current pointer
	cjne	A, #16, t1_int_msb_fail

	; Decode transmitted data
	mov	Temp5, #0			; Reset timestamp
	mov	Temp4, #0			; High byte of receive buffer
	mov	Temp3, #0			; Low byte of receive buffer
	mov	Temp2, #8			; Number of bits per byte
	mov	DPTR, #0			; Set pointer
	mov	Temp1, DShot_Pwm_Thr; DShot pulse width criteria
	mov	A, Clock_Set_At_48MHz
	jnz	t1_int_decode

	clr	C
	mov	A, Temp1			; Scale pulse width criteria
	rrc	A
	mov	Temp1, A

t1_int_decode:
	ajmp	t1_int_decode_msb

t1_int_msb_fail:
	mov	DPTR, #0		 	; Set pointer to start
	setb	IE_EX0			; Enable int0 interrupts
	setb	IE_EX1			; Enable int1 interrupts
	ajmp int0_int_outside_range

t1_int_decode_msb:
	; Decode DShot data Msb. Use more code space to save time (by not using loop)
	Decode_DShot_2Msb
	Decode_DShot_2Msb
	Decode_DShot_2Msb
	Decode_DShot_2Msb
	ajmp	t1_int_decode_lsb

t1_int_lsb_fail:
	mov	DPTR, #0		 	; Set pointer to start
	setb	IE_EX0			; Enable int0 interrupts
	setb	IE_EX1			; Enable int1 interrupts
	ajmp int0_int_outside_range

t1_int_decode_lsb:
	; Decode DShot data Lsb
	Decode_DShot_2Lsb
	Decode_DShot_2Lsb
	Decode_DShot_2Lsb
	Decode_DShot_2Lsb
	; XOR check (in inverted data, which is ok)
	mov	A, Temp4
	swap	A
	xrl	A, Temp4
	xrl	A, Temp3
	anl	A, #0F0h
	mov	Temp2, A
	mov	A, Temp3
	swap	A
	anl	A, #0F0h
	clr	C
	subb	A, Temp2
	jz	t1_int_xor_ok		; XOR check

	mov	DPTR, #0		 	; Set pointer to start
	setb	IE_EX0			; Enable int0 interrupts
	setb	IE_EX1			; Enable int1 interrupts
	ajmp int0_int_outside_range

t1_int_xor_ok:
	; Swap to be LSB aligned to 12 bits (and invert)
	mov	A, Temp4
	cpl	A
	swap A
	anl	A, #0F0h			; Low nibble of high byte
	mov	Temp2, A
	mov	A, Temp3
	cpl	A
	swap	A
	anl	A, #0Fh			; High nibble of low byte 
	orl	A, Temp2
	mov	Temp3, A
	mov	A, Temp4			; High nibble of high byte
	cpl	A
	swap A
	anl	A, #0Fh
	mov	Temp4, A
	; Subtract 96 (still 12 bits)
	clr	C
	mov	A, Temp3
	mov	Temp2, A
	subb	A, #96
	mov	Temp3, A
	mov	A, Temp4
	subb	A, #0
	mov	Temp4, A
	jnc 	t1_normal_range

	clr	C	
	mov	A, Temp2  		; Check for 0 or dshot command
	mov	Temp4, #0
	mov	Temp3, #0
	mov	Temp2, #0
	jz 	t1_normal_range
	
	clr	C				; We are in the special dshot range
	rrc	A 				; Divide by 2
	jnc 	t1_dshot_set_cmd 	; Check for tlm bit set (if not telemetry, Temp2 will be zero and result in invalid command)

	mov 	Temp2, A
	clr	C
	subb A, Dshot_Cmd
	jz 	t1_dshot_inc_cmd_cnt

t1_dshot_set_cmd:
	mov 	A, Temp2
	mov	Dshot_Cmd, A
	mov	Dshot_Cmd_Cnt, #0
	mov	Temp2, #0
	jmp 	t1_normal_range
	
t1_dshot_inc_cmd_cnt:
	inc 	Dshot_Cmd_Cnt
	
t1_normal_range:
	; Check for bidirectional operation (0=stop, 96-2095->fwd, 2096-4095->rev)
	jnb	Flags3.PGM_BIDIR, t1_int_not_bidir	; If not bidirectional operation - branch

	; Subtract 2000 (still 12 bits)
	clr	C
	mov	A, Temp3
	subb	A, #0D0h
	mov	Temp1, A
	mov	A, Temp4
	subb	A, #07h
	mov	Temp2, A
	jc	t1_int_bidir_fwd				; If result is negative - branch

	mov	A, Temp1
	mov	Temp3, A
	mov	A, Temp2
	mov	Temp4, A
	jb	Flags2.RCP_DIR_REV, t1_int_bidir_rev_chk	; If same direction - branch

	setb	Flags2.RCP_DIR_REV
	ajmp	t1_int_bidir_rev_chk

t1_int_bidir_fwd:
	jnb	Flags2.RCP_DIR_REV, t1_int_bidir_rev_chk	; If same direction - branch

	clr	Flags2.RCP_DIR_REV

t1_int_bidir_rev_chk:
	jb	Flags3.PGM_BIDIR_REV, ($+5)

	cpl	Flags2.RCP_DIR_REV

	clr	C							; Multiply throttle value by 2
	mov	A, Temp3
	rlc	A
	mov	Temp3, A
	mov	A, Temp4
	rlc	A
	mov	Temp4, A
t1_int_not_bidir:
	; Generate 4/256
	mov	A, Temp4
	add	A, Temp4
	addc	A, Temp4
	addc	A, Temp4
	mov	Temp2, A
	; Align to 11 bits
	clr	C
	mov	A, Temp4
	rrc	A
	mov	Temp4, A
	mov	A, Temp3
	rrc	A
	mov	Temp3, A
	; Scale from 2000 to 2048
	mov	A, Temp3
	add	A, Temp2	; Holds 4/128
	mov	Temp3, A
	mov	A, Temp4
	addc	A, #0
	mov	Temp4, A
	jnb	ACC.3, ($+7)

	mov	Temp3, #0FFh
	mov	Temp4, #0FFh

	; Boost pwm during direct start
	mov	A, Flags1
	anl	A, #((1 SHL STARTUP_PHASE)+(1 SHL INITIAL_RUN_PHASE))
	jz	t1_int_startup_boosted

	jb	Flags1.MOTOR_STARTED, t1_int_startup_boosted	; Do not boost when changing direction in bidirectional mode

	mov	A, Pwm_Limit_Beg				; Set 25% of max startup power as minimum power
	rlc	A
	mov	Temp2, A
	mov	A, Temp4
	jnz	t1_int_startup_boost_stall

	clr	C
	mov	A, Temp2
	subb	A, Temp3
	jc	t1_int_startup_boost_stall

	mov	A, Temp2
	mov	Temp3, A

t1_int_startup_boost_stall:
	mov	A, Stall_Cnt					; Add an extra power boost during start
	swap	A
	rlc	A
	add	A, Temp3
	mov	Temp3, A
	mov	A, Temp4
	addc	A, #0
	mov	Temp4, A

t1_int_startup_boosted:
	; Set 8bit value
	clr	C
	mov	A, Temp3
	rlc	A
	swap	A
	anl	A, #0Fh
	mov	Temp1, A
	mov	A, Temp4
	rlc	A
	swap	A
	anl	A, #0F0h
	orl	A, Temp1
	mov	Temp1, A
	jnz	t1_int_zero_rcp_checked	; New_Rcp (Temp1) is only zero if all 11 bits are zero

	mov	A, Temp3
	jz	t1_int_zero_rcp_checked

	mov	Temp1, #1

t1_int_zero_rcp_checked:
	; Align to 10 bits for 24MHz MCU
IF MCU_48MHZ == 0
	clr	C
	mov	A, Temp4
	rrc	A
	mov	Temp4, A
	mov	A, Temp3
	rrc	A
	mov	Temp3, A
ENDIF
	mov	DPTR, #0		 			; Set pointer to start
	setb	IE_EX0					; Enable int0 interrupts
	setb	IE_EX1					; Enable int1 interrupts
	; Decrement outside range counter
	mov	A, Rcp_Outside_Range_Cnt
	jz	($+4)

	dec	Rcp_Outside_Range_Cnt

	ajmp	int0_int_pulse_ready

t1_int_frame_fail:
	mov	DPTR, #0		 			; Set pointer to start
	setb	IE_EX0					; Enable int0 interrupts
	setb	IE_EX1					; Enable int1 interrupts
	ajmp int0_int_outside_range

