
;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Int0 interrupt routine
;
; No assumptions
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
int0_int:	; Used for RC pulse timing
	push	ACC
	mov	A, TL0			; Read pwm for DShot immediately
	; Test for DShot
	jnb	Flags2.RCP_DSHOT, int0_int_not_dshot

	mov	TL1, DShot_Timer_Preset	; Reset sync timer
	movx	@DPTR, A			; Store pwm
	inc	DPTR
	pop	ACC
	reti

	; Not DShot
int0_int_not_dshot:
	pop	ACC
	clr	IE_EA
	anl	EIE1, #0EFh		; Disable pca interrupts
	push	PSW				; Preserve registers through interrupt
	push	ACC
	push	B
	setb	PSW.3			; Select register bank 1 for this interrupt 
	setb	IE_EA
	; Get the counter values
	Get_Rcp_Capture_Values
	; Scale down to 10 bits (for 24MHz, and 11 bits for 48MHz)
	jnb	Flags2.RCP_MULTISHOT, int0_int_fall_not_multishot

	; Multishot - Multiply by 2 and add 1/16 and 1/32
	mov	A, Temp1		; Divide by 16
	swap A
	anl	A, #0Fh
	mov	Temp3, A
	mov	A, Temp2
	swap	A
	anl	A, #0F0h
	orl	A, Temp3
	mov	Temp3, A
	clr	C			; Make divided by 32
	rrc	A
	add	A, Temp3		; Add 1/16 to 1/32
	mov	Temp3, A
	clr	C			; Multiply by 2
	mov	A, Temp1
	rlc	A
	mov	Temp1, A
	mov	A, Temp2
	rlc	A
	mov	Temp2, A
	mov	A, Temp1		; Add 1/16 and 1/32
	add	A, Temp3
	mov	Temp3, A
	mov	A, Temp2
IF MCU_48MHZ == 0
	addc	A, #03h		; Add to low end, to make signal look like 20-40us
ELSE
	addc	A, #06h
ENDIF
	mov	Temp4, A
	ajmp	int0_int_fall_gain_done

int0_int_fall_not_multishot:
	jnb	Flags2.RCP_ONESHOT42, int0_int_fall_not_oneshot_42

	; Oneshot42 - Add 2/256
	clr	C
	mov	A, Temp1
	rlc	A
	mov	A, Temp2
	rlc	A
	mov	Temp3, A
	mov	A, Temp1
	add	A, Temp3
	mov	Temp3, A
	mov	A, Temp2
	addc	A, #0
	mov	Temp4, A
	ajmp	int0_int_fall_gain_done

int0_int_fall_not_oneshot_42:
	jnb	Flags2.RCP_ONESHOT125, int0_int_fall_not_oneshot_125

	; Oneshot125 - multiply by 86/256
	mov	A, Temp1		; Multiply by 86 and divide by 256
	mov	B, #56h
	mul	AB
	mov	Temp3, B
	mov	A, Temp2
	mov	B, #56h
	mul	AB
	add	A, Temp3
	mov	Temp3, A
	xch	A, B
	addc	A, #0
	mov	Temp4, A
	ajmp	int0_int_fall_gain_done

int0_int_fall_not_oneshot_125:
	; Regular signal - multiply by 43/1024
IF MCU_48MHZ >= 1
	clr	C
	mov	A, Temp3		; Divide by 2
	rrc	A
	mov	Temp3, A
	mov	A, Temp2
	rrc	A
	mov	Temp2, A
	mov	A, Temp1
	rrc	A
	mov	Temp1, A
ENDIF
	mov	A, Temp1		; Multiply by 43 and divide by 1024
IF MCU_48MHZ == 0
	mov	B, #2Bh
ELSE
	mov	B, #56h		; Multiply by 86
ENDIF
	mul	AB
	mov	Temp3, B
	mov	A, Temp2
IF MCU_48MHZ == 0
	mov	B, #2Bh
ELSE
	mov	B, #56h		; Multiply by 86
ENDIF
	mul	AB
	add	A, Temp3
	mov	Temp3, A
	xch	A, B
	addc	A, #0
	clr	C	
	rrc	A			; Divide by 2 for total 512
	mov	Temp4, A
	mov	A, Temp3
	rrc	A
	mov	Temp3, A
	clr	C
	mov	A, Temp4		; Divide by 2 for total 1024
	rrc	A						
	mov	Temp4, A
	mov	A, Temp3
	rrc	A
	mov	Temp3, A

int0_int_fall_gain_done:
	; Check if 2235us or above (in order to ignore false pulses)
	clr	C
	mov	A, Temp4						; Is pulse 2235us or higher?
IF MCU_48MHZ == 0
	subb A, #09h
ELSE
	subb A, #12h
ENDIF
	jnc	int0_int_outside_range			; Yes - ignore pulse

	; Check if below 900us (in order to ignore false pulses)
	clr	C
	mov	A, Temp3
IF MCU_48MHZ == 0
	subb A, #9Ah
ELSE
	subb A, #34h
ENDIF
	mov	A, Temp4
IF MCU_48MHZ == 0
	subb A, #03h
ELSE
	subb A, #07h
ENDIF
	jnc	int0_int_check_full_range		; No - proceed

int0_int_outside_range:
	inc	Rcp_Outside_Range_Cnt
	mov	A, Rcp_Outside_Range_Cnt
	jnz	($+4)

	dec	Rcp_Outside_Range_Cnt

	clr	C
	mov	A, Rcp_Outside_Range_Cnt
	subb	A, #50						; Allow a given number of outside pulses
	jnc	($+4)
	ajmp	int0_int_set_timeout			; If outside limits - ignore first pulses

	mov	New_Rcp, #0					; Set pulse length to zero
	ajmp	int0_int_exit					; Exit without reseting timeout

int0_int_check_full_range:
	; Decrement outside range counter
	mov	A, Rcp_Outside_Range_Cnt
	jz	($+4)

	dec	Rcp_Outside_Range_Cnt

	; Calculate "1000us" plus throttle minimum
	jnb	Flags2.RCP_FULL_RANGE, int0_int_set_min	; Check if full range is chosen

	mov	Temp5, #0						; Set 1000us as default minimum
IF MCU_48MHZ == 0
	mov	Temp6, #4
ELSE
	mov	Temp6, #8
ENDIF
	ajmp	int0_int_calculate

int0_int_set_min:
	mov	Temp5, Min_Throttle_L			; Min throttle value scaled
	mov	Temp6, Min_Throttle_H
	jnb	Flags3.PGM_BIDIR, ($+7)

	mov	Temp5, Center_Throttle_L			; Center throttle value scaled
	mov	Temp6, Center_Throttle_H

int0_int_calculate:
	clr	C
	mov	A, Temp3						; Subtract minimum
	subb	A, Temp5
	mov	Temp3, A
	mov	A, Temp4					
	subb	A, Temp6
	mov	Temp4, A
	mov	Bit_Access_Int.0, C
	mov	Temp7, Throttle_Gain				; Load Temp7/Temp8 with throttle gain
	mov	Temp8, Throttle_Gain_M
	jnb	Flags3.PGM_BIDIR, int0_int_not_bidir	; If not bidirectional operation - branch

	jnc	int0_int_bidir_fwd					; If result is positive - branch

	jb	Flags2.RCP_DIR_REV, int0_int_bidir_rev_chk	; If same direction - branch

	setb	Flags2.RCP_DIR_REV
	ajmp	int0_int_bidir_rev_chk

int0_int_bidir_fwd:
	jnb	Flags2.RCP_DIR_REV, int0_int_bidir_rev_chk	; If same direction - branch

	clr	Flags2.RCP_DIR_REV

int0_int_bidir_rev_chk:
	jnb	Flags2.RCP_DIR_REV, ($+7)

	mov	Temp7, Throttle_Gain_BD_Rev		; Load Temp7/Temp8 with throttle gain for bidirectional reverse
	mov	Temp8, Throttle_Gain_BD_Rev_M

	jb	Flags3.PGM_BIDIR_REV, ($+5)

	cpl	Flags2.RCP_DIR_REV

	clr	C							; Multiply throttle value by 2
	mov	A, Temp3
	rlc	A
	mov	Temp3, A
	mov	A, Temp4
	rlc	A
	mov	Temp4, A
	mov	C, Bit_Access_Int.0
	jnc	int0_int_bidir_do_deadband		; If result is positive - branch

	mov	A, Temp3						; Change sign
	cpl	A
	add	A, #1
	mov	Temp3, A
	mov	A, Temp4
	cpl	A
	addc	A, #0
	mov	Temp4, A

int0_int_bidir_do_deadband:
	clr	C							; Subtract deadband
	mov	A, Temp3
IF MCU_48MHZ == 0
	subb	A, #40
ELSE
	subb	A, #80
ENDIF
	mov	Temp3, A
	mov	A, Temp4
	subb	A, #0
	mov	Temp4, A
	jnc	int0_int_do_throttle_gain

	mov	Temp1, #0
	mov	Temp3, #0
	mov	Temp4, #0
	ajmp	int0_int_do_throttle_gain

int0_int_not_bidir:
	mov	C, Bit_Access_Int.0
	jnc	int0_int_do_throttle_gain		; If result is positive - branch

int0_int_unidir_neg:
	mov	Temp1, #0						; Yes - set to minimum
	mov	Temp3, #0
	mov	Temp4, #0
	ajmp	int0_int_pulse_ready

int0_int_do_throttle_gain:
	; Boost pwm during direct start
	mov	A, Flags1
	anl	A, #((1 SHL STARTUP_PHASE)+(1 SHL INITIAL_RUN_PHASE))
	jz	int0_int_startup_boosted

	jb	Flags1.MOTOR_STARTED, int0_int_startup_boosted	; Do not boost when changing direction in bidirectional mode

	mov	A, Pwm_Limit_Beg				; Set 25% of max startup power as minimum power
IF MCU_48MHZ >= 1
	rlc	A
ENDIF
	mov	Temp2, A
	mov	A, Temp4
	jnz	int0_int_startup_boost_stall

	clr	C
	mov	A, Temp2
	subb	A, Temp3
	jc	int0_int_startup_boost_stall

	mov	A, Temp2
	mov	Temp3, A

int0_int_startup_boost_stall:
	mov	A, Stall_Cnt					; Add an extra power boost during start
	swap	A
IF MCU_48MHZ >= 1
	rlc	A
ENDIF
	add	A, Temp3
	mov	Temp3, A
	mov	A, Temp4
	addc	A, #0
	mov	Temp4, A

int0_int_startup_boosted:
	mov	A, Temp3						; Multiply throttle value by throttle gain
	mov	B, Temp7						; Temp7 has Throttle_Gain
	mul	AB
	mov	Temp2, A
	mov	Temp3, B
	mov	A, Temp4
	mov	B, Temp7						; Temp7 has Throttle_Gain
	mul	AB
	add	A, Temp3
	mov	Temp3, A
	xch	A, B
	addc	A, #0
	mov	Temp4, A
	clr	C							; Generate 8bit number
	mov	A, Temp4
	rrc	A
	mov	Temp6, A
	mov	A, Temp3
	rrc	A
	mov	Temp1, A
IF MCU_48MHZ >= 1
	clr	C
	mov	A, Temp6
	rrc	A
	mov	Temp6, A
	mov	A, Temp1
	rrc	A
	mov	Temp1, A
ENDIF
	inc	Temp8						; Temp8 has Throttle_Gain_M
int0_int_gain_loop:
	mov	A, Temp8
	dec	A
	jz	int0_int_gain_rcp_done			; Skip one multiply by 2 of New_Rcp

	clr	C
	mov	A, Temp1						; Multiply New_Rcp by 2
	rlc	A
	mov	Temp1, A

int0_int_gain_rcp_done:
	clr	C
	mov	A, Temp2						; Multiply pwm by 2
	rlc	A
	mov	A, Temp3
	rlc	A
	mov	Temp3, A
	mov	A, Temp4
	rlc	A
	mov	Temp4, A
	djnz	Temp8, int0_int_gain_loop

	mov	A, Temp4
IF MCU_48MHZ == 0
	jnb	ACC.2, int0_int_pulse_ready		; Check that RC pulse is within legal range
ELSE
	jnb	ACC.3, int0_int_pulse_ready
ENDIF

	mov	Temp1, #0FFh
	mov	Temp3, #0FFh
IF MCU_48MHZ == 0
	mov	Temp4, #3
ELSE
	mov	Temp4, #7
ENDIF

int0_int_pulse_ready:
	mov	New_Rcp, Temp1					; Store new pulse length
	setb	Flags2.RCP_UPDATED		 		; Set updated flag
	; Check if zero
	mov	A, Temp1						; Load new pulse value
	jz	($+5)						; Check if pulse is zero

	mov	Rcp_Stop_Cnt, #0				; Reset rcp stop counter

	; Set pwm limit
	clr	C
	mov	A, Pwm_Limit					; Limit to the smallest
	mov	Temp5, A						; Store limit in Temp5
	subb	A, Pwm_Limit_By_Rpm
	jc	($+4)

	mov	Temp5, Pwm_Limit_By_Rpm			

	; Check against limit
	clr	C
	mov	A, Temp5
	subb	A, New_Rcp
	jnc	int0_int_set_pwm_registers

	mov	A, Temp5						; Multiply limit by 4 (8 for 48MHz MCUs)
IF MCU_48MHZ == 0
	mov	B, #4
ELSE
	mov	B, #8
ENDIF
	mul	AB
	mov	Temp3, A
	mov	Temp4, B

int0_int_set_pwm_registers:
	mov	A, Temp3
	cpl	A
	mov	Temp1, A
	mov	A, Temp4
	cpl	A
IF MCU_48MHZ == 0
	anl	A, #3
ELSE
	anl	A, #7
ENDIF
	mov	Temp2, A
IF FETON_DELAY != 0
	clr	C
	mov	A, Temp1						; Skew damping fet timing
IF MCU_48MHZ == 0
	subb	A, #FETON_DELAY
ELSE
	subb	A, #(FETON_DELAY SHL 1)
ENDIF
	mov	Temp3, A
	mov	A, Temp2
	subb	A, #0	
	mov	Temp4, A
	jnc	int0_int_set_pwm_damp_set

	mov	Temp3, #0
	mov	Temp4, #0

int0_int_set_pwm_damp_set:
ENDIF
	mov	Power_Pwm_Reg_L, Temp1
	mov	Power_Pwm_Reg_H, Temp2
IF FETON_DELAY != 0
	mov	Damp_Pwm_Reg_L, Temp3
	mov	Damp_Pwm_Reg_H, Temp4
ENDIF
	mov	Rcp_Timeout_Cntd, #10			; Set timeout count
IF FETON_DELAY != 0
	pop	B							; Restore preserved registers
	pop	ACC
	pop	PSW
	Clear_COVF_Interrupt
	Enable_COVF_Interrupt				; Generate a pca interrupt
	orl	EIE1, #10h					; Enable pca interrupts
	reti
ELSE
	mov	A, Current_Power_Pwm_Reg_H
IF MCU_48MHZ == 0
	jnb	ACC.1, int0_int_set_pca_int_hi_pwm
ELSE
	jnb	ACC.2, int0_int_set_pca_int_hi_pwm
ENDIF

	pop	B							; Restore preserved registers
	pop	ACC
	pop	PSW
	Clear_COVF_Interrupt
	Enable_COVF_Interrupt				; Generate a pca interrupt
	orl	EIE1, #10h					; Enable pca interrupts
	reti

int0_int_set_pca_int_hi_pwm:
	pop	B							; Restore preserved registers
	pop	ACC
	pop	PSW
	Clear_CCF_Interrupt
	Enable_CCF_Interrupt				; Generate pca interrupt
	orl	EIE1, #10h					; Enable pca interrupts
	reti
ENDIF

int0_int_set_timeout:
	mov	Rcp_Timeout_Cntd, #10			; Set timeout count
int0_int_exit:
	pop	B							; Restore preserved registers
	pop	ACC
	pop	PSW
	orl	EIE1, #10h					; Enable pca interrupts
	reti
