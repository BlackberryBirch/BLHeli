
;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Set default parameters
;
; No assumptions
;
; Sets default programming parameters
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
set_default_parameters:
	mov	Temp1, #_Pgm_Gov_P_Gain
	mov	@Temp1, #0FFh	; Governor P gain
	inc	Temp1
	mov	@Temp1, #0FFh	; Governor I gain
	inc	Temp1
	mov	@Temp1, #0FFh	; Governor mode
	inc	Temp1
	mov	@Temp1, #0FFh	; Low voltage limit
	inc	Temp1
	mov	@Temp1, #0FFh	; Multi gain
	inc	Temp1
	mov	@Temp1, #0FFh	
	inc	Temp1
	mov	@Temp1, #DEFAULT_PGM_STARTUP_PWR
	inc	Temp1
	mov	@Temp1, #0FFh	; Pwm freq
	inc	Temp1
	mov	@Temp1, #DEFAULT_PGM_DIRECTION

	mov	Temp1, #Pgm_Enable_TX_Program
	mov	@Temp1, #DEFAULT_PGM_ENABLE_TX_PROGRAM
	inc	Temp1
	mov	@Temp1, #0FFh	; Main rearm start
	inc	Temp1
	mov	@Temp1, #0FFh	; Governor setup target
	inc	Temp1
	mov	@Temp1, #0FFh	; Startup rpm	
	inc	Temp1
	mov	@Temp1, #0FFh	; Startup accel
	inc	Temp1
	mov	@Temp1, #0FFh	; Voltage comp
	inc	Temp1
	mov	@Temp1, #DEFAULT_PGM_COMM_TIMING
	inc	Temp1
	mov	@Temp1, #0FFh	; Damping force	
	inc	Temp1
	mov	@Temp1, #0FFh	; Governor range
	inc	Temp1
	mov	@Temp1, #0FFh	; Startup method	
	inc	Temp1
	mov	@Temp1, #DEFAULT_PGM_MIN_THROTTLE
	inc	Temp1
	mov	@Temp1, #DEFAULT_PGM_MAX_THROTTLE
	inc	Temp1
	mov	@Temp1, #DEFAULT_PGM_BEEP_STRENGTH
	inc	Temp1
	mov	@Temp1, #DEFAULT_PGM_BEACON_STRENGTH
	inc	Temp1
	mov	@Temp1, #DEFAULT_PGM_BEACON_DELAY
	inc	Temp1
	mov	@Temp1, #0FFh	; Throttle rate	
	inc	Temp1
	mov	@Temp1, #DEFAULT_PGM_DEMAG_COMP
	inc	Temp1
	mov	@Temp1, #0FFh	; Bec voltage high
	inc	Temp1
	mov	@Temp1, #DEFAULT_PGM_CENTER_THROTTLE
	inc	Temp1
	mov	@Temp1, #0FFh	
	inc	Temp1
	mov	@Temp1, #DEFAULT_PGM_ENABLE_TEMP_PROT
	inc	Temp1
	mov	@Temp1, #DEFAULT_PGM_ENABLE_POWER_PROT
	inc	Temp1
	mov	@Temp1, #0FFh	; Enable pwm input
	inc	Temp1
	mov	@Temp1, #0FFh	; Pwm dither
	inc	Temp1
	mov	@Temp1, #DEFAULT_PGM_BRAKE_ON_STOP
	inc	Temp1
	mov	@Temp1, #DEFAULT_PGM_LED_CONTROL
	ret


;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Scale throttle cal
;
; No assumptions
;
; Scales a throttle cal value
; Input is ACC, output is Temp2/Temp1
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
scale_throttle_cal:
	mov	Temp3, A
	mov	B, #0Ch			; Calculate "3%" (for going from 1000us to numerical 1024)
	mul	AB
	mov	Temp4, B
	mov	A, Temp3
	clr	C				; Shift to 9 bits
	rlc	A
	mov	Temp1, A
	mov	A, #1
	rlc	A
	mov	Temp2, A
	mov	A, Temp1			; Shift to 10 bits
	clr	C
	rlc	A
	mov	Temp1, A
	mov	A, Temp2
	rlc	A
	mov	Temp2, A
	mov	A, Temp1			; Add "3%"
	clr	C
	add	A, Temp4
	mov	Temp1, A
	mov	A, Temp2
	addc	A, #0
	mov	Temp2, A
IF MCU_48MHZ >= 1
	mov	A, Temp1			; Shift to 11 bits
	clr	C
	rlc	A
	mov	Temp1, A
	mov	A, Temp2
	rlc	A
	mov	Temp2, A
ENDIF
ret


;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Decode settings
;
; No assumptions
;
; Decodes various settings
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
decode_settings:
	; Load programmed direction
	mov	Temp1, #Pgm_Direction	
	mov	A, @Temp1				
	clr	C
	subb	A, #3
	setb	Flags3.PGM_BIDIR
	jnc	($+4)

	clr	Flags3.PGM_BIDIR

	clr	Flags3.PGM_DIR_REV
	mov	A, @Temp1				
	jnb	ACC.1, ($+5)
	setb	Flags3.PGM_DIR_REV
	mov	C, Flags3.PGM_DIR_REV
	mov	Flags3.PGM_BIDIR_REV, C
	; Decode startup power
	mov	Temp1, #Pgm_Startup_Pwr		
	mov	A, @Temp1				
	dec	A	
	mov	DPTR, #STARTUP_POWER_TABLE
	movc A, @A+DPTR	
	mov	Temp1, #Pgm_Startup_Pwr_Decoded
	mov	@Temp1, A	
	; Decode low rpm power slope
	mov	Temp1, #Pgm_Startup_Pwr
	mov	A, @Temp1
	mov	Low_Rpm_Pwr_Slope, A
	clr	C
	subb	A, #2
	jnc	($+5)
	mov	Low_Rpm_Pwr_Slope, #2
	; Decode demag compensation
	mov	Temp1, #Pgm_Demag_Comp		
	mov	A, @Temp1				
	mov	Demag_Pwr_Off_Thresh, #255	; Set default

	cjne	A, #2, decode_demag_high

	mov	Demag_Pwr_Off_Thresh, #160	; Settings for demag comp low

decode_demag_high:
	cjne	A, #3, decode_demag_done

	mov	Demag_Pwr_Off_Thresh, #130	; Settings for demag comp high

decode_demag_done:
	; Decode temperature protection limit
	mov	Temp1, #Pgm_Enable_Temp_Prot
	mov	A, @Temp1
	mov	Temp1, A
	jz	decode_temp_done

	mov	A, #(TEMP_LIMIT-TEMP_LIMIT_STEP)
decode_temp_step:
	add	A, #TEMP_LIMIT_STEP
	djnz	Temp1, decode_temp_step

decode_temp_done:
	mov	Temp_Prot_Limit, A
	; Decode throttle cal
	mov	Temp1, #Pgm_Min_Throttle		; Throttle cal is in 4us units
	mov	A, @Temp1
	call	scale_throttle_cal
	mov	Min_Throttle_L, Temp1
	mov	Min_Throttle_H, Temp2
	mov	Temp1, #Pgm_Center_Throttle	; Throttle cal is in 4us units
	mov	A, @Temp1
	call	scale_throttle_cal
	mov	Center_Throttle_L, Temp1
	mov	Center_Throttle_H, Temp2
	mov	Temp1, #Pgm_Max_Throttle		; Throttle cal is in 4us units
	mov	A, @Temp1
	call	scale_throttle_cal
	mov	Max_Throttle_L, Temp1
	mov	Max_Throttle_H, Temp2
	call	switch_power_off
	ret
