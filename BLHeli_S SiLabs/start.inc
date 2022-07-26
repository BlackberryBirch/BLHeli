
;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Start entry point
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
init_start:
	clr	IE_EA
	call switch_power_off
	clr	A
	setb	IE_EA
	clr	A
	mov	Adc_Conversion_Cnt, A
	mov	Flags0, A				; Clear flags0
	mov	Flags1, A				; Clear flags1
	mov	Demag_Detected_Metric, A	; Clear demag metric
	;**** **** **** **** ****
	; Motor start beginning
	;**** **** **** **** **** 
	mov	Adc_Conversion_Cnt, #8				; Make sure a temp reading is done
	call wait1ms
	call start_adc_conversion
read_initial_temp:
	jnb	ADC0CN0_ADINT, read_initial_temp
	Read_Adc_Result						; Read initial temperature
	mov	A, Temp2
	jnz	($+3)							; Is reading below 256?

	mov	Temp1, A							; Yes - set average temperature value to zero

	mov	Current_Average_Temp, Temp1			; Set initial average temperature
	call check_temp_voltage_and_limit_power
	mov	Adc_Conversion_Cnt, #8				; Make sure a temp reading is done next time
	; Set up start operating conditions
	clr	IE_EA				; Disable interrupts
	call set_startup_pwm
	mov	Pwm_Limit, Pwm_Limit_Beg
	mov	Pwm_Limit_By_Rpm, Pwm_Limit_Beg
	setb	IE_EA
	; Begin startup sequence
IF MCU_48MHZ >= 1
	Set_MCU_Clk_48MHz
ENDIF
	jnb	Flags3.PGM_BIDIR, init_start_bidir_done	; Check if bidirectional operation

	clr	Flags3.PGM_DIR_REV			; Set spinning direction. Default fwd
	jnb	Flags2.RCP_DIR_REV, ($+5)	; Check force direction
	setb	Flags3.PGM_DIR_REV			; Set spinning direction

init_start_bidir_done:
	setb	Flags1.STARTUP_PHASE		; Set startup phase flag
	mov	Startup_Cnt, #0			; Reset counter
	call comm5comm6				; Initialize commutation
	call comm6comm1				
	call initialize_timing			; Initialize timing
	call	calc_next_comm_timing		; Set virtual commutation point
	call initialize_timing			; Initialize timing
	call	calc_next_comm_timing		
	call	initialize_timing			; Initialize timing
