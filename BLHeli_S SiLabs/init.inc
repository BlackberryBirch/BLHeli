
;**** **** **** **** **** **** **** **** **** **** **** **** ****
;**** **** **** **** **** **** **** **** **** **** **** **** ****
;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Main program start
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
;**** **** **** **** **** **** **** **** **** **** **** **** ****
;**** **** **** **** **** **** **** **** **** **** **** **** ****

pgm_start:
	; Initialize flash keys to invalid values
	mov	Flash_Key_1, #0
	mov	Flash_Key_2, #0
	; Disable the WDT.
	mov	WDTCN, #0DEh		; Disable watchdog
	mov	WDTCN, #0ADh		
	; Initialize stack
	mov	SP, #0c0h			; Stack = 64 upper bytes of RAM
	; Initialize VDD monitor
IF MCU_48MHZ < 2
	orl	VDM0CN, #080h    	; Enable the VDD monitor
ENDIF
	mov 	RSTSRC, #06h   	; Set missing clock and VDD monitor as a reset source if not 1S capable
	; Set clock frequency
	mov	CLKSEL, #00h		; Set clock divider to 1
	; Switch power off
	call	switch_power_off
	; Ports initialization
	mov	P0, #P0_INIT
	mov	P0MDIN, #P0_DIGITAL
	mov	P0MDOUT, #P0_PUSHPULL
	mov	P0, #P0_INIT
	mov	P0SKIP, #P0_SKIP				
	mov	P1, #P1_INIT
	mov	P1MDIN, #P1_DIGITAL
	mov	P1MDOUT, #P1_PUSHPULL
	mov	P1, #P1_INIT
	mov	P1SKIP, #P1_SKIP				
	mov	P2MDOUT, #P2_PUSHPULL				
	; Initialize the XBAR and related functionality
	Initialize_Xbar
	; Switch power off again, after initializing ports
	call	switch_power_off
	; Clear RAM
	clr	A				; Clear accumulator
	mov	Temp1, A			; Clear Temp1
	clear_ram:	
	mov	@Temp1, A			; Clear RAM
	djnz Temp1, clear_ram	; Is A not zero? - jump
	; Set default programmed parameters
	call	set_default_parameters
	; Read all programmed parameters
	call read_all_eeprom_parameters
	; Set beep strength
	mov	Temp1, #Pgm_Beep_Strength
	mov	Beep_Strength, @Temp1
	; Set initial arm variable
	mov	Initial_Arm, #1
	; Initializing beep
	clr	IE_EA			; Disable interrupts explicitly
	call wait200ms	
	call beep_f1
	call wait30ms
	call beep_f2
	call wait30ms
	call beep_f3
	call wait30ms
	call led_control

bootloader_done:
	; Decode settings
	call	decode_settings
	; Find throttle gain from stored min and max settings
	call	find_throttle_gains
	; Set beep strength
	mov	Temp1, #Pgm_Beep_Strength
	mov	Beep_Strength, @Temp1
	; Switch power off
	call	switch_power_off
	; Set clock frequency
IF MCU_48MHZ >= 1
	Set_MCU_Clk_24MHz
ENDIF
	; Setup timers for pwm input
	mov	IT01CF, #RTX_PIN	; Route RCP input to INT0
	mov	TCON, #11h		; Timer 0 run and INT0 edge triggered
	mov	CKCON0, #04h		; Timer 0 clock is system clock
	mov	TMOD, #09h		; Timer 0 set to 16bits and gated by INT0
	mov	TMR2CN0, #04h		; Timer 2 enabled
	mov	TMR3CN0, #04h		; Timer 3 enabled
	Initialize_PCA			; Initialize PCA
	Set_Pwm_Polarity		; Set pwm polarity
	Enable_Power_Pwm_Module	; Enable power pwm module
	Enable_Damp_Pwm_Module	; Enable damping pwm module
	; Enable interrupts
IF MCU_48MHZ == 0
	mov	IE, #21h			; Enable timer 2 interrupts and INT0 interrupts
ELSE
	mov	IE, #23h			; Enable timer 0, timer 2 interrupts and INT0 interrupts
ENDIF
	mov	EIE1, #90h		; Enable timer 3 and PCA0 interrupts
	mov	IP, #01h			; High priority to INT0 interrupts
	; Initialize comparator
	Initialize_Comparator	; Initialize comparator
	; Initialize ADC
	Initialize_Adc			; Initialize ADC operation
	call	wait1ms
	setb	IE_EA			; Enable all interrupts
	; Reset stall count
	mov	Stall_Cnt, #0
	; Initialize RC pulse
	clr	Flags2.RCP_UPDATED		 	; Clear updated flag
	call wait200ms
	; Clear all shot flags
	clr	Flags2.RCP_ONESHOT125			; Clear OneShot125 flag
	clr	Flags2.RCP_ONESHOT42			; Clear OneShot42 flag
	clr	Flags2.RCP_MULTISHOT			; Clear Multishot flag
	clr	Flags2.RCP_DSHOT				; Clear DShot flag
	mov 	Dshot_Cmd, #0					; Clear Dshot command
	mov 	Dshot_Cmd_Cnt, #0				; Clear Dshot command count
	; Test whether signal is regular pwm
	mov	Rcp_Outside_Range_Cnt, #0		; Reset out of range counter
	call wait100ms						; Wait for new RC pulse
	clr	C
	mov	A, Rcp_Outside_Range_Cnt			; Check how many pulses were outside normal range ("900-2235us")
	subb	A, #10						
	jnc	($+4)
	ajmp	validate_rcp_start

	; Test whether signal is OneShot125
	setb	Flags2.RCP_ONESHOT125			; Set OneShot125 flag
	mov	Rcp_Outside_Range_Cnt, #0		; Reset out of range counter
	call wait100ms						; Wait for new RC pulse
	clr	C
	mov	A, Rcp_Outside_Range_Cnt			; Check how many pulses were outside normal range ("900-2235us")
	subb	A, #10
	jnc	($+4)
	ajmp	validate_rcp_start

	; Test whether signal is OneShot42
	clr	Flags2.RCP_ONESHOT125
	setb	Flags2.RCP_ONESHOT42			; Set OneShot42 flag
	mov	Rcp_Outside_Range_Cnt, #0		; Reset out of range counter
	call wait100ms						; Wait for new RC pulse
	clr	C
	mov	A, Rcp_Outside_Range_Cnt			; Check how many pulses were outside normal range ("900-2235us")
	subb	A, #10
	jnc	($+4)
	ajmp	validate_rcp_start

	; Setup timers for DShot
	mov	IT01CF, #(80h+(RTX_PIN SHL 4)+(RTX_PIN))	; Route RCP input to INT0/1, with INT1 inverted
	mov	TCON, #51h		; Timer 0/1 run and INT0 edge triggered
	mov	CKCON0, #01h		; Timer 0/1 clock is system clock divided by 4 (for DShot150)
	mov	TMOD, #0AAh		; Timer 0/1 set to 8bits auto reload and gated by INT0
	mov	TH0, #0			; Auto reload value zero
	mov	TH1, #0
	; Setup interrupts for DShot
	clr	IE_ET0			; Disable timer 0 interrupts
	setb	IE_ET1			; Enable timer 1 interrupts
	setb	IE_EX1			; Enable int1 interrupts
	; Setup variables for DSshot150
IF MCU_48MHZ >= 1
	mov	DShot_Timer_Preset, #128			; Load DShot sync timer preset (for DShot150)
ELSE
	mov	DShot_Timer_Preset, #192
ENDIF
	mov	DShot_Pwm_Thr, #20				; Load DShot qualification pwm threshold (for DShot150)
	mov	DShot_Frame_Length_Thr, #80		; Load DShot frame length criteria
	; Test whether signal is DShot150
	clr	Flags2.RCP_ONESHOT42
	setb	Flags2.RCP_DSHOT
	mov	Rcp_Outside_Range_Cnt, #10		; Set out of range counter
	call wait100ms						; Wait for new RC pulse
	mov	DShot_Pwm_Thr, #16				; Load DShot regular pwm threshold
	clr	C
	mov	A, Rcp_Outside_Range_Cnt			; Check if pulses were accepted
	subb	A, #10
	mov 	Dshot_Cmd, #0
	mov 	Dshot_Cmd_Cnt, #0
	jc	validate_rcp_start

	; Setup variables for DShot300
	mov	CKCON0, #0Ch					; Timer 0/1 clock is system clock (for DShot300)
IF MCU_48MHZ >= 1
	mov	DShot_Timer_Preset, #0			; Load DShot sync timer preset (for DShot300)
ELSE
	mov	DShot_Timer_Preset, #128
ENDIF
	mov	DShot_Pwm_Thr, #40				; Load DShot qualification pwm threshold (for DShot300)
	mov	DShot_Frame_Length_Thr, #40		; Load DShot frame length criteria
	; Test whether signal is DShot300
	mov	Rcp_Outside_Range_Cnt, #10		; Set out of range counter
	call wait100ms						; Wait for new RC pulse
	mov	DShot_Pwm_Thr, #32				; Load DShot regular pwm threshold
	clr	C
	mov	A, Rcp_Outside_Range_Cnt			; Check if pulses were accepted
	subb	A, #10
	mov 	Dshot_Cmd, #0
	mov 	Dshot_Cmd_Cnt, #0
	jc	validate_rcp_start

	; Setup variables for DShot600
	mov	CKCON0, #0Ch					; Timer 0/1 clock is system clock (for DShot600)
IF MCU_48MHZ >= 1
	mov	DShot_Timer_Preset, #128			; Load DShot sync timer preset (for DShot600)
ELSE
	mov	DShot_Timer_Preset, #192
ENDIF
	mov	DShot_Pwm_Thr, #20				; Load DShot qualification pwm threshold (for DShot600)
	mov	DShot_Frame_Length_Thr, #20		; Load DShot frame length criteria
	; Test whether signal is DShot600
	mov	Rcp_Outside_Range_Cnt, #10		; Set out of range counter
	call wait100ms						; Wait for new RC pulse
	mov	DShot_Pwm_Thr, #16				; Load DShot regular pwm threshold
	clr	C
	mov	A, Rcp_Outside_Range_Cnt			; Check if pulses were accepted
	subb	A, #10
	mov 	Dshot_Cmd, #0
	mov 	Dshot_Cmd_Cnt, #0
	jc	validate_rcp_start

	; Setup timers for Multishot
	mov	IT01CF, #RTX_PIN	; Route RCP input to INT0
	mov	TCON, #11h		; Timer 0 run and INT0 edge triggered
	mov	CKCON0, #04h		; Timer 0 clock is system clock
	mov	TMOD, #09h		; Timer 0 set to 16bits and gated by INT0
	; Setup interrupts for Multishot
	setb	IE_ET0			; Enable timer 0 interrupts
	clr	IE_ET1			; Disable timer 1 interrupts
	clr	IE_EX1			; Disable int1 interrupts
	; Test whether signal is Multishot
	clr	Flags2.RCP_DSHOT
	setb	Flags2.RCP_MULTISHOT			; Set Multishot flag
	mov	Rcp_Outside_Range_Cnt, #0		; Reset out of range counter
	call wait100ms						; Wait for new RC pulse
	clr	C
	mov	A, Rcp_Outside_Range_Cnt			; Check how many pulses were outside normal range ("900-2235us")
	subb	A, #10
	jc	validate_rcp_start

	ajmp	init_no_signal

validate_rcp_start:
	; Validate RC pulse
	call wait3ms						; Wait for new RC pulse
	jb	Flags2.RCP_UPDATED, ($+6)		; Is there an updated RC pulse available - proceed
	ljmp	init_no_signal					; Go back to detect input signal

	; Beep arm sequence start signal
	clr 	IE_EA						; Disable all interrupts
	call beep_f1						; Signal that RC pulse is ready
	call beep_f1
	call beep_f1
	setb	IE_EA						; Enable all interrupts
	call wait200ms	

	; Arming sequence start
arming_start:
	jb	Flags2.RCP_DSHOT, ($+6)	; Disable tx programming for DShot
	jnb	Flags3.PGM_BIDIR, ($+6)

	ljmp	program_by_tx_checked	; Disable tx programming if bidirectional operation

	call wait3ms
	mov	Temp1, #Pgm_Enable_TX_Program; Start programming mode entry if enabled
	mov	A, @Temp1				
	clr	C
	subb	A, #1				; Is TX programming enabled?
	jnc 	arming_initial_arm_check	; Yes - proceed

	jmp	program_by_tx_checked	; No - branch

arming_initial_arm_check:
	mov	A, Initial_Arm			; Yes - check if it is initial arm sequence
	clr	C
	subb	A, #1				; Is it the initial arm sequence?
	jnc 	arming_check			; Yes - proceed

	jmp 	program_by_tx_checked	; No - branch

arming_check:
	; Initialize flash keys to valid values
	mov	Flash_Key_1, #0A5h
	mov	Flash_Key_2, #0F1h
	; Throttle calibration and tx program entry
	mov	Temp8, #2				; Set 1 seconds wait time
throttle_high_cal:			
	setb	Flags2.RCP_FULL_RANGE	; Set range to 1000-2020us
	call	find_throttle_gains		; Set throttle gains
	call wait100ms				; Wait for new throttle value
	clr	IE_EA				; Disable interrupts (freeze New_Rcp value)
	clr	Flags2.RCP_FULL_RANGE	; Set programmed range
	call	find_throttle_gains		; Set throttle gains
	clr	C
	mov	A, New_Rcp			; Load new RC pulse value
	subb	A, #(255/2)			; Is RC pulse above midstick?
	setb	IE_EA				; Enable interrupts
	jc	program_by_tx_checked	; No - branch

	call wait1ms		
	clr	IE_EA				; Disable all interrupts
	call beep_f4
	setb	IE_EA				; Enable all interrupts
	djnz	Temp8, throttle_high_cal	; Continue to wait

	call	average_throttle
	clr	C
	mov	A, Temp8
	mov	Temp1, #Pgm_Max_Throttle	; Store
	mov	@Temp1, A			
	call wait200ms				
	call	success_beep

throttle_low_cal_start:
	mov	Temp8, #10			; Set 3 seconds wait time
throttle_low_cal:			
	setb	Flags2.RCP_FULL_RANGE	; Set range to 1000-2020us
	call	find_throttle_gains		; Set throttle gains
	call wait100ms
	clr	IE_EA				; Disable interrupts (freeze New_Rcp value)
	clr	Flags2.RCP_FULL_RANGE	; Set programmed range
	call	find_throttle_gains		; Set throttle gains
	clr	C
	mov	A, New_Rcp			; Load new RC pulse value
	subb	A, #(255/2)			; Below midstick?
	setb	IE_EA				; Enable interrupts
	jnc	throttle_low_cal_start	; No - start over

	call wait1ms		
	clr	IE_EA				; Disable all interrupts
	call beep_f1
	call wait10ms
	call beep_f1
	setb	IE_EA				; Enable all interrupts
	djnz	Temp8, throttle_low_cal	; Continue to wait

	call	average_throttle
	mov	A, Temp8
	add	A, #3				; Add about 1%
	mov	Temp1, #Pgm_Min_Throttle	; Store
	mov	@Temp1, A			
	mov	Temp1, A				; Min throttle in Temp1
	mov	Temp2, #Pgm_Max_Throttle
	mov	A, @Temp2
	clr	C
	subb	A, #35				; Subtract 35 (140us) from max throttle
	jc	program_by_tx_entry_limit
	subb	A, Temp1				; Subtract min from max
	jnc	program_by_tx_entry_store

program_by_tx_entry_limit:
	mov	A, Temp1				; Load min
	add	A, #35				; Make max 140us higher than min
	mov	Temp1, #Pgm_Max_Throttle	; Store new max
	mov	@Temp1, A

program_by_tx_entry_store:
	call wait200ms				
	call erase_and_store_all_in_eeprom	
	call	success_beep_inverted

program_by_tx_entry_wait:
	call wait100ms
	call	find_throttle_gains		; Set throttle gains
	ljmp	init_no_signal			; Go back

program_by_tx_checked:
	; Initialize flash keys to invalid values
	mov	Flash_Key_1, #0
	mov	Flash_Key_2, #0
	call wait100ms				; Wait for new throttle value
	clr	C
	mov	A, New_Rcp			; Load new RC pulse value
	subb	A, #1				; Below stop?
	jc	arm_end_beep			; Yes - proceed

	jmp	arming_start			; No - start over

arm_end_beep:
	; Beep arm sequence end signal
	clr 	IE_EA				; Disable all interrupts
	call beep_f4				; Signal that rcpulse is ready
	call beep_f4
	call beep_f4
	setb	IE_EA				; Enable all interrupts
	call wait200ms

	; Clear initial arm variable
	mov	Initial_Arm, #0

	; Armed and waiting for power on
wait_for_power_on:
	clr	A
	mov	Power_On_Wait_Cnt_L, A	; Clear wait counter
	mov	Power_On_Wait_Cnt_H, A	
wait_for_power_on_loop:
	inc	Power_On_Wait_Cnt_L		; Increment low wait counter
	mov	A, Power_On_Wait_Cnt_L
	cpl	A
	jnz	wait_for_power_on_no_beep; Counter wrapping (about 3 sec)

	inc	Power_On_Wait_Cnt_H		; Increment high wait counter
	mov	Temp1, #Pgm_Beacon_Delay
	mov	A, @Temp1
	mov	Temp1, #25		; Approximately 1 min
	dec	A
	jz	beep_delay_set

	mov	Temp1, #50		; Approximately 2 min
	dec	A
	jz	beep_delay_set

	mov	Temp1, #125		; Approximately 5 min
	dec	A
	jz	beep_delay_set

	mov	Temp1, #250		; Approximately 10 min
	dec	A
	jz	beep_delay_set

	mov	Power_On_Wait_Cnt_H, #0		; Reset counter for infinite delay

beep_delay_set:
	clr	C
	mov	A, Power_On_Wait_Cnt_H
	subb	A, Temp1				; Check against chosen delay
	jc	wait_for_power_on_no_beep; Has delay elapsed?

	call	switch_power_off		; Switch power off in case braking is set
	call	wait1ms
	dec	Power_On_Wait_Cnt_H		; Decrement high wait counter
	mov	Power_On_Wait_Cnt_L, #0	; Set low wait counter
	mov	Temp1, #Pgm_Beacon_Strength
	mov	Beep_Strength, @Temp1
	clr 	IE_EA				; Disable all interrupts
	call beep_f4				; Signal that there is no signal
	setb	IE_EA				; Enable all interrupts
	mov	Temp1, #Pgm_Beep_Strength
	mov	Beep_Strength, @Temp1
	call wait100ms				; Wait for new RC pulse to be measured

wait_for_power_on_no_beep:
	call wait10ms
	mov	A, Rcp_Timeout_Cntd			; Load RC pulse timeout counter value
	jnz	wait_for_power_on_not_missing	; If it is not zero - proceed

	jmp	init_no_signal				; If pulses missing - go back to detect input signal

wait_for_power_on_not_missing:
	clr	C
	mov	A, New_Rcp			; Load new RC pulse value
	subb	A, #1		 		; Higher than stop
	jnc	wait_for_power_on_nonzero	; Yes - proceed

	clr	C
	mov	A, Dshot_Cmd
	subb	A, #1		 		; 1 or higher
	jnc	check_dshot_cmd		; Check Dshot command

	ljmp	wait_for_power_on_loop	; If not Dshot command - start over

wait_for_power_on_nonzero:
	lcall wait100ms			; Wait to see if start pulse was only a glitch
	mov	A, Rcp_Timeout_Cntd		; Load RC pulse timeout counter value
	jnz	($+5)				; If it is not zero - proceed
	ljmp	init_no_signal			; If it is zero (pulses missing) - go back to detect input signal

	mov 	Dshot_Cmd, #0
	mov 	Dshot_Cmd_Cnt, #0
	ljmp init_start