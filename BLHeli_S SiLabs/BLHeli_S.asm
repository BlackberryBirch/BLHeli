$NOMOD51
;**** **** **** **** ****
;
; BLHeli program for controlling brushless motors in multirotors
;
; Copyright 2011, 2012 Steffen Skaug
; This program is distributed under the terms of the GNU General Public License
;
; This file is part of BLHeli.
;
; BLHeli is free software: you can redistribute it and/or modify
; it under the terms of the GNU General Public License as published by
; the Free Software Foundation, either version 3 of the License, or
; (at your option) any later version.
;
; BLHeli is distributed in the hope that it will be useful,
; but WITHOUT ANY WARRANTY; without even the implied warranty of
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
; GNU General Public License for more details.
;
; You should have received a copy of the GNU General Public License
; along with BLHeli.  If not, see <http://www.gnu.org/licenses/>.
;
;**** **** **** **** ****
;
; This software was initially designed for use with Eflite mCP X, but is now adapted to copters/planes in general
;
; The software was inspired by and started from from Bernard Konze's BLMC: http://home.versanet.de/~bkonze/blc_6a/blc_6a.htm
; And also Simon Kirby's TGY: https://github.com/sim-/tgy
;
; This file is best viewed with tab width set to 5
;
; The code is designed for multirotor applications, running damped light mode
;
; The input signal can be Normal (1-2ms), OneShot125 (125-250us), OneShot42 (41.7-83.3us) or Multishot (5-25us) at rates as high as allowed by the format.
; Three Dshot signal rates are also supported, Dshot150, Dshot300 and Dshot600. A 48MHz MCU is required for Dshot600.
; The code autodetects normal, OneShot125, Oneshot42, Multishot or Dshot.
;
; The first lines of the software must be modified according to the chosen environment:
; ESCNO EQU "ESC"
; MCU_48MHZ EQU "N"
; FETON_DELAY EQU "N"
; 
;**** **** **** **** ****
; Revision history:
; - Rev16.0 Started. Built upon rev 14.5 of base code
;           Using hardware pwm for very smooth throttle response, silent running and support of very high rpms
;           Implemented reverse bidirectional mode
;           Implemented separate throttle gains fwd and rev in bidirectional mode
;           Implemented support for Oneshot42 and Multishot
; - Rev16.1 Made low rpm power limiting programmable through the startup power parameter
; - Rev16.2 Fixed bug that prevented temperature protection
;           Improved robustness to very high input signal rates
;           Beeps can be turned off by programming beep strength to 1
;           Throttle cal difference is checked to be above required minimum before storing. Throttle cal max is not stored until successful min throttle cal
; - Rev16.3 Implemented programmable temperature protection
;           Improved protection of bootloader and generally reduced risk of flash corruption
;           Some small changes for improved sync hold
; - Rev16.4 Fixed bug where bootloader operation could be blocked by a defective "eeprom" signature
; - Rev16.5 Added support for DShot150, DShot300 and DShot600
; - Rev16.6 Fixed signal detection issue of multishot at 32kHz
;           Improved bidirectional mode for high input signal rates
; - Rev16.7 Addition of Dshot commands for beeps and temporary reverse direction (largely by brycedjohnson)
;   
;        
;**** **** **** **** ****
; Minimum 8K Bytes of In-System Self-Programmable Flash
; Minimum 512 Bytes Internal SRAM
;
;**** **** **** **** ****
; Master clock is internal 24MHz oscillator (or 48MHz, for which the times below are halved)
; Although 24/48 are used in the code, the exact clock frequencies are 24.5MHz or 49.0 MHz
; Timer 0 (41.67ns counts) always counts up and is used for
; - RC pulse measurement
; Timer 1 (41.67ns counts) always counts up and is used for
; - DShot frame sync detection
; Timer 2 (500ns counts) always counts up and is used for
; - RC pulse timeout counts and commutation times
; Timer 3 (500ns counts) always counts up and is used for
; - Commutation timeouts
; PCA0 (41.67ns counts) always counts up and is used for
; - Hardware PWM generation
;
;**** **** **** **** ****
; Interrupt handling
; The C8051 does not disable interrupts when entering an interrupt routine.
; Also some interrupt flags need to be cleared by software
; The code disables interrupts in some interrupt routines
; - Interrupts are disabled during beeps, to avoid audible interference from interrupts
;
;**** **** **** **** ****
; Motor control:
; - Brushless motor control with 6 states for each electrical 360 degrees
; - An advance timing of 0deg has zero cross 30deg after one commutation and 30deg before the next
; - Timing advance in this implementation is set to 15deg nominally
; - Motor pwm is always damped light (aka complementary pwm, regenerative braking)
; Motor sequence starting from zero crossing:
; - Timer wait: Wt_Comm			15deg	; Time to wait from zero cross to actual commutation
; - Timer wait: Wt_Advance		15deg	; Time to wait for timing advance. Nominal commutation point is after this
; - Timer wait: Wt_Zc_Scan		7.5deg	; Time to wait before looking for zero cross
; - Scan for zero cross			22.5deg	; Nominal, with some motor variations
;
; Motor startup:
; There is a startup phase and an initial run phase, before normal bemf commutation run begins.
;

;**** **** **** **** ****
; Define board pinouts
;
$incdir(board)
$include(board/board.inc)
$include(globals.inc)
$include(timer0.inc)
$include(timer1_dshot.inc)
$include(timer2_rcpulse.inc)
$include(timer3_commtiming.inc)
$include(int0_rcpulse.inc)
$include(int1_rcpulse.inc)
$include(pca_pwm.inc)
$include(wait.inc)
$include(pwmlimit.inc)
$include(powerlimit.inc)
$include(comm_timing.inc)
$include(comparitor.inc)
$include(wait_for_comm.inc)
$include(comm_driver.inc)
$include(beeper.inc)
$include(settings.inc)
$include(throttle_gain.inc)
$include(led_control.inc)
$include(no_signal_reboot.inc)

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

$include(dshot_cmds.inc)

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



;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Run entry point
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****

; Run 1 = B(p-on) + C(n-pwm) - comparator A evaluated
; Out_cA changes from low to high
run1:
	call wait_for_comp_out_high	; Wait for high
;		setup_comm_wait		; Setup wait time from zero cross to commutation
;		evaluate_comparator_integrity	; Check whether comparator reading has been normal
	call wait_for_comm			; Wait from zero cross to commutation
	call comm1comm2			; Commutate
	call calc_next_comm_timing	; Calculate next timing and wait advance timing wait
;		wait_advance_timing		; Wait advance timing and start zero cross wait
;		calc_new_wait_times
;		wait_before_zc_scan		; Wait zero cross wait and start zero cross timeout

; Run 2 = A(p-on) + C(n-pwm) - comparator B evaluated
; Out_cB changes from high to low
run2:
	call wait_for_comp_out_low
;		setup_comm_wait
;		evaluate_comparator_integrity
	jb	Flags1.HIGH_RPM, ($+6)	; Skip if high rpm
	lcall set_pwm_limit_low_rpm
	jnb	Flags1.HIGH_RPM, ($+6)	; Do if high rpm
	lcall set_pwm_limit_high_rpm
	call wait_for_comm
	call comm2comm3
	call calc_next_comm_timing
;		wait_advance_timing
;		calc_new_wait_times
;		wait_before_zc_scan

; Run 3 = A(p-on) + B(n-pwm) - comparator C evaluated
; Out_cC changes from low to high
run3:
	call wait_for_comp_out_high
;		setup_comm_wait
;		evaluate_comparator_integrity
	call wait_for_comm
	call comm3comm4
	call calc_next_comm_timing
;		wait_advance_timing
;		calc_new_wait_times
;		wait_before_zc_scan

; Run 4 = C(p-on) + B(n-pwm) - comparator A evaluated
; Out_cA changes from high to low
run4:
	call wait_for_comp_out_low
;		setup_comm_wait
;		evaluate_comparator_integrity
	call wait_for_comm
	call comm4comm5
	call calc_next_comm_timing
;		wait_advance_timing
;		calc_new_wait_times
;		wait_before_zc_scan

; Run 5 = C(p-on) + A(n-pwm) - comparator B evaluated
; Out_cB changes from low to high
run5:
	call wait_for_comp_out_high
;		setup_comm_wait
;		evaluate_comparator_integrity
	call wait_for_comm
	call comm5comm6
	call calc_next_comm_timing
;		wait_advance_timing
;		calc_new_wait_times
;		wait_before_zc_scan

; Run 6 = B(p-on) + A(n-pwm) - comparator C evaluated
; Out_cC changes from high to low
run6:
	call start_adc_conversion
	call wait_for_comp_out_low
;		setup_comm_wait
;		evaluate_comparator_integrity
	call wait_for_comm
	call comm6comm1
	call check_temp_voltage_and_limit_power
	call calc_next_comm_timing
;		wait_advance_timing
;		calc_new_wait_times
;		wait_before_zc_scan

	; Check if it is direct startup
	jnb	Flags1.STARTUP_PHASE, normal_run_checks

	; Set spoolup power variables
	mov	Pwm_Limit, Pwm_Limit_Beg		; Set initial max power
	; Check startup counter
	mov	Temp2, #24				; Set nominal startup parameters
	mov	Temp3, #12
	clr	C
	mov	A, Startup_Cnt				; Load counter
	subb	A, Temp2					; Is counter above requirement?
	jc	direct_start_check_rcp		; No - proceed

	clr	Flags1.STARTUP_PHASE		; Clear startup phase flag
	setb	Flags1.INITIAL_RUN_PHASE		; Set initial run phase flag
	mov	Initial_Run_Rot_Cntd, Temp3	; Set initial run rotation count
	mov	Pwm_Limit, Pwm_Limit_Beg
	mov	Pwm_Limit_By_Rpm, Pwm_Limit_Beg
	jmp	normal_run_checks

direct_start_check_rcp:
	clr	C
	mov	A, New_Rcp				; Load new pulse value
	subb	A, #1					; Check if pulse is below stop value
	jc	($+5)

	ljmp	run1						; Continue to run 

	jmp	run_to_wait_for_power_on


normal_run_checks:
	; Check if it is initial run phase
	jnb	Flags1.INITIAL_RUN_PHASE, initial_run_phase_done	; If not initial run phase - branch
	jb	Flags1.DIR_CHANGE_BRAKE, initial_run_phase_done	; If a direction change - branch

	; Decrement startup rotaton count
	mov	A, Initial_Run_Rot_Cntd
	dec	A
	; Check number of initial rotations
	jnz 	initial_run_check_startup_rot	; Branch if counter is not zero

	clr	Flags1.INITIAL_RUN_PHASE		; Clear initial run phase flag
	setb	Flags1.MOTOR_STARTED		; Set motor started
	jmp run1						; Continue with normal run

initial_run_check_startup_rot:
	mov	Initial_Run_Rot_Cntd, A		; Not zero - store counter

	jb	Flags3.PGM_BIDIR, initial_run_continue_run	; Check if bidirectional operation

	clr	C
	mov	A, New_Rcp				; Load new pulse value
	subb	A, #1					; Check if pulse is below stop value
	jc	($+5)

initial_run_continue_run:
	ljmp	run1						; Continue to run 

	jmp	run_to_wait_for_power_on

initial_run_phase_done:
	; Reset stall count
	mov	Stall_Cnt, #0
	; Exit run loop after a given time
	jb	Flags3.PGM_BIDIR, run6_check_timeout	; Check if bidirectional operation

	mov	Temp1, #250
	mov	Temp2, #Pgm_Brake_On_Stop
	mov	A, @Temp2
	jz	($+4)

	mov	Temp1, #3					; About 100ms before stopping when brake is set

	clr	C
	mov	A, Rcp_Stop_Cnt			; Load stop RC pulse counter low byte value
	subb	A, Temp1					; Is number of stop RC pulses above limit?
	jnc	run_to_wait_for_power_on		; Yes, go back to wait for poweron

run6_check_timeout:
	mov	A, Rcp_Timeout_Cntd			; Load RC pulse timeout counter value
	jz	run_to_wait_for_power_on		; If it is zero - go back to wait for poweron

run6_check_dir:
	jnb	Flags3.PGM_BIDIR, run6_check_speed		; Check if bidirectional operation

	jb	Flags3.PGM_DIR_REV, run6_check_dir_rev		; Check if actual rotation direction
	jb	Flags2.RCP_DIR_REV, run6_check_dir_change	; Matches force direction
	jmp	run6_check_speed

run6_check_dir_rev:
	jnb	Flags2.RCP_DIR_REV, run6_check_dir_change
	jmp	run6_check_speed

run6_check_dir_change:
	jb	Flags1.DIR_CHANGE_BRAKE, run6_check_speed

	setb	Flags1.DIR_CHANGE_BRAKE		; Set brake flag
	mov	Pwm_Limit, Pwm_Limit_Beg		; Set max power while braking
	jmp	run4						; Go back to run 4, thereby changing force direction

run6_check_speed:
	mov	Temp1, #0F0h				; Default minimum speed
	jnb	Flags1.DIR_CHANGE_BRAKE, run6_brake_done; Is it a direction change?

	mov	Pwm_Limit, Pwm_Limit_Beg 	; Set max power while braking
	mov	Temp1, #20h 				; Bidirectional braking termination speed

run6_brake_done:
	clr	C
	mov	A, Comm_Period4x_H			; Is Comm_Period4x more than 32ms (~1220 eRPM)?
	subb	A, Temp1
	jnc	($+5)					; Yes - stop or turn direction 
	ljmp	run1						; No - go back to run 1

	jnb	Flags1.DIR_CHANGE_BRAKE, run_to_wait_for_power_on	; If it is not a direction change - stop

	clr	Flags1.DIR_CHANGE_BRAKE		; Clear brake flag
	clr	Flags3.PGM_DIR_REV			; Set spinning direction. Default fwd
	jnb	Flags2.RCP_DIR_REV, ($+5)	; Check force direction
	setb	Flags3.PGM_DIR_REV			; Set spinning direction
	setb	Flags1.INITIAL_RUN_PHASE
	mov	Initial_Run_Rot_Cntd, #18
	mov	Pwm_Limit, Pwm_Limit_Beg		; Set initial max power
	jmp	run1						; Go back to run 1 

run_to_wait_for_power_on_fail:	
	inc	Stall_Cnt					; Increment stall count
	mov	A, New_Rcp				; Check if RCP is zero, then it is a normal stop			
	jz	run_to_wait_for_power_on
	ajmp	run_to_wait_for_power_on_stall_done

run_to_wait_for_power_on:	
	mov	Stall_Cnt, #0

run_to_wait_for_power_on_stall_done:
	clr	IE_EA
	call switch_power_off
	mov	Flags0, #0				; Clear flags0
	mov	Flags1, #0				; Clear flags1
IF MCU_48MHZ >= 1
	Set_MCU_Clk_24MHz
ENDIF
	setb	IE_EA
	call	wait100ms					; Wait for pwm to be stopped
	call switch_power_off
	mov	Temp1, #Pgm_Brake_On_Stop
	mov	A, @Temp1
	jz	run_to_wait_for_power_on_brake_done

	AcomFET_on
	BcomFET_on
	CcomFET_on

run_to_wait_for_power_on_brake_done:
	clr	C
	mov	A, Stall_Cnt
	subb	A, #4
	jc	jmp_wait_for_power_on
	jmp	init_no_signal

jmp_wait_for_power_on:
	jmp	wait_for_power_on			; Go back to wait for power on

;**** **** **** **** **** **** **** **** **** **** **** **** ****

$include (BLHeliPgm.inc)				; Include source code for programming the ESC
$include (BLHeliBootLoad.inc)			; Include source code for bootloader
$include (reset.inc)                    ; Include source code for post-bootloader reset
END