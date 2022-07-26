
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
