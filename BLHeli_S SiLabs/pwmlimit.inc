;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Set pwm limit low rpm
;
; No assumptions
;
; Sets power limit for low rpms and disables demag for low rpms
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
set_pwm_limit_low_rpm:
	; Set pwm limit
	mov	Temp1, #0FFh					; Default full power
	jb	Flags1.STARTUP_PHASE, set_pwm_limit_low_rpm_exit	; Exit if startup phase set

	mov	Temp2, #Pgm_Enable_Power_Prot		; Check if low RPM power protection is enabled
	mov	A, @Temp2
	jz	set_pwm_limit_low_rpm_exit		; Exit if disabled

	mov	A, Comm_Period4x_H
	jz	set_pwm_limit_low_rpm_exit		; Avoid divide by zero

	mov	A, #255						; Divide 255 by Comm_Period4x_H
	mov	B, Comm_Period4x_H
	div	AB
	mov	B, Low_Rpm_Pwr_Slope			; Multiply by slope
	jnb	Flags1.INITIAL_RUN_PHASE, ($+6)	; More protection for initial run phase 
	mov	B, #5
	mul	AB
	mov	Temp1, A						; Set new limit				
	xch	A, B
	jz	($+4)						; Limit to max
	
	mov	Temp1, #0FFh				

	clr	C
	mov	A, Temp1						; Limit to min
	subb	A, Pwm_Limit_Beg
	jnc	set_pwm_limit_low_rpm_exit

	mov	Temp1, Pwm_Limit_Beg

set_pwm_limit_low_rpm_exit:
	mov	Pwm_Limit_By_Rpm, Temp1				
	ret
	

;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Set pwm limit high rpm
;
; No assumptions
;
; Sets power limit for high rpms
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
set_pwm_limit_high_rpm:
IF MCU_48MHZ >= 1
	clr	C
	mov	A, Comm_Period4x_L
	subb	A, #0A0h				; Limit Comm_Period to 160, which is 500k erpm
	mov	A, Comm_Period4x_H
	subb	A, #00h
ELSE
	clr	C
	mov	A, Comm_Period4x_L
	subb	A, #0E4h				; Limit Comm_Period to 228, which is 350k erpm
	mov	A, Comm_Period4x_H
	subb	A, #00h
ENDIF
	mov	A, Pwm_Limit_By_Rpm
	jnc	set_pwm_limit_high_rpm_inc_limit
	
	dec	A
	ajmp	set_pwm_limit_high_rpm_store
	
set_pwm_limit_high_rpm_inc_limit:
	inc	A
set_pwm_limit_high_rpm_store:
	jz	($+4)

	mov	Pwm_Limit_By_Rpm, A

	ret
