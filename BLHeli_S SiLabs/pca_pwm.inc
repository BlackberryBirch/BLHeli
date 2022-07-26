;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; PCA interrupt routine
;
; No assumptions
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
pca_int:	; Used for setting pwm registers
	clr	IE_EA
	push	PSW				; Preserve registers through interrupt
	push	ACC
	setb	PSW.3			; Select register bank 1 for this interrupt

IF FETON_DELAY != 0					; HI/LO enable style drivers

	mov	Temp1, PCA0L				; Read low byte, to transfer high byte to holding register
	mov	A, Current_Power_Pwm_Reg_H
IF MCU_48MHZ == 0
	jnb	ACC.1, pca_int_hi_pwm
ELSE
	jnb	ACC.2, pca_int_hi_pwm
ENDIF
	mov	A, PCA0H
IF MCU_48MHZ == 0
	jb	ACC.1, pca_int_exit			; Power below 50%, update pca in the 0x00-0x0F range
	jb	ACC.0, pca_int_exit
ELSE
	jb	ACC.2, pca_int_exit
	jb	ACC.1, pca_int_exit
ENDIF
	ajmp	pca_int_set_pwm

pca_int_hi_pwm:
	mov	A, PCA0H
IF MCU_48MHZ == 0
	jnb	ACC.1, pca_int_exit			; Power above 50%, update pca in the 0x20-0x2F range
	jb	ACC.0, pca_int_exit
ELSE
	jnb	ACC.2, pca_int_exit
	jb	ACC.1, pca_int_exit
ENDIF

pca_int_set_pwm:
	Set_Power_Pwm_Regs
	Set_Damp_Pwm_Regs
	mov	Current_Power_Pwm_Reg_H, Power_Pwm_Reg_H
	Disable_COVF_Interrupt

ELSE								; EN/PWM style drivers
	Set_Power_Pwm_Regs
	mov	Current_Power_Pwm_Reg_H, Power_Pwm_Reg_H
	Disable_COVF_Interrupt
	Disable_CCF_Interrupt

ENDIF

	; Pwm updated, enable/disable interrupts
	setb	IE_EX0					; Enable int0 interrupts
	jnb	Flags2.RCP_DSHOT, ($+5)
	setb	IE_EX1					; Enable int1 interrupts (DShot only)
	anl	EIE1, #0EFh				; Disable pca interrupts
    
pca_int_exit:
	Clear_COVF_Interrupt
IF FETON_DELAY == 0
	Clear_CCF_Interrupt
ENDIF
	pop	ACC						; Restore preserved registers
	pop	PSW
	setb	IE_EA
	reti
