;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Timer 2 interrupt routine
;
; No assumptions
; Requirements: Temp variables can NOT be used since PSW.x is not set
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
t2_int:	; Happens every 32ms
	push	PSW			; Preserve registers through interrupt
	push	ACC
	clr	TMR2CN0_TF2H				; Clear interrupt flag
	inc	Timer2_X
IF MCU_48MHZ >= 1
	mov	A, Clock_Set_At_48MHz
	jz 	t2_int_start

	; Check skip variable
	mov	A, Skip_T2_Int
	jz	t2_int_start				; Execute this interrupt

	mov	Skip_T2_Int, #0
	ajmp	t2_int_exit

t2_int_start:
	mov	Skip_T2_Int, #1			; Skip next interrupt
ENDIF
	; Update RC pulse timeout counter 
	mov	A, Rcp_Timeout_Cntd			; RC pulse timeout count zero?
	jz	($+4)					; Yes - do not decrement

	dec	Rcp_Timeout_Cntd			; No decrement

	; Check RC pulse against stop value
	clr	C
	mov	A, New_Rcp				; Load new pulse value
	jz	t2_int_rcp_stop			; Check if pulse is below stop value

	; RC pulse higher than stop value, reset stop counter
	mov	Rcp_Stop_Cnt, #0			; Reset rcp stop counter
	ajmp	t2_int_exit

t2_int_rcp_stop:
	; RC pulse less than stop value
	mov	A, Rcp_Stop_Cnt			; Increment stop counter
	add	A, #1
	mov	Rcp_Stop_Cnt, A
	jnc	($+5)					; Branch if counter has not wrapped

	mov	Rcp_Stop_Cnt, #0FFh			; Set stop counter to max

t2_int_exit:
	pop	ACC			; Restore preserved registers
	pop	PSW
	reti
