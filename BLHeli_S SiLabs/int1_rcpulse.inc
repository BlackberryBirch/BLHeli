;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Int1 interrupt routine
;
; No assumptions
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
int1_int:	; Used for RC pulse timing
	clr	IE_EX1			; Disable int1 interrupts
	setb	TCON_TR1			; Start timer 1
	clr	TMR2CN0_TR2				; Timer 2 disabled
	mov	DShot_Frame_Start_L, TMR2L	; Read timer value
	mov	DShot_Frame_Start_H, TMR2H
	setb	TMR2CN0_TR2				; Timer 2 enabled
reti