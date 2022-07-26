;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Timer 3 interrupt routine
;
; No assumptions
; Requirements: Temp variables can NOT be used since PSW.x is not set
;               ACC can not be used, as it is not pushed to stack
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
t3_int:	; Used for commutation timing
	clr 	IE_EA			; Disable all interrupts
	anl	EIE1, #7Fh		; Disable timer 3 interrupts
	mov	TMR3RLL, #0FAh		; Set a short delay before next interrupt
	mov	TMR3RLH, #0FFh
	clr	Flags0.T3_PENDING 	; Flag that timer has wrapped
	anl	TMR3CN0, #07Fh		; Timer 3 interrupt flag cleared
	setb	IE_EA			; Enable all interrupts
	reti