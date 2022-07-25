;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Timer 0 interrupt routine
;
; No assumptions
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
IF MCU_48MHZ >= 1
t0_int:
	inc	Timer0_X
	reti
ENDIF
