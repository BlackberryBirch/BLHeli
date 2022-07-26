
;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Beeper routines (4 different entry points) 
;
; No assumptions
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
beep_f1:	; Entry point 1, load beeper frequency 1 settings
	mov	Temp3, #20	; Off wait loop length
	mov	Temp4, #120	; Number of beep pulses
	jmp	beep

beep_f2:	; Entry point 2, load beeper frequency 2 settings
	mov	Temp3, #16
	mov	Temp4, #140
	jmp	beep

beep_f3:	; Entry point 3, load beeper frequency 3 settings
	mov	Temp3, #13
	mov	Temp4, #180
	jmp	beep

beep_f4:	; Entry point 4, load beeper frequency 4 settings
	mov	Temp3, #11
	mov	Temp4, #200
	jmp	beep

beep:	; Beep loop start
	mov	A, Beep_Strength
	djnz	ACC, beep_start
	ret

beep_start:
	mov	Temp2, #2
beep_onoff:
	clr	A
	BcomFET_off		; BcomFET off
	djnz	ACC, $		; Allow some time after comfet is turned off
	BpwmFET_on		; BpwmFET on (in order to charge the driver of the BcomFET)
	djnz	ACC, $		; Let the pwmfet be turned on a while
	BpwmFET_off		; BpwmFET off again
	djnz	ACC, $		; Allow some time after pwmfet is turned off
	BcomFET_on		; BcomFET on
	djnz	ACC, $		; Allow some time after comfet is turned on
	; Turn on pwmfet
	mov	A, Temp2
	jb	ACC.0, beep_apwmfet_on
	ApwmFET_on		; ApwmFET on
beep_apwmfet_on:
	jnb	ACC.0, beep_cpwmfet_on
	CpwmFET_on		; CpwmFET on
beep_cpwmfet_on:
	mov	A, Beep_Strength
	djnz	ACC, $		
	; Turn off pwmfet
	mov	A, Temp2
	jb	ACC.0, beep_apwmfet_off
	ApwmFET_off		; ApwmFET off
beep_apwmfet_off:
	jnb	ACC.0, beep_cpwmfet_off
	CpwmFET_off		; CpwmFET off
beep_cpwmfet_off:
	mov	A, #150		; 25us off
	djnz	ACC, $		
	djnz	Temp2, beep_onoff
	; Copy variable
	mov	A, Temp3
	mov	Temp1, A	
beep_off:		; Fets off loop
	djnz	ACC, $
	djnz	Temp1,	beep_off
	djnz	Temp4,	beep
	BcomFET_off		; BcomFET off
	ret
