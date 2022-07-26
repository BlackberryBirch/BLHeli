

;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Commutation routines
;
; No assumptions
;
; Performs commutation switching 
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
; Comm phase 1 to comm phase 2
comm1comm2:	
	Set_RPM_Out
	jb	Flags3.PGM_DIR_REV, comm12_rev

	clr 	IE_EA				; Disable all interrupts
	BcomFET_off 				; Turn off comfet
	AcomFET_on				; Turn on comfet
	Set_Pwm_C					; To reapply power after a demag cut
	setb	IE_EA
	Set_Comp_Phase_B 			; Set comparator phase
	jmp	comm_exit

comm12_rev:	
	clr 	IE_EA				; Disable all interrupts
	BcomFET_off 				; Turn off comfet
	CcomFET_on				; Turn on comfet (reverse)
	Set_Pwm_A					; To reapply power after a demag cut
	setb	IE_EA
	Set_Comp_Phase_B 			; Set comparator phase
	jmp	comm_exit


; Comm phase 2 to comm phase 3
comm2comm3:	
	Clear_RPM_Out
	jb	Flags3.PGM_DIR_REV, comm23_rev

	clr 	IE_EA				; Disable all interrupts
	CpwmFET_off				; Turn off pwmfet
	Set_Pwm_B					; To reapply power after a demag cut
	AcomFET_on
	setb	IE_EA
	Set_Comp_Phase_C 			; Set comparator phase
	ajmp	comm_exit

comm23_rev:
	clr 	IE_EA				; Disable all interrupts
	ApwmFET_off				; Turn off pwmfet (reverse)
	Set_Pwm_B					; To reapply power after a demag cut
	CcomFET_on
	setb	IE_EA
	Set_Comp_Phase_A 			; Set comparator phase (reverse)
	ajmp	comm_exit


; Comm phase 3 to comm phase 4
comm3comm4:	
	Set_RPM_Out
	jb	Flags3.PGM_DIR_REV, comm34_rev

	clr 	IE_EA				; Disable all interrupts
	AcomFET_off 				; Turn off comfet
	CcomFET_on				; Turn on comfet
	Set_Pwm_B					; To reapply power after a demag cut
	setb	IE_EA
	Set_Comp_Phase_A 			; Set comparator phase
	jmp	comm_exit

comm34_rev:	
	clr 	IE_EA				; Disable all interrupts
	CcomFET_off 				; Turn off comfet (reverse)
	AcomFET_on				; Turn on comfet (reverse)
	Set_Pwm_B					; To reapply power after a demag cut
	setb	IE_EA
	Set_Comp_Phase_C 			; Set comparator phase (reverse)
	jmp	comm_exit


; Comm phase 4 to comm phase 5
comm4comm5:	
	Clear_RPM_Out
	jb	Flags3.PGM_DIR_REV, comm45_rev

	clr 	IE_EA				; Disable all interrupts
	BpwmFET_off				; Turn off pwmfet
	Set_Pwm_A					; To reapply power after a demag cut
	CcomFET_on
	setb	IE_EA
	Set_Comp_Phase_B 			; Set comparator phase
	jmp	comm_exit

comm45_rev:
	clr 	IE_EA				; Disable all interrupts
	BpwmFET_off				; Turn off pwmfet
	Set_Pwm_C
	AcomFET_on				; To reapply power after a demag cut
	setb	IE_EA
	Set_Comp_Phase_B 			; Set comparator phase
	jmp	comm_exit


; Comm phase 5 to comm phase 6
comm5comm6:	
	Set_RPM_Out
	jb	Flags3.PGM_DIR_REV, comm56_rev

	clr 	IE_EA				; Disable all interrupts
	CcomFET_off 				; Turn off comfet
	BcomFET_on				; Turn on comfet
	Set_Pwm_A					; To reapply power after a demag cut
	setb	IE_EA
	Set_Comp_Phase_C 			; Set comparator phase
	jmp	comm_exit

comm56_rev:
	clr 	IE_EA				; Disable all interrupts
	AcomFET_off 				; Turn off comfet (reverse)
	BcomFET_on				; Turn on comfet
	Set_Pwm_C					; To reapply power after a demag cut
	setb	IE_EA
	Set_Comp_Phase_A 			; Set comparator phase (reverse)
	jmp	comm_exit


; Comm phase 6 to comm phase 1
comm6comm1:	
	Clear_RPM_Out
	jb	Flags3.PGM_DIR_REV, comm61_rev

	clr 	IE_EA				; Disable all interrupts
	ApwmFET_off				; Turn off pwmfet
	Set_Pwm_C
	BcomFET_on				; To reapply power after a demag cut
	setb	IE_EA
	Set_Comp_Phase_A 			; Set comparator phase
	jmp	comm_exit

comm61_rev:
	clr 	IE_EA				; Disable all interrupts
	CpwmFET_off				; Turn off pwmfet (reverse)
	Set_Pwm_A
	BcomFET_on				; To reapply power after a demag cut
	setb	IE_EA
	Set_Comp_Phase_C 			; Set comparator phase (reverse)

comm_exit:
	ret


;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Switch power off routine
;
; No assumptions
;
; Switches all fets off 
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
switch_power_off:
	All_pwmFETs_Off		; Turn off all pwm fets
	All_comFETs_Off		; Turn off all commutation fets
	Set_Pwms_Off
	ret			
