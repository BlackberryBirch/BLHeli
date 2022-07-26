
;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; No signal entry point
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
init_no_signal:
	; Disable interrupts explicitly
	clr	IE_EA
	; Initialize flash keys to invalid values
	mov	Flash_Key_1, #0
	mov	Flash_Key_2, #0
	; Check if input signal is high for more than 15ms
	mov	Temp1, #250
input_high_check_1:
	mov	Temp2, #250
input_high_check_2:
	jnb	RTX_PORT.RTX_PIN, bootloader_done	; Look for low
	djnz	Temp2, input_high_check_2
	djnz	Temp1, input_high_check_1

IF MCU_48MHZ == 2
	ljmp	0F000h			; Jump to bootloader
ELSE
	ljmp	1C00h			; Jump to bootloader
ENDIF
