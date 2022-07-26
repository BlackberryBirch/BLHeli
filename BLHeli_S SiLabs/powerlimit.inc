
;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Start ADC conversion
;
; No assumptions
;
; Start conversion used for measuring power supply voltage
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
start_adc_conversion:
	; Start adc
	Start_Adc 
	ret


;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Check temperature, power supply voltage and limit power
;
; No assumptions
;
; Used to limit main motor power in order to maintain the required voltage
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
check_temp_voltage_and_limit_power:
	inc	Adc_Conversion_Cnt			; Increment conversion counter
	clr	C
	mov	A, Adc_Conversion_Cnt		; Is conversion count equal to temp rate?
	subb	A, #8
	jc	check_voltage_start			; No - check voltage

	; Wait for ADC conversion to complete
	jnb	ADC0CN0_ADINT, check_temp_voltage_and_limit_power
	; Read ADC result
	Read_Adc_Result
	; Stop ADC
	Stop_Adc

	mov	Adc_Conversion_Cnt, #0		; Yes - temperature check. Reset counter
	mov	A, Temp2					; Move ADC MSB to Temp3
	mov	Temp3, A
	mov	Temp2, #Pgm_Enable_Temp_Prot	; Is temp protection enabled?
	mov	A, @Temp2
	jz	temp_check_exit			; No - branch

	mov	A, Temp3					; Is temperature reading below 256?
	jnz	temp_average_inc_dec		; No - proceed

	mov	A, Current_Average_Temp		; Yes -  decrement average
	jz	temp_average_updated		; Already zero - no change
	jmp	temp_average_dec			; Decrement 

temp_average_inc_dec:
	clr	C
	mov	A, Temp1					; Check if current temperature is above or below average
	subb	A, Current_Average_Temp
	jz	temp_average_updated_load_acc	; Equal - no change

	mov	A, Current_Average_Temp		; Above - increment average
	jnc	temp_average_inc				

	jz	temp_average_updated		; Below - decrement average if average is not already zero
temp_average_dec:
	dec	A						; Decrement average
	jmp	temp_average_updated

temp_average_inc:
	inc	A						; Increment average
	jz	temp_average_dec
	jmp	temp_average_updated

temp_average_updated_load_acc:
	mov	A, Current_Average_Temp
temp_average_updated:
	mov	Current_Average_Temp, A
	clr	C
	subb	A, Temp_Prot_Limit			; Is temperature below first limit?
	jc	temp_check_exit			; Yes - exit

	mov  Pwm_Limit, #192			; No - limit pwm

	clr	C
	subb	A, #(TEMP_LIMIT_STEP/2)		; Is temperature below second limit
	jc	temp_check_exit			; Yes - exit

	mov  Pwm_Limit, #128			; No - limit pwm

	clr	C
	subb	A, #(TEMP_LIMIT_STEP/2)		; Is temperature below third limit
	jc	temp_check_exit			; Yes - exit

	mov  Pwm_Limit, #64				; No - limit pwm

	clr	C
	subb	A, #(TEMP_LIMIT_STEP/2)		; Is temperature below final limit
	jc	temp_check_exit			; Yes - exit

	mov  Pwm_Limit, #0				; No - limit pwm

temp_check_exit:
	ret

check_voltage_start:
	; Increase pwm limit
	mov  A, Pwm_Limit
	add	A, #16			
	jnc	($+4)					; If not max - branch

	mov	A, #255

	mov	Pwm_Limit, A				; Increment limit 
	ret

