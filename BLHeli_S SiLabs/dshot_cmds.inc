
check_dshot_cmd:
	clr	C
	mov 	A, Dshot_Cmd
	subb A, #1
	jnz 	dshot_beep_2

	clr 	IE_EA
	call	switch_power_off		; Switch power off in case braking is set
	mov	Temp1, #Pgm_Beacon_Strength
	mov	Beep_Strength, @Temp1
	call beep_f1
	mov	Temp1, #Pgm_Beep_Strength
	mov	Beep_Strength, @Temp1
	setb	IE_EA	
	call wait100ms	
	jmp 	clear_dshot_cmd

dshot_beep_2:	
	clr	C
	mov 	A, Dshot_Cmd
	subb A, #2
	jnz 	dshot_beep_3

	clr 	IE_EA
	call	switch_power_off		; Switch power off in case braking is set
	mov	Temp1, #Pgm_Beacon_Strength
	mov	Beep_Strength, @Temp1
	call beep_f2
	mov	Temp1, #Pgm_Beep_Strength
	mov	Beep_Strength, @Temp1
	setb	IE_EA	
	call wait100ms	
	jmp 	clear_dshot_cmd

dshot_beep_3:		
	clr	C
	mov 	A, Dshot_Cmd
	subb A, #3
	jnz 	dshot_beep_4

	clr 	IE_EA
	call	switch_power_off		; Switch power off in case braking is set
	mov	Temp1, #Pgm_Beacon_Strength
	mov	Beep_Strength, @Temp1
	call beep_f3
	mov	Temp1, #Pgm_Beep_Strength
	mov	Beep_Strength, @Temp1
	setb	IE_EA	
	call wait100ms	
	jmp 	clear_dshot_cmd

dshot_beep_4:
	clr	C
	mov 	A, Dshot_Cmd
	subb A, #4
	jnz 	dshot_beep_5

	clr 	IE_EA
	call	switch_power_off		; Switch power off in case braking is set
	mov	Temp1, #Pgm_Beacon_Strength
	mov	Beep_Strength, @Temp1
	call beep_f4
	mov	Temp1, #Pgm_Beep_Strength
	mov	Beep_Strength, @Temp1
	setb	IE_EA	
	call wait100ms		
	jmp 	clear_dshot_cmd

dshot_beep_5:
	clr	C
	mov 	A, Dshot_Cmd
	subb A, #5
	jnz 	dshot_direction_1

	clr 	IE_EA
	call	switch_power_off		; Switch power off in case braking is set
	mov	Temp1, #Pgm_Beacon_Strength
	mov	Beep_Strength, @Temp1
	call beep_f4
	mov	Temp1, #Pgm_Beep_Strength
	mov	Beep_Strength, @Temp1
	setb	IE_EA	
	call wait100ms	
	jmp 	clear_dshot_cmd

dshot_direction_1:
	clr	C
	mov 	A, Dshot_Cmd
	subb A, #7
	jnz 	dshot_direction_2

	clr 	C
	mov 	A, Dshot_Cmd_Cnt
	subb A, #6 					; Needs to receive it 6 times in a row
	jnc 	($+4) 					; Same as "jc dont_clear_dshot_cmd"
	ajmp wait_for_power_on_not_missing

	mov	A, #1
	jnb	Flags3.PGM_BIDIR, ($+5)
	mov	A, #3
	mov	Temp1, #Pgm_Direction
	mov	@Temp1, A
	clr 	Flags3.PGM_DIR_REV
	clr 	Flags3.PGM_BIDIR_REV
	jmp 	clear_dshot_cmd

dshot_direction_2:
	clr	C
	mov 	A, Dshot_Cmd
	subb A, #8
	jnz 	dshot_direction_bidir_off

	clr 	C
	mov 	A, Dshot_Cmd_Cnt
	subb A, #6 					; Needs to receive it 6 times in a row
	jnc 	($+4) 					; Same as "jc dont_clear_dshot_cmd"
	ajmp wait_for_power_on_not_missing

	mov	A, #2
	jnb	Flags3.PGM_BIDIR, ($+5)
	mov	A, #4
	mov	Temp1, #Pgm_Direction
	mov	@Temp1, A
	setb Flags3.PGM_DIR_REV
	setb Flags3.PGM_BIDIR_REV
	jmp 	clear_dshot_cmd

dshot_direction_bidir_off:
	clr	C
	mov 	A, Dshot_Cmd
	subb A, #9
	jnz 	dshot_direction_bidir_on

	clr 	C
	mov 	A, Dshot_Cmd_Cnt
	subb A, #6 					; Needs to receive it 6 times in a row
	jnc 	($+4) 					; Same as "jc dont_clear_dshot_cmd"
	ajmp wait_for_power_on_not_missing

	jnb	Flags3.PGM_BIDIR, dshot_direction_bidir_on

	clr	C
	mov	Temp1, #Pgm_Direction
	mov	A, @Temp1
	subb	A, #2
	mov	@Temp1, A
	clr 	Flags3.PGM_BIDIR
	jmp 	clear_dshot_cmd

dshot_direction_bidir_on:
	clr	C
	mov 	A, Dshot_Cmd
	subb A, #10
	jnz 	dshot_direction_normal

	clr 	C
	mov 	A, Dshot_Cmd_Cnt
	subb A, #6 					; Needs to receive it 6 times in a row
	jnc 	($+4) 					; Same as "jc dont_clear_dshot_cmd"
	ajmp wait_for_power_on_not_missing

	jb	Flags3.PGM_BIDIR, dshot_direction_normal

	mov	Temp1, #Pgm_Direction
	mov	A, @Temp1
	add	A, #2
	mov	@Temp1, A
	setb	Flags3.PGM_BIDIR
	jmp 	clear_dshot_cmd

dshot_direction_normal: 
	clr	C
	mov 	A, Dshot_Cmd
	subb A, #20
	jnz 	dshot_direction_reverse

	clr 	C
	mov 	A, Dshot_Cmd_Cnt
	subb A, #6 					; Needs to receive it 6 times in a row
	jnc 	($+4) 					; Same as "jc dont_clear_dshot_cmd"
	ajmp wait_for_power_on_not_missing

	clr	IE_EA					; DPTR used in interrupts
	mov	DPTR, #Eep_Pgm_Direction		; Read from flash
	mov	A, #0
	movc	A, @A+DPTR
	setb	IE_EA
	mov	Temp1, #Pgm_Direction
	mov	@Temp1, A
	rrc	A						; Lsb to carry
	clr 	Flags3.PGM_DIR_REV
	clr 	Flags3.PGM_BIDIR_REV
	jc	($+4)
	setb	Flags3.PGM_DIR_REV
	jc	($+4)
	setb	Flags3.PGM_BIDIR_REV
	jmp 	clear_dshot_cmd

dshot_direction_reverse: 			; Temporary reverse
	clr	C
	mov 	A, Dshot_Cmd
	subb A, #21
	jnz 	dshot_save_settings

	clr 	C
	mov 	A, Dshot_Cmd_Cnt
	subb A, #6 					; Needs to receive it 6 times in a row
	jc 	dont_clear_dshot_cmd
	
	clr	IE_EA					; DPTR used in interrupts
	mov	DPTR, #Eep_Pgm_Direction		; Read from flash
	mov	A, #0
	movc	A, @A+DPTR
	setb	IE_EA
	mov	Temp1, A
	cjne	Temp1, #1, ($+5)
	mov	A, #2
	cjne	Temp1, #2, ($+5)
	mov	A, #1
	cjne	Temp1, #3, ($+5)
	mov	A, #4
	cjne	Temp1, #4, ($+5)
	mov	A, #3
	mov	Temp1, #Pgm_Direction
	mov	@Temp1, A
	rrc	A						; Lsb to carry
	clr 	Flags3.PGM_DIR_REV
	clr 	Flags3.PGM_BIDIR_REV
	jc	($+4)
	setb	Flags3.PGM_DIR_REV
	jc	($+4)
	setb	Flags3.PGM_BIDIR_REV
	jmp 	clear_dshot_cmd

dshot_save_settings:
	clr	C
	mov 	A, Dshot_Cmd
	subb A, #12
	jnz 	clear_dshot_cmd

	mov	Flash_Key_1, #0A5h			; Initialize flash keys to valid values
	mov	Flash_Key_2, #0F1h
	clr 	C
	mov 	A, Dshot_Cmd_Cnt
	subb A, #6 					; Needs to receive it 6 times in a row
	jc 	dont_clear_dshot_cmd

	call erase_and_store_all_in_eeprom
	setb	IE_EA
	
clear_dshot_cmd:
	mov 	Dshot_Cmd, #0
	mov 	Dshot_Cmd_Cnt, #0

dont_clear_dshot_cmd:
	mov	Flash_Key_1, #0			; Initialize flash keys to invalid values
	mov	Flash_Key_2, #0
	jmp 	wait_for_power_on_not_missing
