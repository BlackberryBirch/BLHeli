
;**** **** **** **** ****
; Programming defaults
;
DEFAULT_PGM_STARTUP_PWR 				EQU 9 	; 1=0.031 2=0.047 3=0.063 4=0.094 5=0.125 6=0.188	7=0.25  8=0.38  9=0.50  10=0.75 11=1.00 12=1.25 13=1.50
DEFAULT_PGM_COMM_TIMING				EQU 3 	; 1=Low 		2=MediumLow 	3=Medium 		4=MediumHigh 	5=High
DEFAULT_PGM_DEMAG_COMP 				EQU 2 	; 1=Disabled	2=Low		3=High
DEFAULT_PGM_DIRECTION				EQU 1 	; 1=Normal 	2=Reversed	3=Bidir		4=Bidir rev
DEFAULT_PGM_BEEP_STRENGTH			EQU 40	; Beep strength
DEFAULT_PGM_BEACON_STRENGTH			EQU 80	; Beacon strength
DEFAULT_PGM_BEACON_DELAY				EQU 4 	; 1=1m		2=2m			3=5m			4=10m		5=Infinite

; COMMON
DEFAULT_PGM_ENABLE_TX_PROGRAM 		EQU 1 	; 1=Enabled 	0=Disabled
DEFAULT_PGM_MIN_THROTTLE				EQU 37	; 4*37+1000=1148
DEFAULT_PGM_MAX_THROTTLE				EQU 208	; 4*208+1000=1832
DEFAULT_PGM_CENTER_THROTTLE			EQU 122	; 4*122+1000=1488 (used in bidirectional mode)
DEFAULT_PGM_ENABLE_TEMP_PROT	 		EQU 7 	; 0=Disabled	1=80C	2=90C	3=100C	4=110C	5=120C	6=130C	7=140C
DEFAULT_PGM_ENABLE_POWER_PROT 		EQU 1 	; 1=Enabled 	0=Disabled
DEFAULT_PGM_BRAKE_ON_STOP	 		EQU 0 	; 1=Enabled 	0=Disabled
DEFAULT_PGM_LED_CONTROL	 			EQU 0 	; Byte for LED control. 2bits per LED, 0=Off, 1=On

;**** **** **** **** ****
; Temporary register definitions
Temp1		EQU	R0
Temp2		EQU	R1
Temp3		EQU	R2
Temp4		EQU	R3
Temp5		EQU	R4
Temp6		EQU	R5
Temp7		EQU	R6
Temp8		EQU	R7

;**** **** **** **** ****
; Register definitions
DSEG AT 20h					; Variables segment 

Bit_Access:				DS	1		; MUST BE AT THIS ADDRESS. Variable at bit accessible address (for non interrupt routines)
Bit_Access_Int:			DS	1		; Variable at bit accessible address (for interrupts)

Rcp_Outside_Range_Cnt:		DS	1		; RC pulse outside range counter (incrementing) 
Rcp_Timeout_Cntd:			DS	1		; RC pulse timeout counter (decrementing) 


Flags0:					DS	1    	; State flags. Reset upon init_start
T3_PENDING				EQU 	0		; Timer 3 pending flag
DEMAG_DETECTED				EQU 	1		; Set when excessive demag time is detected
COMP_TIMED_OUT				EQU 	2		; Set when comparator reading timed out
;						EQU 	3
;						EQU 	4
;						EQU 	5	
;						EQU 	6	
;						EQU 	7	


Flags1:					DS	1    	; State flags. Reset upon init_start 
STARTUP_PHASE				EQU 	0		; Set when in startup phase
INITIAL_RUN_PHASE			EQU	1		; Set when in initial run phase, before synchronized run is achieved
MOTOR_STARTED				EQU 	2		; Set when motor is started
DIR_CHANGE_BRAKE			EQU 	3		; Set when braking before direction change
HIGH_RPM					EQU 	4		; Set when motor rpm is high (Comm_Period4x_H less than 2)
;						EQU 	5
;						EQU 	6	
;						EQU 	7	

Flags2:					DS	1		; State flags. NOT reset upon init_start
RCP_UPDATED				EQU 	0		; New RC pulse length value available
RCP_ONESHOT125				EQU 	1		; RC pulse input is OneShot125 (125-250us)
RCP_ONESHOT42				EQU 	2		; RC pulse input is OneShot42 (41.67-83us)
RCP_MULTISHOT				EQU 	3		; RC pulse input is Multishot (5-25us)
RCP_DSHOT					EQU 	4		; RC pulse input is digital shot
RCP_DIR_REV				EQU 	5		; RC pulse direction in bidirectional mode
RCP_FULL_RANGE				EQU 	6		; When set full input signal range is used (1000-2000us) and stored calibration values are ignored
;						EQU 	7	

Flags3:					DS	1		; State flags. NOT reset upon init_start
PGM_DIR_REV				EQU 	0		; Programmed direction. 0=normal, 1=reversed
PGM_BIDIR_REV				EQU 	1		; Programmed bidirectional direction. 0=normal, 1=reversed
PGM_BIDIR					EQU 	2		; Programmed bidirectional operation. 0=normal, 1=bidirectional
;						EQU 	3
;						EQU 	4	
;						EQU 	5	
;						EQU 	6	
;						EQU 	7	


;**** **** **** **** ****
; RAM definitions
DSEG AT 30h						; Ram data segment, direct addressing
Initial_Arm:				DS	1		; Variable that is set during the first arm sequence after power on

Min_Throttle_L:			DS	1		; Minimum throttle scaled (lo byte)
Min_Throttle_H:			DS	1		; Minimum throttle scaled (hi byte)
Center_Throttle_L:			DS	1		; Center throttle scaled (lo byte)
Center_Throttle_H:			DS	1		; Center throttle scaled (hi byte)
Max_Throttle_L:			DS	1		; Maximum throttle scaled (lo byte)
Max_Throttle_H:			DS	1		; Maximum throttle scaled (hi byte)

Power_On_Wait_Cnt_L: 		DS	1		; Power on wait counter (lo byte)
Power_On_Wait_Cnt_H: 		DS	1		; Power on wait counter (hi byte)

Startup_Cnt:				DS	1		; Startup phase commutations counter (incrementing)
Startup_Zc_Timeout_Cntd:		DS	1		; Startup zero cross timeout counter (decrementing)
Initial_Run_Rot_Cntd:		DS	1		; Initial run rotations counter (decrementing)
Stall_Cnt:				DS	1		; Counts start/run attempts that resulted in stall. Reset upon a proper stop
Demag_Detected_Metric:		DS	1		; Metric used to gauge demag event frequency
Demag_Pwr_Off_Thresh:		DS	1		; Metric threshold above which power is cut
Low_Rpm_Pwr_Slope:			DS	1		; Sets the slope of power increase for low rpms

Timer0_X:					DS	1		; Timer 0 extended byte
Timer2_X:					DS	1		; Timer 2 extended byte
Prev_Comm_L:				DS	1		; Previous commutation timer 3 timestamp (lo byte)
Prev_Comm_H:				DS	1		; Previous commutation timer 3 timestamp (hi byte)
Prev_Comm_X:				DS	1		; Previous commutation timer 3 timestamp (ext byte)
Prev_Prev_Comm_L:			DS	1		; Pre-previous commutation timer 3 timestamp (lo byte)
Prev_Prev_Comm_H:			DS	1		; Pre-previous commutation timer 3 timestamp (hi byte)
Comm_Period4x_L:			DS	1		; Timer 3 counts between the last 4 commutations (lo byte)
Comm_Period4x_H:			DS	1		; Timer 3 counts between the last 4 commutations (hi byte)
Comparator_Read_Cnt: 		DS	1		; Number of comparator reads done

Wt_Adv_Start_L:			DS	1		; Timer 3 start point for commutation advance timing (lo byte)
Wt_Adv_Start_H:			DS	1		; Timer 3 start point for commutation advance timing (hi byte)
Wt_Zc_Scan_Start_L:			DS	1		; Timer 3 start point from commutation to zero cross scan (lo byte)
Wt_Zc_Scan_Start_H:			DS	1		; Timer 3 start point from commutation to zero cross scan (hi byte)
Wt_Zc_Tout_Start_L:			DS	1		; Timer 3 start point for zero cross scan timeout (lo byte)
Wt_Zc_Tout_Start_H:			DS	1		; Timer 3 start point for zero cross scan timeout (hi byte)
Wt_Comm_Start_L:			DS	1		; Timer 3 start point from zero cross to commutation (lo byte)
Wt_Comm_Start_H:			DS	1		; Timer 3 start point from zero cross to commutation (hi byte)

Dshot_Cmd:				DS	1		; Dshot command
Dshot_Cmd_Cnt:				DS  	1		; Dshot command count

New_Rcp:					DS	1		; New RC pulse value in pca counts
Rcp_Stop_Cnt:				DS	1		; Counter for RC pulses below stop value

Power_Pwm_Reg_L:			DS	1		; Power pwm register setting (lo byte)
Power_Pwm_Reg_H:			DS	1		; Power pwm register setting (hi byte). 0x3F is minimum power
Damp_Pwm_Reg_L:			DS	1		; Damping pwm register setting (lo byte)
Damp_Pwm_Reg_H:			DS	1		; Damping pwm register setting (hi byte)
Current_Power_Pwm_Reg_H:		DS	1		; Current power pwm register setting that is loaded in the PCA register (hi byte)

Pwm_Limit:				DS	1		; Maximum allowed pwm 
Pwm_Limit_By_Rpm:			DS	1		; Maximum allowed pwm for low or high rpms
Pwm_Limit_Beg:				DS	1		; Initial pwm limit

Adc_Conversion_Cnt:			DS	1		; Adc conversion counter

Current_Average_Temp:		DS	1		; Current average temperature (lo byte ADC reading, assuming hi byte is 1)

Throttle_Gain:				DS	1		; Gain to be applied to RCP value
Throttle_Gain_M:			DS	1		; Gain to be applied to RCP value (multiplier 0=1x, 1=2x, 2=4x etc))
Throttle_Gain_BD_Rev:		DS	1		; Gain to be applied to RCP value for reverse direction in bidirectional mode
Throttle_Gain_BD_Rev_M:		DS	1		; Gain to be applied to RCP value for reverse direction in bidirectional mode (multiplier 0=1x, 1=2x, 2=4x etc)
Beep_Strength:				DS	1		; Strength of beeps

Skip_T2_Int:				DS	1		; Set for 48MHz MCUs when timer 2 interrupt shall be ignored
Clock_Set_At_48MHz:			DS	1		; Variable set if 48MHz MCUs run at 48MHz

Flash_Key_1:				DS	1		; Flash key one
Flash_Key_2:				DS	1		; Flash key two

Temp_Prot_Limit:			DS	1		; Temperature protection limit

DShot_Pwm_Thr:				DS	1		; DShot pulse width threshold value
DShot_Timer_Preset:			DS	1		; DShot timer preset for frame sync detection
DShot_Frame_Start_L:		DS	1		; DShot frame start timestamp (lo byte)
DShot_Frame_Start_H:		DS	1		; DShot frame start timestamp (hi byte)
DShot_Frame_Length_Thr:		DS	1		; DShot frame length criteria (in units of 4 timer 2 ticks)

; Indirect addressing data segment. The variables below must be in this sequence
ISEG AT 080h					
_Pgm_Gov_P_Gain:			DS	1		; Programmed governor P gain
_Pgm_Gov_I_Gain:			DS	1		; Programmed governor I gain
_Pgm_Gov_Mode:				DS	1		; Programmed governor mode
_Pgm_Low_Voltage_Lim:		DS	1		; Programmed low voltage limit
_Pgm_Motor_Gain:			DS	1		; Programmed motor gain
_Pgm_Motor_Idle:			DS	1		; Programmed motor idle speed
Pgm_Startup_Pwr:			DS	1		; Programmed startup power
_Pgm_Pwm_Freq:				DS	1		; Programmed pwm frequency
Pgm_Direction:				DS	1		; Programmed rotation direction
Pgm_Input_Pol:				DS	1		; Programmed input pwm polarity
Initialized_L_Dummy:		DS	1		; Place holder
Initialized_H_Dummy:		DS	1		; Place holder
Pgm_Enable_TX_Program:		DS 	1		; Programmed enable/disable value for TX programming
_Pgm_Main_Rearm_Start:		DS 	1		; Programmed enable/disable re-arming main every start 
_Pgm_Gov_Setup_Target:		DS 	1		; Programmed main governor setup target
_Pgm_Startup_Rpm:			DS	1		; Programmed startup rpm (unused - place holder)
_Pgm_Startup_Accel:			DS	1		; Programmed startup acceleration (unused - place holder)
_Pgm_Volt_Comp:			DS	1		; Place holder
Pgm_Comm_Timing:			DS	1		; Programmed commutation timing
_Pgm_Damping_Force:			DS	1		; Programmed damping force (unused - place holder)
_Pgm_Gov_Range:			DS	1		; Programmed governor range
_Pgm_Startup_Method:		DS	1		; Programmed startup method (unused - place holder)
Pgm_Min_Throttle:			DS	1		; Programmed throttle minimum
Pgm_Max_Throttle:			DS	1		; Programmed throttle maximum
Pgm_Beep_Strength:			DS	1		; Programmed beep strength
Pgm_Beacon_Strength:		DS	1		; Programmed beacon strength
Pgm_Beacon_Delay:			DS	1		; Programmed beacon delay
_Pgm_Throttle_Rate:			DS	1		; Programmed throttle rate (unused - place holder)
Pgm_Demag_Comp:			DS	1		; Programmed demag compensation
_Pgm_BEC_Voltage_High:		DS	1		; Programmed BEC voltage
Pgm_Center_Throttle:		DS	1		; Programmed throttle center (in bidirectional mode)
_Pgm_Main_Spoolup_Time:		DS	1		; Programmed main spoolup time
Pgm_Enable_Temp_Prot:		DS	1		; Programmed temperature protection enable
Pgm_Enable_Power_Prot:		DS	1		; Programmed low rpm power protection enable
_Pgm_Enable_Pwm_Input:		DS	1		; Programmed PWM input signal enable
_Pgm_Pwm_Dither:			DS	1		; Programmed output PWM dither
Pgm_Brake_On_Stop:			DS	1		; Programmed braking when throttle is zero
Pgm_LED_Control:			DS	1		; Programmed LED control

; The sequence of the variables below is no longer of importance
Pgm_Startup_Pwr_Decoded:		DS	1		; Programmed startup power decoded


; Indirect addressing data segment
ISEG AT 0D0h					
Temp_Storage:				DS	48		; Temporary storage

;**** **** **** **** ****
IF MCU_48MHZ == 2
CSEG AT 3000h            ; "Eeprom" segment
ELSE
CSEG AT 1A00h            ; "Eeprom" segment
ENDIF
EEPROM_FW_MAIN_REVISION		EQU	16		; Main revision of the firmware
EEPROM_FW_SUB_REVISION		EQU	7		; Sub revision of the firmware
EEPROM_LAYOUT_REVISION		EQU	33		; Revision of the EEPROM layout

Eep_FW_Main_Revision:		DB	EEPROM_FW_MAIN_REVISION			; EEPROM firmware main revision number
Eep_FW_Sub_Revision:		DB	EEPROM_FW_SUB_REVISION			; EEPROM firmware sub revision number
Eep_Layout_Revision:		DB	EEPROM_LAYOUT_REVISION			; EEPROM layout revision number

_Eep_Pgm_Gov_P_Gain:		DB	0FFh	
_Eep_Pgm_Gov_I_Gain:		DB	0FFh	
_Eep_Pgm_Gov_Mode:			DB	0FFh	
_Eep_Pgm_Low_Voltage_Lim:	DB	0FFh							
_Eep_Pgm_Motor_Gain:		DB	0FFh	
_Eep_Pgm_Motor_Idle:		DB	0FFh						
Eep_Pgm_Startup_Pwr:		DB	DEFAULT_PGM_STARTUP_PWR			; EEPROM copy of programmed startup power
_Eep_Pgm_Pwm_Freq:			DB	0FFh	
Eep_Pgm_Direction:			DB	DEFAULT_PGM_DIRECTION			; EEPROM copy of programmed rotation direction
_Eep_Pgm_Input_Pol:			DB	0FFh
Eep_Initialized_L:			DB	055h							; EEPROM initialized signature low byte
Eep_Initialized_H:			DB	0AAh							; EEPROM initialized signature high byte
Eep_Enable_TX_Program:		DB	DEFAULT_PGM_ENABLE_TX_PROGRAM		; EEPROM TX programming enable
_Eep_Main_Rearm_Start:		DB	0FFh							
_Eep_Pgm_Gov_Setup_Target:	DB	0FFh							
_Eep_Pgm_Startup_Rpm:		DB	0FFh
_Eep_Pgm_Startup_Accel:		DB	0FFh
_Eep_Pgm_Volt_Comp:			DB	0FFh	
Eep_Pgm_Comm_Timing:		DB	DEFAULT_PGM_COMM_TIMING			; EEPROM copy of programmed commutation timing
_Eep_Pgm_Damping_Force:		DB	0FFh
_Eep_Pgm_Gov_Range:			DB	0FFh	
_Eep_Pgm_Startup_Method:		DB	0FFh
Eep_Pgm_Min_Throttle:		DB	DEFAULT_PGM_MIN_THROTTLE			; EEPROM copy of programmed minimum throttle
Eep_Pgm_Max_Throttle:		DB	DEFAULT_PGM_MAX_THROTTLE			; EEPROM copy of programmed minimum throttle
Eep_Pgm_Beep_Strength:		DB	DEFAULT_PGM_BEEP_STRENGTH		; EEPROM copy of programmed beep strength
Eep_Pgm_Beacon_Strength:		DB	DEFAULT_PGM_BEACON_STRENGTH		; EEPROM copy of programmed beacon strength
Eep_Pgm_Beacon_Delay:		DB	DEFAULT_PGM_BEACON_DELAY			; EEPROM copy of programmed beacon delay
_Eep_Pgm_Throttle_Rate:		DB	0FFh
Eep_Pgm_Demag_Comp:			DB	DEFAULT_PGM_DEMAG_COMP			; EEPROM copy of programmed demag compensation
_Eep_Pgm_BEC_Voltage_High:	DB	0FFh	
Eep_Pgm_Center_Throttle:		DB	DEFAULT_PGM_CENTER_THROTTLE		; EEPROM copy of programmed center throttle
_Eep_Pgm_Main_Spoolup_Time:	DB	0FFh
Eep_Pgm_Temp_Prot_Enable:	DB	DEFAULT_PGM_ENABLE_TEMP_PROT		; EEPROM copy of programmed temperature protection enable
Eep_Pgm_Enable_Power_Prot:	DB	DEFAULT_PGM_ENABLE_POWER_PROT		; EEPROM copy of programmed low rpm power protection enable
_Eep_Pgm_Enable_Pwm_Input:	DB	0FFh	
_Eep_Pgm_Pwm_Dither:		DB	0FFh	
Eep_Pgm_Brake_On_Stop:		DB	DEFAULT_PGM_BRAKE_ON_STOP		; EEPROM copy of programmed braking when throttle is zero
Eep_Pgm_LED_Control:		DB	DEFAULT_PGM_LED_CONTROL			; EEPROM copy of programmed LED control

Eep_Dummy:				DB	0FFh							; EEPROM address for safety reason

IF MCU_48MHZ == 2
CSEG AT 3060h
ELSE
CSEG AT 1A60h
ENDIF
Eep_Name:					DB	"                "				; Name tag (16 Bytes)

;**** **** **** **** ****
Interrupt_Table_Definition		; SiLabs interrupts
CSEG AT 80h			; Code segment after interrupt vectors 

;**** **** **** **** ****

; Table definitions
STARTUP_POWER_TABLE:  	DB 	04h, 06h, 08h, 0Ch, 10h, 18h, 20h, 30h, 40h, 60h, 80h, 0A0h, 0C0h
