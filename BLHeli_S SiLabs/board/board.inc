;**** **** **** **** ****
; List of enumerated supported ESCs
A_			EQU 1	; X  X  RC X  MC MB MA CC    X  X  Cc Cp Bc Bp Ac Ap
B_			EQU 2	; X  X  RC X  MC MB MA CC    X  X  Ap Ac Bp Bc Cp Cc
C_			EQU 3	; Ac Ap MC MB MA CC X  RC    X  X  X  X  Cc Cp Bc Bp
D_			EQU 4	; X  X  RC X  CC MA MC MB    X  X  Cc Cp Bc Bp Ac Ap	Com fets inverted
E_			EQU 5	; L1 L0 RC X  MC MB MA CC    X  L2 Cc Cp Bc Bp Ac Ap	A with LEDs
F_			EQU 6	; X  X  RC X  MA MB MC CC    X  X  Cc Cp Bc Bp Ac Ap
G_			EQU 7	; X  X  RC X  CC MA MC MB    X  X  Cc Cp Bc Bp Ac Ap	Like D, but noninverted com fets
H_			EQU 8	; RC X  X  X  MA MB CC MC    X  Ap Bp Cp X  Ac Bc Cc
I_			EQU 9	; X  X  RC X  MC MB MA CC    X  X  Ac Bc Cc Ap Bp Cp
J_			EQU 10	; L2 L1 L0 RC CC MB MC MA    X  X  Cc Bc Ac Cp Bp Ap	LEDs
K_			EQU 11	; X  X  MC X  MB CC MA RC    X  X  Ap Bp Cp Cc Bc Ac	Com fets inverted
L_			EQU 12	; X  X  RC X  CC MA MB MC    X  X  Ac Bc Cc Ap Bp Cp
M_			EQU 13	; MA MC CC MB RC L0 X  X     X  Cc Bc Ac Cp Bp Ap X     LED
N_			EQU 14	; X  X  RC X  MC MB MA CC    X  X  Cp Cc Bp Bc Ap Ac
O_			EQU 15	; X  X  RC X  CC MA MC MB    X  X  Cc Cp Bc Bp Ac Ap	Like D, but low side pwm
P_			EQU 16	; X  X  RC MA CC MB MC X     X  Cc Bc Ac Cp Bp Ap X
Q_			EQU 17	; Cp Bp Ap L1 L0 X  RC X     X  MA MB MC CC Cc Bc Ac    LEDs
R_			EQU 18	; X  X  RC X  MC MB MA CC    X  X  Ac Bc Cc Ap Bp Cp
S_			EQU 19  ; X  X  RC X  CC MA MC MB    X  X  Cc Cp Bc Bp Ac Ap    Like O, but com fets inverted
T_			EQU 20	; RC X  MA X  MB CC MC X     X  X  Cp Bp Ap Ac Bc Cc
U_			EQU 21	; MA MC CC MB RC L0 L1 L2    X  Cc Bc Ac Cp Bp Ap X	Like M, but with 3 LEDs
V_			EQU 22	; Cc X  RC X  MC CC MB MA    X  Ap Ac Bp X  X  Bc Cp
W_			EQU 23	; RC MC MB X  CC MA X  X     X  Ap Bp Cp X  X  X  X	Tristate gate driver
X_			EQU 24	; Reserved
Y_			EQU 25	; Reserved
Z_			EQU 26  ; X  X  RC X  CC MA MC MB    X  X  Cp Cc Bp Bc Ap Ac  	Like S, but pwm fets inverted

A_X_		EQU 30	; X  X  RC CC MA MC MB X     X  X  Cc Cp Bc Bp Ac Ap
B_X_		EQU 31	; X  X  RC CC MA MC MB X     X  X  Cc Cp Bc Bp Ac Ap
C_X_		EQU 32	; X  X  RC CC MA MC MB X     X  X  Cp Cc Bp Bc Ap Ac

;**** **** **** **** ****
; Select the port mapping to use (or unselect all for use with external batch compile file)
;ESCNO EQU A_
;ESCNO EQU B_
;ESCNO EQU C_
;ESCNO EQU D_
;ESCNO EQU E_
;ESCNO EQU F_
;ESCNO EQU G_
;ESCNO EQU H_
;ESCNO EQU I_
;ESCNO EQU J_
;ESCNO EQU K_
;ESCNO EQU L_
;ESCNO EQU M_
;ESCNO EQU N_
;ESCNO EQU O_
;ESCNO EQU P_
;ESCNO EQU Q_
;ESCNO EQU R_
;ESCNO EQU S_
;ESCNO EQU T_
;ESCNO EQU U_
;ESCNO EQU V_
;ESCNO EQU W_
;ESCNO EQU X_
;ESCNO EQU Y_
;ESCNO EQU Z_
;ESCNO EQU A_X_ ; Requires MCU_48MHZ=2
;ESCNO EQU B_X_	; Requires MCU_48MHZ=2
;ESCNO EQU C_X_	; Requires MCU_48MHZ=2

;**** **** **** **** ****
; Select the MCU type (or unselect for use with external batch compile file)
;MCU_48MHZ EQU	2

;**** **** **** **** ****
; Select the fet deadtime (or unselect for use with external batch compile file)
;FETON_DELAY EQU 50	; 20.4ns per step


;**** **** **** **** ****
; ESC selection statements
IF ESCNO == A_
$include (A.inc)	; Select pinout A
ENDIF

IF ESCNO == B_
$include (B.inc)	; Select pinout B
ENDIF

IF ESCNO == C_
$include (C.inc)	; Select pinout C
ENDIF

IF ESCNO == D_
$include (D.inc)	; Select pinout D
ENDIF

IF ESCNO == E_
$include (E.inc)	; Select pinout E
ENDIF

IF ESCNO == F_
$include (F.inc)	; Select pinout F
ENDIF

IF ESCNO == G_
$include (G.inc)	; Select pinout G
ENDIF

IF ESCNO == H_
$include (H.inc)	; Select pinout H
ENDIF

IF ESCNO == I_
$include (I.inc)	; Select pinout I
ENDIF

IF ESCNO == J_
$include (J.inc)	; Select pinout J
ENDIF

IF ESCNO == K_
$include (K.inc)	; Select pinout K
ENDIF

IF ESCNO == L_
$include (L.inc)	; Select pinout L
ENDIF

IF ESCNO == M_
$include (M.inc)	; Select pinout M
ENDIF

IF ESCNO == N_
$include (N.inc)	; Select pinout N
ENDIF

IF ESCNO == O_
$include (O.inc)	; Select pinout O
ENDIF

IF ESCNO == P_
$include (P.inc)	; Select pinout P
ENDIF

IF ESCNO == Q_
$include (Q.inc)	; Select pinout Q
ENDIF

IF ESCNO == R_
$include (R.inc)	; Select pinout R
ENDIF

IF ESCNO == S_
$include (S.inc)        ; Select pinout S
ENDIF

IF ESCNO == T_
$include (T.inc)        ; Select pinout T
ENDIF

IF ESCNO == U_
$include (U.inc)        ; Select pinout U
ENDIF

IF ESCNO == V_
$include (V.inc)        ; Select pinout V
ENDIF

IF ESCNO == W_
$include (W.inc)        ; Select pinout W
ENDIF

IF ESCNO == X_
;$include (X.inc)        ; Select pinout X
ENDIF

IF ESCNO == Y_
;$include (Y.inc)        ; Select pinout Y
ENDIF

IF ESCNO == Z_
$include (Z.inc)        ; Select pinout Z
ENDIF

IF ESCNO == A_X_
$include (A_X.inc)      ; Select pinout A_X
ENDIF

IF ESCNO == B_X_
$include (B_X.inc)      ; Select pinout B_X
ENDIF

IF ESCNO == C_X_
$include (C_X.inc)      ; Select pinout C_X
ENDIF
