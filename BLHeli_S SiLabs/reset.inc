IF MCU_48MHZ == 2
CSEG AT 2FFDh
ELSE
CSEG AT 19FDh
ENDIF
reset:
ljmp	pgm_start