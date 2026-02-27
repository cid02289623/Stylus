;========================
; main.s  (SPI WHO_AM_I test)
;========================
#include <xc.inc>

extrn   IMU_Test_Init
extrn   IMU_Test_Step

psect   resetVec, abs
org     0x0000
        goto start

psect   code, class=CODE
start:
        ; LEDs on PORTJ
        clrf    TRISJ, A
        clrf    LATJ,  A

        call    IMU_Test_Init

loop:
        call    IMU_Test_Step
        bra     loop

end
