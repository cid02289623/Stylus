;========================
; imu_lsm9ds1.s  (SPI slow debug with phase markers)
;========================
#include <xc.inc>

global  IMU_Test_Init
global  IMU_Test_Step

; CSAG -> RA4
CSAG_PORT   equ LATA
CSAG_TRIS   equ TRISA
CSAG_BIT    equ 4

psect udata_acs
d0: ds 1
d1: ds 1
d2: ds 1
rx: ds 1

psect code, class=CODE

;---------------------------------------
; visible delay (tweak if needed)
;---------------------------------------
Delay:
        movlw   0x20        ; outer loop (tune this)
        movwf   d0, A

D0:     movlw   0xFF
        movwf   d1, A

D1:     movlw   0xFF
        movwf   d2, A

D2:     decfsz  d2, F, A
        bra     D2

        decfsz  d1, F, A
        bra     D1

        decfsz  d0, F, A
        bra     D0

        return

;---------------------------------------
; SPI1 transfer: write W, return W read
;---------------------------------------
SPI1_Xfer:
        movwf   SSP1BUF, A
WB:     btfss   SSP1STAT, 0, A     ; BF
        bra     WB
        movf    SSP1BUF, W, A
        return

IMU_Test_Init:
        ; PORTJ LEDs output
        clrf    TRISJ, A
        clrf    LATJ,  A

        ; CSAG output, idle high
        bcf     CSAG_TRIS, CSAG_BIT, A
        bsf     CSAG_PORT, CSAG_BIT, A

        ; MSSP1 pins
        bsf     TRISC, 4, A        ; RC4 SDI1 (MISO) input
        bcf     TRISC, 3, A        ; RC3 SCK1 output
        bcf     TRISC, 5, A        ; RC5 SDO1 (MOSI) output

        ; SPI mode try: CKP=1 idle high, CKE=1
        movlw   01000000B
        movwf   SSP1STAT, A

        movlw   00110010B          ; SSPEN=1, CKP=1, Master Fosc/64
        movwf   SSP1CON1, A

        movf    SSP1BUF, W, A
        return

;---------------------------------------
; Step:
; A) Turn on ONLY LED0 while CS is LOW
; B) Turn on ONLY LED1, then sample raw MISO and reflect it on LED2
; C) Do SPI readback and display byte on PORTJ
;---------------------------------------
IMU_Test_Step:

; ---- A: CS LOW marker on LED0 ----
        clrf    LATJ, A
        bcf     CSAG_PORT, CSAG_BIT, A      ; CS low
        bsf     LATJ, 0, A                  ; phase marker (may invert on your board)
        call    Delay

; ---- B: raw MISO check ----
        clrf    LATJ, A
        bsf     LATJ, 1, A                  ; phase marker

        ; sample raw RC4 (MISO) -> show on LED2
        btfsc   PORTC, 4, A
        bsf     LATJ, 2, A
        btfss   PORTC, 4, A
        bcf     LATJ, 2, A
        call    Delay

; ---- C: SPI byte readback ----
        clrf    LATJ, A
        bsf     LATJ, 3, A                  ; phase marker

        ; keep CS low for the transfer
        bcf     CSAG_PORT, CSAG_BIT, A

        movlw   0x00
        call    SPI1_Xfer
        movwf   rx, A

        bsf     CSAG_PORT, CSAG_BIT, A      ; CS high

        ; show rx on LEDs
        movf    rx, W, A
        movwf   LATJ, A
        call    Delay

        return

end