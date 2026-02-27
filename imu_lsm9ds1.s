;========================
; imu_lsm9ds1.s  (SPI WHO_AM_I read with phase LEDs)
; Wiring (your setup):
;   RC3 -> SCL   (SCK)
;   RC5 -> SDA   (MOSI)
;   RC4 -> SDOAG (MISO)
;   RA4 -> CSAG  (CS, active low)  **If flaky, add 10k pull-up to 3.3V or move CS to RB0**
;   CSM -> 3.3V  (mag deselected)
;   3v3 -> 3.3V, GND -> GND
;========================
#include <xc.inc>

global  IMU_Test_Init
global  IMU_Test_Step

;-------------------------
; CSAG -> RA4
;-------------------------
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
        movlw   0x20
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
WB:     btfss   SSP1STAT, 0, A     ; BF = bit0
        bra     WB
        movf    SSP1BUF, W, A
        return

;---------------------------------------
; Init: SPI master on MSSP1
;---------------------------------------
IMU_Test_Init:
        ; LEDs
        clrf    TRISJ, A
        clrf    LATJ,  A

        ; CSAG output, idle HIGH (deselect)
        bcf     CSAG_TRIS, CSAG_BIT, A
        bsf     CSAG_PORT, CSAG_BIT, A

        ; MSSP1 SPI pins
        bsf     TRISC, 4, A        ; RC4 SDI1 (MISO) input
        bcf     TRISC, 3, A        ; RC3 SCK1 output
        bcf     TRISC, 5, A        ; RC5 SDO1 (MOSI) output

        ; SPI mode attempt: CKP=1 idle high, CKE=1 (change to 0 if needed)
        movlw   b'01000000'        ; CKE=1, SMP=0
        movwf   SSP1STAT, A

        movlw   b'00110010'        ; SSPEN=1, CKP=1, Master Fosc/64
        movwf   SSP1CON1, A

        ; clear BF by reading buffer
        movf    SSP1BUF, W, A
        return

;---------------------------------------
; Step:
; A) LED0 while CS LOW
; B) LED1 marker (no longer doing raw MISO check)
; C) Read WHO_AM_I (0x0F) from A/G over SPI and show on PORTJ
;---------------------------------------
IMU_Test_Step:

; ---- A: CS LOW marker on LED0 ----
        clrf    LATJ, A
        bcf     CSAG_PORT, CSAG_BIT, A      ; CS low (select)
        bsf     LATJ, 0, A                  ; phase marker
        call    Delay

; ---- B: marker ----
        clrf    LATJ, A
        bsf     LATJ, 1, A
        call    Delay

; ---- C: SPI WHO_AM_I read ----
        clrf    LATJ, A
        bsf     LATJ, 3, A                  ; phase marker

        ; CS low for whole transaction
        bcf     CSAG_PORT, CSAG_BIT, A
        nop
        nop

        ; Send READ command for WHO_AM_I register 0x0F
        movlw   0x8F                        ; 0x80 | 0x0F = READ
        call    SPI1_Xfer                   ; throw away returned byte

        ; Clock in the WHO_AM_I value
        movlw   0x00                        ; dummy
        call    SPI1_Xfer                   ; W = WHO_AM_I (expect 0x68)
        movwf   rx, A

        ; CS high (deselect)
        bsf     CSAG_PORT, CSAG_BIT, A

        ; show rx on LEDs
        movf    rx, W, A
        movwf   LATJ, A
        call    Delay

        return

end
