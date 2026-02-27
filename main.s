;========================
; main.s  (LSM9DS1 gyro SPI quick test)
; PIC18F87K22
;========================
#include <xc.inc>

; --- exports from imu module ---
extrn   IMU_SPI_Init
extrn   IMU_Gyro_WhoAmI_Test
extrn   IMU_Gyro_Config
extrn   IMU_Gyro_Read_XL

; --- simple delay from your LCD module (or replace with your own) ---
extrn   LCD_delay_ms

psect   code, abs
rst:    org     0x0000
        goto    start

psect   code
start:
        ; --- PORTJ LEDs for visible output ---
        clrf    TRISJ, A
        clrf    LATJ,  A

        ; --- SPI init + WHO_AM_I test ---
        call    IMU_SPI_Init
        call    IMU_Gyro_WhoAmI_Test      ; returns W = WHO_AM_I_XG
        movwf   LATJ, A                  ; show WHO_AM_I on LEDs

        ; if WHO_AM_I != 0x68, blink all LEDs fast forever
        movlw   0x68
        cpfseq  LATJ, A
        bra     bad_sensor

        ; --- configure gyro then stream X low byte to LEDs ---
        call    IMU_Gyro_Config

loop:
        call    IMU_Gyro_Read_XL         ; returns W = OUT_X_L_G
        movwf   LATJ, A                  ; show changing byte on LEDs

        movlw   50
        call    LCD_delay_ms
        bra     loop

bad_sensor:
        movlw   0xFF
        movwf   LATJ, A
        movlw   100
        call    LCD_delay_ms
        clrf    LATJ, A
        movlw   100
        call    LCD_delay_ms
        bra     bad_sensor

end rst
