;========================
; imu_lsm9ds1.s  (LSM9DS1 gyro over SPI using MSSP1)
;========================
#include <xc.inc>

global  IMU_SPI_Init
global  IMU_Gyro_WhoAmI_Test
global  IMU_Gyro_Config
global  IMU_Gyro_Read_XL

; optional delay (only used for tiny CS settle pauses)
extrn   LCD_delay_ms

;------------------------
; Pin choice for CS_G
; Using RA4 as chip select
;------------------------
CSG_LAT     equ LATAbits.LATA4
CSG_TRIS    equ TRISAbits.TRISA4

;------------------------
; LSM9DS1 gyro registers
;------------------------
WHO_AM_I_XG     equ 0x0F
CTRL_REG1_G     equ 0x10
CTRL_REG4       equ 0x1E
OUT_X_L_G       equ 0x18

; SPI R/W + auto-inc bits (LSM9DS1 style)
SPI_READ        equ 0x80
SPI_AUTOINC     equ 0x40

psect udata_acs
tmp:    ds 1

psect code

;========================
; IMU_SPI_Init
; - MSSP1 SPI master, Mode 0-ish (CKP=0, CKE=1)
; - SCK1=RC3 output, SDO1=RC5 output, SDI1=RC4 input
; - CS_G on RA4 (manual)
;========================
IMU_SPI_Init:
        ; --- CS pin ---
        bcf     CSG_TRIS, A          ; RA4 output
        bsf     CSG_LAT,  A          ; CS inactive high

        ; --- SPI pins direction ---
        bsf     TRISC, 4, A          ; RC4/SDI1 input
        bcf     TRISC, 3, A          ; RC3/SCK1 output (master)
        bcf     TRISC, 5, A          ; RC5/SDO1 output

        ; --- MSSP1 in SPI master mode ---
        ; SSP1STAT: CKE=1 (transmit on active->idle), SMP=0
        movlw   b'01000000'          ; CKE=1
        movwf   SSP1STAT, A

        ; SSP1CON1:
        ; SSPEN=1 enable
        ; CKP=0 idle low
        ; SSPM=0001 => Fosc/16 (pick slower if your wiring is long)
        movlw   b'00100001'          ; SSPEN=1, SSPM=0001
        movwf   SSP1CON1, A

        return

;========================
; Low-level SPI transfer: W out, W in
;========================
SPI1_Xfer:
        movwf   SSP1BUF, A
wait_bf:
        btfss   SSP1STAT, 0, A       ; BF bit
        bra     wait_bf
        movf    SSP1BUF, W, A
        return

;========================
; Read one gyro register
; input: W = reg
; output: W = data
;========================
IMU_Gyro_ReadReg:
        bcf     CSG_LAT, A                   ; CS low

        iorlw   SPI_READ                     ; set read bit
        call    SPI1_Xfer                    ; send address

        movlw   0x00
        call    SPI1_Xfer                    ; dummy -> read data in W

        bsf     CSG_LAT, A                   ; CS high
        return

;========================
; Write one gyro register
; input: W = reg, tmp = data
;========================
IMU_Gyro_WriteReg:
        bcf     CSG_LAT, A                   ; CS low

        andlw   0x7F                         ; ensure write
        call    SPI1_Xfer                    ; send address

        movf    tmp, W, A
        call    SPI1_Xfer                    ; send data

        bsf     CSG_LAT, A                   ; CS high
        return

;========================
; IMU_Gyro_WhoAmI_Test
; returns W = WHO_AM_I_XG
;========================
IMU_Gyro_WhoAmI_Test:
        movlw   WHO_AM_I_XG
        call    IMU_Gyro_ReadReg
        return

;========================
; IMU_Gyro_Config
; - CTRL_REG1_G = 0x60 : ODR=119Hz, FS=245 dps, BW default
; - CTRL_REG4   = 0x38 : enable X/Y/Z
;========================
IMU_Gyro_Config:
        ; CTRL_REG1_G
        movlw   0x60
        movwf   tmp, A
        movlw   CTRL_REG1_G
        call    IMU_Gyro_WriteReg

        ; CTRL_REG4
        movlw   0x38
        movwf   tmp, A
        movlw   CTRL_REG4
        call    IMU_Gyro_WriteReg

        return

;========================
; IMU_Gyro_Read_XL
; returns W = OUT_X_L_G (low byte only)
;========================
IMU_Gyro_Read_XL:
        movlw   OUT_X_L_G
        call    IMU_Gyro_ReadReg
        return
