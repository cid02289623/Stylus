            PROCESSOR 18F87K22
            #include <xc.inc>

;===========================================================
; uart1.s
; Owns: InitUART1, UART1_WriteByte, DelayVisible
; RAM (dly1, dly2) declared in main.s, accessed via extrn.
;===========================================================

            GLOBAL  InitUART1
            GLOBAL  UART1_WriteByte
            GLOBAL  DelayVisible

            extrn   dly1, dly2

            psect   uart1_code,class=CODE,reloc=2

;-----------------------------------------------------------
; InitUART1
; Assumes Fosc = 64 MHz
; 38400 baud, BRGH=1, BRG16=1, SPBRGH:SPBRG = 0x01A0
;-----------------------------------------------------------
InitUART1:
            ; Enable UART1 peripheral module if PMD disabled it.
            bcf     PMD0, 3, A

            ; Set UART pin directions.
            bsf     TRISC, 7, A
            bcf     TRISC, 6, A

            ; Async UART, high-speed baud, 16-bit BRG.
            bcf     TXSTA1, 4, A
            bsf     TXSTA1, 2, A
            bsf     TXSTA1, 5, A
            bsf     BAUDCON1, 3, A

            ; Set baud generator for 38400 baud at 64 MHz.
            movlw   0x01
            movwf   SPBRGH1, A
            movlw   0xA0
            movwf   SPBRG1, A

            ; Enable serial port and continuous receive.
            bsf     RCSTA1, 7, A
            bsf     RCSTA1, 4, A

FlushRX1:
            ; Drain stale bytes before enabling interrupt.
            btfss   PIR1, 5, A
            bra     FlushDone
            movf    RCREG1, W, A
            bra     FlushRX1

FlushDone:
            ; Enable UART receive interrupt.
            bsf     PIE1, 5, A
            return

;-----------------------------------------------------------
; UART1_WriteByte
; Input: WREG = byte to transmit
; Blocks until TXREG1 is ready, then writes the byte.
;-----------------------------------------------------------
UART1_WriteByte:
TX1_Wait:
            btfss   PIR1, 4, A
            bra     TX1_Wait
            movwf   TXREG1, A
            return

;-----------------------------------------------------------
; DelayVisible
; Blocking software delay, used for the startup LED self-test.
; Uses dly1 and dly2 from main.s access RAM.
;-----------------------------------------------------------
DelayVisible:
            movlw   0xFF
            movwf   dly1, A
DV_Outer:
            movlw   0xFF
            movwf   dly2, A
DV_Inner:
            decfsz  dly2, F, A
            bra     DV_Inner
            decfsz  dly1, F, A
            bra     DV_Outer
            return

            end
