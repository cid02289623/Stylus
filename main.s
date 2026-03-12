            PROCESSOR 18F87K22
            #include <xc.inc>
	    
	    GLOBAL  rx_ok_count, rx_ferr_count, rx_oerr_count, rx_ovf_count
            GLOBAL  last_byte, rx_head, rx_tail

;===========================================================
; Phase 2 UART RX bring-up
; Uses UART1 on RC7/RC6
; Uses RC0..RC5 as visible debug LEDs
;===========================================================

;-----------------------------------------------------------
; Access RAM
;-----------------------------------------------------------
            psect udata_acs

isr_wreg:       ds 1
isr_status:     ds 1
isr_bsr:        ds 1
isr_fsr0l:      ds 1
isr_fsr0h:      ds 1

dly1:           ds 1
dly2:           ds 1

rx_head:        ds 1
rx_tail:        ds 1
rx_next:        ds 1
rx_byte:        ds 1
last_byte:      ds 1
led_temp:       ds 1

rx_ok_count:    ds 1
rx_ferr_count:  ds 1
rx_oerr_count:  ds 1
rx_ovf_count:   ds 1

rx_buf:         ds 32

;-----------------------------------------------------------
; Code
;-----------------------------------------------------------
            psect code,abs

rst:        org 0x0000
            goto    Start

hi_isr:     org 0x0008
            goto    ISR_Main

;-----------------------------------------------------------
; Start
;-----------------------------------------------------------
Start:
            ; Make analog-capable pins digital
            movlw   0xFF
            movwf   ANCON0, A
            movwf   ANCON1, A

            ; Clear software state
            clrf    rx_head, A
            clrf    rx_tail, A
            clrf    rx_ok_count, A
            clrf    rx_ferr_count, A
            clrf    rx_oerr_count, A
            clrf    rx_ovf_count, A
            clrf    last_byte, A
            clrf    led_temp, A

            ; RC7 = RX1 input
            ; RC6 = TX1 output
            ; RC5..RC0 = LED debug outputs
            movlw   10000000B
            movwf   TRISC, A

            ; Visible startup self-test on RC0..RC5 only
            movlw   00101010B        ; 0x2A, visible on RC1/3/5
            movwf   LATC, A
            call    DelayVisible

            movlw   00010101B         ; 0x15, visible on RC0/2/4
            movwf   LATC, A
            call    DelayVisible

            clrf    LATC, A

            call    InitUART1_RX

MainLoop:
            call    ServiceRxDebug
            bra     MainLoop

;-----------------------------------------------------------
; Delay
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

;-----------------------------------------------------------
; UART1 RX init
; Assumes Fosc = 64 MHz
; For 115200 baud with BRGH=1 and BRG16=1:
; SPBRGH1:SPBRG1 = 0x008A
;-----------------------------------------------------------
InitUART1_RX:
            ; Enable UART1 module
            bcf     PMD0, 3, A          ; UART1MD = 0

            ; RC7 input, RC6 output preserved
            bsf     TRISC, 7, A
            bcf     TRISC, 6, A

            ; Async mode, high-speed baud, 16-bit BRG
            bcf     TXSTA1, 4, A        ; SYNC = 0
            bsf     TXSTA1, 2, A        ; BRGH = 1
            bsf     BAUDCON1, 3, A      ; BRG16 = 1

            ; 115200 @ 64 MHz
            clrf    SPBRGH1, A
            movlw   0x8A
            movwf   SPBRG1, A

            ; Enable serial port and continuous receive
            bsf     RCSTA1, 7, A        ; SPEN = 1
            bsf     RCSTA1, 4, A        ; CREN = 1

FlushRX1:
            btfss   PIR1, 5, A          ; RC1IF?
            bra     FlushDone
            movf    RCREG1, W, A
            bra     FlushRX1
FlushDone:

            ; Single priority mode
            bcf     RCON, 7, A          ; IPEN = 0

            ; Enable RX interrupt
            bsf     PIE1, 5, A          ; RC1IE = 1
            bsf     INTCON, 6, A        ; PEIE = 1
            bsf     INTCON, 7, A        ; GIE = 1

            return

;-----------------------------------------------------------
; ISR
;-----------------------------------------------------------
ISR_Main:
            movff   WREG, isr_wreg
            movff   STATUS, isr_status
            movff   BSR, isr_bsr
            movff   FSR0L, isr_fsr0l
            movff   FSR0H, isr_fsr0h

ISR_CheckRX:
            btfss   PIR1, 5, A          ; RC1IF
            bra     ISR_Exit

            btfsc   RCSTA1, 1, A        ; OERR
            bra     ISR_HandleOERR

            btfsc   RCSTA1, 2, A        ; FERR
            bra     ISR_HandleFERR

            ; Read byte immediately
            movf    RCREG1, W, A
            movwf   rx_byte, A

            ; next = (head + 1) & 0x1F
            movf    rx_head, W, A
            addlw   0x01
            andlw   0x1F
            movwf   rx_next, A

            ; if next == tail, overflow -> drop byte
            movf    rx_tail, W, A
            cpfseq  rx_next, A
            bra     ISR_StoreByte

            incf    rx_ovf_count, F, A
            bra     ISR_CheckRX

ISR_StoreByte:
            lfsr    0, rx_buf
            movf    rx_head, W, A
            addwf   FSR0L, F, A
            btfsc   STATUS, 0, A
            incf    FSR0H, F, A

            movf    rx_byte, W, A
            movwf   INDF0, A

            movf    rx_next, W, A
            movwf   rx_head, A

            incf    rx_ok_count, F, A
            bra     ISR_CheckRX

ISR_HandleFERR:
            movf    RCREG1, W, A
            incf    rx_ferr_count, F, A
            bra     ISR_CheckRX

ISR_HandleOERR:
            bcf     RCSTA1, 4, A
            bsf     RCSTA1, 4, A
            incf    rx_oerr_count, F, A
            bra     ISR_CheckRX

ISR_Exit:
            movff   isr_fsr0h, FSR0H
            movff   isr_fsr0l, FSR0L
            movff   isr_bsr, BSR
            movff   isr_status, STATUS
            movff   isr_wreg, WREG
            retfie

;-----------------------------------------------------------
; Pop one byte from ring buffer and show low 6 bits on RC0..RC5
; RC6/RC7 are preserved for UART function
;-----------------------------------------------------------
ServiceRxDebug:
            movf    rx_head, W, A
            cpfseq  rx_tail, A
            bra     SRX_HaveByte
            return

SRX_HaveByte:
            lfsr    0, rx_buf
            movf    rx_tail, W, A
            addwf   FSR0L, F, A
            btfsc   STATUS, 0, A
            incf    FSR0H, F, A

            movf    INDF0, W, A
            movwf   last_byte, A

            ; Preserve RC6 and RC7, update RC0..RC5 only
            movf    LATC, W, A
            andlw   11000000B
            movwf   led_temp, A

            movf    last_byte, W, A
            andlw   00111110B
            iorwf   led_temp, W, A
            movwf   LATC, A

            incf    rx_tail, F, A
            movlw   0x1F
            andwf   rx_tail, F, A

            return

            end     rst
