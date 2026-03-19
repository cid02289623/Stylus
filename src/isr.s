            PROCESSOR 18F87K22
            #include <xc.inc>

;===========================================================
; isr.s
; Owns: ISR_Main
; Handles UART1 RX ring-buffer fill and Timer1 20ms tick.
; All RAM lives in main.s; accessed here via extrn.
;===========================================================

            GLOBAL  ISR_Main

;-----------------------------------------------------------
; RAM declared in main.s
;-----------------------------------------------------------
            extrn   isr_wreg, isr_status, isr_bsr, isr_fsr0l, isr_fsr0h
            extrn   rx_head, rx_tail, rx_next, rx_byte, rx_buf
            extrn   rx_ok_count, rx_ferr_count, rx_oerr_count, rx_ovf_count
            extrn   pending_ticks, tick_count_l, tick_count_h

            psect   isr_code,class=CODE,reloc=2

;-----------------------------------------------------------
; ISR_Main
; Services UART1 RX first, then Timer1.
;-----------------------------------------------------------
ISR_Main:
            ; Save context used by this ISR.
            movff   WREG,   isr_wreg
            movff   STATUS, isr_status
            movff   BSR,    isr_bsr
            movff   FSR0L,  isr_fsr0l
            movff   FSR0H,  isr_fsr0h

ISR_CheckRX:
            ; If UART1 receive interrupt is pending, service it first.
            btfss   PIR1, 5, A
            bra     ISR_CheckTMR1

            ; If hardware overrun occurred, cycle CREN to clear.
            btfsc   RCSTA1, 1, A
            bra     ISR_HandleOERR

            ; If framing error occurred, read and discard the bad byte.
            btfsc   RCSTA1, 2, A
            bra     ISR_HandleFERR

            ; Read received byte from RCREG1.
            movf    RCREG1, W, A
            movwf   rx_byte, A

            ; Compute next head index: (head + 1) & 0x1F.
            movf    rx_head, W, A
            addlw   0x01
            andlw   0x1F
            movwf   rx_next, A

            ; If next == tail the ring buffer is full - drop byte.
            movf    rx_tail, W, A
            cpfseq  rx_next, A
            bra     ISR_StoreByte

            incf    rx_ovf_count, F, A
            bra     ISR_CheckRX

ISR_StoreByte:
            ; Point FSR0 at rx_buf[rx_head].
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

ISR_CheckTMR1:
            btfss   PIR1, 0, A
            bra     ISR_Exit

            ; Reload Timer1 for the next 20 ms interval.
            movlw   0x63
            movwf   TMR1H, A
            movlw   0xC0
            movwf   TMR1L, A

            bcf     PIR1, 0, A

            ; pending_ticks++ with saturation at 0xFF.
            movlw   0xFF
            cpfseq  pending_ticks, A
            incf    pending_ticks, F, A

            ; Increment total tick counter.
            incf    tick_count_l, F, A
            bnz     ISR_Exit
            incf    tick_count_h, F, A

ISR_Exit:
            movff   isr_fsr0h, FSR0H
            movff   isr_fsr0l, FSR0L
            movff   isr_bsr,   BSR
            movff   isr_status, STATUS
            movff   isr_wreg,  WREG
            retfie

            end
