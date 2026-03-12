            PROCESSOR 18F87K22
            #include <xc.inc>

            GLOBAL  ParserService

            GLOBAL  rx_ok_count, rx_ferr_count, rx_oerr_count, rx_ovf_count
            GLOBAL  last_byte, rx_head, rx_tail, rx_buf

            GLOBAL  parser_state, parser_checksum
            GLOBAL  stg_gx_l, stg_gx_h
            GLOBAL  stg_gy_l, stg_gy_h
            GLOBAL  stg_gz_l, stg_gz_h
            GLOBAL  stg_btn

            GLOBAL  latest_gx_l, latest_gx_h
            GLOBAL  latest_gy_l, latest_gy_h
            GLOBAL  latest_gz_l, latest_gz_h
            GLOBAL  latest_btn

            GLOBAL  sample_valid, new_sample
            GLOBAL  frame_ok_count_l, frame_ok_count_h
            GLOBAL  frame_badck_count_l, frame_badck_count_h

;===========================================================
; Phase 3 main
; - UART1 RX ISR fills ring buffer
; - ParserService in parser.s consumes ring buffer
; - Shared variables are GLOBAL for MPLAB X watch
;===========================================================

;-----------------------------------------------------------
; Access RAM
;-----------------------------------------------------------
            psect   udata_acs

isr_wreg:               ds 1
isr_status:             ds 1
isr_bsr:                ds 1
isr_fsr0l:              ds 1
isr_fsr0h:              ds 1

dly1:                   ds 1
dly2:                   ds 1

rx_head:                ds 1
rx_tail:                ds 1
rx_next:                ds 1
rx_byte:                ds 1
last_byte:              ds 1
led_temp:               ds 1

rx_ok_count:            ds 1
rx_ferr_count:          ds 1
rx_oerr_count:          ds 1
rx_ovf_count:           ds 1

rx_buf:                 ds 32

parser_state:           ds 1
parser_checksum:        ds 1

stg_gx_l:               ds 1
stg_gx_h:               ds 1
stg_gy_l:               ds 1
stg_gy_h:               ds 1
stg_gz_l:               ds 1
stg_gz_h:               ds 1
stg_btn:                ds 1

latest_gx_l:            ds 1
latest_gx_h:            ds 1
latest_gy_l:            ds 1
latest_gy_h:            ds 1
latest_gz_l:            ds 1
latest_gz_h:            ds 1
latest_btn:             ds 1

sample_valid:           ds 1
new_sample:             ds 1

frame_ok_count_l:       ds 1
frame_ok_count_h:       ds 1
frame_badck_count_l:    ds 1
frame_badck_count_h:    ds 1

;-----------------------------------------------------------
; Code
;-----------------------------------------------------------
            psect   code,abs

rst:        org     0x0000
            goto    Start

hi_isr:     org     0x0008
            goto    ISR_Main

;-----------------------------------------------------------
; Start
;-----------------------------------------------------------
Start:
            ; Make analog-capable pins digital so PORT/TRIS/LAT
            ; behave as digital I/O instead of analog inputs.
            movlw   0xFF
            movwf   ANCON0, A
            movwf   ANCON1, A

            ; Clear UART ring buffer state.
            clrf    rx_head, A
            clrf    rx_tail, A
            clrf    rx_ok_count, A
            clrf    rx_ferr_count, A
            clrf    rx_oerr_count, A
            clrf    rx_ovf_count, A
            clrf    last_byte, A
            clrf    led_temp, A

            ; Clear parser state and checksum accumulator.
            clrf    parser_state, A
            clrf    parser_checksum, A

            ; Clear staging registers used while a frame is being built.
            clrf    stg_gx_l, A
            clrf    stg_gx_h, A
            clrf    stg_gy_l, A
            clrf    stg_gy_h, A
            clrf    stg_gz_l, A
            clrf    stg_gz_h, A
            clrf    stg_btn, A

            ; Clear latest committed valid sample.
            clrf    latest_gx_l, A
            clrf    latest_gx_h, A
            clrf    latest_gy_l, A
            clrf    latest_gy_h, A
            clrf    latest_gz_l, A
            clrf    latest_gz_h, A
            clrf    latest_btn, A

            ; Clear sample flags.
            clrf    sample_valid, A
            clrf    new_sample, A

            ; Clear parser statistics.
            clrf    frame_ok_count_l, A
            clrf    frame_ok_count_h, A
            clrf    frame_badck_count_l, A
            clrf    frame_badck_count_h, A

            ; RC7 = RX1 input.
            ; RC6 = TX1 output.
            ; RC5..RC0 = debug LEDs.
            movlw   10000000B
            movwf   TRISC, A

            ; Visible startup self-test on RC0..RC5.
            movlw   00101010B
            movwf   LATC, A
            call    DelayVisible

            movlw   00010101B
            movwf   LATC, A
            call    DelayVisible

            clrf    LATC, A

            ; Configure UART1 receive path and interrupts.
            call    InitUART1_RX

MainLoop:
            ; ParserService consumes bytes from rx_buf, validates frames,
            ; and commits latest_* only after checksum passes.
            call    ParserService
            bra     MainLoop

;-----------------------------------------------------------
; Delay
;-----------------------------------------------------------
DelayVisible:
            ; Outer loop preset for visible LED delay.
            movlw   0xFF
            movwf   dly1, A
DV_Outer:
            ; Inner loop preset for visible LED delay.
            movlw   0xFF
            movwf   dly2, A
DV_Inner:
            ; Count inner loop down to zero.
            decfsz  dly2, F, A
            bra     DV_Inner
            ; Count outer loop down to zero.
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
            ; Enable UART1 peripheral module if PMD disabled it.
            bcf     PMD0, 3, A

            ; Preserve UART pin directions.
            bsf     TRISC, 7, A
            bcf     TRISC, 6, A

            ; Async UART, high-speed baud rate generator, 16-bit BRG.
            bcf     TXSTA1, 4, A
            bsf     TXSTA1, 2, A
            bsf     BAUDCON1, 3, A

            ; Set baud generator for 115200 baud at 64 MHz.
            clrf    SPBRGH1, A
            movlw   0x8A
            movwf   SPBRG1, A

            ; Enable serial port and continuous receive.
            bsf     RCSTA1, 7, A
            bsf     RCSTA1, 4, A

FlushRX1:
            ; Drain any stale receive bytes before enabling interrupts.
            btfss   PIR1, 5, A
            bra     FlushDone
            movf    RCREG1, W, A
            bra     FlushRX1
FlushDone:

            ; Single priority interrupt mode.
            bcf     RCON, 7, A

            ; Enable UART receive interrupt and global interrupts.
            bsf     PIE1, 5, A
            bsf     INTCON, 6, A
            bsf     INTCON, 7, A

            return

;-----------------------------------------------------------
; ISR
;-----------------------------------------------------------
ISR_Main:
            ; Save context that this ISR uses.
            movff   WREG, isr_wreg
            movff   STATUS, isr_status
            movff   BSR, isr_bsr
            movff   FSR0L, isr_fsr0l
            movff   FSR0H, isr_fsr0h

ISR_CheckRX:
            ; If no UART1 receive interrupt pending, exit ISR.
            btfss   PIR1, 5, A
            bra     ISR_Exit

            ; If hardware overrun occurred, receiver is jammed until
            ; CREN is cycled off then on.
            btfsc   RCSTA1, 1, A
            bra     ISR_HandleOERR

            ; If framing error occurred, read and discard bad byte so
            ; the receiver can continue with the next one.
            btfsc   RCSTA1, 2, A
            bra     ISR_HandleFERR

            ; Read received byte immediately from RCREG1.
            movf    RCREG1, W, A
            movwf   rx_byte, A

            ; Compute next head index: (head + 1) & 0x1F for 32-byte ring.
            movf    rx_head, W, A
            addlw   0x01
            andlw   0x1F
            movwf   rx_next, A

            ; If next == tail, software ring buffer is full, so drop byte.
            movf    rx_tail, W, A
            cpfseq  rx_next, A
            bra     ISR_StoreByte

            ; Count software overflow event.
            incf    rx_ovf_count, F, A
            bra     ISR_CheckRX

ISR_StoreByte:
            ; Point FSR0 at rx_buf[rx_head].
            lfsr    0, rx_buf
            movf    rx_head, W, A
            addwf   FSR0L, F, A
            btfsc   STATUS, 0, A
            incf    FSR0H, F, A

            ; Store received byte into ring buffer.
            movf    rx_byte, W, A
            movwf   INDF0, A

            ; Publish new head index.
            movf    rx_next, W, A
            movwf   rx_head, A

            ; Count successfully queued bytes.
            incf    rx_ok_count, F, A
            bra     ISR_CheckRX

ISR_HandleFERR:
            ; Read and discard the bad byte to clear the hardware state.
            movf    RCREG1, W, A
            incf    rx_ferr_count, F, A
            bra     ISR_CheckRX

ISR_HandleOERR:
            ; Clear continuous receive, then re-enable it, which clears OERR.
            bcf     RCSTA1, 4, A
            bsf     RCSTA1, 4, A
            incf    rx_oerr_count, F, A
            bra     ISR_CheckRX

ISR_Exit:
            ; Restore saved context.
            movff   isr_fsr0h, FSR0H
            movff   isr_fsr0l, FSR0L
            movff   isr_bsr, BSR
            movff   isr_status, STATUS
            movff   isr_wreg, WREG
            retfie

            end     rst
