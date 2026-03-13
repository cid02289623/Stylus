PROCESSOR 18F87K22
            #include <xc.inc>

            GLOBAL  ParserService
            GLOBAL  GestureInit
            GLOBAL  ProcessTickService

            GLOBAL  UART1_WriteByte

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

            GLOBAL  pending_ticks, tick_count_l, tick_count_h
            GLOBAL  debug_tickdiv
            GLOBAL  proc_gx_l, proc_gx_h
            GLOBAL  proc_gy_l, proc_gy_h
            GLOBAL  proc_gz_l, proc_gz_h
            GLOBAL  proc_btn

;===========================================================
; main.s
; - UART1 RX ISR fills ring buffer
; - ParserService consumes ring buffer and validates frames
; - Timer1 generates fixed 20 ms processing ticks
; - Main loop consumes one pending tick at a time
; - ProcessTickService in gesture.s snapshots the latest sample,
;   runs startup calibration first, then runs the gesture engine
;===========================================================

;-----------------------------------------------------------
; Access RAM
;-----------------------------------------------------------
            psect   udata_acs

isr_wreg:               ds 1          ; Saved WREG during ISR
isr_status:             ds 1          ; Saved STATUS during ISR
isr_bsr:                ds 1          ; Saved BSR during ISR
isr_fsr0l:              ds 1          ; Saved FSR0L during ISR
isr_fsr0h:              ds 1          ; Saved FSR0H during ISR

dly1:                   ds 1          ; Outer visible delay counter
dly2:                   ds 1          ; Inner visible delay counter

rx_head:                ds 1          ; Ring buffer head index
rx_tail:                ds 1          ; Ring buffer tail index
rx_next:                ds 1          ; Temporary next-head index
rx_byte:                ds 1          ; Temporary received byte
last_byte:              ds 1          ; Last byte consumed by parser

rx_ok_count:            ds 1          ; RX bytes stored successfully
rx_ferr_count:          ds 1          ; Framing error counter
rx_oerr_count:          ds 1          ; Hardware overrun counter
rx_ovf_count:           ds 1          ; Software ring overflow counter

;-----------------------------------------------------------
; Move rx_buf out of access RAM.
; It is always reached indirectly through FSR0, so it does not
; need access-bank placement.
;-----------------------------------------------------------
            psect   udata

rx_buf:                 ds 32         ; 32-byte UART RX ring buffer

;-----------------------------------------------------------
; Back to access RAM for direct-access variables
;-----------------------------------------------------------
            psect   udata_acs

parser_state:           ds 1          ; Parser state machine state
parser_checksum:        ds 1          ; Running checksum over bytes 1..7

stg_gx_l:               ds 1          ; Staged Gx low byte
stg_gx_h:               ds 1          ; Staged Gx high byte
stg_gy_l:               ds 1          ; Staged Gy low byte
stg_gy_h:               ds 1          ; Staged Gy high byte
stg_gz_l:               ds 1          ; Staged Gz low byte
stg_gz_h:               ds 1          ; Staged Gz high byte
stg_btn:                ds 1          ; Staged button byte

latest_gx_l:            ds 1          ; Latest valid Gx low byte
latest_gx_h:            ds 1          ; Latest valid Gx high byte
latest_gy_l:            ds 1          ; Latest valid Gy low byte
latest_gy_h:            ds 1          ; Latest valid Gy high byte
latest_gz_l:            ds 1          ; Latest valid Gz low byte
latest_gz_h:            ds 1          ; Latest valid Gz high byte
latest_btn:             ds 1          ; Latest valid button byte

sample_valid:           ds 1          ; 1 once at least one frame parsed
new_sample:             ds 1          ; 1 when parser committed a fresh frame

frame_ok_count_l:       ds 1          ; Good frame counter low byte
frame_ok_count_h:       ds 1          ; Good frame counter high byte
frame_badck_count_l:    ds 1          ; Bad-checksum counter low byte
frame_badck_count_h:    ds 1          ; Bad-checksum counter high byte

pending_ticks:          ds 1          ; Number of 20 ms ticks waiting
tick_count_l:           ds 1          ; Total Timer1 tick counter low byte
tick_count_h:           ds 1          ; Total Timer1 tick counter high byte
debug_tickdiv:          ds 1          ; Heartbeat divider

proc_gx_l:              ds 1          ; Snapshot Gx low for current tick
proc_gx_h:              ds 1          ; Snapshot Gx high for current tick
proc_gy_l:              ds 1          ; Snapshot Gy low for current tick
proc_gy_h:              ds 1          ; Snapshot Gy high for current tick
proc_gz_l:              ds 1          ; Snapshot Gz low for current tick
proc_gz_h:              ds 1          ; Snapshot Gz high for current tick
proc_btn:               ds 1          ; Snapshot button byte for current tick

;-----------------------------------------------------------
; Code
;-----------------------------------------------------------
            psect   code,abs

rst:        org     0x0000
            goto    Start             ; Reset vector

hi_isr:     org     0x0008
            goto    ISR_Main          ; Single-priority interrupt vector

;-----------------------------------------------------------
; Start
;-----------------------------------------------------------
Start:
            ; Make analog-capable pins digital.
            movlw   0xFF
            movwf   ANCON0, A
            movwf   ANCON1, A

            ; Clear UART ring buffer state.
            clrf    rx_head, A
            clrf    rx_tail, A
            clrf    rx_next, A
            clrf    rx_byte, A
            clrf    rx_ok_count, A
            clrf    rx_ferr_count, A
            clrf    rx_oerr_count, A
            clrf    rx_ovf_count, A
            clrf    last_byte, A

            ; Clear parser state and checksum accumulator.
            clrf    parser_state, A
            clrf    parser_checksum, A

            ; Clear staging registers.
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

            ; Clear timing / processing state.
            clrf    pending_ticks, A
            clrf    tick_count_l, A
            clrf    tick_count_h, A
            clrf    debug_tickdiv, A

            ; Clear current per-tick processing snapshot.
            clrf    proc_gx_l, A
            clrf    proc_gx_h, A
            clrf    proc_gy_l, A
            clrf    proc_gy_h, A
            clrf    proc_gz_l, A
            clrf    proc_gz_h, A
            clrf    proc_btn, A

            ; RC7 = RX1 input
            ; RC6 = TX1 output
            ; RC5..RC0 = outputs
            movlw   10000000B
            movwf   TRISC, A

            ; Clear output latch.
            clrf    LATC, A

            ; Visible startup self-test on RC0..RC5.
            movlw   00101010B
            movwf   LATC, A
            call    DelayVisible

            movlw   00010101B
            movwf   LATC, A
            call    DelayVisible

            ; End self-test with all LEDs off.
            clrf    LATC, A

            ; Use single-priority interrupt mode.
            bcf     RCON, 7, A        ; IPEN = 0

            ; Configure UART1 RX/TX path.
            call    InitUART1

            ; Configure Timer1 for a deterministic 20 ms tick.
            call    InitTimer1_20ms

            ; Initialise the gesture module.
            ; This also starts startup calibration.
            call    GestureInit

            ; Enable peripheral and global interrupts.
            bsf     INTCON, 6, A      ; PEIE = 1
            bsf     INTCON, 7, A      ; GIE  = 1

;-----------------------------------------------------------
; Main loop
; - If a tick is pending, process exactly one tick.
; - Otherwise keep draining the parser.
;-----------------------------------------------------------
MainLoop:
            ; Check whether a Timer1 processing slot is pending.
            movf    pending_ticks, W, A
            bz      Main_DoParse

            ; Short critical section so the decrement is atomic.
            bcf     INTCON, 7, A      ; GIE = 0

            ; Re-check after disabling interrupts.
            movf    pending_ticks, W, A
            bz      Main_ReEnable

            ; Consume exactly one pending tick.
            decf    pending_ticks, F, A

            ; Re-enable interrupts quickly.
            bsf     INTCON, 7, A

            ; Run one fixed-rate 20 ms service step.
            call    ProcessTickService
            bra     MainLoop

Main_ReEnable:
            ; Nothing pending after re-check, so re-enable and continue.
            bsf     INTCON, 7, A
            bra     MainLoop

Main_DoParse:
            ; ParserService drains bytes from the ring buffer,
            ; validates frames, and only then publishes latest_*.
            call    ParserService
            bra     MainLoop

;-----------------------------------------------------------
; Visible delay
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
; UART1 init
; Assumes Fosc = 64 MHz
; 115200 baud, BRGH = 1, BRG16 = 1, SPBRG = 0x008A
;-----------------------------------------------------------
InitUART1:
            ; Enable UART1 peripheral module if PMD disabled it.
            bcf     PMD0, 3, A        ; UART1MD = 0

            ; Set UART pin directions.
            bsf     TRISC, 7, A       ; RC7 = RX1 input
            bcf     TRISC, 6, A       ; RC6 = TX1 output

            ; Async UART, high-speed baud, 16-bit BRG.
            bcf     TXSTA1, 4, A      ; SYNC = 0
            bsf     TXSTA1, 2, A      ; BRGH = 1
            bsf     TXSTA1, 5, A      ; TXEN = 1
            bsf     BAUDCON1, 3, A    ; BRG16 = 1

            ; Set baud generator for 115200 baud at 64 MHz.
            clrf    SPBRGH1, A
            movlw   0x8A
            movwf   SPBRG1, A

            ; Enable serial port and continuous receive.
            bsf     RCSTA1, 7, A      ; SPEN = 1
            bsf     RCSTA1, 4, A      ; CREN = 1

FlushRX1:
            ; Drain any stale receive bytes before enabling interrupt.
            btfss   PIR1, 5, A        ; RC1IF
            bra     FlushDone
            movf    RCREG1, W, A
            bra     FlushRX1

FlushDone:
            ; Enable UART receive interrupt.
            bsf     PIE1, 5, A        ; RC1IE = 1
            return

;-----------------------------------------------------------
; Timer1 init for 20 ms tick
; Assumes Fosc = 64 MHz
; Internal clock = Fosc/4 = 16 MHz
; Prescaler = 1:8 => Timer tick = 0.5 us
; 20 ms => 40000 counts
; Preload = 65536 - 40000 = 25536 = 0x63C0
;-----------------------------------------------------------
InitTimer1_20ms:
            ; Ensure Timer1 peripheral is enabled.
            bcf     PMD1, 1, A        ; TMR1MD = 0

            ; Disable Timer1 gate function.
            clrf    T1GCON, A

            ; Stop Timer1 while configuring.
            clrf    T1CON, A

            ; T1CON = 00110010b
            ; TMR1CS<1:0> = 00 -> internal clock (Fosc/4)
            ; T1CKPS<1:0> = 11 -> 1:8 prescale
            ; RD16 = 1
            movlw   00110010B
            movwf   T1CON, A

            ; Load 20 ms preload.
            movlw   0x63
            movwf   TMR1H, A
            movlw   0xC0
            movwf   TMR1L, A

            ; Clear stale Timer1 interrupt flag.
            bcf     PIR1, 0, A        ; TMR1IF = 0

            ; Enable Timer1 interrupt.
            bsf     PIE1, 0, A        ; TMR1IE = 1

            ; Start Timer1.
            bsf     T1CON, 0, A       ; TMR1ON = 1
            return

;-----------------------------------------------------------
; UART1_WriteByte
; Input: WREG = byte to send
; Blocks until TXREG1 is ready, then writes the byte.
;-----------------------------------------------------------
UART1_WriteByte:
TX1_Wait:
            btfss   PIR1, 4, A        ; TX1IF = 1 when TXREG1 is empty
            bra     TX1_Wait
            movwf   TXREG1, A
            return

;-----------------------------------------------------------
; ISR
; - Services UART RX first
; - Then services Timer1
;-----------------------------------------------------------
ISR_Main:
            ; Save context touched by this ISR.
            movff   WREG,   isr_wreg
            movff   STATUS, isr_status
            movff   BSR,    isr_bsr
            movff   FSR0L,  isr_fsr0l
            movff   FSR0H,  isr_fsr0h

ISR_CheckRX:
            ; If UART1 receive interrupt is pending, service it first.
            btfss   PIR1, 5, A        ; RC1IF
            bra     ISR_CheckTMR1

            ; If hardware overrun occurred, receiver is jammed until
            ; CREN is cycled off and on again.
            btfsc   RCSTA1, 1, A      ; OERR
            bra     ISR_HandleOERR

            ; If framing error occurred, read and discard the bad byte.
            btfsc   RCSTA1, 2, A      ; FERR
            bra     ISR_HandleFERR

            ; Read received byte immediately from RCREG1.
            movf    RCREG1, W, A
            movwf   rx_byte, A

            ; Compute next head index: (head + 1) & 0x1F.
            movf    rx_head, W, A
            addlw   0x01
            andlw   0x1F
            movwf   rx_next, A

            ; If next == tail, the software ring buffer is full.
            movf    rx_tail, W, A
            cpfseq  rx_next, A
            bra     ISR_StoreByte

            ; Drop the byte and count the overflow.
            incf    rx_ovf_count, F, A
            bra     ISR_CheckRX

ISR_StoreByte:
            ; Point FSR0 at rx_buf[rx_head].
            lfsr    0, rx_buf
            movf    rx_head, W, A
            addwf   FSR0L, F, A
            btfsc   STATUS, 0, A
            incf    FSR0H, F, A

            ; Store received byte into the ring buffer.
            movf    rx_byte, W, A
            movwf   INDF0, A

            ; Publish new head index.
            movf    rx_next, W, A
            movwf   rx_head, A

            ; Count the stored byte.
            incf    rx_ok_count, F, A
            bra     ISR_CheckRX

ISR_HandleFERR:
            ; Read and discard the bad byte to clear receiver state.
            movf    RCREG1, W, A
            incf    rx_ferr_count, F, A
            bra     ISR_CheckRX

ISR_HandleOERR:
            ; Clear continuous receive, then re-enable it.
            bcf     RCSTA1, 4, A      ; CREN = 0
            bsf     RCSTA1, 4, A      ; CREN = 1
            incf    rx_oerr_count, F, A
            bra     ISR_CheckRX

ISR_CheckTMR1:
            ; If Timer1 overflow happened, service it.
            btfss   PIR1, 0, A        ; TMR1IF
            bra     ISR_Exit

            ; Reload Timer1 for the next 20 ms interval.
            movlw   0x63
            movwf   TMR1H, A
            movlw   0xC0
            movwf   TMR1L, A

            ; Clear Timer1 interrupt flag.
            bcf     PIR1, 0, A        ; TMR1IF = 0

            ; pending_ticks++ with saturation at 0xFF.
            movlw   0xFF
            cpfseq  pending_ticks, A
            incf    pending_ticks, F, A

            ; Increment total tick counter.
            incf    tick_count_l, F, A
            bnz     ISR_Exit
            incf    tick_count_h, F, A

ISR_Exit:
            ; Restore saved context in reverse order.
            movff   isr_fsr0h, FSR0H
            movff   isr_fsr0l, FSR0L
            movff   isr_bsr,   BSR
            movff   isr_status, STATUS
            movff   isr_wreg,  WREG
            retfie

            end
