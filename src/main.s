            PROCESSOR 18F87K22
            #include <xc.inc>

;===========================================================
; main.s
; Owns:
;   - Reset and ISR vectors
;   - All shared RAM declarations (access + udata)
;   - Start: hardware init sequence
;   - MainLoop: tick dispatch and parser call
;
; All subroutines have been moved to their own modules.
; Cross-module calls are declared with extrn here.
;===========================================================

;-----------------------------------------------------------
; Symbols defined in this file, visible to other modules
;-----------------------------------------------------------
            GLOBAL  rx_ok_count, rx_ferr_count, rx_oerr_count, rx_ovf_count
            GLOBAL  last_byte, rx_head, rx_tail, rx_buf
            GLOBAL  rx_next, rx_byte

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

            GLOBAL  isr_wreg, isr_status, isr_bsr, isr_fsr0l, isr_fsr0h
            GLOBAL  dly1, dly2

;-----------------------------------------------------------
; Symbols defined in other modules, used here
;-----------------------------------------------------------
            extrn   ISR_Main
            extrn   InitUART1
            extrn   InitTimer1_20ms
            extrn   GestureInit
            extrn   ParserService
            extrn   ProcessTickService
            extrn   DelayVisible

;-----------------------------------------------------------
; Access RAM - ISR scratch, delay, ring buffer pointers
;-----------------------------------------------------------
            psect   main_acs0,class=COMRAM,space=1,noexec

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

rx_ok_count:            ds 1
rx_ferr_count:          ds 1
rx_oerr_count:          ds 1
rx_ovf_count:           ds 1

;-----------------------------------------------------------
; Ring buffer - normal RAM (too large for access bank)
;-----------------------------------------------------------
            psect   udata
rx_buf:                 ds 32

;-----------------------------------------------------------
; Access RAM - parser state and sample staging/latest
;-----------------------------------------------------------
            psect   main_acs1,class=COMRAM,space=1,noexec

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

;-----------------------------------------------------------
; Access RAM - flags, counters, tick state, proc snapshot
;-----------------------------------------------------------
            psect   main_acs2,class=COMRAM,space=1,noexec

sample_valid:           ds 1
new_sample:             ds 1

frame_ok_count_l:       ds 1
frame_ok_count_h:       ds 1
frame_badck_count_l:    ds 1
frame_badck_count_h:    ds 1

pending_ticks:          ds 1
tick_count_l:           ds 1
tick_count_h:           ds 1
debug_tickdiv:          ds 1

proc_gx_l:              ds 1
proc_gx_h:              ds 1
proc_gy_l:              ds 1
proc_gy_h:              ds 1
proc_gz_l:              ds 1
proc_gz_h:              ds 1
proc_btn:               ds 1

;-----------------------------------------------------------
; Code - reset and interrupt vectors (absolute)
;-----------------------------------------------------------
            psect   code,abs

rst:        org     0x0000
            goto    Start

hi_isr:     org     0x0008
            goto    ISR_Main

;-----------------------------------------------------------
; Start - hardware init, then fall into MainLoop
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

            ; Clear per-tick processing snapshot.
            clrf    proc_gx_l, A
            clrf    proc_gx_h, A
            clrf    proc_gy_l, A
            clrf    proc_gy_h, A
            clrf    proc_gz_l, A
            clrf    proc_gz_h, A
            clrf    proc_btn, A

            ; RC7 = RX1 input, RC6 = TX1 output, RC5..RC0 = outputs.
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

            ; Single-priority interrupt mode.
            bcf     RCON, 7, A

            ; Configure UART1 RX/TX path.
            call    InitUART1

            ; Configure Timer1 20 ms tick.
            call    InitTimer1_20ms

            ; Initialise gesture module.
            call    GestureInit

            ; Enable peripheral and global interrupts.
            bsf     INTCON, 6, A
            bsf     INTCON, 7, A

;-----------------------------------------------------------
; MainLoop
; - If a tick is pending, consume exactly one then loop.
; - Otherwise run ParserService and loop.
;-----------------------------------------------------------
MainLoop:
            movf    pending_ticks, W, A
            bz      Main_DoParse

            ; Short critical section: atomic decrement of pending_ticks.
            bcf     INTCON, 7, A

            movf    pending_ticks, W, A
            bz      Main_ReEnable

            decf    pending_ticks, F, A
            bsf     INTCON, 7, A

            call    ProcessTickService
            bra     MainLoop

Main_ReEnable:
            bsf     INTCON, 7, A
            bra     MainLoop

Main_DoParse:
            call    ParserService
            bra     MainLoop

            end     rst
