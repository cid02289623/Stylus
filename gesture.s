            PROCESSOR 18F87K22
            #include <xc.inc>

            GLOBAL  GestureInit
            GLOBAL  ProcessTickService

            EXTERN  sample_valid, new_sample
            EXTERN  latest_gx_l, latest_gx_h
            EXTERN  latest_gy_l, latest_gy_h
            EXTERN  latest_gz_l, latest_gz_h
            EXTERN  latest_btn

            EXTERN  proc_gx_l, proc_gx_h
            EXTERN  proc_gy_l, proc_gy_h
            EXTERN  proc_gz_l, proc_gz_h
            EXTERN  proc_btn

            EXTERN  debug_tickdiv
            EXTERN  UART1_WriteByte

;-----------------------------------------------------------
; Phase 5 constants
;-----------------------------------------------------------
BTN_BIT     equ 0

G_IDLE      equ 0x00
G_CAPTURE   equ 0x01

RES_LEFT    equ 0x4C        ; 'L'
RES_RIGHT   equ 0x52        ; 'R'
RES_UP      equ 0x55        ; 'U'
RES_DOWN    equ 0x44        ; 'D'
RES_UNK     equ 0x3F        ; '?'

; Initial threshold for |Dx| or |Dy|.
; Tune later using measured data.
TDISP_H     equ 0x04
TDISP_L     equ 0x00

;-----------------------------------------------------------
; Gesture RAM
;-----------------------------------------------------------
            psect   udata_acs

btn_now:                 ds 1
prev_btn_down:           ds 1
gesture_state:           ds 1

acc_dx_l:                ds 1
acc_dx_h:                ds 1
acc_dy_l:                ds 1
acc_dy_h:                ds 1

mag_dx_l:                ds 1
mag_dx_h:                ds 1
mag_dy_l:                ds 1
mag_dy_h:                ds 1

gesture_result:          ds 1
cmp_result:              ds 1

;-----------------------------------------------------------
; Code
;-----------------------------------------------------------
            psect   gesture_code,class=CODE,reloc=2

;-----------------------------------------------------------
; GestureInit
;-----------------------------------------------------------
GestureInit:
            clrf    btn_now, A
            clrf    prev_btn_down, A
            clrf    gesture_state, A

            clrf    acc_dx_l, A
            clrf    acc_dx_h, A
            clrf    acc_dy_l, A
            clrf    acc_dy_h, A

            clrf    mag_dx_l, A
            clrf    mag_dx_h, A
            clrf    mag_dy_l, A
            clrf    mag_dy_h, A

            clrf    cmp_result, A
            movlw   RES_UNK
            movwf   gesture_result, A
            return

;-----------------------------------------------------------
; ProcessTickService
; Called once per 20 ms processing slot.
; - toggles RC0 heartbeat
; - snapshots latest valid sample into proc_*
; - runs gesture state machine
;-----------------------------------------------------------
ProcessTickService:
            ; Toggle RC0 every 25 ticks.
            ; 25 * 20 ms = 500 ms, full blink cycle = 1 s.
            incf    debug_tickdiv, F, A
            movlw   25
            cpfseq  debug_tickdiv, A
            bra     PTS_NoToggle

            clrf    debug_tickdiv, A
            btg     LATC, 0, A

PTS_NoToggle:
            ; No valid parsed sample yet -> nothing to process.
            movf    sample_valid, W, A
            bz      PTS_Done

            ; Snapshot latest valid sample into proc_*.
            ; This keeps processing tied to Timer1, not to UART arrival time.
            movff   latest_gx_l, proc_gx_l
            movff   latest_gx_h, proc_gx_h
            movff   latest_gy_l, proc_gy_l
            movff   latest_gy_h, proc_gy_h
            movff   latest_gz_l, proc_gz_l
            movff   latest_gz_h, proc_gz_h
            movff   latest_btn,  proc_btn

            ; This tick has consumed the "fresh sample" condition.
            clrf    new_sample, A

            ; Run the gesture engine.
            call    GestureService

PTS_Done:
            return

;-----------------------------------------------------------
; GestureService
; - detects button edges
; - captures while button held
; - classifies on release
;-----------------------------------------------------------
GestureService:
            ; btn_now = 1 if proc_btn bit is high, else 0
            clrf    btn_now, A
            btfsc   proc_btn, BTN_BIT, A
            incf    btn_now, F, A

            ; Branch by gesture state.
            movf    gesture_state, W, A
            bz      GS_Idle
            bra     GS_Capture

GS_Idle:
            ; Wait for rising edge: prev=0, now=1
            movf    btn_now, W, A
            bz      GS_UpdatePrev

            movf    prev_btn_down, W, A
            bnz     GS_UpdatePrev

            ; Rising edge -> start capture
            call    GestureStartCapture
            bra     GS_UpdatePrev

GS_Capture:
            ; While pressed, accumulate every 20 ms tick.
            movf    btn_now, W, A
            bz      GS_Released

            call    GestureAccumulate
            bra     GS_UpdatePrev

GS_Released:
            ; Falling edge -> classify once.
            movf    prev_btn_down, W, A
            bz      GS_UpdatePrev

            call    GestureClassifyAndReport
            clrf    gesture_state, A

GS_UpdatePrev:
            movf    btn_now, W, A
            movwf   prev_btn_down, A
            return

;-----------------------------------------------------------
; GestureStartCapture
; - clear accumulators
; - clear result LEDs
; - enter capture state
; - accumulate current pressed sample immediately
;-----------------------------------------------------------
GestureStartCapture:
            clrf    acc_dx_l, A
            clrf    acc_dx_h, A
            clrf    acc_dy_l, A
            clrf    acc_dy_h, A

            ; Clear result LEDs RC1..RC5, preserve RC0/RC6/RC7.
            movlw   11000001B
            andwf   LATC, F, A

            movlw   G_CAPTURE
            movwf   gesture_state, A

            ; Count the first pressed sample immediately.
            call    GestureAccumulate
            return

;-----------------------------------------------------------
; GestureAccumulate
; acc_dx += proc_gx
; acc_dy += proc_gy
;-----------------------------------------------------------
GestureAccumulate:
            movf    proc_gx_l, W, A
            addwf   acc_dx_l, F, A
            movf    proc_gx_h, W, A
            addwfc  acc_dx_h, F, A

            movf    proc_gy_l, W, A
            addwf   acc_dy_l, F, A
            movf    proc_gy_h, W, A
            addwfc  acc_dy_h, F, A
            return

;-----------------------------------------------------------
; GestureClassifyAndReport
; Decision:
;   Ax = |Dx|
;   Ay = |Dy|
;   if dominant magnitude <= threshold -> UNKNOWN
;   else dominant axis + sign -> L/R/U/D
;-----------------------------------------------------------
GestureClassifyAndReport:
            call    MakeAbsDx
            call    MakeAbsDy

            ; Compare |Dx| against |Dy| to choose dominant axis.
            movf    mag_dy_h, W, A
            cpfsgt  mag_dx_h, A
            bra     GCR_XHiNotGreater
            bra     GCR_DomX

GCR_XHiNotGreater:
            movf    mag_dy_h, W, A
            cpfslt  mag_dx_h, A
            bra     GCR_CompareLow
            bra     GCR_DomY

GCR_CompareLow:
            movf    mag_dy_l, W, A
            cpfsgt  mag_dx_l, A
            bra     GCR_XLoNotGreater
            bra     GCR_DomX

GCR_XLoNotGreater:
            movf    mag_dy_l, W, A
            cpfslt  mag_dx_l, A
            bra     GCR_Tie
            bra     GCR_DomY

GCR_DomX:
            call    MagDxAboveThreshold
            bz      GCR_Unknown

            ; Negative Dx = LEFT, positive Dx = RIGHT
            btfsc   acc_dx_h, 7, A
            bra     GCR_Left
            bra     GCR_Right

GCR_DomY:
            call    MagDyAboveThreshold
            bz      GCR_Unknown

            ; Negative Dy = DOWN, positive Dy = UP
            btfsc   acc_dy_h, 7, A
            bra     GCR_Down
            bra     GCR_Up

GCR_Tie:
            bra     GCR_Unknown

GCR_Left:
            movlw   RES_LEFT
            movwf   gesture_result, A
            bra     GCR_Report

GCR_Right:
            movlw   RES_RIGHT
            movwf   gesture_result, A
            bra     GCR_Report

GCR_Up:
            movlw   RES_UP
            movwf   gesture_result, A
            bra     GCR_Report

GCR_Down:
            movlw   RES_DOWN
            movwf   gesture_result, A
            bra     GCR_Report

GCR_Unknown:
            movlw   RES_UNK
            movwf   gesture_result, A

GCR_Report:
            call    ShowGestureOnLEDs
            call    SendGestureResultByte
            return

;-----------------------------------------------------------
; MakeAbsDx
; mag_dx = |acc_dx|
;-----------------------------------------------------------
MakeAbsDx:
            movff   acc_dx_l, mag_dx_l
            movff   acc_dx_h, mag_dx_h

            btfss   mag_dx_h, 7, A
            return

            comf    mag_dx_l, F, A
            comf    mag_dx_h, F, A
            incf    mag_dx_l, F, A
            bnz     MADX_Done
            incf    mag_dx_h, F, A
MADX_Done:
            return

;-----------------------------------------------------------
; MakeAbsDy
; mag_dy = |acc_dy|
;-----------------------------------------------------------
MakeAbsDy:
            movff   acc_dy_l, mag_dy_l
            movff   acc_dy_h, mag_dy_h

            btfss   mag_dy_h, 7, A
            return

            comf    mag_dy_l, F, A
            comf    mag_dy_h, F, A
            incf    mag_dy_l, F, A
            bnz     MADY_Done
            incf    mag_dy_h, F, A
MADY_Done:
            return

;-----------------------------------------------------------
; MagDxAboveThreshold
; Returns W=1 if |Dx| > threshold, else W=0
;-----------------------------------------------------------
MagDxAboveThreshold:
            clrf    cmp_result, A

            movlw   TDISP_H
            cpfsgt  mag_dx_h, A
            bra     MDTX_HNotGreater

            movlw   0x01
            movwf   cmp_result, A
            bra     MDTX_Done

MDTX_HNotGreater:
            movlw   TDISP_H
            cpfseq  mag_dx_h, A
            bra     MDTX_Done

            movlw   TDISP_L
            cpfsgt  mag_dx_l, A
            bra     MDTX_Done

            movlw   0x01
            movwf   cmp_result, A

MDTX_Done:
            movf    cmp_result, W, A
            return

;-----------------------------------------------------------
; MagDyAboveThreshold
; Returns W=1 if |Dy| > threshold, else W=0
;-----------------------------------------------------------
MagDyAboveThreshold:
            clrf    cmp_result, A

            movlw   TDISP_H
            cpfsgt  mag_dy_h, A
            bra     MDTY_HNotGreater

            movlw   0x01
            movwf   cmp_result, A
            bra     MDTY_Done

MDTY_HNotGreater:
            movlw   TDISP_H
            cpfseq  mag_dy_h, A
            bra     MDTY_Done

            movlw   TDISP_L
            cpfsgt  mag_dy_l, A
            bra     MDTY_Done

            movlw   0x01
            movwf   cmp_result, A

MDTY_Done:
            movf    cmp_result, W, A
            return

;-----------------------------------------------------------
; ShowGestureOnLEDs
; RC1 = LEFT
; RC2 = RIGHT
; RC3 = UP
; RC4 = DOWN
; RC5 = UNKNOWN
; RC0 remains heartbeat
;-----------------------------------------------------------
ShowGestureOnLEDs:
            ; Clear RC1..RC5 only.
            movlw   11000001B
            andwf   LATC, F, A

            movf    gesture_result, W, A
            xorlw   RES_LEFT
            bnz     SG_NotLeft
            bsf     LATC, 1, A
            return

SG_NotLeft:
            movf    gesture_result, W, A
            xorlw   RES_RIGHT
            bnz     SG_NotRight
            bsf     LATC, 2, A
            return

SG_NotRight:
            movf    gesture_result, W, A
            xorlw   RES_UP
            bnz     SG_NotUp
            bsf     LATC, 3, A
            return

SG_NotUp:
            movf    gesture_result, W, A
            xorlw   RES_DOWN
            bnz     SG_NotDown
            bsf     LATC, 4, A
            return

SG_NotDown:
            bsf     LATC, 5, A
            return

;-----------------------------------------------------------
; SendGestureResultByte
; Sends one UART byte for logging.
;-----------------------------------------------------------
SendGestureResultByte:
            movf    gesture_result, W, A
            call    UART1_WriteByte
            return

            end
