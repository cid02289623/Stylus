            PROCESSOR 18F87K22
            #include <xc.inc>

;===========================================================
; gesture_core.s
; Owns: GestureInit, ProcessTickService, GestureService,
;       GestureStartCapture, GestureAccumulate
; Owns: all gesture RAM psects (gest_acs0, gest_acs1,
;       gest_acs2a, gest_acs2b, gest_udata)
;
; Other gesture sub-modules (calibration, classify, output)
; extrn the RAM declared here.
;===========================================================

            GLOBAL  GestureInit
            GLOBAL  ProcessTickService

;-----------------------------------------------------------
; Gesture RAM exported to other gesture sub-modules
;-----------------------------------------------------------
            GLOBAL  btn_now, prev_btn_down, gesture_state, tick_has_fresh
            GLOBAL  acc_v_b0, acc_v_b1, acc_v_b2, acc_v_b3
            GLOBAL  acc_h_b0, acc_h_b1, acc_h_b2, acc_h_b3
            GLOBAL  mag_dx_l, mag_dx_h, mag_dy_l, mag_dy_h
            GLOBAL  gesture_result, cmp_result

            GLOBAL  peak_v_pos_l, peak_v_pos_h
            GLOBAL  peak_v_neg_l, peak_v_neg_h
            GLOBAL  peak_h_pos_l, peak_h_pos_h
            GLOBAL  peak_h_neg_l, peak_h_neg_h
            GLOBAL  calib_active, calib_count, cal_prev_valid
            GLOBAL  bias_gx_l, bias_gx_h, bias_gz_l, bias_gz_h
            GLOBAL  tx_cksum, dbg_flags

            GLOBAL  sumx_l, sumx_h, sumx_u
            GLOBAL  sumz_l, sumz_h, sumz_u
            GLOBAL  tmp_dx_l, tmp_dx_h, tmp_dy_l, tmp_dy_h
            GLOBAL  shift_count

            GLOBAL  cal_prev_gx_l, cal_prev_gx_h
            GLOBAL  cal_prev_gz_l, cal_prev_gz_h

;-----------------------------------------------------------
; Symbols from main.s
;-----------------------------------------------------------
            extrn   sample_valid, new_sample
            extrn   latest_gx_l, latest_gx_h
            extrn   latest_gy_l, latest_gy_h
            extrn   latest_gz_l, latest_gz_h
            extrn   latest_btn
            extrn   proc_gx_l, proc_gx_h
            extrn   proc_gy_l, proc_gy_h
            extrn   proc_gz_l, proc_gz_h
            extrn   proc_btn
            extrn   debug_tickdiv

;-----------------------------------------------------------
; Symbols from other gesture sub-modules
;-----------------------------------------------------------
            extrn   CalibrationService
            extrn   GestureClassifyAndReport
            extrn   MakeAbsTmpDx, MakeAbsTmpDy
            extrn   UpdatePeakVPos, UpdatePeakVNeg
            extrn   UpdatePeakHPos, UpdatePeakHNeg

;-----------------------------------------------------------
; Gesture constants
;-----------------------------------------------------------
BTN_BIT         equ 0
G_IDLE          equ 0x00
G_CAPTURE       equ 0x01
RES_LEFT        equ 0x4C        ; 'L'
RES_RIGHT       equ 0x52        ; 'R'
RES_UP          equ 0x55        ; 'U'
RES_DOWN        equ 0x44        ; 'D'
RES_UNK         equ 0x3F        ; '?'

CAL_ACTIVE      equ 0x01
CAL_DONE        equ 0x00
CAL_SAMPLES     equ 64

;-----------------------------------------------------------
; Gesture RAM - access bank psects
; These must not move; other modules extrn these symbols.
;-----------------------------------------------------------
            psect   gest_acs0,class=COMRAM,space=1,noexec

btn_now:                ds 1
prev_btn_down:          ds 1
gesture_state:          ds 1
tick_has_fresh:         ds 1

acc_v_b0:               ds 1
acc_v_b1:               ds 1
acc_v_b2:               ds 1
acc_v_b3:               ds 1

acc_h_b0:               ds 1
acc_h_b1:               ds 1
acc_h_b2:               ds 1
acc_h_b3:               ds 1

mag_dx_l:               ds 1
mag_dx_h:               ds 1
mag_dy_l:               ds 1
mag_dy_h:               ds 1

gesture_result:         ds 1
cmp_result:             ds 1

            psect   gest_acs1,class=COMRAM,space=1,noexec

peak_v_pos_l:           ds 1
peak_v_pos_h:           ds 1
peak_v_neg_l:           ds 1
peak_v_neg_h:           ds 1

peak_h_pos_l:           ds 1
peak_h_pos_h:           ds 1
peak_h_neg_l:           ds 1
peak_h_neg_h:           ds 1

calib_active:           ds 1
calib_count:            ds 1
cal_prev_valid:         ds 1

bias_gx_l:              ds 1
bias_gx_h:              ds 1
bias_gz_l:              ds 1
bias_gz_h:              ds 1

tx_cksum:               ds 1
dbg_flags:              ds 1

            psect   gest_acs2a,class=COMRAM,space=1,noexec

sumx_l:                 ds 1
sumx_h:                 ds 1
sumx_u:                 ds 1

sumz_l:                 ds 1
sumz_h:                 ds 1
sumz_u:                 ds 1

            psect   gest_acs2b,class=COMRAM,space=1,noexec

tmp_dx_l:               ds 1
tmp_dx_h:               ds 1
tmp_dy_l:               ds 1
tmp_dy_h:               ds 1

shift_count:            ds 1

            psect   gest_udata

cal_prev_gx_l:          ds 1
cal_prev_gx_h:          ds 1
cal_prev_gz_l:          ds 1
cal_prev_gz_h:          ds 1

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
            clrf    tick_has_fresh, A

            clrf    acc_v_b0, A
            clrf    acc_v_b1, A
            clrf    acc_v_b2, A
            clrf    acc_v_b3, A

            clrf    acc_h_b0, A
            clrf    acc_h_b1, A
            clrf    acc_h_b2, A
            clrf    acc_h_b3, A

            clrf    mag_dx_l, A
            clrf    mag_dx_h, A
            clrf    mag_dy_l, A
            clrf    mag_dy_h, A

            clrf    peak_v_pos_l, A
            clrf    peak_v_pos_h, A
            clrf    peak_v_neg_l, A
            clrf    peak_v_neg_h, A

            clrf    peak_h_pos_l, A
            clrf    peak_h_pos_h, A
            clrf    peak_h_neg_l, A
            clrf    peak_h_neg_h, A

            clrf    cmp_result, A
            clrf    dbg_flags, A
            movlw   RES_UNK
            movwf   gesture_result, A

            movlw   CAL_ACTIVE
            movwf   calib_active, A
            clrf    calib_count, A
            clrf    cal_prev_valid, A

            clrf    bias_gx_l, A
            clrf    bias_gx_h, A
            clrf    bias_gz_l, A
            clrf    bias_gz_h, A

            clrf    tx_cksum, A

            clrf    sumx_l, A
            clrf    sumx_h, A
            clrf    sumx_u, A
            clrf    sumz_l, A
            clrf    sumz_h, A
            clrf    sumz_u, A

            clrf    tmp_dx_l, A
            clrf    tmp_dx_h, A
            clrf    tmp_dy_l, A
            clrf    tmp_dy_h, A
            clrf    shift_count, A

            clrf    cal_prev_gx_l, A
            clrf    cal_prev_gx_h, A
            clrf    cal_prev_gz_l, A
            clrf    cal_prev_gz_h, A

            ; RC5 on during startup calibration.
            bsf     LATC, 5, A
            return

;-----------------------------------------------------------
; ProcessTickService
; Called once per 20 ms processing slot.
;-----------------------------------------------------------
ProcessTickService:
            ; Toggle RC0 every 25 ticks = 500 ms heartbeat.
            incf    debug_tickdiv, F, A
            movlw   25
            cpfseq  debug_tickdiv, A
            bra     PTS_NoToggle

            clrf    debug_tickdiv, A
            btg     LATC, 0, A

PTS_NoToggle:
            ; No valid parsed sample yet - nothing to process.
            movf    sample_valid, W, A
            bz      PTS_Done

            ; Snapshot latest valid sample into proc_*.
            movff   latest_gx_l, proc_gx_l
            movff   latest_gx_h, proc_gx_h
            movff   latest_gy_l, proc_gy_l
            movff   latest_gy_h, proc_gy_h
            movff   latest_gz_l, proc_gz_l
            movff   latest_gz_h, proc_gz_h
            movff   latest_btn,  proc_btn

            ; Latch whether this tick has a fresh parser update.
            movf    new_sample, W, A
            movwf   tick_has_fresh, A
            clrf    new_sample, A

            ; Calibration runs first.
            movf    calib_active, W, A
            bz      PTS_RunGesture

            call    CalibrationService
            bra     PTS_Done

PTS_RunGesture:
            ; Gesture logic runs only on fresh frames.
            movf    tick_has_fresh, W, A
            bz      PTS_Done

            call    GestureService

PTS_Done:
            return

;-----------------------------------------------------------
; GestureService
; Button bit gates capture window.
; Accumulation occurs while pressed.
; Classification occurs once on falling edge.
;-----------------------------------------------------------
GestureService:
            clrf    btn_now, A
            btfss   proc_btn, BTN_BIT, A
            bra     GS_BtnDone
            movlw   0x01
            movwf   btn_now, A

GS_BtnDone:
            movf    gesture_state, W, A
            bz      GS_Idle

            ; Capture state.
            movf    btn_now, W, A
            bz      GS_Released

            call    GestureAccumulate
            bra     GS_UpdatePrev

GS_Released:
            ; Falling edge: classify once.
            movf    prev_btn_down, W, A
            bz      GS_UpdatePrev

            call    GestureClassifyAndReport
            clrf    gesture_state, A
            bra     GS_UpdatePrev

GS_Idle:
            ; Rising edge starts a new capture.
            movf    prev_btn_down, W, A
            bnz     GS_UpdatePrev

            movf    btn_now, W, A
            bz      GS_UpdatePrev

            call    GestureStartCapture

GS_UpdatePrev:
            movf    btn_now, W, A
            movwf   prev_btn_down, A
            return

;-----------------------------------------------------------
; GestureStartCapture
;-----------------------------------------------------------
GestureStartCapture:
            clrf    acc_v_b0, A
            clrf    acc_v_b1, A
            clrf    acc_v_b2, A
            clrf    acc_v_b3, A

            clrf    acc_h_b0, A
            clrf    acc_h_b1, A
            clrf    acc_h_b2, A
            clrf    acc_h_b3, A

            clrf    mag_dx_l, A
            clrf    mag_dx_h, A
            clrf    mag_dy_l, A
            clrf    mag_dy_h, A

            clrf    peak_v_pos_l, A
            clrf    peak_v_pos_h, A
            clrf    peak_v_neg_l, A
            clrf    peak_v_neg_h, A

            clrf    peak_h_pos_l, A
            clrf    peak_h_pos_h, A
            clrf    peak_h_neg_l, A
            clrf    peak_h_neg_h, A

            clrf    dbg_flags, A

            ; Clear RC1..RC5 only.
            movlw   11000001B
            andwf   LATC, F, A

            movlw   G_CAPTURE
            movwf   gesture_state, A

            ; Count first pressed sample immediately.
            call    GestureAccumulate
            return

;-----------------------------------------------------------
; GestureAccumulate
; vertical   = proc_gx - bias_gx
; horizontal = proc_gz - bias_gz
; Maintains 32-bit signed accumulators and signed peaks.
;-----------------------------------------------------------
GestureAccumulate:
            ; tmp_dx = proc_gx - bias_gx
            movf    bias_gx_l, W, A
            subwf   proc_gx_l, W, A
            movwf   tmp_dx_l, A
            movf    bias_gx_h, W, A
            subwfb  proc_gx_h, W, A
            movwf   tmp_dx_h, A

            ; acc_v += sign-extended tmp_dx
            movf    tmp_dx_l, W, A
            addwf   acc_v_b0, F, A
            movf    tmp_dx_h, W, A
            addwfc  acc_v_b1, F, A
            movlw   0x00
            btfsc   tmp_dx_h, 7, A
            movlw   0xFF
            addwfc  acc_v_b2, F, A
            movlw   0x00
            btfsc   tmp_dx_h, 7, A
            movlw   0xFF
            addwfc  acc_v_b3, F, A

            ; Update vertical peak.
            btfss   tmp_dx_h, 7, A
            bra     GA_V_Pos

            call    MakeAbsTmpDx
            call    UpdatePeakVNeg
            bra     GA_H

GA_V_Pos:
            call    UpdatePeakVPos

GA_H:
            ; tmp_dy = proc_gz - bias_gz
            movf    bias_gz_l, W, A
            subwf   proc_gz_l, W, A
            movwf   tmp_dy_l, A
            movf    bias_gz_h, W, A
            subwfb  proc_gz_h, W, A
            movwf   tmp_dy_h, A

            ; acc_h += sign-extended tmp_dy
            movf    tmp_dy_l, W, A
            addwf   acc_h_b0, F, A
            movf    tmp_dy_h, W, A
            addwfc  acc_h_b1, F, A
            movlw   0x00
            btfsc   tmp_dy_h, 7, A
            movlw   0xFF
            addwfc  acc_h_b2, F, A
            movlw   0x00
            btfsc   tmp_dy_h, 7, A
            movlw   0xFF
            addwfc  acc_h_b3, F, A

            ; Update horizontal peak.
            btfss   tmp_dy_h, 7, A
            bra     GA_H_Pos

            call    MakeAbsTmpDy
            call    UpdatePeakHNeg
            return

GA_H_Pos:
            call    UpdatePeakHPos
            return

            end
