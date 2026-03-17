PROCESSOR 18F87K22
            #include <xc.inc>

            GLOBAL  GestureInit
            GLOBAL  ProcessTickService

            GLOBAL  sample_valid, new_sample
            GLOBAL  latest_gx_l, latest_gx_h
            GLOBAL  latest_gy_l, latest_gy_h
            GLOBAL  latest_gz_l, latest_gz_h
            GLOBAL  latest_btn

            GLOBAL  proc_gx_l, proc_gx_h
            GLOBAL  proc_gy_l, proc_gy_h
            GLOBAL  proc_gz_l, proc_gz_h
            GLOBAL  proc_btn

            GLOBAL  debug_tickdiv
            GLOBAL  UART1_WriteByte

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

;-----------------------------------------------------------
; Debug frame SOFs
;
; Bias frame:
;   0xB1, bias_gx_l, bias_gx_h, bias_gz_l, bias_gz_h, checksum
;
; Classifier frame:
;   0xE2,
;   proc_gx_l, proc_gx_h,
;   proc_gy_l, proc_gy_h,
;   proc_gz_l, proc_gz_h,
;   proc_btn,
;   bias_gx_l, bias_gx_h,
;   bias_gz_l, bias_gz_h,
;   acc_v_b0, acc_v_b1, acc_v_b2, acc_v_b3,
;   acc_h_b0, acc_h_b1, acc_h_b2, acc_h_b3,
;   peak_v_pos_l, peak_v_pos_h,
;   peak_v_neg_l, peak_v_neg_h,
;   peak_h_pos_l, peak_h_pos_h,
;   peak_h_neg_l, peak_h_neg_h,
;   dbg_flags,
;   gesture_result,
;   checksum
;-----------------------------------------------------------
BIAS_SOF        equ 0xB1
DBG_SOF         equ 0xE2

;-----------------------------------------------------------
; Classifier parameters
;-----------------------------------------------------------
PEAK_THR_H      equ 0x08
PEAK_THR_L      equ 0x00        ; 0x0800 = 2048 counts

; Dominance check:
;   winner must be > other + (other >> 2)
; i.e. about 1.25x larger, less brittle than 1.5x
;
; This helps because your real captures are often mixed-axis.
;-----------------------------------------------------------

;-----------------------------------------------------------
; Startup calibration constants
; 64 accepted still samples = about 1.28 s at 50 Hz.
;-----------------------------------------------------------
CAL_ACTIVE      equ 0x01
CAL_DONE        equ 0x00
CAL_SAMPLES     equ 64

;-----------------------------------------------------------
; Stillness gate for startup calibration.
; Compare Gx and Gz because those are the two axes used by
; this classifier.
;-----------------------------------------------------------
CAL_DDELTA_H    equ 0x01
CAL_DDELTA_L    equ 0x00        ; 0x0100 = 256 counts
;-----------------------------------------------------------
; Gesture RAM split into several smaller COMRAM psects
;-----------------------------------------------------------
            psect   gest_acs0,class=COMRAM,space=1,noexec

btn_now:                ds 1
prev_btn_down:          ds 1
gesture_state:          ds 1
tick_has_fresh:         ds 1

; 32-bit signed accumulators for debug / analysis
acc_v_b0:               ds 1
acc_v_b1:               ds 1
acc_v_b2:               ds 1
acc_v_b3:               ds 1

acc_h_b0:               ds 1
acc_h_b1:               ds 1
acc_h_b2:               ds 1
acc_h_b3:               ds 1

; Best magnitudes chosen from positive/negative peaks
mag_dx_l:               ds 1
mag_dx_h:               ds 1
mag_dy_l:               ds 1
mag_dy_h:               ds 1

gesture_result:         ds 1
cmp_result:             ds 1

;-----------------------------------------------------------
; More access RAM, separate psect
;-----------------------------------------------------------
            psect   gest_acs1,class=COMRAM,space=1,noexec

; Signed per-sample peaks captured during the button window
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

;-----------------------------------------------------------
; More access RAM, separate psect
;-----------------------------------------------------------
                       psect   gest_acs2a,class=COMRAM,space=1,noexec

; 24-bit sums for calibration averaging
sumx_l:                 ds 1
sumx_h:                 ds 1
sumx_u:                 ds 1

sumz_l:                 ds 1
sumz_h:                 ds 1
sumz_u:                 ds 1

            psect   gest_acs2b,class=COMRAM,space=1,noexec

; Temporary signed 16-bit working values
tmp_dx_l:               ds 1
tmp_dx_h:               ds 1
tmp_dy_l:               ds 1
tmp_dy_h:               ds 1

shift_count:            ds 1

            psect   gest_udata

; Previous calibration sample
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

            ; Start calibration mode.
            movlw   CAL_ACTIVE
            movwf   calib_active, A
            clrf    calib_count, A
            clrf    cal_prev_valid, A

            ; Clear learned biases.
            clrf    bias_gx_l, A
            clrf    bias_gx_h, A
            clrf    bias_gz_l, A
            clrf    bias_gz_h, A

            ; Clear UART checksum temp.
            clrf    tx_cksum, A

            ; Clear running sums.
            clrf    sumx_l, A
            clrf    sumx_h, A
            clrf    sumx_u, A
            clrf    sumz_l, A
            clrf    sumz_h, A
            clrf    sumz_u, A

            ; Clear temporaries.
            clrf    tmp_dx_l, A
            clrf    tmp_dx_h, A
            clrf    tmp_dy_l, A
            clrf    tmp_dy_h, A
            clrf    shift_count, A

            ; Clear previous calibration sample.
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
; - toggles RC0 heartbeat
; - snapshots latest valid sample into proc_*
; - latches whether this tick has a fresh parsed frame
; - while calibration is active, runs calibration only
; - after calibration, runs gesture logic only on fresh frames
;-----------------------------------------------------------
ProcessTickService:
            ; Toggle RC0 every 25 ticks = 500 ms.
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
            movff   latest_gx_l, proc_gx_l
            movff   latest_gx_h, proc_gx_h
            movff   latest_gy_l, proc_gy_l
            movff   latest_gy_h, proc_gy_h
            movff   latest_gz_l, proc_gz_l
            movff   latest_gz_h, proc_gz_h
            movff   latest_btn,  proc_btn

            ; Latch whether this processing slot has a fresh parser update.
            movf    new_sample, W, A
            movwf   tick_has_fresh, A
            clrf    new_sample, A

            ; Calibration runs first.
            movf    calib_active, W, A
            bz      PTS_RunGesture

            call    CalibrationService
            bra     PTS_Done

PTS_RunGesture:
            ; Gesture logic runs only when a fresh frame exists.
            movf    tick_has_fresh, W, A
            bz      PTS_Done

            call    GestureService

PTS_Done:
            return

;-----------------------------------------------------------
; CalibrationService
; - uses only fresh parsed samples
; - applies a stillness gate
; - accumulates 64 still samples for Gx and Gz
; - computes signed average bias = sum / 64
;-----------------------------------------------------------
CalibrationService:
            movf    tick_has_fresh, W, A
            bz      CAL_Done

            call    CalibrationCheckStill
            bz      CAL_Done

            ; sumx += sign-extended proc_gx
            movf    proc_gx_l, W, A
            addwf   sumx_l, F, A
            movf    proc_gx_h, W, A
            addwfc  sumx_h, F, A
            movlw   0x00
            btfsc   proc_gx_h, 7, A
            movlw   0xFF
            addwfc  sumx_u, F, A

            ; sumz += sign-extended proc_gz
            movf    proc_gz_l, W, A
            addwf   sumz_l, F, A
            movf    proc_gz_h, W, A
            addwfc  sumz_h, F, A
            movlw   0x00
            btfsc   proc_gz_h, 7, A
            movlw   0xFF
            addwfc  sumz_u, F, A

            incf    calib_count, F, A

            movlw   CAL_SAMPLES
            cpfseq  calib_count, A
            bra     CAL_Done

            call    FinishCalibration

CAL_Done:
            return

;-----------------------------------------------------------
; CalibrationCheckStill
; Returns W = 1 if the current fresh sample is still enough.
;-----------------------------------------------------------
CalibrationCheckStill:
            clrf    cmp_result, A

            ; First fresh sample only seeds the previous registers.
            movf    cal_prev_valid, W, A
            bnz     CCS_HavePrev

            call    CalibrationStorePrev
            return

CCS_HavePrev:
            ; Load previous Gx from normal RAM into tmp_dy as scratch
            movff   cal_prev_gx_l, tmp_dy_l
            movff   cal_prev_gx_h, tmp_dy_h

            ; tmp_dx = proc_gx - cal_prev_gx
            movf    tmp_dy_l, W, A
            subwf   proc_gx_l, W, A
            movwf   tmp_dx_l, A
            movf    tmp_dy_h, W, A
            subwfb  proc_gx_h, W, A
            movwf   tmp_dx_h, A

            call    MakeAbsTmpDx
            call    TmpDxWithinCalDelta
            bz      CCS_UpdatePrev

            ; Load previous Gz from normal RAM into tmp_dx as scratch
            movff   cal_prev_gz_l, tmp_dx_l
            movff   cal_prev_gz_h, tmp_dx_h

            ; tmp_dy = proc_gz - cal_prev_gz
            movf    tmp_dx_l, W, A
            subwf   proc_gz_l, W, A
            movwf   tmp_dy_l, A
            movf    tmp_dx_h, W, A
            subwfb  proc_gz_h, W, A
            movwf   tmp_dy_h, A

            call    MakeAbsTmpDy
            call    TmpDyWithinCalDelta
            bz      CCS_UpdatePrev

            movlw   0x01
            movwf   cmp_result, A

CCS_UpdatePrev:
            call    CalibrationStorePrev
            movf    cmp_result, W, A
            return

;-----------------------------------------------------------
; CalibrationStorePrev
;-----------------------------------------------------------
CalibrationStorePrev:
            movff   proc_gx_l, cal_prev_gx_l
            movff   proc_gx_h, cal_prev_gx_h
            movff   proc_gz_l, cal_prev_gz_l
            movff   proc_gz_h, cal_prev_gz_h
            movlw   0x01
            movwf   cal_prev_valid, A
            return
	    
;-----------------------------------------------------------
; FinishCalibration
;-----------------------------------------------------------
FinishCalibration:
            ; bias_gx = sumx / 64
            movlw   6
            movwf   shift_count, A

FC_ShiftX:
            bcf     STATUS, 0, A
            btfsc   sumx_u, 7, A
            bsf     STATUS, 0, A
            rrcf    sumx_u, F, A
            rrcf    sumx_h, F, A
            rrcf    sumx_l, F, A
            decfsz  shift_count, F, A
            bra     FC_ShiftX

            movff   sumx_l, bias_gx_l
            movff   sumx_h, bias_gx_h

            ; bias_gz = sumz / 64
            movlw   6
            movwf   shift_count, A

FC_ShiftZ:
            bcf     STATUS, 0, A
            btfsc   sumz_u, 7, A
            bsf     STATUS, 0, A
            rrcf    sumz_u, F, A
            rrcf    sumz_h, F, A
            rrcf    sumz_l, F, A
            decfsz  shift_count, F, A
            bra     FC_ShiftZ

            movff   sumz_l, bias_gz_l
            movff   sumz_h, bias_gz_h

            ; Calibration complete.
            clrf    calib_active, A
            clrf    calib_count, A
            clrf    cal_prev_valid, A

            ; Emit one bias frame for debug.
            call    SendBiasDebugFrame

            ; Clear RC1..RC5, preserve heartbeat on RC0 and UART pins.
            movlw   11000001B
            andwf   LATC, F, A
            return

;-----------------------------------------------------------
; SendByteWithChecksum
; Input: WREG = byte to send
;-----------------------------------------------------------
SendByteWithChecksum:
            addwf   tx_cksum, F, A
            call    UART1_WriteByte
            return

;-----------------------------------------------------------
; SendBiasDebugFrame
; Sends:
;   0xB1, bias_gx_l, bias_gx_h, bias_gz_l, bias_gz_h, checksum
;-----------------------------------------------------------
SendBiasDebugFrame:
            movlw   BIAS_SOF
            call    UART1_WriteByte

            clrf    tx_cksum, A

            movf    bias_gx_l, W, A
            call    SendByteWithChecksum
            movf    bias_gx_h, W, A
            call    SendByteWithChecksum
            movf    bias_gz_l, W, A
            call    SendByteWithChecksum
            movf    bias_gz_h, W, A
            call    SendByteWithChecksum

            movf    tx_cksum, W, A
            call    UART1_WriteByte
            return

;-----------------------------------------------------------
; GestureService
; - button bit gates capture
; - accumulation occurs only while pressed
; - classification occurs once on falling edge
;-----------------------------------------------------------
GestureService:
            ; btn_now = 1 if capture bit is high, else 0
            clrf    btn_now, A
            btfss   proc_btn, BTN_BIT, A
            bra     GS_BtnDone
            movlw   0x01
            movwf   btn_now, A

GS_BtnDone:
            movf    gesture_state, W, A
            bz      GS_Idle

            ; Capture state
            movf    btn_now, W, A
            bz      GS_Released

            call    GestureAccumulate
            bra     GS_UpdatePrev

GS_Released:
            ; Falling edge -> classify once
            movf    prev_btn_down, W, A
            bz      GS_UpdatePrev

            call    GestureClassifyAndReport
            clrf    gesture_state, A
            bra     GS_UpdatePrev

GS_Idle:
            ; Rising edge starts a new capture
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
            ; Clear 32-bit accumulators
            clrf    acc_v_b0, A
            clrf    acc_v_b1, A
            clrf    acc_v_b2, A
            clrf    acc_v_b3, A

            clrf    acc_h_b0, A
            clrf    acc_h_b1, A
            clrf    acc_h_b2, A
            clrf    acc_h_b3, A

            ; Clear selected magnitudes
            clrf    mag_dx_l, A
            clrf    mag_dx_h, A
            clrf    mag_dy_l, A
            clrf    mag_dy_h, A

            ; Clear per-axis signed peaks
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
; Uses:
;   vertical   = proc_gx - bias_gx
;   horizontal = proc_gz - bias_gz
;
; We keep:
; - 32-bit signed accumulators for debug / evidence
; - positive and negative peaks for robust classification
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

            ; Update vertical peak
            btfss   tmp_dx_h, 7, A
            bra     GA_V_Pos

            ; Negative vertical: store magnitude in peak_v_neg
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

            ; Update horizontal peak
            btfss   tmp_dy_h, 7, A
            bra     GA_H_Pos

            ; Negative horizontal: store magnitude in peak_h_neg
            call    MakeAbsTmpDy
            call    UpdatePeakHNeg
            return

GA_H_Pos:
            call    UpdatePeakHPos
            return

;-----------------------------------------------------------
; UpdatePeakVPos
; Updates peak_v_pos if tmp_dx is larger
;-----------------------------------------------------------
UpdatePeakVPos:
            movf    peak_v_pos_h, W, A
            cpfsgt  tmp_dx_h, A
            bra     UVP_HNotGreater

            movff   tmp_dx_l, peak_v_pos_l
            movff   tmp_dx_h, peak_v_pos_h
            return

UVP_HNotGreater:
            movf    peak_v_pos_h, W, A
            cpfslt  tmp_dx_h, A
            bra     UVP_CheckLow
            return

UVP_CheckLow:
            movf    peak_v_pos_l, W, A
            cpfsgt  tmp_dx_l, A
            return

            movff   tmp_dx_l, peak_v_pos_l
            movff   tmp_dx_h, peak_v_pos_h
            return

;-----------------------------------------------------------
; UpdatePeakVNeg
; Updates peak_v_neg if |tmp_dx| is larger
;-----------------------------------------------------------
UpdatePeakVNeg:
            movf    peak_v_neg_h, W, A
            cpfsgt  tmp_dx_h, A
            bra     UVN_HNotGreater

            movff   tmp_dx_l, peak_v_neg_l
            movff   tmp_dx_h, peak_v_neg_h
            return

UVN_HNotGreater:
            movf    peak_v_neg_h, W, A
            cpfslt  tmp_dx_h, A
            bra     UVN_CheckLow
            return

UVN_CheckLow:
            movf    peak_v_neg_l, W, A
            cpfsgt  tmp_dx_l, A
            return

            movff   tmp_dx_l, peak_v_neg_l
            movff   tmp_dx_h, peak_v_neg_h
            return

;-----------------------------------------------------------
; UpdatePeakHPos
; Updates peak_h_pos if tmp_dy is larger
;-----------------------------------------------------------
UpdatePeakHPos:
            movf    peak_h_pos_h, W, A
            cpfsgt  tmp_dy_h, A
            bra     UHP_HNotGreater

            movff   tmp_dy_l, peak_h_pos_l
            movff   tmp_dy_h, peak_h_pos_h
            return

UHP_HNotGreater:
            movf    peak_h_pos_h, W, A
            cpfslt  tmp_dy_h, A
            bra     UHP_CheckLow
            return

UHP_CheckLow:
            movf    peak_h_pos_l, W, A
            cpfsgt  tmp_dy_l, A
            return

            movff   tmp_dy_l, peak_h_pos_l
            movff   tmp_dy_h, peak_h_pos_h
            return

;-----------------------------------------------------------
; UpdatePeakHNeg
; Updates peak_h_neg if |tmp_dy| is larger
;-----------------------------------------------------------
UpdatePeakHNeg:
            movf    peak_h_neg_h, W, A
            cpfsgt  tmp_dy_h, A
            bra     UHN_HNotGreater

            movff   tmp_dy_l, peak_h_neg_l
            movff   tmp_dy_h, peak_h_neg_h
            return

UHN_HNotGreater:
            movf    peak_h_neg_h, W, A
            cpfslt  tmp_dy_h, A
            bra     UHN_CheckLow
            return

UHN_CheckLow:
            movf    peak_h_neg_l, W, A
            cpfsgt  tmp_dy_l, A
            return

            movff   tmp_dy_l, peak_h_neg_l
            movff   tmp_dy_h, peak_h_neg_h
            return

;-----------------------------------------------------------
; SelectVerticalBest
; mag_dx = max(peak_v_pos, peak_v_neg)
;-----------------------------------------------------------
SelectVerticalBest:
            movff   peak_v_pos_l, mag_dx_l
            movff   peak_v_pos_h, mag_dx_h

            movf    peak_v_pos_h, W, A
            cpfsgt  peak_v_neg_h, A
            bra     SVB_HNotGreater

            movff   peak_v_neg_l, mag_dx_l
            movff   peak_v_neg_h, mag_dx_h
            return

SVB_HNotGreater:
            movf    peak_v_pos_h, W, A
            cpfslt  peak_v_neg_h, A
            bra     SVB_CheckLow
            return

SVB_CheckLow:
            movf    peak_v_pos_l, W, A
            cpfsgt  peak_v_neg_l, A
            return

            movff   peak_v_neg_l, mag_dx_l
            movff   peak_v_neg_h, mag_dx_h
            return

;-----------------------------------------------------------
; SelectHorizontalBest
; mag_dy = max(peak_h_pos, peak_h_neg)
;-----------------------------------------------------------
SelectHorizontalBest:
            movff   peak_h_pos_l, mag_dy_l
            movff   peak_h_pos_h, mag_dy_h

            movf    peak_h_pos_h, W, A
            cpfsgt  peak_h_neg_h, A
            bra     SHB_HNotGreater

            movff   peak_h_neg_l, mag_dy_l
            movff   peak_h_neg_h, mag_dy_h
            return

SHB_HNotGreater:
            movf    peak_h_pos_h, W, A
            cpfslt  peak_h_neg_h, A
            bra     SHB_CheckLow
            return

SHB_CheckLow:
            movf    peak_h_pos_l, W, A
            cpfsgt  peak_h_neg_l, A
            return

            movff   peak_h_neg_l, mag_dy_l
            movff   peak_h_neg_h, mag_dy_h
            return

;-----------------------------------------------------------
; BuildDebugFlags
; bit0 = btn_now
; bit1 = vertical magnitude > threshold
; bit2 = horizontal magnitude > threshold
; bit3 = vertical dominates
; bit4 = horizontal dominates
; bit5 = classifier chose vertical branch
; bit6 = classifier chose horizontal branch
;-----------------------------------------------------------
BuildDebugFlags:
            clrf    dbg_flags, A

            btfsc   btn_now, 0, A
            bsf     dbg_flags, 0, A

            call    PeakVAboveThreshold
            bz      BDF_NoVThr
            bsf     dbg_flags, 1, A
BDF_NoVThr:

            call    PeakHAboveThreshold
            bz      BDF_NoHThr
            bsf     dbg_flags, 2, A
BDF_NoHThr:

            call    PeakVDominates
            bz      BDF_NoVDom
            bsf     dbg_flags, 3, A
BDF_NoVDom:

            call    PeakHDominates
            bz      BDF_NoHDom
            bsf     dbg_flags, 4, A
BDF_NoHDom:
            return

;-----------------------------------------------------------
; GestureClassifyAndReport
;
; Mapping used here:
;   +Gx => UP
;   -Gx => DOWN
;   +Gz => LEFT
;   -Gz => RIGHT
;
; Classification uses peaks, not final signed sums, because your
; real button windows often contain return-swing cancellation.
;-----------------------------------------------------------
GestureClassifyAndReport:
            call    SelectVerticalBest
            call    SelectHorizontalBest
            call    BuildDebugFlags

            ; Compare best vertical against best horizontal
            movf    mag_dy_h, W, A
            cpfsgt  mag_dx_h, A
            bra     GCR_VHiNotGreater
            bra     GCR_DomV

GCR_VHiNotGreater:
            movf    mag_dy_h, W, A
            cpfslt  mag_dx_h, A
            bra     GCR_CompareLow
            bra     GCR_DomH

GCR_CompareLow:
            movf    mag_dy_l, W, A
            cpfsgt  mag_dx_l, A
            bra     GCR_VLoNotGreater
            bra     GCR_DomV

GCR_VLoNotGreater:
            movf    mag_dy_l, W, A
            cpfslt  mag_dx_l, A
            bra     GCR_Tie
            bra     GCR_DomH

GCR_DomV:
            bsf     dbg_flags, 5, A
            call    PeakVAboveThreshold
            bz      GCR_Unknown

            call    PeakVDominates
            bz      GCR_Unknown

            ; If negative peak is larger -> DOWN, else UP
            movf    peak_v_pos_h, W, A
            cpfsgt  peak_v_neg_h, A
            bra     GCR_VSignHiNotGreater
            bra     GCR_Down

GCR_VSignHiNotGreater:
            movf    peak_v_pos_h, W, A
            cpfslt  peak_v_neg_h, A
            bra     GCR_VSignLow
            bra     GCR_Up

GCR_VSignLow:
            movf    peak_v_pos_l, W, A
            cpfsgt  peak_v_neg_l, A
            bra     GCR_Up
            bra     GCR_Down

GCR_DomH:
            bsf     dbg_flags, 6, A
            call    PeakHAboveThreshold
            bz      GCR_Unknown

            call    PeakHDominates
            bz      GCR_Unknown

            ; If negative peak is larger -> RIGHT, else LEFT
            movf    peak_h_pos_h, W, A
            cpfsgt  peak_h_neg_h, A
            bra     GCR_HSignHiNotGreater
            bra     GCR_Right

GCR_HSignHiNotGreater:
            movf    peak_h_pos_h, W, A
            cpfslt  peak_h_neg_h, A
            bra     GCR_HSignLow
            bra     GCR_Left

GCR_HSignLow:
            movf    peak_h_pos_l, W, A
            cpfsgt  peak_h_neg_l, A
            bra     GCR_Left
            bra     GCR_Right

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
            call    SendClassifierDebugFrame
            return

;-----------------------------------------------------------
; SendClassifierDebugFrame
;-----------------------------------------------------------
SendClassifierDebugFrame:
            movlw   DBG_SOF
            call    UART1_WriteByte

            clrf    tx_cksum, A

            ; proc_gx
            movf    proc_gx_l, W, A
            call    SendByteWithChecksum
            movf    proc_gx_h, W, A
            call    SendByteWithChecksum

            ; proc_gy
            movf    proc_gy_l, W, A
            call    SendByteWithChecksum
            movf    proc_gy_h, W, A
            call    SendByteWithChecksum

            ; proc_gz
            movf    proc_gz_l, W, A
            call    SendByteWithChecksum
            movf    proc_gz_h, W, A
            call    SendByteWithChecksum

            ; proc_btn
            movf    proc_btn, W, A
            call    SendByteWithChecksum

            ; bias_gx
            movf    bias_gx_l, W, A
            call    SendByteWithChecksum
            movf    bias_gx_h, W, A
            call    SendByteWithChecksum

            ; bias_gz
            movf    bias_gz_l, W, A
            call    SendByteWithChecksum
            movf    bias_gz_h, W, A
            call    SendByteWithChecksum

            ; acc_v (32-bit signed)
            movf    acc_v_b0, W, A
            call    SendByteWithChecksum
            movf    acc_v_b1, W, A
            call    SendByteWithChecksum
            movf    acc_v_b2, W, A
            call    SendByteWithChecksum
            movf    acc_v_b3, W, A
            call    SendByteWithChecksum

            ; acc_h (32-bit signed)
            movf    acc_h_b0, W, A
            call    SendByteWithChecksum
            movf    acc_h_b1, W, A
            call    SendByteWithChecksum
            movf    acc_h_b2, W, A
            call    SendByteWithChecksum
            movf    acc_h_b3, W, A
            call    SendByteWithChecksum

            ; peak_v_pos
            movf    peak_v_pos_l, W, A
            call    SendByteWithChecksum
            movf    peak_v_pos_h, W, A
            call    SendByteWithChecksum

            ; peak_v_neg
            movf    peak_v_neg_l, W, A
            call    SendByteWithChecksum
            movf    peak_v_neg_h, W, A
            call    SendByteWithChecksum

            ; peak_h_pos
            movf    peak_h_pos_l, W, A
            call    SendByteWithChecksum
            movf    peak_h_pos_h, W, A
            call    SendByteWithChecksum

            ; peak_h_neg
            movf    peak_h_neg_l, W, A
            call    SendByteWithChecksum
            movf    peak_h_neg_h, W, A
            call    SendByteWithChecksum

            ; flags
            movf    dbg_flags, W, A
            call    SendByteWithChecksum

            ; result
            movf    gesture_result, W, A
            call    SendByteWithChecksum

            ; checksum
            movf    tx_cksum, W, A
            call    UART1_WriteByte
            return

;-----------------------------------------------------------
; MakeAbsTmpDx
; tmp_dx = |tmp_dx|
;-----------------------------------------------------------
MakeAbsTmpDx:
            btfss   tmp_dx_h, 7, A
            return

            comf    tmp_dx_l, F, A
            comf    tmp_dx_h, F, A
            incf    tmp_dx_l, F, A
            bnz     MATDX_Done
            incf    tmp_dx_h, F, A
MATDX_Done:
            return

;-----------------------------------------------------------
; MakeAbsTmpDy
; tmp_dy = |tmp_dy|
;-----------------------------------------------------------
MakeAbsTmpDy:
            btfss   tmp_dy_h, 7, A
            return

            comf    tmp_dy_l, F, A
            comf    tmp_dy_h, F, A
            incf    tmp_dy_l, F, A
            bnz     MATDY_Done
            incf    tmp_dy_h, F, A
MATDY_Done:
            return

;-----------------------------------------------------------
; PeakVAboveThreshold
; Returns W = 1 if best vertical magnitude > threshold
;-----------------------------------------------------------
PeakVAboveThreshold:
            clrf    cmp_result, A

            movlw   PEAK_THR_H
            cpfsgt  mag_dx_h, A
            bra     PVT_HNotGreater

            movlw   0x01
            movwf   cmp_result, A
            bra     PVT_Done

PVT_HNotGreater:
            movlw   PEAK_THR_H
            cpfseq  mag_dx_h, A
            bra     PVT_Done

            movlw   PEAK_THR_L
            cpfsgt  mag_dx_l, A
            bra     PVT_Done

            movlw   0x01
            movwf   cmp_result, A

PVT_Done:
            movf    cmp_result, W, A
            return

;-----------------------------------------------------------
; PeakHAboveThreshold
; Returns W = 1 if best horizontal magnitude > threshold
;-----------------------------------------------------------
PeakHAboveThreshold:
            clrf    cmp_result, A

            movlw   PEAK_THR_H
            cpfsgt  mag_dy_h, A
            bra     PHT_HNotGreater

            movlw   0x01
            movwf   cmp_result, A
            bra     PHT_Done

PHT_HNotGreater:
            movlw   PEAK_THR_H
            cpfseq  mag_dy_h, A
            bra     PHT_Done

            movlw   PEAK_THR_L
            cpfsgt  mag_dy_l, A
            bra     PHT_Done

            movlw   0x01
            movwf   cmp_result, A

PHT_Done:
            movf    cmp_result, W, A
            return

;-----------------------------------------------------------
; PeakVDominates
; Returns W = 1 if vertical > horizontal + (horizontal >> 2)
;-----------------------------------------------------------
PeakVDominates:
            ; tmp_dx = horizontal >> 2
            movff   mag_dy_l, tmp_dx_l
            movff   mag_dy_h, tmp_dx_h
            bcf     STATUS, 0, A
            rrcf    tmp_dx_h, F, A
            rrcf    tmp_dx_l, F, A
            bcf     STATUS, 0, A
            rrcf    tmp_dx_h, F, A
            rrcf    tmp_dx_l, F, A

            ; tmp_dy = horizontal + (horizontal >> 2)
            movff   mag_dy_l, tmp_dy_l
            movff   mag_dy_h, tmp_dy_h
            movf    tmp_dx_l, W, A
            addwf   tmp_dy_l, F, A
            movf    tmp_dx_h, W, A
            addwfc  tmp_dy_h, F, A

            ; Return 1 only if vertical is strictly greater.
            clrf    cmp_result, A

            movf    tmp_dy_h, W, A
            cpfsgt  mag_dx_h, A
            bra     PVD_HNotGreater

            movlw   0x01
            movwf   cmp_result, A
            bra     PVD_Done

PVD_HNotGreater:
            movf    tmp_dy_h, W, A
            cpfslt  mag_dx_h, A
            bra     PVD_CheckLow
            bra     PVD_Done

PVD_CheckLow:
            movf    tmp_dy_l, W, A
            cpfsgt  mag_dx_l, A
            bra     PVD_Done

            movlw   0x01
            movwf   cmp_result, A

PVD_Done:
            movf    cmp_result, W, A
            return

;-----------------------------------------------------------
; PeakHDominates
; Returns W = 1 if horizontal > vertical + (vertical >> 2)
;-----------------------------------------------------------
PeakHDominates:
            ; tmp_dx = vertical >> 2
            movff   mag_dx_l, tmp_dx_l
            movff   mag_dx_h, tmp_dx_h
            bcf     STATUS, 0, A
            rrcf    tmp_dx_h, F, A
            rrcf    tmp_dx_l, F, A
            bcf     STATUS, 0, A
            rrcf    tmp_dx_h, F, A
            rrcf    tmp_dx_l, F, A

            ; tmp_dy = vertical + (vertical >> 2)
            movff   mag_dx_l, tmp_dy_l
            movff   mag_dx_h, tmp_dy_h
            movf    tmp_dx_l, W, A
            addwf   tmp_dy_l, F, A
            movf    tmp_dx_h, W, A
            addwfc  tmp_dy_h, F, A

            ; Return 1 only if horizontal is strictly greater.
            clrf    cmp_result, A

            movf    tmp_dy_h, W, A
            cpfsgt  mag_dy_h, A
            bra     PHD_HNotGreater

            movlw   0x01
            movwf   cmp_result, A
            bra     PHD_Done

PHD_HNotGreater:
            movf    tmp_dy_h, W, A
            cpfslt  mag_dy_h, A
            bra     PHD_CheckLow
            bra     PHD_Done

PHD_CheckLow:
            movf    tmp_dy_l, W, A
            cpfsgt  mag_dy_l, A
            bra     PHD_Done

            movlw   0x01
            movwf   cmp_result, A

PHD_Done:
            movf    cmp_result, W, A
            return

;-----------------------------------------------------------
; TmpDxWithinCalDelta
; Returns W = 1 if |tmp_dx| <= CAL_DDELTA
;-----------------------------------------------------------
TmpDxWithinCalDelta:
            clrf    cmp_result, A

            movlw   CAL_DDELTA_H
            cpfsgt  tmp_dx_h, A
            bra     TDX_HNotGreater
            bra     TDX_Done

TDX_HNotGreater:
            movlw   CAL_DDELTA_H
            cpfslt  tmp_dx_h, A
            bra     TDX_CheckLow

            movlw   0x01
            movwf   cmp_result, A
            bra     TDX_Done

TDX_CheckLow:
            movlw   CAL_DDELTA_L
            cpfsgt  tmp_dx_l, A
            bra     TDX_LowOK
            bra     TDX_Done

TDX_LowOK:
            movlw   0x01
            movwf   cmp_result, A

TDX_Done:
            movf    cmp_result, W, A
            return

;-----------------------------------------------------------
; TmpDyWithinCalDelta
; Returns W = 1 if |tmp_dy| <= CAL_DDELTA
;-----------------------------------------------------------
TmpDyWithinCalDelta:
            clrf    cmp_result, A

            movlw   CAL_DDELTA_H
            cpfsgt  tmp_dy_h, A
            bra     TDY_HNotGreater
            bra     TDY_Done

TDY_HNotGreater:
            movlw   CAL_DDELTA_H
            cpfslt  tmp_dy_h, A
            bra     TDY_CheckLow

            movlw   0x01
            movwf   cmp_result, A
            bra     TDY_Done

TDY_CheckLow:
            movlw   CAL_DDELTA_L
            cpfsgt  tmp_dy_l, A
            bra     TDY_LowOK
            bra     TDY_Done

TDY_LowOK:
            movlw   0x01
            movwf   cmp_result, A

TDY_Done:
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

            end
