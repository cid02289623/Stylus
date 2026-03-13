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
; Phase 6 constants
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
; Gesture decision threshold
; Tune later from measured Dx/Dy pilot data.
;-----------------------------------------------------------
TDISP_H         equ 0x04
TDISP_L         equ 0x00

;-----------------------------------------------------------
; Startup calibration constants
; CAL_SAMPLES = 64 gives 64 fresh still samples, which is
; about 1.28 s at 50 Hz.
;-----------------------------------------------------------
CAL_ACTIVE      equ 0x01
CAL_DONE        equ 0x00
CAL_SAMPLES     equ 64

;-----------------------------------------------------------
; Stillness gate during calibration
; This threshold applies to the absolute sample-to-sample
; change |current - previous| on each axis.
; Tune if startup calibration is too strict or too loose.
;-----------------------------------------------------------
CAL_DDELTA_H    equ 0x01
CAL_DDELTA_L    equ 0x00        ; 0x0100 = 256 counts

;-----------------------------------------------------------
; Debug telemetry frame start byte
; Output format after each result:
;   raw result byte
;   0xF1, result, DxL, DxH, DyL, DyH, checksum
; checksum = sum(result + DxL + DxH + DyL + DyH) mod 256
;-----------------------------------------------------------
TELEM_SOF       equ 0xF1

;-----------------------------------------------------------
; Gesture RAM
;-----------------------------------------------------------
            psect   udata_acs

btn_now:                ds 1          ; Current button-down flag (0/1)
prev_btn_down:          ds 1          ; Previous button-down flag (0/1)
gesture_state:          ds 1          ; IDLE or CAPTURE

acc_dx_l:               ds 1          ; Signed accumulated Dx low byte
acc_dx_h:               ds 1          ; Signed accumulated Dx high byte
acc_dy_l:               ds 1          ; Signed accumulated Dy low byte
acc_dy_h:               ds 1          ; Signed accumulated Dy high byte

mag_dx_l:               ds 1          ; |Dx| low byte
mag_dx_h:               ds 1          ; |Dx| high byte
mag_dy_l:               ds 1          ; |Dy| low byte
mag_dy_h:               ds 1          ; |Dy| high byte

gesture_result:         ds 1          ; Final result code
cmp_result:             ds 1          ; General-purpose boolean result

calib_active:           ds 1          ; 1 while startup calibration runs
calib_count:            ds 1          ; Number of accepted still samples
cal_prev_valid:         ds 1          ; 1 after first calibration sample saved

bias_gx_l:              ds 1          ; Learned Gx bias low byte
bias_gx_h:              ds 1          ; Learned Gx bias high byte
bias_gy_l:              ds 1          ; Learned Gy bias low byte
bias_gy_h:              ds 1          ; Learned Gy bias high byte

sumx_l:                 ds 1          ; 24-bit signed sum of Gx samples low
sumx_h:                 ds 1          ; 24-bit signed sum of Gx samples mid
sumx_u:                 ds 1          ; 24-bit signed sum of Gx samples high

sumy_l:                 ds 1          ; 24-bit signed sum of Gy samples low
sumy_h:                 ds 1          ; 24-bit signed sum of Gy samples mid
sumy_u:                 ds 1          ; 24-bit signed sum of Gy samples high

tmp_dx_l:               ds 1          ; Temporary 16-bit low byte
tmp_dx_h:               ds 1          ; Temporary 16-bit high byte
tmp_dy_l:               ds 1          ; Temporary 16-bit low byte
tmp_dy_h:               ds 1          ; Temporary 16-bit high byte

shift_count:            ds 1          ; Loop counter for /64 right shifts

cal_prev_gx_l:          ds 1          ; Previous fresh calibration Gx low
cal_prev_gx_h:          ds 1          ; Previous fresh calibration Gx high
cal_prev_gy_l:          ds 1          ; Previous fresh calibration Gy low
cal_prev_gy_h:          ds 1          ; Previous fresh calibration Gy high

telem_cksum:            ds 1          ; Telemetry checksum accumulator

;-----------------------------------------------------------
; Code
;-----------------------------------------------------------
            psect   gesture_code,class=CODE,reloc=2

;-----------------------------------------------------------
; GestureInit
; - clears gesture state
; - starts startup calibration
; - lights RC5 during calibration
;-----------------------------------------------------------
GestureInit:
            ; Clear edge-detection and state variables.
            clrf    btn_now, A
            clrf    prev_btn_down, A
            clrf    gesture_state, A

            ; Clear signed accumulators.
            clrf    acc_dx_l, A
            clrf    acc_dx_h, A
            clrf    acc_dy_l, A
            clrf    acc_dy_h, A

            ; Clear magnitude work registers.
            clrf    mag_dx_l, A
            clrf    mag_dx_h, A
            clrf    mag_dy_l, A
            clrf    mag_dy_h, A

            ; Default result is UNKNOWN.
            clrf    cmp_result, A
            movlw   RES_UNK
            movwf   gesture_result, A

            ; Start calibration.
            movlw   CAL_ACTIVE
            movwf   calib_active, A
            clrf    calib_count, A
            clrf    cal_prev_valid, A

            ; Clear learned biases.
            clrf    bias_gx_l, A
            clrf    bias_gx_h, A
            clrf    bias_gy_l, A
            clrf    bias_gy_h, A

            ; Clear running sums.
            clrf    sumx_l, A
            clrf    sumx_h, A
            clrf    sumx_u, A
            clrf    sumy_l, A
            clrf    sumy_h, A
            clrf    sumy_u, A

            ; Clear temporaries.
            clrf    tmp_dx_l, A
            clrf    tmp_dx_h, A
            clrf    tmp_dy_l, A
            clrf    tmp_dy_h, A
            clrf    shift_count, A
            clrf    telem_cksum, A

            ; Clear saved previous calibration sample.
            clrf    cal_prev_gx_l, A
            clrf    cal_prev_gx_h, A
            clrf    cal_prev_gy_l, A
            clrf    cal_prev_gy_h, A

            ; Indicate startup calibration with RC5 on.
            bsf     LATC, 5, A
            return

;-----------------------------------------------------------
; ProcessTickService
; Called once per 20 ms processing slot.
; - toggles RC0 heartbeat
; - snapshots latest valid sample into proc_*
; - while calibration is active, services calibration only
; - after calibration, runs the gesture state machine
;-----------------------------------------------------------
ProcessTickService:
            ; Toggle RC0 every 25 ticks.
            ; 25 * 20 ms = 500 ms, so full blink cycle = 1 s.
            incf    debug_tickdiv, F, A
            movlw   25
            cpfseq  debug_tickdiv, A
            bra     PTS_NoToggle

            clrf    debug_tickdiv, A
            btg     LATC, 0, A

PTS_NoToggle:
            ; If no valid frame has ever been parsed, nothing to do.
            movf    sample_valid, W, A
            bz      PTS_Done

            ; Snapshot latest valid sample into proc_*.
            ; This decouples processing from UART arrival jitter.
            movff   latest_gx_l, proc_gx_l
            movff   latest_gx_h, proc_gx_h
            movff   latest_gy_l, proc_gy_l
            movff   latest_gy_h, proc_gy_h
            movff   latest_gz_l, proc_gz_l
            movff   latest_gz_h, proc_gz_h
            movff   latest_btn,  proc_btn

            ; If startup calibration is still active, run that first.
            movf    calib_active, W, A
            bz      PTS_RunGesture

            call    CalibrationService
            clrf    new_sample, A
            bra     PTS_Done

PTS_RunGesture:
            ; This tick has now consumed the "fresh sample" condition.
            clrf    new_sample, A

            ; Run the gesture engine.
            call    GestureService

PTS_Done:
            return

;-----------------------------------------------------------
; CalibrationService
; - uses only fresh parsed samples
; - applies a sample-to-sample stillness gate
; - accumulates 64 still samples for Gx and Gy
; - computes signed average bias = sum / 64
;-----------------------------------------------------------
CalibrationService:
            ; Ignore reused samples. Only fresh parser commits count.
            movf    new_sample, W, A
            bz      CAL_Done

            ; Require startup stillness before accepting the sample.
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

            ; sumy += sign-extended proc_gy
            movf    proc_gy_l, W, A
            addwf   sumy_l, F, A
            movf    proc_gy_h, W, A
            addwfc  sumy_h, F, A
            movlw   0x00
            btfsc   proc_gy_h, 7, A
            movlw   0xFF
            addwfc  sumy_u, F, A

            ; Count this accepted still sample.
            incf    calib_count, F, A

            ; If not yet at CAL_SAMPLES, continue calibrating.
            movlw   CAL_SAMPLES
            cpfseq  calib_count, A
            bra     CAL_Done

            ; Reached target count -> compute average biases.
            call    FinishCalibration

CAL_Done:
            return

;-----------------------------------------------------------
; CalibrationCheckStill
; Returns W = 1 if the current fresh sample is considered still.
; Strategy:
; - first fresh sample seeds the previous-sample register and is not used
; - afterwards, if both |Gx(n)-Gx(n-1)| and |Gy(n)-Gy(n-1)| are
;   <= CAL_DDELTA, the sample is accepted as still
; - the previous sample is always updated at the end
;-----------------------------------------------------------
CalibrationCheckStill:
            ; Default return = 0 (not accepted yet).
            clrf    cmp_result, A

            ; If no previous fresh sample exists, seed it and return 0.
            movf    cal_prev_valid, W, A
            bnz     CCS_HavePrev

            call    CalibrationStorePrev
            return

CCS_HavePrev:
            ; tmp_dx = proc_gx - cal_prev_gx
            movf    cal_prev_gx_l, W, A
            subwf   proc_gx_l, W, A
            movwf   tmp_dx_l, A
            movf    cal_prev_gx_h, W, A
            subwfb  proc_gx_h, W, A
            movwf   tmp_dx_h, A

            ; tmp_dx = |tmp_dx|
            call    MakeAbsTmpDx

            ; If delta-x is too large, reject this sample.
            call    TmpDxWithinCalDelta
            bz      CCS_UpdatePrev

            ; tmp_dy = proc_gy - cal_prev_gy
            movf    cal_prev_gy_l, W, A
            subwf   proc_gy_l, W, A
            movwf   tmp_dy_l, A
            movf    cal_prev_gy_h, W, A
            subwfb  proc_gy_h, W, A
            movwf   tmp_dy_h, A

            ; tmp_dy = |tmp_dy|
            call    MakeAbsTmpDy

            ; If delta-y is too large, reject this sample.
            call    TmpDyWithinCalDelta
            bz      CCS_UpdatePrev

            ; Both deltas are inside threshold -> accept sample.
            movlw   0x01
            movwf   cmp_result, A

CCS_UpdatePrev:
            ; Always update the previous-sample register.
            call    CalibrationStorePrev

            ; Return W = 1 if still, else 0.
            movf    cmp_result, W, A
            return

;-----------------------------------------------------------
; CalibrationStorePrev
; Saves current proc_gx / proc_gy as the previous calibration sample.
;-----------------------------------------------------------
CalibrationStorePrev:
            movff   proc_gx_l, cal_prev_gx_l
            movff   proc_gx_h, cal_prev_gx_h
            movff   proc_gy_l, cal_prev_gy_l
            movff   proc_gy_h, cal_prev_gy_h
            movlw   0x01
            movwf   cal_prev_valid, A
            return

;-----------------------------------------------------------
; FinishCalibration
; - divides signed 24-bit sums by 64 using arithmetic right shifts
; - stores low 16 bits as bias_gx and bias_gy
; - ends calibration and clears result LEDs RC1..RC5
;-----------------------------------------------------------
FinishCalibration:
            ; Compute bias_gx = sumx / 64 using 6 arithmetic right shifts.
            movlw   6
            movwf   shift_count, A

FC_ShiftX:
            bcf     STATUS, 0, A      ; Clear C before injecting sign bit
            btfsc   sumx_u, 7, A      ; If sign bit = 1,
            bsf     STATUS, 0, A      ; preload C = 1 for arithmetic shift
            rrcf    sumx_u, F, A
            rrcf    sumx_h, F, A
            rrcf    sumx_l, F, A
            decfsz  shift_count, F, A
            bra     FC_ShiftX

            movff   sumx_l, bias_gx_l
            movff   sumx_h, bias_gx_h

            ; Compute bias_gy = sumy / 64 using 6 arithmetic right shifts.
            movlw   6
            movwf   shift_count, A

FC_ShiftY:
            bcf     STATUS, 0, A
            btfsc   sumy_u, 7, A
            bsf     STATUS, 0, A
            rrcf    sumy_u, F, A
            rrcf    sumy_h, F, A
            rrcf    sumy_l, F, A
            decfsz  shift_count, F, A
            bra     FC_ShiftY

            movff   sumy_l, bias_gy_l
            movff   sumy_h, bias_gy_h

            ; Calibration is complete.
            clrf    calib_active, A
            clrf    calib_count, A
            clrf    cal_prev_valid, A

            ; Clear result LEDs RC1..RC5, preserve heartbeat RC0 and UART pins.
            movlw   11000001B
            andwf   LATC, F, A
            return

;-----------------------------------------------------------
; GestureService
; - detects button edges
; - captures while button held
; - classifies on release
;-----------------------------------------------------------
GestureService:
            ; btn_now = 1 if proc_btn bit is high, else 0.
            clrf    btn_now, A
            btfsc   proc_btn, BTN_BIT, A
            incf    btn_now, F, A

            ; Branch by gesture state.
            movf    gesture_state, W, A
            bz      GS_Idle
            bra     GS_Capture

GS_Idle:
            ; Wait for rising edge: prev = 0, now = 1.
            movf    btn_now, W, A
            bz      GS_UpdatePrev

            movf    prev_btn_down, W, A
            bnz     GS_UpdatePrev

            ; Rising edge -> start a new capture.
            call    GestureStartCapture
            bra     GS_UpdatePrev

GS_Capture:
            ; While the button is held, accumulate every 20 ms tick.
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
            ; Save current button state for next edge test.
            movf    btn_now, W, A
            movwf   prev_btn_down, A
            return

;-----------------------------------------------------------
; GestureStartCapture
; - clears accumulators
; - clears result LEDs
; - enters capture state
; - accumulates the current pressed sample immediately
;-----------------------------------------------------------
GestureStartCapture:
            ; Clear signed motion accumulators.
            clrf    acc_dx_l, A
            clrf    acc_dx_h, A
            clrf    acc_dy_l, A
            clrf    acc_dy_h, A

            ; Clear result LEDs RC1..RC5, preserve RC0/RC6/RC7.
            movlw   11000001B
            andwf   LATC, F, A

            ; Enter capture state.
            movlw   G_CAPTURE
            movwf   gesture_state, A

            ; Count the first pressed sample immediately.
            call    GestureAccumulate
            return

;-----------------------------------------------------------
; GestureAccumulate
; acc_dx += (proc_gx - bias_gx)
; acc_dy += (proc_gy - bias_gy)
;-----------------------------------------------------------
GestureAccumulate:
            ; tmp_dx = proc_gx - bias_gx
            movf    bias_gx_l, W, A
            subwf   proc_gx_l, W, A
            movwf   tmp_dx_l, A
            movf    bias_gx_h, W, A
            subwfb  proc_gx_h, W, A
            movwf   tmp_dx_h, A

            ; acc_dx += tmp_dx
            movf    tmp_dx_l, W, A
            addwf   acc_dx_l, F, A
            movf    tmp_dx_h, W, A
            addwfc  acc_dx_h, F, A

            ; tmp_dy = proc_gy - bias_gy
            movf    bias_gy_l, W, A
            subwf   proc_gy_l, W, A
            movwf   tmp_dy_l, A
            movf    bias_gy_h, W, A
            subwfb  proc_gy_h, W, A
            movwf   tmp_dy_h, A

            ; acc_dy += tmp_dy
            movf    tmp_dy_l, W, A
            addwf   acc_dy_l, F, A
            movf    tmp_dy_h, W, A
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
            ; Compute |Dx| and |Dy|.
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
            ; Require |Dx| to exceed threshold.
            call    MagDxAboveThreshold
            bz      GCR_Unknown

            ; Negative Dx = LEFT, positive Dx = RIGHT.
            btfsc   acc_dx_h, 7, A
            bra     GCR_Left
            bra     GCR_Right

GCR_DomY:
            ; Require |Dy| to exceed threshold.
            call    MagDyAboveThreshold
            bz      GCR_Unknown

            ; Negative Dy = DOWN, positive Dy = UP.
            btfsc   acc_dy_h, 7, A
            bra     GCR_Down
            bra     GCR_Up

GCR_Tie:
            ; Equal magnitudes are treated as UNKNOWN.
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
            ; Show result locally on LEDs.
            call    ShowGestureOnLEDs

            ; Send the original one-byte result for normal logging.
            call    SendGestureResultByte

            ; Also send a structured telemetry frame carrying Dx and Dy
            ; to help threshold tuning in Phase 6 pilot tests.
            call    SendGestureDebugFrame
            return

;-----------------------------------------------------------
; MakeAbsDx
; mag_dx = |acc_dx|
;-----------------------------------------------------------
MakeAbsDx:
            movff   acc_dx_l, mag_dx_l
            movff   acc_dx_h, mag_dx_h

            ; If sign bit is clear, value is already non-negative.
            btfss   mag_dx_h, 7, A
            return

            ; Two's-complement negate to form absolute value.
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

            ; If sign bit is clear, value is already non-negative.
            btfss   mag_dy_h, 7, A
            return

            ; Two's-complement negate to form absolute value.
            comf    mag_dy_l, F, A
            comf    mag_dy_h, F, A
            incf    mag_dy_l, F, A
            bnz     MADY_Done
            incf    mag_dy_h, F, A
MADY_Done:
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
; MagDxAboveThreshold
; Returns W = 1 if |Dx| > threshold, else W = 0
;-----------------------------------------------------------
MagDxAboveThreshold:
            clrf    cmp_result, A

            ; If high byte > threshold high, definitely above.
            movlw   TDISP_H
            cpfsgt  mag_dx_h, A
            bra     MDTX_HNotGreater

            movlw   0x01
            movwf   cmp_result, A
            bra     MDTX_Done

MDTX_HNotGreater:
            ; If high byte != threshold high and not greater, then below.
            movlw   TDISP_H
            cpfseq  mag_dx_h, A
            bra     MDTX_Done

            ; High bytes are equal, so compare low byte.
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
; Returns W = 1 if |Dy| > threshold, else W = 0
;-----------------------------------------------------------
MagDyAboveThreshold:
            clrf    cmp_result, A

            ; If high byte > threshold high, definitely above.
            movlw   TDISP_H
            cpfsgt  mag_dy_h, A
            bra     MDTY_HNotGreater

            movlw   0x01
            movwf   cmp_result, A
            bra     MDTY_Done

MDTY_HNotGreater:
            ; If high byte != threshold high and not greater, then below.
            movlw   TDISP_H
            cpfseq  mag_dy_h, A
            bra     MDTY_Done

            ; High bytes are equal, so compare low byte.
            movlw   TDISP_L
            cpfsgt  mag_dy_l, A
            bra     MDTY_Done

            movlw   0x01
            movwf   cmp_result, A

MDTY_Done:
            movf    cmp_result, W, A
            return

;-----------------------------------------------------------
; TmpDxWithinCalDelta
; Returns W = 1 if |tmp_dx| <= CAL_DDELTA, else W = 0
;-----------------------------------------------------------
TmpDxWithinCalDelta:
            clrf    cmp_result, A

            ; If high byte > threshold high, fail.
            movlw   CAL_DDELTA_H
            cpfsgt  tmp_dx_h, A
            bra     TDX_HNotGreater
            bra     TDX_Done

TDX_HNotGreater:
            ; If high byte < threshold high, pass immediately.
            movlw   CAL_DDELTA_H
            cpfslt  tmp_dx_h, A
            bra     TDX_CheckLow

            movlw   0x01
            movwf   cmp_result, A
            bra     TDX_Done

TDX_CheckLow:
            ; High bytes equal, so low byte must be <= threshold low.
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
; Returns W = 1 if |tmp_dy| <= CAL_DDELTA, else W = 0
;-----------------------------------------------------------
TmpDyWithinCalDelta:
            clrf    cmp_result, A

            ; If high byte > threshold high, fail.
            movlw   CAL_DDELTA_H
            cpfsgt  tmp_dy_h, A
            bra     TDY_HNotGreater
            bra     TDY_Done

TDY_HNotGreater:
            ; If high byte < threshold high, pass immediately.
            movlw   CAL_DDELTA_H
            cpfslt  tmp_dy_h, A
            bra     TDY_CheckLow

            movlw   0x01
            movwf   cmp_result, A
            bra     TDY_Done

TDY_CheckLow:
            ; High bytes equal, so low byte must be <= threshold low.
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

            ; LEFT
            movf    gesture_result, W, A
            xorlw   RES_LEFT
            bnz     SG_NotLeft
            bsf     LATC, 1, A
            return

SG_NotLeft:
            ; RIGHT
            movf    gesture_result, W, A
            xorlw   RES_RIGHT
            bnz     SG_NotRight
            bsf     LATC, 2, A
            return

SG_NotRight:
            ; UP
            movf    gesture_result, W, A
            xorlw   RES_UP
            bnz     SG_NotUp
            bsf     LATC, 3, A
            return

SG_NotUp:
            ; DOWN
            movf    gesture_result, W, A
            xorlw   RES_DOWN
            bnz     SG_NotDown
            bsf     LATC, 4, A
            return

SG_NotDown:
            ; Otherwise show UNKNOWN on RC5.
            bsf     LATC, 5, A
            return

;-----------------------------------------------------------
; SendGestureResultByte
; Sends one UART byte for the normal logging path.
;-----------------------------------------------------------
SendGestureResultByte:
            movf    gesture_result, W, A
            call    UART1_WriteByte
            return

;-----------------------------------------------------------
; SendGestureDebugFrame
; Sends:
;   0xF1, result, DxL, DxH, DyL, DyH, checksum
; where checksum = sum(result + DxL + DxH + DyL + DyH) mod 256
;-----------------------------------------------------------
SendGestureDebugFrame:
            ; Send telemetry start byte.
            movlw   TELEM_SOF
            call    UART1_WriteByte

            ; Clear checksum accumulator.
            clrf    telem_cksum, A

            ; Send result and accumulate checksum.
            movf    gesture_result, W, A
            addwf   telem_cksum, F, A
            call    UART1_WriteByte

            ; Send final Dx low byte.
            movf    acc_dx_l, W, A
            addwf   telem_cksum, F, A
            call    UART1_WriteByte

            ; Send final Dx high byte.
            movf    acc_dx_h, W, A
            addwf   telem_cksum, F, A
            call    UART1_WriteByte

            ; Send final Dy low byte.
            movf    acc_dy_l, W, A
            addwf   telem_cksum, F, A
            call    UART1_WriteByte

            ; Send final Dy high byte.
            movf    acc_dy_h, W, A
            addwf   telem_cksum, F, A
            call    UART1_WriteByte

            ; Send checksum byte.
            movf    telem_cksum, W, A
            call    UART1_WriteByte
            return

            end
