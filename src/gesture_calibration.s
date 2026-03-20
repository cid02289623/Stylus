            PROCESSOR 18F87K22
            #include <xc.inc>

;===========================================================
; gesture_calibration.s
; Owns: CalibrationService, CalibrationCheckStill,
;       CalibrationStorePrev, FinishCalibration,
;       SendBiasDebugFrame
;
; All RAM declared in gesture_core.s - accessed via extrn.
;===========================================================

            GLOBAL  CalibrationService

            extrn   UART1_WriteByte

            extrn   tick_has_fresh
            extrn   proc_gx_l, proc_gx_h
            extrn   proc_gz_l, proc_gz_h
            extrn   calib_active, calib_count, cal_prev_valid
            extrn   bias_gx_l, bias_gx_h, bias_gz_l, bias_gz_h
            extrn   tx_cksum
            extrn   sumx_l, sumx_h, sumx_u
            extrn   sumz_l, sumz_h, sumz_u
            extrn   tmp_dx_l, tmp_dx_h, tmp_dy_l, tmp_dy_h
            extrn   shift_count, cmp_result
            extrn   cal_prev_gx_l, cal_prev_gx_h
            extrn   cal_prev_gz_l, cal_prev_gz_h

            extrn   MakeAbsTmpDx, MakeAbsTmpDy

;-----------------------------------------------------------
; Calibration constants
;-----------------------------------------------------------
CAL_SAMPLES     equ 64
BIAS_SOF        equ 0xB1
CAL_DDELTA_H    equ 0x01
CAL_DDELTA_L    equ 0x00        ; 0x0100 = 256 counts

            psect   cal_code,class=CODE,reloc=2

;-----------------------------------------------------------
; CalibrationService
; Uses only fresh parsed samples.
; Applies a stillness gate.
; Accumulates 64 still samples for Gx and Gz.
; Computes signed average bias = sum / 64.
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
; Returns W = 1 if current fresh sample is still enough.
;-----------------------------------------------------------
CalibrationCheckStill:
            clrf    cmp_result, A

            movf    cal_prev_valid, W, A
            bnz     CCS_HavePrev

            call    CalibrationStorePrev
            return

CCS_HavePrev:
            ; tmp_dy = cal_prev_gx (scratch for subtraction)
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

            ; tmp_dx = cal_prev_gz (scratch for subtraction)
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
; bias_gx = sumx / 64, bias_gz = sumz / 64 (arithmetic shift)
;-----------------------------------------------------------
FinishCalibration:
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

            clrf    calib_active, A
            clrf    calib_count, A
            clrf    cal_prev_valid, A

            call    SendBiasDebugFrame

            ; Clear RC1..RC5, preserve heartbeat on RC0 and UART pins.
            movlw   11000001B
            andwf   LATC, F, A
            return

;-----------------------------------------------------------
; SendBiasDebugFrame
; Sends: 0xB1, bias_gx_l, bias_gx_h, bias_gz_l, bias_gz_h, checksum
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
; SendByteWithChecksum (local helper)
; Input: WREG = byte to send
;-----------------------------------------------------------
SendByteWithChecksum:
            addwf   tx_cksum, F, A
            call    UART1_WriteByte
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

            end
