            PROCESSOR 18F87K22
            #include <xc.inc>

;===========================================================
; gesture_capture.s
; Owns: UpdatePeakVPos, UpdatePeakVNeg,
;       UpdatePeakHPos, UpdatePeakHNeg,
;       MakeAbsTmpDx, MakeAbsTmpDy
;
; These are the low-level peak update helpers called from
; GestureAccumulate (gesture_core.s) during capture.
; All RAM declared in gesture_core.s - accessed via extrn.
;===========================================================

            GLOBAL  UpdatePeakVPos, UpdatePeakVNeg
            GLOBAL  UpdatePeakHPos, UpdatePeakHNeg
            GLOBAL  MakeAbsTmpDx, MakeAbsTmpDy

            extrn   peak_v_pos_l, peak_v_pos_h
            extrn   peak_v_neg_l, peak_v_neg_h
            extrn   peak_h_pos_l, peak_h_pos_h
            extrn   peak_h_neg_l, peak_h_neg_h
            extrn   tmp_dx_l, tmp_dx_h, tmp_dy_l, tmp_dy_h

            psect   capture_code,class=CODE,reloc=2

;-----------------------------------------------------------
; MakeAbsTmpDx  -  tmp_dx = |tmp_dx|
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
; MakeAbsTmpDy  -  tmp_dy = |tmp_dy|
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
; UpdatePeakVPos
; Updates peak_v_pos if tmp_dx is larger (unsigned 16-bit)
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
; Updates peak_v_neg if |tmp_dx| is larger (unsigned 16-bit)
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
; Updates peak_h_pos if tmp_dy is larger (unsigned 16-bit)
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
; Updates peak_h_neg if |tmp_dy| is larger (unsigned 16-bit)
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

            end
