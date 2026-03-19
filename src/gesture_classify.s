            PROCESSOR 18F87K22
            #include <xc.inc>

;===========================================================
; gesture_classify.s
; Owns: GestureClassifyAndReport, SelectVerticalBest,
;       SelectHorizontalBest, BuildDebugFlags,
;       PeakVAboveThreshold, PeakHAboveThreshold,
;       PeakVDominates, PeakHDominates
;
; All RAM declared in gesture_core.s - accessed via extrn.
;===========================================================

            GLOBAL  GestureClassifyAndReport

            extrn   ShowGestureOnLEDs
            extrn   SendClassifierDebugFrame

            extrn   mag_dx_l, mag_dx_h, mag_dy_l, mag_dy_h
            extrn   peak_v_pos_l, peak_v_pos_h
            extrn   peak_v_neg_l, peak_v_neg_h
            extrn   peak_h_pos_l, peak_h_pos_h
            extrn   peak_h_neg_l, peak_h_neg_h
            extrn   gesture_result, cmp_result, dbg_flags, btn_now
            extrn   tmp_dx_l, tmp_dx_h, tmp_dy_l, tmp_dy_h

;-----------------------------------------------------------
; Gesture result constants
;-----------------------------------------------------------
RES_LEFT        equ 0x4C
RES_RIGHT       equ 0x52
RES_UP          equ 0x55
RES_DOWN        equ 0x44
RES_UNK         equ 0x3F

PEAK_THR_H      equ 0x08
PEAK_THR_L      equ 0x00        ; 0x0800 = 2048 counts

            psect   classify_code,class=CODE,reloc=2

;-----------------------------------------------------------
; GestureClassifyAndReport
; Mapping: +Gx=>UP, -Gx=>DOWN, +Gz=>LEFT, -Gz=>RIGHT
; Uses peaks not final sums for robust classification.
;-----------------------------------------------------------
GestureClassifyAndReport:
            call    SelectVerticalBest
            call    SelectHorizontalBest
            call    BuildDebugFlags

            ; Compare best vertical against best horizontal.
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

            ; Negative peak larger -> DOWN, else UP.
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

            ; Negative peak larger -> RIGHT, else LEFT.
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
; bit0=btn_now, bit1=V>thr, bit2=H>thr,
; bit3=V dominates, bit4=H dominates
; (bits 5 and 6 are set by GCR_DomV / GCR_DomH above)
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
            movff   mag_dy_l, tmp_dx_l
            movff   mag_dy_h, tmp_dx_h
            bcf     STATUS, 0, A
            rrcf    tmp_dx_h, F, A
            rrcf    tmp_dx_l, F, A
            bcf     STATUS, 0, A
            rrcf    tmp_dx_h, F, A
            rrcf    tmp_dx_l, F, A

            movff   mag_dy_l, tmp_dy_l
            movff   mag_dy_h, tmp_dy_h
            movf    tmp_dx_l, W, A
            addwf   tmp_dy_l, F, A
            movf    tmp_dx_h, W, A
            addwfc  tmp_dy_h, F, A

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
            movff   mag_dx_l, tmp_dx_l
            movff   mag_dx_h, tmp_dx_h
            bcf     STATUS, 0, A
            rrcf    tmp_dx_h, F, A
            rrcf    tmp_dx_l, F, A
            bcf     STATUS, 0, A
            rrcf    tmp_dx_h, F, A
            rrcf    tmp_dx_l, F, A

            movff   mag_dx_l, tmp_dy_l
            movff   mag_dx_h, tmp_dy_h
            movf    tmp_dx_l, W, A
            addwf   tmp_dy_l, F, A
            movf    tmp_dx_h, W, A
            addwfc  tmp_dy_h, F, A

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

            end
