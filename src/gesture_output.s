            PROCESSOR 18F87K22
            #include <xc.inc>

;===========================================================
; gesture_output.s
; Owns: ShowGestureOnLEDs, SendClassifierDebugFrame,
;       SendByteWithChecksum
;
; All RAM declared in gesture_core.s - accessed via extrn.
; UART1_WriteByte declared in uart1.s - accessed via extrn.
;===========================================================

            GLOBAL  ShowGestureOnLEDs
            GLOBAL  SendClassifierDebugFrame

            extrn   UART1_WriteByte

            extrn   gesture_result, dbg_flags, tx_cksum
            extrn   proc_gx_l, proc_gx_h
            extrn   proc_gy_l, proc_gy_h
            extrn   proc_gz_l, proc_gz_h
            extrn   proc_btn
            extrn   bias_gx_l, bias_gx_h, bias_gz_l, bias_gz_h
            extrn   acc_v_b0, acc_v_b1, acc_v_b2, acc_v_b3
            extrn   acc_h_b0, acc_h_b1, acc_h_b2, acc_h_b3
            extrn   peak_v_pos_l, peak_v_pos_h
            extrn   peak_v_neg_l, peak_v_neg_h
            extrn   peak_h_pos_l, peak_h_pos_h
            extrn   peak_h_neg_l, peak_h_neg_h

;-----------------------------------------------------------
; Result constants
;-----------------------------------------------------------
RES_LEFT        equ 0x4C
RES_RIGHT       equ 0x52
RES_UP          equ 0x55
RES_DOWN        equ 0x44
DBG_SOF         equ 0xE2

            psect   output_code,class=CODE,reloc=2

;-----------------------------------------------------------
; ShowGestureOnLEDs
; RC1=LEFT RC2=RIGHT RC3=UP RC4=DOWN RC5=UNKNOWN RC0=heartbeat
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
; SendClassifierDebugFrame
; Sends 0xE2 + all classification data + checksum.
;-----------------------------------------------------------
SendClassifierDebugFrame:
            movlw   DBG_SOF
            call    UART1_WriteByte

            clrf    tx_cksum, A

            movf    proc_gx_l, W, A
            call    SendByteWithChecksum
            movf    proc_gx_h, W, A
            call    SendByteWithChecksum

            movf    proc_gy_l, W, A
            call    SendByteWithChecksum
            movf    proc_gy_h, W, A
            call    SendByteWithChecksum

            movf    proc_gz_l, W, A
            call    SendByteWithChecksum
            movf    proc_gz_h, W, A
            call    SendByteWithChecksum

            movf    proc_btn, W, A
            call    SendByteWithChecksum

            movf    bias_gx_l, W, A
            call    SendByteWithChecksum
            movf    bias_gx_h, W, A
            call    SendByteWithChecksum

            movf    bias_gz_l, W, A
            call    SendByteWithChecksum
            movf    bias_gz_h, W, A
            call    SendByteWithChecksum

            movf    acc_v_b0, W, A
            call    SendByteWithChecksum
            movf    acc_v_b1, W, A
            call    SendByteWithChecksum
            movf    acc_v_b2, W, A
            call    SendByteWithChecksum
            movf    acc_v_b3, W, A
            call    SendByteWithChecksum

            movf    acc_h_b0, W, A
            call    SendByteWithChecksum
            movf    acc_h_b1, W, A
            call    SendByteWithChecksum
            movf    acc_h_b2, W, A
            call    SendByteWithChecksum
            movf    acc_h_b3, W, A
            call    SendByteWithChecksum

            movf    peak_v_pos_l, W, A
            call    SendByteWithChecksum
            movf    peak_v_pos_h, W, A
            call    SendByteWithChecksum

            movf    peak_v_neg_l, W, A
            call    SendByteWithChecksum
            movf    peak_v_neg_h, W, A
            call    SendByteWithChecksum

            movf    peak_h_pos_l, W, A
            call    SendByteWithChecksum
            movf    peak_h_pos_h, W, A
            call    SendByteWithChecksum

            movf    peak_h_neg_l, W, A
            call    SendByteWithChecksum
            movf    peak_h_neg_h, W, A
            call    SendByteWithChecksum

            movf    dbg_flags, W, A
            call    SendByteWithChecksum

            movf    gesture_result, W, A
            call    SendByteWithChecksum

            movf    tx_cksum, W, A
            call    UART1_WriteByte
            return

;-----------------------------------------------------------
; SendByteWithChecksum
; Input: WREG = byte to send; accumulates into tx_cksum.
;-----------------------------------------------------------
SendByteWithChecksum:
            addwf   tx_cksum, F, A
            call    UART1_WriteByte
            return

            end
