PROCESSOR 18F87K22
            #include <xc.inc>

            GLOBAL  ParserService

            GLOBAL  rx_head, rx_tail, rx_buf, last_byte
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

;===========================================================
; parser.s
; - Consumes bytes from the UART RX ring buffer
; - Searches for 0xAA start byte
; - Collects 8 remaining bytes
; - Validates checksum over bytes 1..7
; - Commits latest valid sample only after checksum passes
;===========================================================

P_SEARCH    equ 0x00
P_GX_L      equ 0x01
P_GX_H      equ 0x02
P_GY_L      equ 0x03
P_GY_H      equ 0x04
P_GZ_L      equ 0x05
P_GZ_H      equ 0x06
P_BTN       equ 0x07
P_CHK       equ 0x08

            psect   parser_code,class=CODE,reloc=2

;-----------------------------------------------------------
; ParserService
; Packet format:
;   byte 0 = 0xAA
;   byte 1 = Gx low
;   byte 2 = Gx high
;   byte 3 = Gy low
;   byte 4 = Gy high
;   byte 5 = Gz low
;   byte 6 = Gz high
;   byte 7 = button
;   byte 8 = checksum = sum(bytes 1..7) mod 256
;-----------------------------------------------------------
ParserService:
PS_Loop:
            ; If rx_head == rx_tail, ring buffer is empty.
            movf    rx_head, W, A
            cpfseq  rx_tail, A
            bra     PS_HaveByte
            return

PS_HaveByte:
            ; Point FSR0 at rx_buf[rx_tail].
            lfsr    0, rx_buf
            movf    rx_tail, W, A
            addwf   FSR0L, F, A
            btfsc   STATUS, 0, A
            incf    FSR0H, F, A

            ; Read one byte from the ring buffer.
            movf    INDF0, W, A
            movwf   last_byte, A

            ; Advance tail: (tail + 1) & 0x1F.
            incf    rx_tail, F, A
            movlw   0x1F
            andwf   rx_tail, F, A

            ; Dispatch based on current parser state.
            movf    parser_state, W, A
            bz      PS_Search

            movlw   P_GX_L
            cpfseq  parser_state, A
            bra     PS_Check2
            bra     PS_GxL

PS_Check2:
            movlw   P_GX_H
            cpfseq  parser_state, A
            bra     PS_Check3
            bra     PS_GxH

PS_Check3:
            movlw   P_GY_L
            cpfseq  parser_state, A
            bra     PS_Check4
            bra     PS_GyL

PS_Check4:
            movlw   P_GY_H
            cpfseq  parser_state, A
            bra     PS_Check5
            bra     PS_GyH

PS_Check5:
            movlw   P_GZ_L
            cpfseq  parser_state, A
            bra     PS_Check6
            bra     PS_GzL

PS_Check6:
            movlw   P_GZ_H
            cpfseq  parser_state, A
            bra     PS_Check7
            bra     PS_GzH

PS_Check7:
            movlw   P_BTN
            cpfseq  parser_state, A
            bra     PS_Check8
            bra     PS_Btn

PS_Check8:
            ; Any remaining non-zero state here is checksum state.
            bra     PS_Chk

;-----------------------------------------------------------
; State 0: wait for start-of-frame 0xAA
;-----------------------------------------------------------
PS_Search:
            ; Ignore bytes until 0xAA appears.
            movf    last_byte, W, A
            xorlw   0xAA
            bnz     PS_Loop

            ; Start of frame found, so clear checksum and move to Gx low.
            clrf    parser_checksum, A
            movlw   P_GX_L
            movwf   parser_state, A
            bra     PS_Loop

;-----------------------------------------------------------
; States 1..7: collect payload bytes and accumulate checksum
;-----------------------------------------------------------
PS_GxL:
            movff   last_byte, stg_gx_l
            movf    last_byte, W, A
            addwf   parser_checksum, F, A
            movlw   P_GX_H
            movwf   parser_state, A
            bra     PS_Loop

PS_GxH:
            movff   last_byte, stg_gx_h
            movf    last_byte, W, A
            addwf   parser_checksum, F, A
            movlw   P_GY_L
            movwf   parser_state, A
            bra     PS_Loop

PS_GyL:
            movff   last_byte, stg_gy_l
            movf    last_byte, W, A
            addwf   parser_checksum, F, A
            movlw   P_GY_H
            movwf   parser_state, A
            bra     PS_Loop

PS_GyH:
            movff   last_byte, stg_gy_h
            movf    last_byte, W, A
            addwf   parser_checksum, F, A
            movlw   P_GZ_L
            movwf   parser_state, A
            bra     PS_Loop

PS_GzL:
            movff   last_byte, stg_gz_l
            movf    last_byte, W, A
            addwf   parser_checksum, F, A
            movlw   P_GZ_H
            movwf   parser_state, A
            bra     PS_Loop

PS_GzH:
            movff   last_byte, stg_gz_h
            movf    last_byte, W, A
            addwf   parser_checksum, F, A
            movlw   P_BTN
            movwf   parser_state, A
            bra     PS_Loop

PS_Btn:
            movff   last_byte, stg_btn
            movf    last_byte, W, A
            addwf   parser_checksum, F, A
            movlw   P_CHK
            movwf   parser_state, A
            bra     PS_Loop

;-----------------------------------------------------------
; State 8: compare received checksum with computed checksum
;-----------------------------------------------------------
PS_Chk:
            movf    last_byte, W, A
            xorwf   parser_checksum, W, A
            bnz     PS_BadChecksum

            ; Publish staged frame as latest valid data.
            movff   stg_gx_l, latest_gx_l
            movff   stg_gx_h, latest_gx_h
            movff   stg_gy_l, latest_gy_l
            movff   stg_gy_h, latest_gy_h
            movff   stg_gz_l, latest_gz_l
            movff   stg_gz_h, latest_gz_h
            movff   stg_btn,  latest_btn

            ; Set "sample valid" and "new sample" flags.
            movlw   0x01
            movwf   sample_valid, A
            movwf   new_sample, A

            ; Increment 16-bit good-frame counter.
            incf    frame_ok_count_l, F, A
            bnz     PS_GoodDone
            incf    frame_ok_count_h, F, A

PS_GoodDone:
            ; Reset parser and continue scanning.
            clrf    parser_state, A
            clrf    parser_checksum, A
            bra     PS_Loop

PS_BadChecksum:
            ; Count bad-checksum frame.
            incf    frame_badck_count_l, F, A
            bnz     PS_BadDone
            incf    frame_badck_count_h, F, A

PS_BadDone:
            ; Reset parser and continue scanning.
            clrf    parser_state, A
            clrf    parser_checksum, A
            bra     PS_Loop

            end
