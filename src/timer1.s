            PROCESSOR 18F87K22
            #include <xc.inc>

;===========================================================
; timer1.s
; Owns: InitTimer1_20ms
; No RAM required - operates only on SFRs.
;===========================================================

            GLOBAL  InitTimer1_20ms

            psect   timer1_code,class=CODE,reloc=2

;-----------------------------------------------------------
; InitTimer1_20ms
; Assumes Fosc = 64 MHz
; Internal clock = Fosc/4 = 16 MHz
; Prescaler = 1:8  =>  timer tick = 0.5 us
; 20 ms = 40000 counts
; Preload = 65536 - 40000 = 25536 = 0x63C0
;-----------------------------------------------------------
InitTimer1_20ms:
            ; Ensure Timer1 peripheral is enabled.
            bcf     PMD1, 1, A

            ; Disable Timer1 gate function.
            clrf    T1GCON, A

            ; Stop Timer1 while configuring.
            clrf    T1CON, A

            ; T1CON = 00110010b
            ; TMR1CS<1:0> = 00  internal clock (Fosc/4)
            ; T1CKPS<1:0> = 11  1:8 prescale
            ; RD16         = 1
            movlw   00110010B
            movwf   T1CON, A

            ; Load 20 ms preload value.
            movlw   0x63
            movwf   TMR1H, A
            movlw   0xC0
            movwf   TMR1L, A

            ; Clear stale Timer1 interrupt flag.
            bcf     PIR1, 0, A

            ; Enable Timer1 interrupt.
            bsf     PIE1, 0, A

            ; Start Timer1.
            bsf     T1CON, 0, A
            return

            end
