#include <xc.inc>

GLOBAL  Start

PSECT resetVec, class=CODE, reloc=2
resetVec:
    goto    Start

PSECT udata_acs
hb0:        DS 1
hb1:        DS 1
hb2:        DS 1
tick_count: DS 1
t1_div:     DS 1

PSECT code, class=CODE, reloc=2

Start:
    ; Make analog-capable pins digital
    BANKSEL ANCON0
    clrf    BANKMASK(ANCON0), b
    clrf    BANKMASK(ANCON1), b
    clrf    BANKMASK(ANCON2), b
    movlb   0

    ; RA0 heartbeat LED, RA1 timer LED
    clrf    LATA, A
    movlw   0xFC
    movwf   TRISA, A

    ; Heartbeat preload
    clrf    hb0, A
    clrf    hb1, A
    movlw   0xF0
    movwf   hb2, A

    ; 25 x 20 ms = 500 ms per toggle
    movlw   25
    movwf   t1_div, A
    clrf    tick_count, A

    ; Timer1 setup
    clrf    T1GCON, A
    bcf     TMR1IF              ; NOT: bcf PIR1, TMR1IF, A

    movlw   0x63
    movwf   TMR1H, A
    movlw   0xC0
    movwf   TMR1L, A

    movlw   0x32                ; Fosc/4, 1:8 prescale, RD16=1, off
    movwf   T1CON, A
    bsf     TMR1ON              ; simpler bit-symbol style

MainLoop:
    ; Foreground heartbeat
    incfsz  hb0, F, A
    bra     CheckTimer1
    incfsz  hb1, F, A
    bra     CheckTimer1
    incfsz  hb2, F, A
    bra     CheckTimer1

    btg     LATA, 0, A
    clrf    hb0, A
    clrf    hb1, A
    movlw   0xF0
    movwf   hb2, A

CheckTimer1:
    btfss   TMR1IF              ; NOT: btfss PIR1, TMR1IF, A
    bra     MainLoop

    bcf     TMR1ON
    bcf     TMR1IF              ; NOT: bcf PIR1, TMR1IF, A

    movlw   0x63
    movwf   TMR1H, A
    movlw   0xC0
    movwf   TMR1L, A

    bsf     TMR1ON
    incf    tick_count, F, A

    decfsz  t1_div, F, A
    bra     MainLoop

    movlw   25
    movwf   t1_div, A
    btg     LATA, 1, A
    bra     MainLoop

    END     resetVec
