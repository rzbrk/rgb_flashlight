;==============================================================================
; rgb_flashlight.asm
; Source: https://www.mikrocontroller.net/articles/AVR-Tutorial:_Timer
; Beschreibung:
; Blinkende LEDs an Ausgang Port C, gesteuert mittels 8Bit Timer_0 des
; ATmega.
;==============================================================================

.include "m8def.inc"

.def temp = r16
.def leds = r17

.org 0x0000
        rjmp    main                  ; Reset Handler
.org OVF0addr
        rjmp    timer0_overflow       ; Timer Overflow Handler

main:
        ; Stackpointer initialisieren
        ldi     temp, HIGH(RAMEND)
        out     SPH, temp
        ldi     temp, LOW(RAMEND)     
        out     SPL, temp

        ldi     temp, 0xFF            ; Port C auf Ausgang
        out     DDRC, temp

        ldi     leds, 0xFF

        ; Teiler definieren
        ldi     temp, (1<<CS00)|(0<<CS01)|(1<<CS02)
        out     TCCR0, temp

        ldi     temp, (1<<TOIE0)      ; TOIE0: Interrupt bei Timer Overflow
        out     TIMSK, temp

        sei

loop:   rjmp    loop

timer0_overflow:                      ; Timer 0 Overflow Handler
        out     PORTC, leds
        com     leds
        reti

    .exit

