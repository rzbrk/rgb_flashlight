;==============================================================================
; rgb_flashlight.asm
;==============================================================================

    .include "m8def.inc"                ; We are using ATmega8

    .cseg                               ; Code segment
    .org $10                            ; No interrupts

.reset:

;==============================================================================
; Initialization
;==============================================================================

init:

    ; Set-up stack
    ldi r16, low(ramend)
    out spl, r16
    ldi r16, high(ramend)
    out sph, r16

    ; Configure Port C for output
    ldi r16, $ff
    out DDRC, r16

    ldi r16, $ff                        ; initialize output

;==============================================================================
; Main loop
;==============================================================================

main:
    out PORTC, r16
    ldi r24, $00                        ; set wait time to 0.3 sec @ 1 MHz
    ldi r25, $00
    rcall wait                          ; call wait routine
    com r16                             ; bit-wise invert of r16 
    rjmp main

;==============================================================================
; Wait routine
;==============================================================================

; Calculation of wait time (t):
; r24:  start value for register r24 [1]
; r25:  start value for register r25 [1]
; f_mc: Clock frequency of Microcontroller [Hz]
;
; t = 4 * (256 - r24) * (256 - r25) / f_mc

wait:
    sbiw r24,1                      ; 2 clock steps
    brne wait                       ; 2 clock steps
    ret

.exit

