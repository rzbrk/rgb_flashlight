;==============================================================================
; rgb_flashlight.asm
;==============================================================================

    .include "m8def.inc"                ; We are using ATmega8

    .def cnt = r18                      ; Global counter for PWM
    .def pwm_led = r19                  ; holds PWM value for LED
    .equ lstend = $01                   ; End mark of list "vals"

    .cseg                               ; Code segment
    rjmp init                           ; Jump here after reset
;    .org $06
;    rjmp timer                          ; Jump for Timer_0 overflow interrupt
    .org $10                            ; Skip all other interrupts

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

    ; Configure pointer address for list "vals". Store values in word register
    ; X (XH,XL)
    ldi ZL, low(addr*2)
    ldi ZH, high(addr*2)
    lpm                                 ; r0 <= low(addr)
    mov XL, r0
    adiw ZL, 1                          ; Z <= Z+1
    lpm                                 ; r0 <= high(addr)
    mov XH, r0
    lsl XL                              ; low(byte addr.) <= 2 * word addr.
    rol XH                              ; high(byte addr.) <= 2 * word addr.

    ; Initialize PWM (r0 holds PWM value after execution of lpm)
    rcall list_begin    
    lpm                                 ; r0 <= first list element
    mov pwm_led, r0                     ; Initialize pwm_led

    ; Configute Timer_0 interrupt
    ldi r16, 0b011                      ; System clock divider: 64
    out TCCR0, r16
    in r16, TIMSK                       ; Timer interrupt mask
    ori r16, 0b00000010                 ; TOIE0 = 1
    out TIMSK, r16
    clr cnt                             ; Initialize PWM counter
    sei                                 ; Activate interrupt

;==============================================================================
; Main loop
;==============================================================================

main:
    rcall button
    rcall led
    rjmp main

;==============================================================================
; Check for buttons pressed
;==============================================================================

; Button is connected to PINB0

button:
    push r16

btn_pr:
    sbic PINB, PINB0                    ; Button pressed?
    rjmp btn_end                        ; No? Jump to btn_end
    adiw ZL, 1                          ; Yes? Move pointer in "vals" one
                                        ; element forward.
    lpm                                 ; r0 holds list element
    ldi r16, lstend                     ; Check if end of list is reached
    cp r0, r16
    brne btn_skp                        ; No? Jump to btn_skp
    rcall list_begin                    ; Yes? Go to beginning of list "vals".
    lpm                                 ; Now, read the first element to r0.

btn_skp:
    mov pwm_led, r0                     ; pwm_led <= r0
    ldi r24, $00                        ; Debounce: set wait time ~10 ms
    ldi r25, $0a
    rcall wait

btn_rls:
    sbis PINB, PINB0                    ; Button released?
    rjmp btn_rls                        ; No? Jump back to btn_rls

    ldi r24, $00                        ; Debounce: Set wait time ~10 ms
    ldi r25, $0a
    rcall wait

btn_end:
    pop r16
    ret

;==============================================================================
; Output PWM to LED
;==============================================================================

; LED is connected somewhere on Port C

led:
    push r16

    dec cnt

    ldi r16, $ff                        ; Assume LED to be on
    cp pwm_led, cnt                     ; Compare
    brsh led_skp                        ; Skip following command if
                                        ; pwm_led > cnt
    ldi r16, $00                        ; LED off
led_skp:
    out PORTC, r16

    pop r16
    ret

;==============================================================================
; Timer_0 interrupt routine
;==============================================================================

timer:
    push r16
    in r16, SREG                        ; Save SREG
    push r16

;    dec cnt                             ; Decrement counter

    pop r16
    out SREG, r16                      ; Restore SREG from stack
    pop r16
        
    reti

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

;==============================================================================
; Return pointer address to beginning of list "vals".
;==============================================================================

list_begin:
    mov ZL, XL
    mov ZH, XH
    ret

;==============================================================================
; List definitions
;==============================================================================

addr:   .dw vals                    ; Start address of list vals
vals:   .db 0,16,32,64,128,255,1    ; List with PWM value definitions. "1"
                                    ; denotes end of list.

    .exit

