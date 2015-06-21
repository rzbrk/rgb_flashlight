;==============================================================================
; rgb_flashlight.asm
;==============================================================================

    .include "m8def.inc"                ; We are using ATmega8

    .def cnt = r18                      ; Global counter for PWM
    .def pwm_led = r19                  ; holds PWM value for LED
    .equ lstend = $01                   ; End mark of list "vals"

    .org $0000
    rjmp init                           ; Jump here after reset
    .org OVF0addr
    rjmp timer                          ; Jump for Timer_0 overflow interrupt

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

    ; Configute Timer_0 interrupt (divider: 1)
    ldi r16, (1<<CS00)|(0<<CS01)|(0<<CS02)
    out TCCR0, r16
    ldi r16, (1<<TOIE0)
    out TIMSK, r16
    sei                                 ; Activate interrupt

    ; Initialize counter
    clr cnt

;==============================================================================
; Main loop
;==============================================================================

main:
    rcall button
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
; Timer_0 interrupt routine
;==============================================================================

timer:
    push r16
    in r16, SREG                        ; Save SREG
    push r16

    ldi r16, $ff                        ; Assume LED to be on ($ff)
    cp pwm_led, cnt                     ; Skip following cmd if pwm_led > cnt
    brsh led_skp
    ldi r16, $00                        ; LED off

led_skp:
    out PORTC, r16                      ; Output to Port C

    inc cnt                             ; Increment counter
    cpi cnt, $20                        ; cnt >= 32?
    brlo cnt_skp                        ; No: Jump to cnt_skp
    ldi cnt, $00                        ; Yes: Reset cnt to zero

cnt_skp:
    pop r16
    out SREG, r16                       ; Restore SREG from stack
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
vals:   .db 0,2,4,8,16,32,1,0       ; List with PWM value definitions. "1"
                                    ; denotes end of list. # of list elements
                                    ; shall be even, hence added extra "0".

    .exit

