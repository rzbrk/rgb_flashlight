;==============================================================================
; rgb_flashlight.asm
;==============================================================================

    .include "m8def.inc"                ; We are using ATmega8
    .include "wait.inc"
    .include "button.inc"

    .def cnt = r18                      ; Global counter for PWM
    .def pwm_red = r20                  ; Holds PWM for RED led
    .def pwm_grn = r21                  ; Holds PWM for GREEN led
    .def pwm_blu = r22                  ; Holds PWM for BLUE led
    .equ pwm_max = $20                  ; Max value for PWM
;    .equ lstend = $01                   ; End mark of list "vals"

    .equ port_btn = PINB                ; Port to which buttons are connected
    .equ pin_btn_red = PINB0            ; Button RED led
    .equ pin_btn_grn = PINB1            ; Button GREEN led
    .equ pin_btn_blu = PINB2            ; Button BLUE led

    .equ port_led = PORTC               ; Port to which leds are connected
    .equ pddr_led = DDRC                ; and its DDR
    ; Define the pins, where the leds are connected to
    .equ pin_led_red = $00              ; RED led
    .equ pin_led_grn = $01              ; GREEN led
    .equ pin_led_blu = $02              ; BLUE led

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

    ; Configure port_led for output
    ldi r16, $ff
    out pddr_led, r16
    ldi r16, (0<<pin_led_red)|(0<<pin_led_grn)|(0<<pin_led_blu)
    out port_led, r16

;    ; Configure pointer address for list "vals". Store values in word register
;    ; X (XH,XL)
;    ldi ZL, low(addr*2)
;    ldi ZH, high(addr*2)
;    lpm                                 ; r0 <= low(addr)
;    mov XL, r0
;    adiw ZL, 1                          ; Z <= Z+1
;    lpm                                 ; r0 <= high(addr)
;    mov XH, r0
;    lsl XL                              ; low(byte addr.) <= 2 * word addr.
;    rol XH                              ; high(byte addr.) <= 2 * word addr.
;
;    ; Initialize PWM (r0 holds PWM value after execution of lpm)
;    rcall list_begin    
;    lpm                                 ; r0 <= first list element
;    mov pwm_led, r0                     ; Initialize pwm_led

    ; Configute Timer_0 interrupt (divider: 1)
    ldi r16, (1<<CS00)|(0<<CS01)|(0<<CS02)
    out TCCR0, r16
    ldi r16, (1<<TOIE0)
    out TIMSK, r16
    sei                                 ; Activate interrupt

    ; Initialize counter and PWMs
    clr cnt
    ldi pwm_red, $01
    ldi pwm_grn, $01
    ldi pwm_blu, $01

;==============================================================================
; Main loop
;==============================================================================

main:
    button port_btn, pin_btn_red, pwm_red
    button port_btn, pin_btn_grn, pwm_grn
    button port_btn, pin_btn_blu, pwm_blu

    rjmp main

;==============================================================================
; Timer_0 interrupt routine
;==============================================================================

timer:
    push r16
    in r16, SREG                        ; Save SREG
    push r16

    ; R16 holds output. Clear this register.
    clr r16

    ; RED led: compare pwm_red with cnt and decide whether led on/off
    ; On  if cnt <= pwm_red
    ; Off if cnt >  pwm_red
    cp pwm_red, cnt                     ; Skip following ori cmd if pwm_led < cnt
    brlo skp_to_grn
    ori r16, (1<<pin_led_red)           ; turn led on

    ; GREEN led
skp_to_grn:
    cp pwm_grn, cnt
    brlo skp_to_blu
    ori r16, (1<<pin_led_grn)

    ; BLUE led
skp_to_blu:
    cp pwm_blu, cnt
    brlo skp_led_out
    ori r16, (1<<pin_led_blu)

skp_led_out:
    out port_led, r16                   ; Output to port C

    inc cnt                             ; Increment counter
    cpi cnt, pwm_max                    ; cnt < pwm_max?
    brlo cnt_skp                        ; No: Jump to cnt_skp
    ldi cnt, $00                        ; Yes: Reset cnt to zero

cnt_skp:
    pop r16
    out SREG, r16                       ; Restore SREG from stack
    pop r16
        
    reti

;==============================================================================
; Return pointer address to beginning of list "vals".
;==============================================================================

;list_begin:
;    mov ZL, XL
;    mov ZH, XH
;    ret

;==============================================================================
; List definitions
;==============================================================================

;addr:   .dw vals                    ; Start address of list vals
;vals:   .db 0,2,4,8,16,32,1,0       ; List with PWM value definitions. "1"
;                                    ; denotes end of list. # of list elements
;                                    ; shall be even, hence added extra "0".

    .exit

