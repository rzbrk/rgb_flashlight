;==============================================================================
; Check for buttons pressed
;==============================================================================

.MACRO button
    push r16

btn_pr:
    sbic @0, @1                         ; Button pressed?
    rjmp btn_end                        ; No? Jump to btn_end
    lsl @2                              ; Yes? Multiply pwm with 2

    ldi r16, pwm_max
    cp r16, @2                          ; @2 <= pwm_max ?
    brsh btn_skp                        ; Yes: Go ahead to btn_skp
    ldi @2, $01                         ; No: Reset pwm value

;    adiw ZL, 1                          ; Yes? Move pointer in "vals" one
;                                        ; element forward.
;    lpm                                 ; r0 holds list element
;    ldi r16, lstend                     ; Check if end of list is reached
;    cp r0, r16
;    brne btn_skp                        ; No? Jump to btn_skp
;    rcall list_begin                    ; Yes? Go to beginning of list "vals".
;    lpm                                 ; Now, read the first element to r0.

btn_skp:
;    mov pwm_led, r0                     ; pwm_led <= r0
    wait $00, $0a                       ; Debounce: wait time ~10 ms

btn_rls:
    sbis @0, @1                         ; Button released?
    rjmp btn_rls                        ; No? Jump back to btn_rls
    wait $00, $0a                       ; Debounce: wait time ~10 ms

btn_end:
    pop r16
;    ret
    .ENDM
