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
