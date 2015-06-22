;==============================================================================
; Wait routine
;==============================================================================

; Calculation of wait time (t):
; r24:  start value for register r24 [1]
; r25:  start value for register r25 [1]
; f_mc: Clock frequency of Microcontroller [Hz]
;
; t = 4 * (256 - r24) * (256 - r25) / f_mc

.MACRO wait
    ldi r24, @0                         ; Debounce: set wait time ~10 ms
    ldi r25, @1
waitloop:
    sbiw r24,1                          ; 2 clock steps
    brne waitloop                       ; 2 clock steps
;    ret
    .ENDM

