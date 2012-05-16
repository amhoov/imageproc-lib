;-----------------------------------------------------------------------;
; div16.s: Q16 division elementary operation.
;
; Based on divsi3.s from the compact math library for the dsPIC30.
; (c) Microchip Technology. 2006.
;
;-----------------------------------------------------------------------;
;
; C prototype: _siq _IQ16div(_siq, _siq);
;
; __IQ16div
;
;       Signed 32-bit Q16 division.
;
; Input:
;
;       (w1:w0) Dividend (Q)
;       (w3:w2) Divisor (D)
;
; Output:
;
;       (w1:w0) Quotient (Q)
;
; Description:
;
;       Caclulates the quotient. The sign of the quotient is the
;       exclusive OR of the operand signs.
;-----------------------------------------------------------------------;
        .global __IQ16div

__IQ16div:
;-----------------------------------------------------------------------;
        xor     w1,w3,[w15++]   ; Save sign of result		; 1
        cp0     w1              ; Q < 0 ?			; 2
        bra     ge,divtestb     ; No ...			; 3

;------ Q < 0

        subr    w0,#0,w0        ; (w1:w0) := Q = -Q		; 1
        subbr   w1,#0,w1        ; *				; 2

divtestb:
        cp0     w3              ; D < 0 ?			; 4
        bra     ge,calcquot     ; No ...			; 5

;------ D < 0

        subr    w2,#0,w2        ; (w3:w2) := D = -D		; 1
        subbr   w3,#0,w3        ; *				; 2

;------ Compute the quotient

calcquot:
        rcall   __uIQ16div      ; (w1:w0) = quotient		; 6

        cp0     [--w15]         ; Result -ve ?			; 7
        bra     nn,returnq      ; No ...			; 8

;------ Result is negative

        subr    w0,#0,w0        ; (w1:w0) := Q = -Q		; 1
        subbr   w1,#0,w1        ; *				; 2
returnq:
        return							; 9
;-----------------------------------------------------------------------;
        .end

; run-time: min                        max
;           9 + 326 == 335 cycles      9 + 2 + 2 + 2 + 422 == 437 cycles

