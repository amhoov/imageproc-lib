;-----------------------------------------------------------------------;
; udiv16.s: Q16 division elementary operation.
;
; Based on udivsi3.s from the compact math library for the dsPIC30.
; (c) Microchip Technology. 2006.
;
;-----------------------------------------------------------------------;
;
; C prototype: _iq _uIQ16div(_iq, _iq);
;
; __uIQ16div
;
;       Unsigned 32-bit Q16 division.
;
; Input:
;
;       (w1:w0) Dividend (Q)
;       (w3:w2) Divisor (D)
;
; Output:
;
;       (w1:w0) Quotient
;       (w5:w4) Remainder
;
; Description:
;
;       Restoring, sequential divison.
;
;-----------------------------------------------------------------------;
        .global __uIQ16div

__uIQ16div:
	mul.uu  w1, #1, w4      ; (w4:w5) (R) = Q << 16		; 1
        mov     w0, w1          ; (w1:w0) = Q << 16		; 2
        clr     w0						; 3
        mov     #32,w6          ; (w6) = iterator		; 4

;------ Calculate the next quotient & remainder bit

nextbit:

;------ (R,Q) <<= 1

        sl      w0,w0           ; (w1:w0) = Q <<= 1		; 1
        rlc     w1,w1           ; *				; 2
        rlc     w4,w4           ; (w5:w4) = R <<= 1		; 3
        rlc     w5,w5           ; *				; 4

;------ R -= D

        bset    w0,#0           ; (w1:w0) = Q += 1		; 5
        sub     w4,w2,w4        ; (w5:w4) = R -= D		; 6
        subb    w5,w3,w5        ; *				; 7
        bra     nn,iterate      ; No restore needed ...		; 8

;------ Restore R: R += D

        add     w4,w2,w4        ; (w5:w4) = R += D		; 1
        addc    w5,w3,w5        ; *				; 2
        bclr    w0,#0           ; (w1:w0) = Q -= 1		; 3

;------ Iterate through the bits

iterate:
        dec     w6,w6           ; (w6) = iterator--		; 9
        bra     nz,nextbit      ; Get all 32 bits ...		; 10

        return                  ; Done				; 1

;-----------------------------------------------------------------------;
        .end


; run-time: min                           max
;           4 + 10*32 + 1 == 326 cycles   4 + 10*32 + 3*32 + 1  == 422 cycles
