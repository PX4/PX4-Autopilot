;*************************************************************************
; arch/z80/src/z180/z180_saveusercontext.asm
;
;   Copyright (C) 2012 Gregory Nutt. All rights reserved.
;   Author: Gregory Nutt <gnutt@nuttx.org>
;
; Redistribution and use in source and binary forms, with or without
; modification, are permitted provided that the following conditions
; are met:
;
; 1. Redistributions of source code must retain the above copyright
;    notice, this list of conditions and the following disclaimer.
; 2. Redistributions in binary form must reproduce the above copyright
;    notice, this list of conditions and the following disclaimer in
;    the documentation and/or other materials provided with the
;    distribution.
; 3. Neither the name NuttX nor the names of its contributors may be
;    used to endorse or promote products derived from this software
;    without specific prior written permission.
;
; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
; "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
; LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
; FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
; COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
; INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
; BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
; OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
; AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
; LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
; ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
; POSSIBILITY OF SUCH DAMAGE.
;
;*************************************************************************

;*************************************************************************
; Constants
;*************************************************************************

	; Register save area layout

	.globl	XCPT_I		; Offset 0: Saved I w/interrupt state in parity
	.globl	XCPT_BC		; Offset 1: Saved BC register
	.globl	XCPT_DE		; Offset 2: Saved DE register
	.globl	XCPT_IX		; Offset 3: Saved IX register
	.globl	XCPT_IY		; Offset 4: Saved IY register
	.globl	XCPT_SP		; Offset 5: Offset to SP at time of interrupt
	.globl	XCPT_HL		; Offset 6: Saved HL register
	.globl	XCPT_AF		; Offset 7: Saved AF register
	.globl	XCPT_PC		; Offset 8: Offset to PC at time of interrupt

	; Stack frame

	FRAME_IY	==  0	; Location of IY on the stack
	FRAME_IX	==  2	; Location of IX on the stack
	FRAME_RET	==  4	; Location of return address on the stack
	FRAME_REGS	==  6	; Location of reg save area on stack

	SP_OFFSET	==  6

;*************************************************************************
; Name: z180_saveusercontext
;*************************************************************************

	.area	_CODE
_z180_saveusercontext:
	; Set up a stack frame

	push	ix					; Save IX and IY
	push	iy
	ld		ix, #0
	add		ix, sp				; IX = stack frame

	; Fetch the address of the save area

	ld		e, FRAME_REGS(ix)	; HL = save area address
	ld		d, FRAME_REGS+1(ix)	;
	ld		iy, #0
	add		iy, de				; IY = save area address

	; Then save the registers

	; Save the current interrupt state at offset 0

	ld		a, i				; Get interrupt state
	push	af
	pop		hl
	ld		XCPT_I(iy), l		; Offset 0: I w/interrupt state in parity
	ld		XCPT_I+1(iy), h

	; Save BC at offset 1

	ld		XCPT_BC(iy), c		; Offset 1: BC
	ld		XCPT_BC+1(iy), b

	; DE is not preserved (offset 2)

	; Save IX at offset 3

	ld		l, FRAME_IX(ix)		; HL = Saved alue of IX
	ld		h, FRAME_IX+1(ix)	;
	ld		XCPT_IX(iy), l		; Offset 3: IX
	ld		XCPT_IX+1(iy), h	;

	; Save IY at offset 4

	ld		l, FRAME_IY(ix)		; HL = Saved value of IY
	ld		h, FRAME_IY+1(ix)	;
	ld		XCPT_IY(iy), l		; Offset 4: IY
	ld		XCPT_IY+1(iy), h

	; Save that stack pointer as it would be upon return in offset 5

	ld		hl, #SP_OFFSET		; Value of stack pointer on return
	add		hl, sp
	ld		XCPT_SP(iy), l		; Offset 5 SP
	ld		XCPT_SP+1(iy), h

	; HL is saved as the value 1 at offset 6

	xor		a					; A = 0
	ld		XCPT_HL+1(iy), a	; Offset 2: HL on return (=1)
	inc		a					; A = 1
	ld		XCPT_HL(iy), a		;

	; AF is not preserved (offset 7)

	; Save the return address in offset 8

	ld		l, FRAME_RET(ix)	; HL = Saved return address
	ld		h, FRAME_RET+1(ix)	;
	ld		XCPT_PC(iy), l		; Offset 8: PC
	ld		XCPT_PC+1(iy), h

	; Return the value 0

	xor		a					; HL = return value of zero
	ld		l, a
	ld		h, a

	pop		iy
	pop		ix
	ret
