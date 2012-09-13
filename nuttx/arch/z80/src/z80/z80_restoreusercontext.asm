;**************************************************************************
; arch/z80/src/z80/z80_restoreusercontext.asm
;
;   Copyright (C) 2007-2009 Gregory Nutt. All rights reserved.
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
;**************************************************************************

	; Register save area layout

	.globl	XCPT_I		; Offset 0: Saved I w/interrupt state in carry
	.globl	XCPT_BC		; Offset 1: Saved BC register
	.globl	XCPT_DE		; Offset 2: Saved DE register
	.globl	XCPT_IX		; Offset 3: Saved IX register
	.globl	XCPT_IY		; Offset 4: Saved IY register
	.globl	XCPT_SP		; Offset 5: Offset to SP at time of interrupt
	.globl	XCPT_HL		; Offset 6: Saved HL register
	.globl	XCPT_AF		; Offset 7: Saved AF register
	.globl	XCPT_PC		; Offset 8: Offset to PC at time of interrupt

;**************************************************************************
; z80_restoreusercontext
;**************************************************************************

	.area	_CODE
_z80_restoreusercontext:
	; On entry, stack contains return address (not used), then address
	; of the register save structure

	; Discard the return address, we won't be returning

	pop	hl

	; Get the address of the beginning of the state save area.  Each
	; pop will increment to the next element of the structure

	pop	hl		; BC = Address of save structure
	ld	sp, hl		; SP points to top of storage area

	; Disable interrupts while we muck with the alternative registers.  The
	; Correct interrupt state will be restore below

	di

	; Restore registers.  HL points to the beginning of the reg structure to restore

	ex	af, af'			; Select alternate AF
	pop	af			; Offset 0: AF' = I with interrupt state in parity
	ex	af, af'			;   Restore original AF
	pop	bc			; Offset 1: BC
	pop	de			; Offset 2: DE
	pop	ix			; Offset 3: IX
	pop	iy			; Offset 4: IY
	exx				;   Use alternate BC/DE/HL
	pop	hl			; Offset 5: HL' = Stack pointer after return
	exx				;   Restore original BC/DE/HL
	pop	hl			; Offset 6: HL
	pop	af			; Offset 7: AF

	; Restore the stack pointer

	exx				; Use alternate BC/DE/HL
	pop	de			; DE' = return address
	ld	sp, hl			; Set SP = saved stack pointer value before return
	push	de			; Save return address for ret instruction
	exx				; Restore original BC/DE/HL

	; Restore interrupt state

	ex	af, af'			; Recover interrupt state
	jp	po, noinrestore		; Odd parity, IFF2=0, means disabled
	ex	af, af'			; Restore AF (before enabling interrupts)
	ei				; yes.. Enable interrupts
	ret				; and return
noinrestore:
	ex	af, af'			; Restore AF
	ret				; Return with interrupts disabled
