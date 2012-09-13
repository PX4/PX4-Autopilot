;**************************************************************************
; arch/z80/src/ez80/ez80_irqsave.asm
;
;   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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

;**************************************************************************
; Global Symbols Imported
;**************************************************************************

;**************************************************************************
; Global Symbols Expported
;**************************************************************************

	xdef	_irqsave
	xdef	_irqrestore

;**************************************************************************
; Code
;**************************************************************************

	segment CODE
	.assume ADL=1

;**************************************************************************
;* Name: irqstate_t irqsave(void)
;*
;* Description:
;*   Disable all interrupts; return previous interrupt state
;*
;**************************************************************************

_irqsave:
	ld	a, i		; AF = interrupt state
	di			; Interrupts are disabled (does not affect F)
	push	af		; Transfer to HL via the stack
	pop	hl		;
	ret			; And return

;**************************************************************************
;* Name: void irqrestore(irqstate_t flags)
;*
;* Description:
;*   Restore previous interrupt state
;*
;**************************************************************************

_irqrestore:
	di			; Assume disabled
	pop	hl		; HL = return address
	pop	af		; AF Parity bit holds interrupt state
	jp	po, _disabled	; Skip over re-enable if Parity odd
	ei			; Re-enable interrupts
_disabled:
	push	af		; Restore stack
	push	hl		;
	ret			; and return

	end
