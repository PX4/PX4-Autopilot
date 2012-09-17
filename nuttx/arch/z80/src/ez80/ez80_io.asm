;**************************************************************************
; arch/z80/src/ze80/ez80_io.c
;
;   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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

	xdef	_outp
	xdef	_inp

;**************************************************************************
; Global Symbols Expported
;**************************************************************************

	CONFIG_EZ80_Z80MODE equ 0

;**************************************************************************
; Code
;**************************************************************************

	segment CODE
	.assume ADL=1

;**************************************************************************
; Name: void outp(uint16_t p, uint8_t c)
;
; Description:
;   Output byte c on port p
;
;**************************************************************************

_outp:
	; Create a stack frame

	push	ix
	ld	ix, #0
	add	ix, sp

	; Get the arguments from the stack

	.if CONFIG_EZ80_Z80MODE
	; Stack looks like:
	;
	;	7-8	Unused
	;	 6	Value
	;	4-5	Port
	;	2-3	Return address
	; SP:	0-1	Caller's fame pointer

	ld	bc, (ix + 4)	; Port
	ld	a, (ix + 6)	; Value

	.else
	; Stack looks like:
	;
	;	10-11	Unused
	;	 9	Value
	;	 8	Unused
	;	6-7	Port
	;	3-5	Return address
	; SP:	0-2	Caller's frame pointer

	ld	bc, (ix + 6)	; Port (upper 8 bits not used)
	ld	a, (ix + 9)	; Value

	.endif

	; Output the specified by to the specified 8-bit I/O address

	out	(bc), a
	pop	ix
	ret

;**************************************************************************
; Name: uint8_t inp(uint16_t p)
;
; Description:
;   Input byte from port p
;
;**************************************************************************

_inp:
	; Create a stack frame

	push	ix
	ld	ix, #0
	add	ix, sp

	; Get the arguments from the stack

	.if CONFIG_EZ80_Z80MODE
	; Stack looks like:
	;
	;	4-5	Port
	;	2-3	Return address
	; SP:	0-1	Caller's fame pointer

	ld	bc, (ix + 4)	; Port

	.else
	; Stack looks like:
	;
	;	 8	Unused
	;	6-7	Port
	;	3-5	Return address
	; SP:	0-2	Caller's frame pointer

	ld	bc, (ix + 6)	; Port (upper 8 bits not used)

	.endif

	; Return port value in A

	in	a, (bc)
	pop	ix
	ret

	end
