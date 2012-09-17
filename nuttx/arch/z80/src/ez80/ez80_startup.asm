;**************************************************************************
; arch/z80/src/ez80/ez80_startup.asm
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
; Included Files
;**************************************************************************

;**************************************************************************
; Constants
;**************************************************************************

;**************************************************************************
; Global symbols used
;**************************************************************************

	xref	__stack
	xref	_ez80_init
	xref	_ez80_initvectors
	xref	_ez80_initsysclk
	xref	_ez80_lowinit
	xref	__low_bss	; Low address of bss segment
	xref	__len_bss	; Length of bss segment

	xref	__low_data	; Address of initialized data section
	xref	__low_romdata	; Addr of initialized data section in ROM
	xref	__len_data	; Length of initialized data section

	xref	__copy_code_to_ram
	xref	__len_code
	xref	__low_code
	xref	__low_romcode
	xref	_os_start
	xdef	_ez80_startup
	xdef	_ez80_halt

;**************************************************************************
; Code
;**************************************************************************

	segment	CODE
	.assume ADL=1

;**************************************************************************
; System reset start logic
;**************************************************************************

_ez80_startup:
	; Set up the stack pointer at the location determined the lincmd
	; file

	ld	sp, __stack

	; Peform chip-specific initialization

	call	_ez80_init

	; initialize the interrupt vector table

	call 	_ez80_initvectors

	; Initialize the system clock

	call	_ez80_initsysclk

	; Perform C initializations
	; Clear the uninitialized data section

	ld	bc, __len_bss		; Check for non-zero length
	ld	a, __len_bss >> 16
	or	a, c
	or	a, b
	jr	z, _ez80_bssdone	; BSS is zero-length ...
	xor	a, a
	ld	(__low_bss), a
	sbc	hl, hl			; hl = 0
	dec	bc			; 1st byte's taken care of
	sbc	hl, bc
	jr	z, _ez80_bssdone	; Just 1 byte ...
	ld	hl, __low_bss		; reset hl
	ld	de, __low_bss + 1	; [de] = bss + 1
	ldir
_ez80_bssdone:

	; Copy the initialized data section
	ld	bc, __len_data		; [bc] = data length
	ld	a, __len_data >> 16	; Check for non-zero length
	or	a, c
	or	a, b
	jr	z, _ez80_datadone	; __len_data is zero-length ...
	ld	hl, __low_romdata	; [hl] = data_copy
	ld	de, __low_data		; [de] = data
	ldir				; Copy the data section
_ez80_datadone:

	; Copy CODE (which may be in FLASH) to RAM if the
	; copy_code_to_ram symbol is set in the link control file
	ld	a, __copy_code_to_ram
	or	a, a
	jr	z, _ez80_codedone
	ld	bc, __len_code		; [bc] = code length
	ld	a, __len_code >> 16	; Check for non-zero length
	or	a, c
	or	a, b
	jr	z, _ez80_codedone	; __len_code is zero-length
	ld	hl, __low_romcode	; [hl] = code_copy
	ld	de, __low_code		; [de] = code
	ldir				; Copy the code section
_ez80_codedone:

	; Perform board-specific intialization

	call	_ez80_lowinit

	; Then start NuttX

	call	_os_start		; jump to the OS entry point

	; NuttX will never return, but just in case...

_ez80_halt:
	halt				; We should never get here
	jp	_ez80_halt

