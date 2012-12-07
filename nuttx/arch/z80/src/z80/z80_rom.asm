;**************************************************************************
; arch/z80/src/z80/z80_rom.asm
;
;   Copyright (C) 2008-2009, 2012 Gregory Nutt. All rights reserved.
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

	.title	NuttX for the Z80
	.module	z80_head

;**************************************************************************
; Constants
;**************************************************************************

	; Register save area layout

	XCPT_I 	==  0		; Offset 0: Saved I w/interrupt state in carry
	XCPT_BC	==  2		; Offset 1: Saved BC register
	XCPT_DE	==  4		; Offset 2: Saved DE register
	XCPT_IX	==  6		; Offset 3: Saved IX register
	XCPT_IY	==  8		; Offset 4: Saved IY register
	XCPT_SP	== 10		; Offset 5: Offset to SP at time of interrupt
	XCPT_HL	== 12		; Offset 6: Saved HL register
	XCPT_AF	== 14		; Offset 7: Saved AF register
	XCPT_PC	== 16		; Offset 8: Offset to PC at time of interrupt

	; Default stack base (needs to be fixed)

	.include	"asm_mem.h"

;**************************************************************************
; Global symbols used
;**************************************************************************

	.globl	_os_start		; OS entry point
	.globl	_up_doirq		; Interrupt decoding logic

;**************************************************************************
; System start logic
;**************************************************************************

_up_reset:
	; Set up the stack pointer at the location determined the Makefile
	; and stored in asm_mem.h

	ld		SP, #CONFIG_STACK_END	; Set stack pointer

	; Performed initialization unique to the SDCC toolchain

	call	gsinit			; Initialize the data section

	; Copy the reset vectors

	ld		hl, #_up_rstvectors	; code for RAM
	ld		de, #0x4000		; move it here
	ld		bc, #3*7		; 7 vectors / 3 bytes each
	ldir

	; Then start NuttX

	call	_os_start		; jump to the OS entry point

	; NuttX will never return, but just in case...

_up_halt::
	halt					; We should never get here
	jp		_up_halt

	; Data to copy to address 0x4000

_up_rstvectors:
	jp		_up_rst1		; 0x4000 : RST 1
	jp		_up_rst2		; 0x4003 : RST 2
	jp		_up_rst3		; 0x4006 : RST 3
	jp		_up_rst4		; 0x4009 : RST 4
	jp		_up_rst5		; 0x400c : RST 5
	jp		_up_rst6		; 0x400f : RST 6
	jp		_up_rst7		; 0x4012 : RST 7

;**************************************************************************
; Other reset handlers
;
; Interrupt mode 1 behavior:
; 
; 1. M1 cycle: 7 ticks
;    Acknowledge interrupt and decrements SP
; 2. M2 cycle: 3 ticks
;    Writes the MS byte of the PC onto the stack and decrements SP
; 3. M3 cycle: 3 ticks
;    Writes the LS byte of the PC onto the stack and sets the PC to 0x0038.
;
;**************************************************************************

_up_rst1:					; RST 1
	; Save AF on the stack, set the interrupt number and jump to the
	; common reset handling logic.
							; Offset 8: Return PC is already on the stack
	push	af				; Offset 7: AF (retaining flags)
	ld		a, #1			; 1 = Z80_RST1
	jr		_up_rstcommon	; Remaining RST handling is common

_up_rst2:					; RST 2
	; Save AF on the stack, set the interrupt number and jump to the
	; common reset handling logic.
							; Offset 8: Return PC is already on the stack
	push	af				; Offset 7: AF (retaining flags)
	ld		a, #2			; 2 = Z80_RST2
	jr		_up_rstcommon	; Remaining RST handling is common

_up_rst3:					; RST 3
	; Save AF on the stack, set the interrupt number and jump to the
	; common reset handling logic.
							; Offset 8: Return PC is already on the stack
	push	af				; Offset 7: AF (retaining flags)
	ld		a, #3			; 1 = Z80_RST3
	jr		_up_rstcommon	; Remaining RST handling is common

_up_rst4:					; RST 4
	; Save AF on the stack, set the interrupt number and jump to the
	; common reset handling logic.
							; Offset 8: Return PC is already on the stack
	push	af				; Offset 7: AF (retaining flags)
	ld		a, #4			; 1 = Z80_RST4
	jr		_up_rstcommon	; Remaining RST handling is common

_up_rst5:					; RST 5
	; Save AF on the stack, set the interrupt number and jump to the
	; common reset handling logic.
							; Offset 8: Return PC is already on the stack
	push	af				; Offset 7: AF (retaining flags)
	ld		a, #5			; 1 = Z80_RST5
	jr		_up_rstcommon	; Remaining RST handling is common

_up_rst6:					; RST 6
	; Save AF on the stack, set the interrupt number and jump to the
	; common reset handling logic.
							; Offset 8: Return PC is already on the stack
	push	af				; Offset 7: AF (retaining flags)
	ld		a, #6			; 1 = Z80_RST6
	jr		_up_rstcommon	; Remaining RST handling is common

_up_rst7:					; RST 7
	; Save AF on the stack, set the interrupt number and jump to the
	; common reset handling logic.
							; Offset 8: Return PC is already on the stack
	push	af				; Offset 7: AF (retaining flags)
	ld		a, #7			; 7 = Z80_RST7
	jr		_up_rstcommon	; Remaining RST handling is common

;**************************************************************************
; Common Interrupt handler
;**************************************************************************

_up_rstcommon:
	; Create a register frame.  SP points to top of frame + 4, pushes
	; decrement the stack pointer.  Already have
	;
	;   Offset 8: Return PC is already on the stack
	;   Offset 7: AF (retaining flags)
	;
	; IRQ number is in A

	push	hl				; Offset 6: HL
	ld		hl, #(3*2)		;    HL is the value of the stack pointer before
	add		hl, sp			;    the interrupt occurred
	push	hl				; Offset 5: Stack pointer
	push	iy				; Offset 4: IY
	push	ix				; Offset 3: IX
	push	de				; Offset 2: DE
	push	bc				; Offset 1: BC

	ld		b, a			;   Save the reset number in B
	ld		a, i			;   Parity bit holds interrupt state
	push	af				; Offset 0: I with interrupt state in parity
	di

	; Call the interrupt decode logic. SP points to the beginning of the reg structure

	ld		hl, #0			; Argument #2 is the beginning of the reg structure
	add		hl, sp			;
	push	hl				; Place argument #2 at the top of stack
	push	bc				; Argument #1 is the Reset number
	inc		sp				; (make byte sized)
	call	_up_doirq		; Decode the IRQ

	; On return, HL points to the beginning of the reg structure to restore
	; Note that (1) the arguments pushed on the stack are not popped, and (2) the
	; original stack pointer is lost.  In the normal case (no context switch),
	; HL will contain the value of the SP before the arguments were pushed.

	ld		sp, hl			; Use the new stack pointer

	; Restore registers.  HL points to the beginning of the reg structure to restore

	ex		af, af'			; Select alternate AF
	pop		af				; Offset 0: AF' = I with interrupt state in carry
	ex		af, af'			;   Restore original AF
	pop		bc				; Offset 1: BC
	pop		de				; Offset 2: DE
	pop		ix				; Offset 3: IX
	pop		iy				; Offset 4: IY
	exx						;   Use alternate BC/DE/HL
	ld		hl, #-2			;   Offset of SP to account for ret addr on stack
	pop		de				; Offset 5: HL' = Stack pointer after return
	add		hl, de			;   HL = Stack pointer value before return
	exx						;   Restore original BC/DE/HL
	pop		hl				; Offset 6: HL
	pop		af				; Offset 7: AF

	; Restore the stack pointer

	exx						; Use alternate BC/DE/HL
	ld		sp, hl			; Set SP = saved stack pointer value before return
	exx						; Restore original BC/DE/HL

	; Restore interrupt state

	ex		af, af'			; Recover interrupt state
	jp		po, nointenable	; Odd parity, IFF2=0, means disabled
	ex		af, af'			; Restore AF (before enabling interrupts)
	ei						; yes
	reti
nointenable::
	ex		af, af'			; Restore AF
	reti

;**************************************************************************
; Ordering of segments for the linker (SDCC only)
;**************************************************************************

	.area	_HOME
	.area	_CODE
	.area	_INITIALIZER
	.area	_GSINIT
	.area	_GSFINAL

	.area	_DATA
	.area	_INITIALIZED
	.area	_BSEG
	.area	_BSS
	.area	_HEAP

;**************************************************************************
; Global data initialization logic (SDCC only)
;**************************************************************************

	.area	_GSINIT
gsinit::
	ld		bc, #l__INITIALIZER
	ld		a, b
	or		a, c
	jr		Z, gsinit_next
	ld		de, #s__INITIALIZED
	ld		hl, #s__INITIALIZER
	ldir
gsinit_next:

	.area   _GSFINAL
	ret

;**************************************************************************
; The start of the heap (SDCC only).  Note that is actually resides in
; the _CODE area (which may be FLASH or ROM)
;**************************************************************************

	.area	_CODE
_g_heapbase::
	.dw		#s__HEAP

