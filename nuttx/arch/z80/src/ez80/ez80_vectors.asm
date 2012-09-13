;**************************************************************************
; arch/z80/src/ez80/ez80_vectors.asm
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
; Constants
;**************************************************************************

NVECTORS	EQU	64		; max possible interrupt vectors

;* Bits in the Z80 FLAGS register *****************************************

EZ80_C_FLAG	EQU	01h		; Bit 0: Carry flag
EZ80_N_FLAG	EQU	02h		; Bit 1: Add/Subtract flag
EZ80_PV_FLAG	EQU	04h		; Bit 2: Parity/Overflow flag
EZ80_H_FLAG	EQU	10h		; Bit 4: Half carry flag
EZ80_Z_FLAG	EQU	40h		; Bit 5: Zero flag
EZ80_S_FLAG	EQU	80h		; Bit 7: Sign flag

;* The IRQ number to use for unused vectors

EZ80_UNUSED	EQU	40h

;**************************************************************************
; Global Symbols Imported
;**************************************************************************

	xref	_ez80_startup
	xref	_up_doirq

;**************************************************************************
; Global Symbols Exported
;**************************************************************************

	xdef	_ez80_reset
	xdef	_ez80_initvectors
	xdef	_ez80_handlers
	xdef	_ez80_rstcommon
	xdef	_ez80_initvectors
	xdef	_ez80_vectable

;**************************************************************************
; Macros
;**************************************************************************

; Define one reset handler
;  1. Disable interrupts
;  2. Dlear mixed memory mode (MADL) flag
;  3. jump to initialization procedure with jp.lil to set ADL
rstvector: macro
	di
	rsmix
	jp.lil	_ez80_startup
	endmac	rstvector

; Define one interrupt handler
irqhandler: macro vectno
	; Save AF on the stack, set the interrupt number and jump to the
	; common reset handling logic.
					; Offset 8: Return PC is already on the stack
	push	af			; Offset 7: AF (retaining flags)
	ld	a, #vectno		; A = vector number
	jp	_ez80_rstcommon		; Remaining RST handling is common
	endmac	irqhandler

;**************************************************************************
; Reset entry points
;**************************************************************************

	define	.RESET, space = ROM
	segment	.RESET

_ez80_reset:
_rst0:
	rstvector
_rst8:
	rstvector
_rst10:
	rstvector
_rst18:
	rstvector
_rst20:
	rstvector
_rst28:
	rstvector
_rst30:
	rstvector
_rst38:
	rstvector
	ds %26
_nmi:
	retn

;**************************************************************************
; Startup logic
;**************************************************************************

	define .STARTUP, space = ROM
	segment .STARTUP
	.assume ADL=1

;**************************************************************************
; Interrupt Vector Handling
;**************************************************************************

					; Symbol           Val VecNo Addr
					;----------------- --- ----- -----
_ez80_handlers:
	irqhandler	 0		; EZ80_EMACRX_IRQ   0    0   0x040   
	handlersize equ $-_ez80handlers
	irqhandler	 1		; EZ80_EMACTX_IRQ   1    1   0x044   
	irqhandler	 2		; EZ80_EMACSYS_IRQ  2    2   0x048   
	irqhandler	 3		; EZ80_PLL_IRQ      3    3   0x04c   
	irqhandler	 4		; EZ80_FLASH_IRQ    4    4   0x050   
	irqhandler	 5		; EZ80_TIMER0_IRQ   5    5   0x054   
	irqhandler	 6		; EZ80_TIMER1_IRQ   6    6   0x058   
	irqhandler	 7		; EZ80_TIMER2_IRQ   7    7   0x05c   
	irqhandler	 8		; EZ80_TIMER3_IRQ   8    8   0x060   
	irqhandler	EZ80_UNUSED	;                        9   0x064   
	irqhandler	EZ80_UNUSED+1	;                       10   0x068   
	irqhandler	 9		; EZ80_RTC_IRQ      9   11   0x06C   
	irqhandler	10		; EZ80_UART0_IRQ   10   12   0x070   
	irqhandler	11		; EZ80_UART1_IRQ   11   13   0x074   
	irqhandler	12		; EZ80_I2C_IRQ     12   14   0x078   
	irqhandler	13		; EZ80_SPI_IRQ     13   15   0x07c   
	irqhandler	14		; EZ80_PORTA0_IRQ  14   16   0x080   
	irqhandler	15		; EZ80_PORTA1_IRQ  15   17   0x084   
	irqhandler	16		; EZ80_PORTA2_IRQ  16   18   0x088   
	irqhandler	17		; EZ80_PORTA3_IRQ  17   19   0x08c   
	irqhandler	18		; EZ80_PORTA4_IRQ  18   20   0x090   
	irqhandler	19		; EZ80_PORTA5_IRQ  19   21   0x094   
	irqhandler	20		; EZ80_PORTA6_IRQ  20   22   0x098   
	irqhandler	21		; EZ80_PORTA7_IRQ  21   23   0x09c   
	irqhandler	22		; EZ80_PORTB0_IRQ  22   24   0x0a0   
	irqhandler	23		; EZ80_PORTB1_IRQ  23   25   0x0a4   
	irqhandler	24		; EZ80_PORTB2_IRQ  24   26   0x0a8   
	irqhandler	25		; EZ80_PORTB3_IRQ  25   27   0x0ac   
	irqhandler	26		; EZ80_PORTB4_IRQ  26   28   0x0b0   
	irqhandler	27		; EZ80_PORTB5_IRQ  27   29   0x0b4   
	irqhandler	28		; EZ80_PORTB6_IRQ  28   20   0x0b8   
	irqhandler	29		; EZ80_PORTB7_IRQ  29   21   0x0bc   
	irqhandler	30		; EZ80_PORTC0_IRQ  30   22   0x0c0   
	irqhandler	31		; EZ80_PORTC1_IRQ  31   23   0x0c4   
	irqhandler	32		; EZ80_PORTC2_IRQ  32   24   0x0c8   
	irqhandler	33		; EZ80_PORTC3_IRQ  33   25   0x0cc   
	irqhandler	34		; EZ80_PORTC4_IRQ  34   26   0x0d0   
	irqhandler	35		; EZ80_PORTC5_IRQ  35   27   0x0d4   
	irqhandler	36		; EZ80_PORTC6_IRQ  36   28   0x0d8   
	irqhandler	37		; EZ80_PORTC7_IRQ  37   29   0x0dc   
	irqhandler	38		; EZ80_PORTD0_IRQ  38   40   0x0e0   
	irqhandler	39		; EZ80_PORTD1_IRQ  39   41   0x0e4   
	irqhandler	40		; EZ80_PORTD2_IRQ  40   42   0x0e8   
	irqhandler	41		; EZ80_PORTD3_IRQ  41   43   0x0ec   
	irqhandler	42		; EZ80_PORTD4_IRQ  42   44   0x0f0   
	irqhandler	43		; EZ80_PORTD5_IRQ  43   45   0x0f4   
	irqhandler	44		; EZ80_PORTD6_IRQ  44   46   0x0f8   
	irqhandler	45		; EZ80_PORTD7_IRQ  45   47   0x0fc   
	irqhandler	EZ80_UNUSED+1	;                       48   0x100   
	irqhandler	EZ80_UNUSED+2	;                       49   0x104   
	irqhandler	EZ80_UNUSED+3	;                       50   0x108   
	irqhandler	EZ80_UNUSED+4	;                       51   0x10c   
	irqhandler	EZ80_UNUSED+5	;                       52   0x110   
	irqhandler	EZ80_UNUSED+6	;                       53   0x114   
	irqhandler	EZ80_UNUSED+7	;                       54   0x118   
	irqhandler	EZ80_UNUSED+8	;                       55   0x11c   
	irqhandler	EZ80_UNUSED+9	;                       56   0x120   
	irqhandler	EZ80_UNUSED+10	;                       57   0x124   
	irqhandler	EZ80_UNUSED+11	;                       58   0x128   
	irqhandler	EZ80_UNUSED+12	;                       59   0x12c   
	irqhandler	EZ80_UNUSED+13	;                       60   0x130   
	irqhandler	EZ80_UNUSED+14	;                       61   0x134   
	irqhandler	EZ80_UNUSED+15	;                       62   0x138   
	irqhandler	EZ80_UNUSED+16	;                       63   0x13c   

;**************************************************************************
; Common Interrupt handler
;**************************************************************************

_ez80_rstcommon:
	; Create a register frame.  SP points to top of frame + 4, pushes
	; decrement the stack pointer.  Already have
	;
	;   Offset 8: Return PC is already on the stack
	;   Offset 7: AF (retaining flags)
	;
	; IRQ number is in A

	push	hl			; Offset 6: HL
	ld	hl, #(3*3)		;    HL is the value of the stack pointer before
	add	hl, sp			;    the interrupt occurred (3 for PC, AF, HL)
	push	hl			; Offset 5: Stack pointer
	push	iy			; Offset 4: IY
	push	ix			; Offset 3: IX
	push	de			; Offset 2: DE
	push	bc			; Offset 1: BC

	; At this point, we know that interrupts were enabled (or we wouldn't be here
	; so we can save a fake indicationn that will cause interrupts to restored when
	; this context is restored

	ld	bc, #EZ80_PV_FLAG	; Parity bit.  1=parity odd, IEF2=1
	push	bc			; Offset 0: I with interrupt state in parity
	di				; (not necessary)

	; Call the interrupt decode logic. SP points to the beggining of the reg structure

	ld	hl, #0			; Argument #2 is the beginning of the reg structure
	add	hl, sp			;
	push	hl			; Place argument #2 at the top of stack
        ld      bc, #0			; BC = reset number
	ld	c, a			;   Save the reset number in C
	push	bc			; Argument #1 is the Reset number
	call	_up_doirq		; Decode the IRQ

	; On return, HL points to the beginning of the reg structure to restore
	; Note that (1) the arguments pushed on the stack are not popped, and (2) the
	; original stack pointer is lost.  In the normal case (no context switch),
	; HL will contain the value of the SP before the arguments were pushed.

	ld	sp, hl			; Use the new stack pointer

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
	pop	de			; Offset 8: Return address
	ld	sp, hl			; Set SP = saved stack pointer value before return
	push	de			; Set up for reti
	exx				; Restore original BC/DE/HL

	; Restore interrupt state

	ex	af, af'			; Recover interrupt state
	jp	po, nointenable		; Odd parity, IFF2=0, means disabled
	ex	af, af'			; Restore AF (before enabling interrupts)
	ei				; yes
	reti
nointenable:
	ex	af, af'			; Restore AF
	reti

;**************************************************************************
; Vector Setup Logic
;**************************************************************************

_ez80_initvectors:
	; Initialize the vector table

	ld	iy, _ez80_vectable
	ld	ix, 4
	ld	bc, 4
	ld	b, NVECTORS
	xor	a, a			; Clear carry
	ld	hl, handlersize
	ld	de, _ez80_handlers
	sbc	hl, de			; Length of irq handler in hl
	ld	d, h
	ld	e, l
	ld	hl, _ez80_handlers 	; Start of handlers in hl

	ld	a, 0
$1:
	ld	(iy), hl		; Store IRQ handler
	ld	(iy+3), a		; Pad to 4 bytes
	add	hl, de			; Point to next handler
	push	de
	ld	de, 4
	add	iy, de			; Point to next entry in vector table
	pop	de
	djnz	$1			; Loop until all vectors have been written

	; Select interrupt mode 2

	im	2			; Interrupt mode 2

	; Write the address of the vector table into the interrupt vector base

	ld	hl, _ez80_vectable >> 8
	ld	i, hl
	ret

;**************************************************************************
; Vector Table
;**************************************************************************
; This segment must be aligned on a 512 byte boundary anywhere in RAM
; Each entry will be a 3-byte address in a 4-byte space

	define	.IVECTS, space = RAM, align = 200h
	segment	.IVECTS

	; The first 64 bytes are not used... the vectors actually start at +0x40
_ez80_vecreserve:
	ds	64
_ez80_vectable:
	ds	NVECTORS * 4
