;**************************************************************************
; arch/z80/src/z180/z180_vectors.asm
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
;**************************************************************************

	.title	NuttX for the Z180
	.module	z180_vectors

;**************************************************************************
; Constants
;**************************************************************************

;**************************************************************************
; Global symbols used
;**************************************************************************

	.globl	_up_int1		; Vector offset 0: External /INT1
	.globl	_up_int2		; Vector offset 2: External /INT2
	.globl	_up_prt0		; Vector offset 4: PRT channel 0
	.globl	_up_prt1		; Vector offset 6: PRT channel 1
	.globl	_up_dma0		; Vector offset 8: DMA channel 0
	.globl	_up_dma1		; Vector offset 8: DMA channel 1
	.globl	_up_csio		; Vector offset 12: Clocked serial I/O
	.globl	_up_asci0		; Vector offset 14: Async channel 0
	.globl	_up_asci1		; Vector offset 16: Async channel 1
	.globl	_up_unused		; Vector offset 18: Unused

;**************************************************************************
; Interrupt Vector Table
;**************************************************************************

; The Vector Table is located at address 0x0040, between the RST vectors
; and the NMI vector

	.area	_VECTORS (ABS)
	.org	0x0040

_up_vectors::
	.dw		_up_int1		; Vector offset 0: External /INT1
	.dw		_up_int2		; Vector offset 2: External /INT2
	.dw		_up_prt0		; Vector offset 4: PRT channel 0
	.dw		_up_prt1		; Vector offset 6: PRT channel 1
	.dw		_up_dma0		; Vector offset 8: DMA channel 0
	.dw		_up_dma1		; Vector offset 8: DMA channel 1
	.dw		_up_csio		; Vector offset 12: Clocked serial I/O
	.dw		_up_asci0		; Vector offset 14: Async channel 0
	.dw		_up_asci1		; Vector offset 16: Async channel 1
	.dw		_up_unused		; Vector offset 18: Unused
	.dw		_up_unused		; Vector offset 20: Unused
	.dw		_up_unused		; Vector offset 22: Unused
	.dw		_up_unused		; Vector offset 24: Unused
	.dw		_up_unused		; Vector offset 26: Unused
	.dw		_up_unused		; Vector offset 28: Unused
	.dw		_up_unused		; Vector offset 30: Unused