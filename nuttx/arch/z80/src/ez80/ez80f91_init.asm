;**************************************************************************
; arch/z80/src/ez80/ez80f91_init.asm
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

	include "ez80F91.inc"

;**************************************************************************
; Constants
;**************************************************************************

;PLL_DIV_L    EQU %5C
;PLL_DIV_H    EQU %5D
;PLL_CTL0     EQU %5E
;PLL_CTL1     EQU %5F

OSC           EQU 0
PLL           EQU 1
RTC           EQU 2

CLK_MUX_OSC   EQU %00
CLK_MUX_PLL   EQU %01
CLK_MUX_RTC   EQU %02

CHRP_CTL_0    EQU %00
CHRP_CTL_1    EQU %40
CHRP_CTL_2    EQU %80
CHRP_CTL_3    EQU %C0

LDS_CTL_0     EQU %00
LDS_CTL_1     EQU %04
LDS_CTL_2     EQU %08
LDS_CTL_3     EQU %0C

LCK_STATUS    EQU %20
INT_LOCK      EQU %10
INT_UNLOCK    EQU %08
INT_LOCK_EN   EQU %04
INT_UNLOCK_EN EQU %02
PLL_ENABLE    EQU %01

;**************************************************************************
; Global symbols used
;**************************************************************************

; Exported symbols
	xdef _ez80_init
	xdef _ez80_initsysclk

; Imported symbols
	xref __CS0_LBR_INIT_PARAM
	xref __CS0_UBR_INIT_PARAM
	xref __CS0_CTL_INIT_PARAM
	xref __CS1_LBR_INIT_PARAM
	xref __CS1_UBR_INIT_PARAM
	xref __CS1_CTL_INIT_PARAM
	xref __CS2_LBR_INIT_PARAM
	xref __CS2_UBR_INIT_PARAM
	xref __CS2_CTL_INIT_PARAM
	xref __CS3_LBR_INIT_PARAM
	xref __CS3_UBR_INIT_PARAM
	xref __CS3_CTL_INIT_PARAM
	xref __CS0_BMC_INIT_PARAM
	xref __CS1_BMC_INIT_PARAM
	xref __CS2_BMC_INIT_PARAM
	xref __CS3_BMC_INIT_PARAM
	xref __FLASH_CTL_INIT_PARAM
	xref __FLASH_ADDR_U_INIT_PARAM
	xref __RAM_CTL_INIT_PARAM
	xref __RAM_ADDR_U_INIT_PARAM
	xref _SYS_CLK_SRC
	xref _SYS_CLK_FREQ
	xref _OSC_FREQ
	xref _OSC_FREQ_MULT
	xref __PLL_CTL0_INIT_PARAM

;**************************************************************************
; Chip-specific initialization logic
;**************************************************************************
; Minimum default initialization for eZ80F91

	define	.STARTUP, space = ROM
	segment	.STARTUP
	.assume	ADL = 1

_ez80_init:
	; Disable internal peripheral interrupt sources

	ld	a, %ff
	out0	(PA_DDR), a		; GPIO
	out0	(PB_DDR), a
	out0	(PC_DDR), a
	out0	(PD_DDR), a
	ld	a, %00
	out0	(PA_ALT1), a
	out0	(PB_ALT1), a
	out0	(PC_ALT1), a
	out0	(PD_ALT1), a
	out0	(PA_ALT2), a
	out0	(PB_ALT2), a
	out0	(PC_ALT2), a
	out0	(PD_ALT2), a
	out0	(PLL_CTL1), a		; PLL
	out0	(TMR0_IER), a		; timers
	out0	(TMR1_IER), a
	out0	(TMR2_IER), a
	out0	(TMR3_IER), a
	out0	(UART0_IER), a		; UARTs
	out0	(UART1_IER), a
	out0	(I2C_CTL), a		; I2C
	out0	(EMAC_IEN), a		; EMAC
	out0	(FLASH_IRQ), a		; Flash
	ld	a, %04
	out0	(SPI_CTL), a		; SPI
	in0	a, (RTC_CTRL)		; RTC,
	and	a, %be
	out0	(RTC_CTRL), a

	; Configure external memory/io

	ld	a, __CS0_LBR_INIT_PARAM
	out0	(CS0_LBR), a
	ld	a, __CS0_UBR_INIT_PARAM
	out0	(CS0_UBR), a
	ld	a, __CS0_BMC_INIT_PARAM
	out0	(CS0_BMC), a
	ld	a, __CS0_CTL_INIT_PARAM
	out0	(CS0_CTL), a

	ld	a, __CS1_LBR_INIT_PARAM
	out0	(CS1_LBR), a
	ld	a, __CS1_UBR_INIT_PARAM
	out0	(CS1_UBR), a
	ld	a, __CS1_BMC_INIT_PARAM
	out0	(CS1_BMC), a
	ld	a, __CS1_CTL_INIT_PARAM
	out0	(CS1_CTL), a

	ld	a, __CS2_LBR_INIT_PARAM
	out0	(CS2_LBR), a
 	ld	a, __CS2_UBR_INIT_PARAM
	out0	(CS2_UBR), a
	ld	a, __CS2_BMC_INIT_PARAM
	out0	(CS2_BMC), a
	ld	a, __CS2_CTL_INIT_PARAM
	out0	(CS2_CTL), a

	ld	a, __CS3_LBR_INIT_PARAM
	out0	(CS3_LBR), a
	ld	a, __CS3_UBR_INIT_PARAM
	out0	(CS3_UBR), a
	ld	a, __CS3_BMC_INIT_PARAM
	out0	(CS3_BMC), a
	ld	a, __CS3_CTL_INIT_PARAM
	out0	(CS3_CTL), a

	; Enable internal memory

	ld	a, __FLASH_ADDR_U_INIT_PARAM
	out0	(FLASH_ADDR_U), a
	ld	a, __FLASH_CTL_INIT_PARAM
	out0	(FLASH_CTRL), a

	ld	a, __RAM_ADDR_U_INIT_PARAM
	out0	(RAM_ADDR_U), a
	ld	a, __RAM_CTL_INIT_PARAM
	out0	(RAM_CTL), a
	ret

;*****************************************************************************
; eZ80F91 System Clock Initialization
;*****************************************************************************

_ez80_initsysclk:
	; check if the PLL should be used
	ld	a, (_ez80_sysclksrc)
	cp	a, PLL
	jr	nz, _ez80_initsysclkdone

	; Load PLL divider

	ld	a, (_ez80_oscfreqmult)		;CR 6202
	out0	(PLL_DIV_L), a
	ld	a, (_ez80_oscfreqmult+1)
	out0	(PLL_DIV_H), a
	
	; Set charge pump and lock criteria

	ld	a, __PLL_CTL0_INIT_PARAM
	and	a, %CC  ; mask off reserved and clock source bits
	out0	(PLL_CTL0), a

	; Enable PLL

	in0	a, (PLL_CTL1)
	set	0, a
	out0	(PLL_CTL1), a

	; Wait for PLL to lock
_ez80_initsysclkwait:
	in0	a, (PLL_CTL1)
	and	a, LCK_STATUS
	cp	a, LCK_STATUS
	jr	nz, _ez80_initsysclkwait

	; Select PLL as system clock source

	ld	a, __PLL_CTL0_INIT_PARAM
	set	0, a
	out0	(PLL_CTL0), a

_ez80_initsysclkdone:
	ret

;_ez80_oscfreq:
;	dl _OSC_FREQ
_ez80_oscfreqmult:
	dw _OSC_FREQ_MULT
;_ez80_sysclkfreq:
;	dl _SYS_CLK_FREQ
_ez80_sysclksrc:
	db _SYS_CLK_SRC
	end
