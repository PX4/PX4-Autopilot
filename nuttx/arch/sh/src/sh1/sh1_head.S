/*****************************************************************************
 * arch/sh/src/sh1/sh1_head.S
 *
 *   Copyright (C) 2008-2009, 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>	/* NuttX configuration settings */
#include <arch/board/board.h>	/* Board-specific settings */
#include <arch/irq.h>		/* IRQ definitons */

#include "chip.h"		/* Chip-specific settings */
#include "up_internal.h"
#include "up_arch.h"

/*****************************************************************************
 * Definitions
 *****************************************************************************/

/* This file holds the NuttX start logic that runs when the SH-1/US7032EVB1
 * is reset.  This logic must be located in SRAM at 0x0a00:2000.  On that
 * platform, the entire PROM and the first 8Kb of SRAM are reserved for CMON.
 */

/*****************************************************************************
 * External references
 *****************************************************************************/

/* Called functions */

	.globl	_up_lowsetup		/* Early initialization of UART */
#ifdef USE_EARLYSERIALINIT
	.globl	_up_earlyconsoleinit	/* Early initialization of console driver */
#endif
#ifdef CONFIG_ARCH_LEDS
	.globl	_up_ledinit		/* Boot LED setup */
#endif
#ifdef CONFIG_DEBUG
	.globl	_up_lowputc		/* Low-level debug output */
#endif
	.globl	_os_start		/* NuttX entry point */

/* Variables set up by the linker script */

	.globl	_sbss			/* Start of BSS */
	.globl	_ebss			/* End of BSS */
	.globl	_svect			/* Start of the new vector location */

#ifdef CONFIG_BOOT_RUNFROMFLASH
	.globl	_eronly			/* Where .data defaults are stored in FLASH */
	.global	_sdata			/* Start of .data in RAM */
	.globl	_edata			/* End of .data in RAM */
#endif

/* Interrupt handlers */
	.globl	_up_invalid_handler
#ifdef CONFIG_SH1_DMAC0
	.globl	_up_dmac0_handler
#endif
#ifdef CONFIG_SH1_DMAC1
	.globl	_up_dmac1_handler
#endif
#ifdef CONFIG_SH1_DMAC2
	.globl	_up_dmac2_handler
#endif
#ifdef CONFIG_SH1_DMAC3
	.globl	_up_dmac3_handler
#endif
	.globl	_up_imia0_handler
	.globl	_up_imib0_handler
	.globl	_up_ovi0_handler
#ifdef CONFIG_SH1_ITU1
	.globl	_up_imia1_handler
	.globl	_up_imib1_handler
	.globl	_up_ovi1_handler
#endif
#ifdef CONFIG_SH1_ITU2
	.globl	_up_imia2_handler
	.globl	_up_imib2_handler
	.globl	_up_ovi2_handler
#endif
#ifdef CONFIG_SH1_ITU3
	.globl	_up_imia3_handler
	.globl	_up_imib3_handler
	.globl	_up_ovi3_handler
#endif
#ifdef CONFIG_SH1_ITU4
	.globl	_up_imia4_handler
	.globl	_up_imib4_handler
	.globl	_up_ovi4_handler
#endif
#ifdef CONFIG_SH1_SCI0
	.globl	_up_eri0_handler
	.globl	_up_rxi0_handler
	.globl	_up_txi0_handler
	.globl	_up_tei0_handler
#endif
#ifdef CONFIG_SH1_SCI1
	.globl	_up_eri1_handler
	.globl	_up_rxi1_handler
	.globl	_up_txi1_handler
	.globl	_up_tei1_handler
#endif
#ifdef CONFIG_SH1_PCU
	.globl	_up_pei_handler
#endif
#ifdef CONFIG_SH1_AD
	.globl	_up_aditi_handler
#endif
#ifdef CONFIG_SH1_WDT
	.globl	_up_wdt_handler
#endif
#ifdef CONFIG_SH1_CMI
	.globl	_up_cmi_handler
#endif

/*****************************************************************************
 * Macros
 *****************************************************************************/

/*****************************************************************************
 * Name: showprogress
 *
 * Description:
 *   Print a character on the UART to show boot status. This macro will
 *   modify r0, r1, r2 and r14
 *
 *****************************************************************************/

	.macro	showprogress, code
#ifdef CONFIG_DEBUG
	mov.l   .Llowputc, r0		/* Address of up_earlyconsoleinit */
	jsr     @r0			/* Call it */
	mov	#\code, r4		/* Delay slot */
#endif
	.endm

/*****************************************************************************
 * Vectors
 *****************************************************************************/

	.section	.vects

/*****************************************************************************
 * Name: __vector_table
 *
 * Description:
 *   Interrupt vector table.  The actual vectors are managed by CMON.  For
 *   any non-zero settings in the following table, CMON will redirect interrupt
 *   handling to that function.
 *
 *****************************************************************************/

	.globl		__vector_table
	.type		__vector_table, %object
__vector_table:
	/* All of the SH-1 common vectors are copied from the CMON vector
	 * area to here.  As a result, CMON will continue to intercept these
	 * vectors.
	 */

	.long	__start		/* 0-1: Power-on reset (hard, NMI high) PC & SP */
	.long	_ebss+CONFIG_IDLETHREAD_STACKSIZE-4
	.long	__start		/* 2-3: Manual reset (soft, NMI low) PC & SP */
	.long	_ebss+CONFIG_IDLETHREAD_STACKSIZE-4

	.rept SH1_NCMN_VECTORS-4
	.long	_up_invalid_handler
	.endr

	/* The remaining vectors are unique to the SH-1 703x family */

#ifdef CONFIG_SH1_DMAC0
	.long	_up_dmac0_handler		/* 72: DMAC0 DEI0 */
#else
	.long	_up_invalid_handler		/* 72: DMAC0 DEI0 */
#endif
	.long	_up_invalid_handler		/* 73: Reserved */
#ifdef CONFIG_SH1_DMAC1
	.long	_up_dmac1_handler		/* 74: DMAC1 DEI1 */
#else
	.long	_up_invalid_handler		/* 74: DMAC1 DEI1 */
#endif
	.long	_up_invalid_handler		/* 75: Reserved */
#ifdef CONFIG_SH1_DMAC2
	.long	_up_dmac2_handler		/* 76: DMAC2 DEI2 */
#else
	.long	_up_invalid_handler		/* 76: DMAC2 DEI2 */
#endif
	.long	_up_invalid_handler		/* 77: Reserved */
#ifdef CONFIG_SH1_DMAC3
	.long	_up_dmac3_handler		/* 78: DMAC3 DEI3 */
#else
	.long	_up_invalid_handler		/* 78: DMAC3 DEI3 */
#endif
	.long	_up_invalid_handler		/* 79: Reserved */
	.long	_up_imia0_handler		/* 80: ITU0 IMIA0 */
	.long	_up_imib0_handler		/* 81:      IMIB0 */
	.long	_up_ovi0_handler		/* 82:      OVI0 */
	.long	_up_invalid_handler		/* 83: Reserved */
#ifdef CONFIG_SH1_ITU1
	.long	_up_imia1_handler		/* 84: ITU1 IMIA1 */
	.long	_up_imib1_handler		/* 85:      IMIB1 */
	.long	_up_ovi1_handler		/* 86:      OVI1 */
#else
	.long	_up_invalid_handler		/* 84: ITU1 IMIA1 */
	.long	_up_invalid_handler		/* 85:      IMIB1 */
	.long	_up_invalid_handler		/* 86:      OVI1 */
#endif
	.long	_up_invalid_handler		/* 87: Reserved */
#ifdef CONFIG_SH1_ITU2
	.long	_up_imia2_handler		/* 88: ITU2 IMIA2 */
	.long	_up_imib2_handler		/* 89:      IMIB2 */
	.long	_up_ovi2_handler		/* 90:      OVI2 */
#else
	.long	_up_invalid_handler		/* 88: ITU2 IMIA2 */
	.long	_up_invalid_handler		/* 89:      IMIB2 */
	.long	_up_invalid_handler		/* 90:      OVI2 */
#endif
	.long	_up_invalid_handler		/* 91: Reserved */
#ifdef CONFIG_SH1_ITU3
	.long	_up_imia3_handler		/* 92: ITU3 IMIA3 */
	.long	_up_imib3_handler		/* 93:      IMIB3 */
	.long	_up_ovi3_handler		/* 94:      OVI3 */
#else
	.long	_up_invalid_handler		/* 92: ITU3 IMIA3 */
	.long	_up_invalid_handler		/* 93:      IMIB3 */
	.long	_up_invalid_handler		/* 94:      OVI3 */
#endif
	.long	_up_invalid_handler		/* 95: Reserved */
#ifdef CONFIG_SH1_ITU4
	.long	_up_imia4_handler		/* 96: ITU4 IMIA4 */
	.long	_up_imib4_handler		/* 97:      IMIB4 */
	.long	_up_ovi4_handler		/* 98:      OVI4 */
#else
	.long	_up_invalid_handler		/* 96: ITU4 IMIA4 */
	.long	_up_invalid_handler		/* 97:      IMIB4 */
	.long	_up_invalid_handler		/* 98:      OVI4 */
#endif
	.long	_up_invalid_handler		/* 99: Reserved */
#ifdef CONFIG_SH1_SCI0
	.long	_up_eri0_handler		/* 100: SCI0 ERI0 */
	.long	_up_rxi0_handler		/* 101:      RxI0 */
	.long	_up_txi0_handler		/* 102:      TxI0 */
	.long	_up_tei0_handler		/* 103:      TEI0 */
#else
	.long	_up_invalid_handler		/* 100: SCI0 ERI0 */
	.long	_up_invalid_handler		/* 101:      RxI0 */
	.long	_up_invalid_handler		/* 102:      TxI0 */
	.long	_up_invalid_handler		/* 103:      TEI0 */
#endif
#ifdef CONFIG_SH1_SCI1
	.long	_up_eri1_handler		/* 104: SCI1 ERI1 */
	.long	_up_rxi1_handler		/* 105:      RxI1 */
	.long	_up_txi1_handler		/* 106:      TxI1 */
	.long	_up_tei1_handler		/* 107:      TEI1 */
#else
	.long	_up_invalid_handler		/* 104: SCI1 ERI1 */
	.long	_up_invalid_handler		/* 105:      RxI1 */
	.long	_up_invalid_handler		/* 106:      TxI1 */
	.long	_up_invalid_handler		/* 107:      TEI1 */
#endif
#ifdef CONFIG_SH1_PCU
	.long	_up_pei_handler			/* 108: Parity control unit PEI */
#else
	.long	_up_invalid_handler		/* 108: Parity control unit PEI */
#endif
#ifdef CONFIG_SH1_AD
	.long	_up_aditi_handler		/* 109: A/D ITI */
#else
	.long	_up_invalid_handler		/* 109: A/D ITI */
#endif
	.long	_up_invalid_handler		/* 110: Reserved */
	.long	_up_invalid_handler		/* 111: Reserved */
#ifdef CONFIG_SH1_WDT
	.long	_up_wdt_handler			/* 112: WDT ITI */
#else
	.long	_up_invalid_handler		/* 112: WDT ITI */
#endif
#ifdef CONFIG_SH1_CMI
	.long	_up_cmi_handler			/* 113: REF CMI */
#else
	.long	_up_invalid_handler		/* 113: REF CMI */
#endif
	.rept (SH1_LAST_VNDX-SH1_CMI_VNDX)	/* 114-255: Reserved */
	.long	_up_invalid_handler
	.endr
	.size	__vector_table, . - __vector_table

/*****************************************************************************
 * Text
 *****************************************************************************/

	.section	.reset

/*****************************************************************************
 * Name: __start
 *
 * Description:
 *   Reset entry point.  This is the first function to execute when the
 *   processor is reset.  It initializes hardware and then gives control to
 *   NuttX.  Nearly all SH-1 resources have already been setup by CMON so all
 *   that is necessary for us to do here is setup the stack pointer and BSS.
 *
 *****************************************************************************/

	.global __start
	.type	__start, #function

__start:
	/* Initialize stack pointer to the preallocated stack */
	mov.l	.Lstack, r15
	
	/* set up the bus controller for the EVB */

	mov.l	.Lwcr1, r0
	sub	r1,r1
	mov.w	r1, @r0

	/* Configure the BSR to use /LBS, /HBS, /WR */

	mov.l   .Lbcr, r0
	mov.w   .Lbas, r1
	bra     __start0
	mov.w   r1, @r0
	
	.align 2
.Lstack:
	.long	_ebss+CONFIG_IDLETHREAD_STACKSIZE-4
.Lwcr1:
	.long	0x5ffffa2
.Lbcr:
	.long  0x5ffffa0
.Lbas:
	.word  0x0800

__start0:
	/* Copy the monitor vectors to a002000-a00211f */

	mov	#0, r0			/* R0: Monitor vector table at address 0 in PROM */
	mov.l	.Lsvect, r1		/* R1: Redirected vector table in SRAM */
	mov.l	.Lvectend, r3		/* R3: Copy only up to external interrupts */
1:
	mov.l	@r0, r2			/* R2: Value from mnitor monitor vector table */
	mov.l	r2, @r1			/* Write into SRAM vector table */
	add	#4, r0			/* R0: Address of next vector to read from monitor vector table */
	add	#4, r1			/* R1: Address of next vector to write to SRAM vector table */
	cmp/gt	r0, r3			/* Copy only only up to external interrupts at */
	bt	1b			/* Continue looping until all copied */
	nop				/* Delay slot */

	/* Update the VBR to show new adddress of vector table */

	mov.l	.Lsvect, r0		/* R0: Address of SRAM vector table */
	ldc	r0, vbr			/* Set VBR to start of SRAM vector table */

	/* Initialize data segement */

#ifdef CONFIG_BOOT_RUNFROMFLASH
	mov.l	.Lsdata, r0		/* R0: Start of .data segment */
	mov.l	.Ledata, r1		/* R1: End+1 of .data segment */
	mov.l	.Leronly, r2		/* R2: Start of FLASH .data segment copy */
2:
	mov.l	@r2, r3			/* R3: Next byte from FLASH copy */
	mov.l	r3, @r0			/* Copy to .data */
	add	#4, r2			/* R2: Address of next byte to read from FLASH */
	add	#4, r0			/* R0: Address to write next byte to .data */
	cmp/gt	r0, r1			/* End of .data? */
	bt	2b			/* Loop until end of data */
	nop				/* Delay slot */
#endif

	/* Clear BSS */

	mov.l   .Lsbss, r0		/* R0: Start of BSS segment */
	mov.l   .Lebss, r1		/* R1: End+1 of BSS segment */
	mov	#0, r2			/* R2: Value = 0 */
3:
	mov.l	r2, @r0			/* Clear the next word in BSS */
	add	#4, r0			/* R0: Address of next byte to clear in BSS */
	cmp/ge	r0, r1			/* End of BSS? */
	bt	3b			/* Loop until the end of BSS */
	nop				/* Delay slot */

	/* Configure the uart so that we can get debug output as soon
	 * as possible.
	 */

	mov.l   .Llowsetup, r0		/* Address of up_lowsetup */
	jsr     @r0			/* Call it */
	or      r0, r0			/* Delay slot */

	showprogress 'A'

	/* Perform early console initialization */

#ifdef USE_EARLYSERIALINIT
	mov.l   .Learlyconsole, r0	/* Address of up_earlyconsoleinit */
	jsr     @r0			/* Call it */
	or      r0, r0			/* Delay slot */
#endif

	showprogress 'B'

	/* Call C++ constructors */

#ifdef CONFIG_CPLUSPLUS
#  warning "No C++ support yet"
	showprogress 'C'
#endif
	showprogress '\n'

	/* Initialize onboard LEDs */

#ifdef CONFIG_ARCH_LEDS
	mov.l   .Lledinit, r0		/* Address of up_ledinit */
	jsr     @r0			/* Call it */
	or      r0, r0			/* Delay slot */
#endif

	/* Then jump to NuttX entry */

	mov.l   .Losstart,r0
	jsr     @r0
	or      r0, r0

	/* Shouldn't get here */

	/* Call destructors  -- never get here */

#ifdef CONFIG_CPLUSPLUS
#  warning "No C++ support yet"
#endif

4:  nop
	bra 4b
	nop

	.align 2
#ifdef CONFIG_BOOT_RUNFROMFLASH
.Leronly:
	.long   _eronly
.Lsdata:
	.long   _sdata
.Ledata:
	.long   _edata
#endif
.Lsbss:  
	.long   _sbss
.Lebss:
	.long   _ebss
#ifdef USE_EARLYSERIALINIT
.Learlyconsole:
	.long	_up_earlyconsoleinit
#endif
.Llowsetup:
	.long	_up_lowsetup
#ifdef CONFIG_DEBUG
.Llowputc:
	.long	_up_lowputc
#endif
.Lledinit:
	.long	_up_ledinit
.Losstart:
	.long   _os_start
.Lsvect:
	.long   _svect
.Lvectend:
	.long   ((4*SH1_NCMN_VECTORS)-1)
	.size	__start, .-__start

/*****************************************************************************
 * DATA
 *****************************************************************************/

	.section	.data

	/* This global variable is unsigned long g_heapbase and is
	 * exported from here only because of its coupling to the stack
	 * above.
	 */

	.data
	.align	4
	.globl	_g_heapbase
	.type	_g_heapbase, object
_g_heapbase:
	.long	_ebss+CONFIG_IDLETHREAD_STACKSIZE
	.size	_g_heapbase, .-_g_heapbase

	.end

