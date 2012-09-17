/************************************************************************************
 * arch/sh/src/m16c/m16c_head.S
 *
 *	 Copyright (C) 2009, 2012 Gregory Nutt. All rights reserved.
 *	 Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *		notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *		notice, this list of conditions and the following disclaimer in
 *		the documentation and/or other materials provided with the
 *		distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *		used to endorse or promote products derived from this software
 *		without specific prior written permission.
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip.h"
#include "up_internal.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Macro Definitions
 ************************************************************************************/

/*****************************************************************************
 * Name: showprogress
 *
 * Description:
 *   Print a character on the UART to show boot status.
 *
 *****************************************************************************/

	.macro	showprogress, code
#ifdef CONFIG_DEBUG
	.globl	_up_lowputc
	mov.b	r#\code1l		/* Character to print */
	jsr.a	_up_lowputc		/* Print it */
#endif
	.endm

/************************************************************************************
 * Data
 ************************************************************************************/

/* The near RAM memory map is as follows:
 * 
 * 0x00400 - DATA		Size: Determined by linker
 *           BSS		Size: Determined by linker
 *           Interrupt stack	Size: CONFIG_ARCH_INTERRUPTSTACK
 *           Idle stack		Size: CONFIG_IDLETHREAD_STACKSIZE
 *           Heap		Size: Everything remaining
 * 0x00bff - (end+1)
 */

	.section .rodata
	.globl	_g_snbss
	.type	_g_snbss, object
_g_snbss:
	.word	_g_snbss
	.size	_g_snbss, .-_g_snbss

	.globl	_g_enbss
	.type	_g_enbss, object
_g_enbss:
	.word	_g_enbss
	.size	_g_enbss, .-_g_enbss

	.globl	_g_sndata
	.type	_g_sndata, object
_g_sndata:
	.word	_g_sndata
	.size	_g_sndata, .-_g_sndata

	.globl	_g_endata
	.type	_g_endata, object
_g_endata:
	.word	_g_endata
	.size	_g_endata, .-_g_endata

	.globl	_g_enronly
	.type	_g_enronly, object
_g_enronly:
	.long	_g_enronly
	.size	_g_enronly, .-_g_efronly

#ifdef CONFIG_M16C_HAVEFARRAM
	.globl	_g_sfbss
	.type	_g_sfbss, object
_g_sfbss:
	.long	_g_sfbss
	.size	_g_sfbss, .-_g_sfbss

	.globl	_g_efbss
	.type	_g_efbss, object
_g_efbss:
	.long	_g_efbss
	.size	_g_efbss, .-_g_efbss

	.globl	_g_sfdata
	.type	_g_sfdata, object
_g_sfdata:
	.long	_g_sfdata
	.size	_g_sfdata, .-_g_sfdata

	.globl	_g_efdata
	.type	_g_efdata, object
_g_efdata:
	.long	_g_efdata
	.size	_g_efdata, .-_g_efdata

	.globl	_g_efronly
	.type	_g_efronly, object
_g_efronly:
	.long	_g_efronly
	.size	_g_efronly, .-_g_efronly
#endif
	.globl	_g_svarvect
	.type	_g_svarvect, object
_g_svarvect:
	.long	_svarvect
	.size	_g_svarvect, .-_g_svarvect

	.globl	_g_heapbase
	.type	_g_heapbase, object
_g_heapbase:
	.long	_enbss+CONFIG_ARCH_INTERRUPTSTACK+CONFIG_IDLETHREAD_STACKSIZE
	.size	_g_heapbase, .-_g_heapbase

/************************************************************************************
 * Code
 ************************************************************************************/

/************************************************************************************
 * Name: __start
 *
 * Description:
 *   After reset, program execution starts here.
 *
 ************************************************************************************/

 	.text
	.globl	__start
	.globl	_enbss
	.type	__start, #function
__start:

/* Set the interrupt and user stack pointers */

	mov.w	#_enbss, R0
	ldc	R0, isp			/* Set the interrupt stack pointer to the end of BSS */
	add.w	#CONFIG_IDLETHREAD_STACKSIZE, R0
	fset	U			/* Set bit 7 (U) to select the user stack pointer */
	ldc	R0, sp			/* Set the user stack pointer */

/* Set BCLK speed. At reset, the processor clock  (BLCK) defaults to a divisor of 8.
 * This sets clock to F1 (divide by 1) on XIN: BCLK = XIN frequency.
 */

 	mov.b	#0x01, M16C_PRCR	/* Unprotect CM0 to change clock setting */
	mov.b	#0x08, M16C_CM0		/* enable CM17 and CM16 to set BCLK to F1
					 * CM17 & CM16 defaults to 0 after reset and
					 * so we only need to reset CM06 to 0 */
	mov.b	#0x00,M16C_PRCR		/* protect CM0 */

/* The two MS bits of the interrupt cause select register must be set to
 * enable the use of INT4 and INT5
 */

 	mov.b	#0xc0, M16C_IFSR	/* Set b7 & b6 if application will use INT4 & INT5 */
	ldc	#M16C_IRAM_BASE, sb	/* Set sb register (to what?) */

/* Set up INTB to point to location of variable vector table */

	mov.w	_g_svarvect, r0		/* R0 = lower 16-bits */
	mov.w	_g_svarvect+2, r1	/* R1 = upper 4-bits */
	ldc	r1, intbh
	ldc	r0, intbl

/* Configure the uart so that we can get debug output as soon as possible. */

	.globl	_up_lowsetup		/* Early initialization of UART */
	jsr.a	_up_lowsetup
	showprogress 'A'

/* Clear near .bss sections */

	mov.b	#0x00, r0l		/* r0l: 0 */
	mov.w	_g_snbss, a1		/* a1: start of near .bss */
	mov.w	_g_enbss, r3		/* r3: end of near .bss */
	sub.w	a1, r3			/* r3: size of near .bss */
	sstr.b				/* Clear near .bss */

/* Clear far .bss sections */

	showprogress 'B'

#ifdef CONFIG_M16C_HAVEFARRAM
# warning "Far RAM support not implemented"
#endif

/* Initialize near .data sections (.rodata is not moved) */

	mov.w	_g_enronly, a0		/* a0: Low 16 bits of source address */
	mov.b	_g_enronly+2, r1h	/* 4 MS of 20-bit source address */
	mov.w	_g_sndata, a1		/* a1: start of near .data */
	mov.w	_g_endata, r3		/* r3: end of near .data */
	sub.w	a1, r3			/* r3: size of near .data */
	smovf.b				/* Copy source to near .data */

/* Initialize far .data sections (.rodata is not moved) */

	showprogress 'C'

#ifdef CONFIG_M16C_HAVEFARRAM
# warning "Far RAM support not implemented"
#endif

/* Perform early console initialization */

#ifdef USE_EARLYSERIALINIT
	.globl	_up_earlyconsoleinit	/* Early initialization of console driver */
	jsr.a	 _up_earlyconsoleinit	/* Call it */
	showprogress 'D'
#endif

/* Call C++ constructors */

#ifdef CONFIG_CPLUSPLUS
#  warning "No C++ support yet"
	showprogress 'E'
#endif

/* Initialize onboard LEDs */

#ifdef CONFIG_ARCH_LEDS
	.globl	_up_ledinit		/* Boot LED setup */
	jsr.a	_up_ledinit		/* Call it */
#endif
	showprogress '\n'

/* Pass control to NuttX */

	.globl	_os_start
	jsr.a	_os_start

/* NuttX will not return, but just in case... */

_os_exit:
	jmp.s	_os_exit
	.size	__start, .-__start
	.end
