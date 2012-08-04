/****************************************************************************
 * arch/arm/src/arm/up_nommuhead.S
 *
 *   Copyright (C) 2007, 2009-2010, 2012 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "arm.h"
#include "up_internal.h"
#include "up_arch.h"

/****************************************************************************
 * Macros
 ****************************************************************************/

	/* This macro will modify r0, r1, r2 and r14 */

#ifdef CONFIG_DEBUG
	.macro	showprogress, code
	mov	r0, #\code
	bl	up_lowputc
	.endm
#else
	.macro	showprogress, code
	.endm
#endif

/****************************************************************************
 * OS Entry Point
 ****************************************************************************/

/* We assume the bootloader has already initialized most of the h/w for
 * us and that only leaves us having to do some os specific things
 * below.
 */
	.text
        .global __start
	.type	__start, #function
__start:

	/* First, setup initial processor mode */

	mov	r0, #(SVC_MODE | PSR_I_BIT | PSR_F_BIT )
	msr	cpsr, r0

	showprogress 'A'

	/* Setup system stack (and get the BSS range) */

	adr	r0, LC0
	ldmia   r0, {r4, r5, sp}

	/* Clear system BSS section */

	mov	r0, #0
1:	cmp	r4, r5
	strcc	r0, [r4], #4
	bcc	1b

	showprogress 'B'

	/* Copy system .data sections to new home in RAM. */

#ifdef CONFIG_BOOT_RUNFROMFLASH

	adr	r3, LC2
	ldmia	r3, {r0, r1, r2}

1:	ldmia	r0!, {r3 - r10}
	stmia	r1!, {r3 - r10}
	cmp	r1, r2
	blt	1b

#endif

	/* Perform early serial initialization */

	mov	fp, #0
#ifdef USE_EARLYSERIALINIT
	bl	up_earlyserialinit
#endif

#ifdef CONFIG_DEBUG
	mov	r0, #'C'
	bl	up_putc
	mov	r0, #'\n'
	bl	up_putc
#endif
	/* Initialize onboard LEDs */

#ifdef CONFIG_ARCH_LEDS
	bl	up_ledinit
#endif

	/* Then jump to OS entry */

	b	os_start

	/* Variables:
	 * _sbss is the start of the BSS region (see ld.script)
	 * _ebss is the end of the BSS regsion (see ld.script)
	 * The idle task stack starts at the end of BSS and is
	 * of size CONFIG_IDLETHREAD_STACKSIZE.  The heap continues
	 * from there until the end of memory.  See g_heapbase
	 * below.
	 */

LC0:	.long	_sbss
	.long	_ebss
	.long	_ebss+CONFIG_IDLETHREAD_STACKSIZE-4

#ifdef CONFIG_BOOT_RUNFROMFLASH
LC2:	.long	_eronly	/* Where .data defaults are stored in FLASH */
	.long	_sdata	/* Where .data needs to reside in SDRAM */
	.long	_edata
#endif
	.size	__start, .-__start

	/* This global variable is unsigned long g_heapbase and is
	 * exported from here only because of its coupling to LCO
	 * above.
	 */

	.data
	.align	4
	.globl	g_heapbase
	.type	g_heapbase, object
g_heapbase:
	.long	_ebss+CONFIG_IDLETHREAD_STACKSIZE
	.size	g_heapbase, .-g_heapbase

	.end

