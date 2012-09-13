/****************************************************************************
 * arch/avr32/src/avr32/up_nommuhead.S
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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
#include <arch/avr32/avr32.h>

#include "up_internal.h"
#include "up_arch.h"

/****************************************************************************
 * External Symbols
 ****************************************************************************/

	.file		"up_nommuhead.S"
	.global		_sbss			/* Start of .bss.  Defined by ld.script */
	.global		_ebss			/* End of .bss.  Defined by ld.script */
#ifdef CONFIG_BOOT_RUNFROMFLASH
	.global		_sdata			/* Start of .data section in RAM */
	.global		_edata			/* End of .data section in RAM */
	.global		_eronly			/* Start of .data section in FLASH */
#endif
	.global		up_lowinit		/* Perform low level initialization */
	.global		os_start		/* NuttX entry point */
	.global		vectortab		/* Vector base address */

/****************************************************************************
 * Macros
 ****************************************************************************/

/****************************************************************************
 * OS Entry Point
 ****************************************************************************/

/* The following entry point must be linked to execute at 0x80000000 if it
 * is to execute upon reset.
 */
	.text
	.global 	__start
	.type		__start, @function
__start:

	/* Set the IDLE thread stack pointer.  This stack will be used
	 * through NuttX initialization and will, eventually, be inherited
	 * by the IDLE thread when threading is enabled.
	 */

	lddpc		sp, .Lstackbase

	/* Set up the vector base address so interrupts can be enabled. */

	lda.w		r0, vectortab
	mtsr		AVR32_EVBA, r0

	/* Enable exception processing */

	csrf		AVR32_SR_EM_SHIFT

	/* Clear system BSS section */

	lda.w		r0, _sbss		/* R =Start of .bss */
	lda.w		r1, _ebss		/* r1=End of .bss */
	mov			r2, 0			/* Value to write to .bss */
	rjmp		2f				/* Start at the bottom of the loop */
1:
	st.d		r0++, r2		/* Zero .bss */
2:
	cp			r0, r1			/* Finished? */
	brlo		1b				/* No... keep looping */

	/* Copy system .data sections to new home in RAM. */

#ifdef CONFIG_BOOT_RUNFROMFLASH

	lda.w		r0, _sdata		/* r0=Start of .data section in RAM */
	lda.w		r1, _edata		/* r1=End of .data section in RAM */
	lda.w 		r2, _eronly		/* r2=Start of .data in FLASH */
	rjmp		4f				/* Start at the bottom of the loop */
3:
	ld.d		r4, r2++		/* Fetch the next data value */
	st.d		r0++, r4		/* Write it to the .data section */
4:
	cp			r0, r1			/* Finished? */
	brlo		3b				/* No... keep looping */

#endif

	/* Clear the frame pointer and link register since this is the outermost
	 * frame.
	 */

	mov			r7, 0
	mov			lr, 0

	/* Perform low-level initialization */

	mcall		.Lup_lowinit

	/* Then jump to OS entry (will not return) */

	lda.w		pc, os_start

.Lstackbase:
	.word	_ebss+CONFIG_IDLETHREAD_STACKSIZE-4
.Lup_lowinit:
	.word	up_lowinit
	.size	__start, .-__start

	/* This global variable is unsigned long g_heapbase and is
	 * exported from here only because of its coupling to other
	 * uses of _ebss in this file
	 */

	.data
	.align	4
	.globl	g_heapbase
	.type	g_heapbase, object
g_heapbase:
	.long	_ebss+CONFIG_IDLETHREAD_STACKSIZE
	.size	g_heapbase, .-g_heapbase
	.end

