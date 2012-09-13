/*************************************************************************
 * arch/z16/src/z16f/z16f_restoreusercontext.asm
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSeqUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *************************************************************************/

/*************************************************************************
 * Included Files
 *************************************************************************/

#include <nuttx/config.h>
#include <arch/irq.h>
#include "chip/chip.h"

/*************************************************************************
 * External References / External Definitions
 *************************************************************************/

	xdef	_up_restoreusercontext

/*************************************************************************
 * Data Allocation
 *************************************************************************/

	define CODESEG, SPACE=EROM
	segment CODESEG

/*************************************************************************
 * Code
 *************************************************************************/

/*************************************************************************
 * Name: up_restoreusercontext
 *
 * Description:
 *   Restore the current user context.
 *
 * Parameters:
 *   r1: pointer to the register save array in the XCPT structure
 *
 *************************************************************************/

_up_restoreusercontext:
	/* Disable interrupts while we are accessing the TCB's register
	 * save structure.  The correct interrupt state will be retored by
	 * the iret instruction below
	 */

	di

	/* Set up the stack for return from 'interrupt' */

	ld	sp, 2*REG_SP(r1)			/* sp=Value of SP on return */
	ld.w	r2, 2*REG_FLAGS(r1)			/* r1=padded flags value */
	push.w	r2					/* Push padded flags value onto the stack */
	ld	r2, 2*REG_PC(r1)			/* r2=return address */
	push	r2					/* Push the return address onto the stack */
	ld	r7, 2*REG_R7(r1)			/* r7=saved value of r7 */
	push	r7					/* Save this so that we can use r7 */
	ld	r7, sp					/* r7=saved SP value */

	/* Restore registers and return from interrupt */

	ld	sp, r1					/* sp=Pointer to register save structure */
	popmhi	<r8-r14>				/* Restore r8-r14 */
	add	sp, #4					/* Skip over restore of r15=sp */
	popmlo	<r0-r6>					/* Restore r0-r6 */
	ld	sp, r7					/* Switch to the correct stack for return */
	pop	r7					/* Recover the saved value of r7 from the stack */
	iret2						/* Return from interrupt */

	end


