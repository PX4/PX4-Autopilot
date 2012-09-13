/**************************************************************************
 * arch/z80/src/z8/z8_saveusercontext.S
 * Save the state of the current user thread
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
 * "AS IS" AND ANY EXPRESS or IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER or CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, or CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS or SERVICES; LOSS
 * OF USE, DATA, or PROFITS; or BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, or TORT (INCLUDING NEGLIGENCE or OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 **************************************************************************/

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <nuttx/config.h>
#include <arch/irq.h>
#include <ez8.inc>

/**************************************************************************
 * Definitions
 **************************************************************************/

	xdef	_z8_restorecontext

/**************************************************************************
 * Code
 **************************************************************************/

	segment	CODE

/****************************************************************************
 * Name: _z8_restorecontext
 *
 * Description:
 *   Restore the task context that was previously saved via
 *   _z8_saveusercontext() or by interrupt handling.  Unlike the
 *   _z8_saveusercontext() counterpart, we do not know the context of the
 *   restored task and, hence, we must handle the worst case -- restore
 *   everythihng.
 *
 * Parameters:
 *   On entry, the following stack organization is assumed:
 *
 *          Pointer to the context save structure
 *   TOS -> Return address (2)
 *
 * Assumptions:
 *   Large model, dynamic frames
 *
 **************************************************************************/

_z8_restorecontext:
	/* Disable all interrupts because we are going to be using
	 * the IRQ register set.
	 */

	di

	/* Switch to IRQ register set */

	srp     #%f0

	/* Get the rr0 = the current value of the stack pointer */

	ldx	r0, sph			/* rr0 = stack pointer */
	ldx	r1, spl

	/* Get rr6 = the pointer to the context save structure */

	ldx	r6, 2(rr0)		/* rr6 = pointer to context structure */
	ldx	r7, 3(rr0)

	/* Copy all registers into the user register area.  NOTE: we
	 * use the saved RP value to determine the destination adress.
	 */

	clr	r0			/* rr0 = destination address */
	ldx	r1, XCPT_RP_OFFS(rr6)
	ld	r2, r6			/* rr2 = source address */
	ld	r3, r7
	ld	r4, #16			/* r4 = number of bytes to copy */

_z8_restore:
	ldx	r5, @rr2
	ldx	@rr0, r5
	incw	rr0
	incw	rr2
	djnz	r4, _z8_restore

	/* Set the new stack pointer */

	ldx	r0, XCPT_SPH_OFFS(rr6)
	ldx	r1, XCPT_SPL_OFFS(rr6)
	ldx	sph, r0
	ldx	spl, r1

	/* Push the return address onto the stack */

	ldx	r0, XCPT_PCH_OFFS(rr6)
	ldx	r1, XCPT_PCL_OFFS(rr6)
	push	r1
	push	r0

	/* Recover the flags and RP settings.. but don't restore them yet */

	ldx	r1, XCPT_FLAGS_OFFS(rr6)
	ldx	r2, XCPT_RP_OFFS(rr6)

	/* Determine whether interrupts must be enabled on return.  This
	 * would be nicer to do below, but later we will need to preserve
	 * the condition codes in the flags.
	 */

	ldx	r0, XCPT_IRQCTL_OFFS(rr6)
	tm	r0, #%80
	jr	nz, _z8_returnenabled

	/* Restore the flag settings */

	ldx	flags, r1

	/* Restore the user register page and return with interrupts disabled */

	ldx	rp, r2				/* Does not effect flags */
	ret					/* Does not effect flags */

_z8_returnenabled:
	/* Restore the flag settings */

	ldx	flags, r1

	/* Restore the user register page, re-enable interrupts and return */

	ldx	rp, r2				/* Does not effect flags */
	ei					/* Does not effect flags */
	ret					/* Does not effect flags */

	end
