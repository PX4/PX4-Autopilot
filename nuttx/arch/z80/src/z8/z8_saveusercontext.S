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

	xdef	_z8_saveusercontext

/**************************************************************************
 * Code
 **************************************************************************/

	segment	CODE

/****************************************************************************
 * Name: _z8_saveusercontext
 *
 * Description:
 *   Save the current state of the user thread.  Since this function is
 *   called from user code, it is only necessary to save the parts of the
 *   context that must be preserved between function calls.  This includes
 *
 *   - Frame pointer (r14, r15)
 *   - Register pointer (RP)
 *   - Interrupt state (flags)
 *   - Stack pointer (sph, spl)
 *   - Return address
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

_z8_saveusercontext:
	/* Get the rr6 = the current value of the stack pointer */

	ldx	r6, sph			/* rr6 = stack pointer */
	ldx	r7, spl

	/* Get rr2 = the pointer to the context save structure */

	ldx	r2, 2(rr6)		/* rr2 = pointer to context structure */
	ldx	r3, 3(rr6)

	/* Get the value currently in the interrupt control register.
	 * Bit 7 (IRQE) determines whether or not interrupts are
	 * currently enabled (0:disabled, 1:enabled)
	 */

	ldx	r4, IRQCTL		/* r4 = IRQCTL value */

	/* Disable all interrupts so that there can be no concurrent
	 * modification of the TCB state save area.
	 */

	di

	/* Fetch and save the return address from the stack */

	ldx	r0, @rr6		/* rr0 = return address */
	ldx	r1, 1(rr6)
	ldx	XCPT_PCH_OFFS(rr2), r0
	ldx	XCPT_PCL_OFFS(rr2), r1

	/* Fetch and save the register pointer */

	ldx	r0, rp			/* r0 = register pointer */
	ldx	XCPT_RP_OFFS(rr2), r0

	/* Calculate the value of the stack pointer on return
	 * from this function
	 */

	ld	r1, #3			/* rr0 = 3 */
	clr	r0
	add	r1, r7			/* rr0 = SP + 3 */
	adc	r0, r6
	ldx	XCPT_SPH_OFFS(rr2), r0
	ldx	XCPT_SPL_OFFS(rr2), r1

	/* Save the IRQCTL register value */

	clr	r0
	ldx	XCPT_UNUSED_OFFS(rr2), r0
	ldx	XCPT_IRQCTL_OFFS(rr2), r4

	/* Save the frame pointer (rr14) in the context structure */

	ldx	XCPT_R14_OFFS(rr2), r14
	ldx	XCPT_R15_OFFS(rr2), r15

	/* Set the return value of 1 in the context structure.  When the
	 * state is restored (via z8_restorecontext() or an interrupt
	 * return), the return value of 1 distinguishes the no-context-
	 * switch case.
	 */

	/* clr	r0 */
	ld	r1, #1
	ldx	XCPT_R0_OFFS(rr2), r0
	ldx	XCPT_R1_OFFS(rr2), r1

	/* Setup to return zero for the no-context-switch case */

	/* clr	r0 */
	clr	r1

	/* Now decide if we need to re-enable interrupts or not */

	tm	r4, #%80
	jr	z, _z8_noenable
	ei
_z8_noenable:
	ret

	end

