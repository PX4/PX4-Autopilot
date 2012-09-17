/************************************************************************************
 * arch/avr/src/avr32/up_switchcontext.S
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <arch/irq.h>
#include <arch/avr32/avr32.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Global Symbols
 ************************************************************************************/

	.file		"up_switchcontext.S"

/************************************************************************************
 * Macros
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: up_switchcontext
 *
 * Description:
 *   Save the current thread context and restore the specified context.  The full
 *   C function prototype is:
 *
 *   void up_switchcontext(uint32_t *saveregs, uint32_t *restoreregs);
 *
 * Return:
 *   up_switchcontext forces a context switch to the task "canned" in restoreregs.
 *   It does not 'return' in the normal sense, rather, it will context switch back
 *   to the function point.  When it does 'return,' it is because the blocked
 *   task hat was "pickeled" in the saveregs "can" is again ready to run and has
 *   execution priority.
 *
 * Assumptions:
 *   global interrupts disabled by the caller.
 *
 ************************************************************************************/

	.text
	.globl		up_switchcontext
	.type		up_switchcontext, @function
up_switchcontext:
	/* "Pickle" the current thread context in the saveregs "can."  r12=saveregs.    */
	/*     xx xx xx xx xx xx xx xx xx xx xx xx xx xx xx xx xx                       */
	/*     ^r12                                                                     */
	/* Sample SR and set r12 to just after the LR storage location.                 */
	/*     xx xx xx xx xx xx xx xx xx xx xx xx xx xx xx xx xx                       */
	/*                                         ^r12                                 */

	mfsr	r10, AVR32_SR
	sub		r12, -4*(REG_LR+1)

	/* Then "push" PC=LR, LR, SR, and SP as they are on entry.                      */
	/*     xx xx xx xx xx xx xx xx SP SR PC LR xx xx xx xx xx                       */
	/*                             ^r12                                             */

	st.w	--r12, lr
	st.w	--r12, lr
	st.w	--r12, r10
	st.w	--r12, sp

	/* Save the preserved/static registers, r0-r7.  There is no reason to save the  */
	/* scratch/volatile registers, r8-r12, in this context.                         */
	/*     07 06 05 04 03 02 01 00 SP SR PC LR xx xx xx xx xx                       */
	/*     ^r12                                                                     */

	stm		--r12, r0-r7

	/* Finally, let up_fullcontextrestore handling the re-instatement of the thread */
	/* "canned" in restoregs.                                                       */

	mov		r12, r11
	lddpc	pc, .Lup_fullcontextrestore

.Lup_fullcontextrestore:
	.word	up_fullcontextrestore
	.size	up_switchcontext, .-up_switchcontext
	.end

