/****************************************************************************
 * arch/avr32/src/avr32/up_fullcontextrestore.S
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

/****************************************************************************
 * External Symbols
 ****************************************************************************/

	.file		"up_fullcontextrestore.S"

/****************************************************************************
 * Macros
 ****************************************************************************/

/****************************************************************************
 * Name: up_fullcontextrestore
 *
 * Descripion:
 *   Restore the full-running contex of a thread.
 *
 *   NOTE: Thus function must handle one very strange case.  That is when
 *   this function is called with up_sigdeliver().  That case is strange in
 *   two ways:
 *
 *   1. It is not a context switch between threads.  Rather, up_fullcontextrestore
 *      must behave more it more like a longjmp within the same task, using the
 *      same stack.
 *   2. In this case, this function is called with r12 pointing to a register
 *      save area on the stack to be destroyed.  This is dangerous for two
 *      reasons: (a) there is a period of time where the stack contents still
 *      contain valid data, but are outside of range protected by the stack
 *      pointer (hence, interrupts must be disabled), and (b) there is the
 *      very real possibility that the new stack pointer might overlap with
 *      the register save area and stack usage in this function might corrupt
 *      the register save data before the state is restored.  It turns that
 *      an extra 3 words in the register save structure size will protect its
 *      contents (because that is the number of temporaries pushed onto the
 *      stack).
 *
 * Input Parameters:
 *   r12 = A pointer to the register save area of the thread to be restored.
 *
 * C Prototype:
 *  void up_fullcontextrestore(uint32_t *regs);
 *
 * Assumptions:
 *   Interrupts are disabled.
 *
 ****************************************************************************/

	.text
	.global	up_fullcontextrestore
	.type	up_fullcontextrestore, @function
up_fullcontextrestore:

	/* Initially, r12 points to the r7 save area. Restore r0-r7.            */
	/* regs: 07 06 05 04 03 02 01 00 xx xx xx xx xx xx xx xx xx             */
	/*                               ^r12                                   */

	ldm 	r12++, r0-r7
	
    /* Now, r12 points to the SP (r13) save area.  Recover the value of     */
	/* the stack pointer (r13).  We will need to save some temporaries on   */
	/* the final stack.                                                     */
	/* regs: 07 06 05 04 03 02 01 00 SP xx xx xx xx xx xx xx xx             */
	/*                                  ^r12                                */

	ld.w	sp, r12++

	/* Now r12 points to the SR save area.  Skip over the SR for now.       */
	/* regs: 07 06 05 04 03 02 01 00 SP xx xx xx xx xx xx xx xx             */
	/*                                     ^r12                             */
	
    sub		r12, -1*4

	/* Get the pc, lr, and r12 (in r10, r9, and r8) and move them to the    */
    /* stack.  We can now use r12 and lr as scratch registers.              */
	/* regs: 07 06 05 04 03 02 01 00 SP xx PC LR 12 xx xx xx xx             */
	/*                                              ^r12                    */
	/* stack: lr, r12, pc                                                   */
	/*        ^sp                                                           */

	ldm		r12++, r8-r10		/* Get r10=pc, r9=lr, r8=r12                */

#if 0 /* See comments below */
	stm		--sp, r8-r10		/* Save r12, lr, and pc from the stack      */
#else
	st.w	--sp, r10			/* Save R10=PC on the stack                 */
	st.w	--sp, r8			/* Save R8=r12 on the stack                 */
	st.w	--sp, r9			/* Save R9=lr on the stack                  */
#endif

	/* Now r12 now points to the r11 save area.  Restore r8-r11.            */
	/* regs: 07 06 05 04 03 02 01 00 SP xx PC LR 12 11 10 09 08             */
	/*                                                          ^r12        */
	
	ldm		r12++, r8-r11
	
	/* r12 now points +4 beyond the end of the register save area. Restore  */
	/* SR.  NOTE:  This may enable interrupts!                              */
	/* regs: 07 06 05 04 03 02 01 00 SP SR PC LR 12 11 10 09 08             */
	/*                                  ^r12-4*8                ^r12        */
	
	ld.w	lr, r12[-4*8]
	mtsr	AVR32_SR, lr
	
	/* Restore PC, LR and r12.  Hmmm.. I need to study the behaviour of ldm */
	/* when r12,lr, and pc on in ldm reglist16.  I'm not sure that I want   */
	/* that behavior.                                                       */
	/* stack: lr, r12, pc                                                   */
	/*        ^sp                                                           */

#if 0
	ldm		sp++, r12, lr, pc	/* Restore r12, lr, and pc from the stack   */
#else
	ld.w	lr, sp++			/* Recover lr from the stack                */
	ld.w	r12, sp++			/* Recover r12 from the stack               */
	ld.w	pc, sp++			/* Jump to the address on the stack         */
#endif
	.end
