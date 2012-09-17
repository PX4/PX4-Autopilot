/************************************************************************************
 * arch/avr/src/avr/up_switchcontext.S
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

#include "excptmacros.h"

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
 *   void up_switchcontext(uint8_t *saveregs, uint8_t *restoreregs);
 *
 * On Entry:
 *   r24-r25: savregs
 *   r22-r23: restoreregs
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
	.globl	up_switchcontext
	.func	up_switchcontext
up_switchcontext:
	/* Use X [r26:r27] to reference the save structure.  (X is Call-used) */

	movw	r26, r24

	/* Save the context to saveregs */

	USER_SAVE

	/* Then fall through to do the full context restore with r24-r5 = restoreregs */

	movw	r24, r22
	.endfunc

/****************************************************************************
 * Name: up_fullcontextrestore
 *
 * Descripion:
 *   Restore the full-running context of a thread.
 *
 * Input Parameters:
 *   r24-r25 = A pointer to the register save area of the thread to be restored.
 *
 * C Prototype:
 *  void up_fullcontextrestore(uint8_t *regs);
 *
 * Assumptions:
 *   Interrupts are disabled.
 *
 ****************************************************************************/

	.text
	.global	up_fullcontextrestore
	.func	up_fullcontextrestore
up_fullcontextrestore:
	/* Use X [r26:r27] to reference the restore structure.  */

	movw	r26, r24

	/* Restore the context from the TCB saved registers */

	TCB_RESTORE

	/* And "return" to the new task (using ret not reti so that the interrupt
	 * state that we just restored will be preserved).
	 */

	ret
	.endfunc
	.end

