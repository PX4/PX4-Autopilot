/**************************************************************************
 * arch/arm/src/arm/up_fullcontextrestore.S
 *
 *   Copyright (C) 2007, 2009-2010 Gregory Nutt. All rights reserved.
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
 **************************************************************************/

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <nuttx/irq.h>
#include "up_internal.h"

/**************************************************************************
 * Private Definitions
 **************************************************************************/

/**************************************************************************
 * Private Types
 **************************************************************************/

/**************************************************************************
 * Private Function Prototypes
 **************************************************************************/

/**************************************************************************
 * Global Variables
 **************************************************************************/

/**************************************************************************
 * Private Variables
 **************************************************************************/

/**************************************************************************
 * Private Functions
 **************************************************************************/

/**************************************************************************
 * Public Functions
 **************************************************************************/

/**************************************************************************
 * Name: up_fullcontextrestore
 **************************************************************************/

	.globl	up_fullcontextrestore
	.type	up_fullcontextrestore, function
up_fullcontextrestore:

	/* On entry, a1 (r0) holds address of the register save area */

	/* Recover all registers except for r0, r1, R15, and CPSR */

	add	r1, r0, #(4*REG_R2)	/* Offset to REG_R2 storage */
	ldmia	r1, {r2-r14}		/* Recover registers */

	/* Create a stack frame to hold the PC */

	sub	sp, sp, #(3*4)		/* Frame for three registers */
	ldr	r1, [r0, #(4*REG_R0)]	/* Fetch the stored r0 value */
	str	r1, [sp]		/* Save it at the top of the stack */
	ldr	r1, [r0, #(4*REG_R1)]	/* Fetch the stored r1 value */
	str	r1, [sp, #4]		/* Save it in the stack */
	ldr	r1, [r0, #(4*REG_PC)]	/* Fetch the stored pc value */
	str	r1, [sp, #8]		/* Save it at the bottom of the frame */

	/* Now we can restore the CPSR.  We wait until we are completely
	 * finished with the context save data to do this. Restore the CPSR
	 * may re-enable and interrupts and we could be in a context
	 * where the save structure is only protected by interrupts being
	 * disabled.
	 */

	ldr	r1, [r0, #(4*REG_CPSR)]	/* Fetch the stored CPSR value */
	msr	cpsr, r1		/* Set the CPSR */

	/* Now recover r0 and r1 */

	ldr	r0, [sp]
	ldr	r1, [sp, #4]
	add	sp, sp, #(2*4)

	/* Then return to the address at the stop of the stack,
	 * destroying the stack frame
	 */

	ldr	pc, [sp], #4
	.size up_fullcontextrestore, . - up_fullcontextrestore

