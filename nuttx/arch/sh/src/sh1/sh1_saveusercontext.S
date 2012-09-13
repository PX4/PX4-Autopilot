/**************************************************************************
 * arch/sh/src/sh1/sh1_saveusercontext.S
 *
 *   Copyright (C) 2008, 2009 Gregory Nutt. All rights reserved.
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
 * Name: up_saveusercontext
 **************************************************************************/

	.text
	.globl	_up_saveusercontext
	.type	_up_saveusercontext, function
_up_saveusercontext:

	/* On entry, R4 contains a reference to the register save structure
	 * to populate.
	 *
	 * Re-position to the end of the structure (+4_ so that we can use
	 * auto decrement 
	 */

	add	#(XCPTCONTEXT_SIZE), r4	/* R4: Address of last entry + 4 */

	/* Save the necessary registers */

	stc     sr, r0			/* Get and save the SR */
	mov.l	r0, @-r4
	sts.l	pr, @-r4		/* The return address is the restore PC */
	add	#-(7*4), r4		/* Skip over r4, r7-r1.  R4 points at R1 storage */

	/* Save a nonzero value for R0 in the register save area.  This will
	 * will be the apparent return value when a context switch back to
	 * this thread is performed.
	 */

	mov	#1, r0			/* Save R0=1 (return value) */
	mov.l	r0, @-r4
	add	#-(2*4), r4		/* Skip over mach and macl.  R4 points at R15 storage. */
	mov.l	r15, @-r4		/* SP */
	stc.l	gbr, @-r4		/* GBR & PR */
	sts.l	pr, @-r4

	mov.l	r14, @-r4		/* R14-R8 */
	mov.l	r13, @-r4
	mov.l	r12, @-r4
	mov.l	r11, @-r4
	mov.l	r10, @-r4
	mov.l	r9, @-r4
	mov.l	r8, @-r4

	/* And return zero */

	rts
	mov	#0, r0
	.size	up_saveusercontext, . - up_saveusercontext
	.end
