/************************************************************************************
 * arch/arm/src/armv7-m/vfork.S
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

#include "up_vfork.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Global Symbols
 ************************************************************************************/

	.syntax	unified
	.thumb
	.file	"vfork.S"
	.globl	up_vfork

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: vfork
 *
 * Description:
 *   The vfork() function has the same effect as fork(), except that the behavior is
 *   undefined if the process created by vfork() either modifies any data other than
 *   a variable of type pid_t used to store the return value from vfork(), or returns
 *   from the function in which vfork() was called, or calls any other function before
 *   successfully calling _exit() or one of the exec family of functions. 
 *
 *   This thin layer implements vfork by simply calling up_vfork() with the vfork()
 *   context as an argument.  The overall sequence is:
 *
 *   1) User code calls vfork().  vfork() collects context information and
 *      transfers control up up_vfork().
 *   2) up_vfork()and calls task_vforksetup().
 *   3) task_vforksetup() allocates and configures the child task's TCB.  This
 *      consists of:
 *      - Allocation of the child task's TCB.
 *      - Initialization of file descriptors and streams
 *      - Configuration of environment variables
 *      - Setup the intput parameters for the task.
 *      - Initialization of the TCB (including call to up_initial_state()
 *   4) up_vfork() provides any additional operating context. up_vfork must:
 *      - Allocate and initialize the stack
 *      - Initialize special values in any CPU registers that were not
 *        already configured by up_initial_state()
 *   5) up_vfork() then calls task_vforkstart()
 *   6) task_vforkstart() then executes the child thread.
 *
 * Input Paremeters:
 *   None
 *
 * Return:
 *   Upon successful completion, vfork() returns 0 to the child process and returns
 *   the process ID of the child process to the parent process. Otherwise, -1 is
 *   returned to the parent, no child process is created, and errno is set to
 *   indicate the error. 
 *
 ************************************************************************************/

	.thumb_func
	.globl	vfork
	.type	vfork, function
vfork:
	/* Create a stack frame */

	mov		r0, sp					/* Save the value of the stack on entry */
	sub		sp, sp, #VFORK_SIZEOF	/* Allocate the structure on the stack */

	/* CPU registers */
	/* Save the volatile registers */

	str		r4, [sp, #VFORK_R4_OFFSET]
	str		r5, [sp, #VFORK_R5_OFFSET]
	str		r6, [sp, #VFORK_R6_OFFSET]
	str		r7, [sp, #VFORK_R7_OFFSET]
	str		r8, [sp, #VFORK_R8_OFFSET]
	str		r9, [sp, #VFORK_R9_OFFSET]
	str		r10, [sp, #VFORK_R10_OFFSET]

	/* Save the frame pointer, stack pointer, and return address */

	str		fp, [sp, #VFORK_FP_OFFSET]
	str		r0, [sp, #VFORK_SP_OFFSET]
	str		lr, [sp, #VFORK_LR_OFFSET]

	/* Floating point registers (not yet) */

	/* Then, call up_vfork(), passing it a pointer to the stack structure */

	mov		r0, sp
	bl		up_vfork

	/* Release the stack data and return the value returned by up_vfork */

	ldr		lr, [sp, #VFORK_LR_OFFSET]
	add		sp, sp, #VFORK_SIZEOF
	bx		lr
	.size	vfork, .-vfork
	.end

