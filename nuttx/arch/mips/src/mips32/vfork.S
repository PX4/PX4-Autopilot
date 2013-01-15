/************************************************************************************
 * arch/mips/src/mips32/vfork.S
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

	.text
	.align  2
	.globl	vfork
	.type	vfork, function
	.set	nomips16
	.ent	vfork

vfork:
	/* Create a stack frame */

	move	$t0, $sp					/* Save the value of the stack on entry */
	addiu	$sp, $sp, -VFORK_SIZEOF		/* Allocate the structure on the stack */

	/* CPU registers */
	/* Save the saved registers */

	sw		$s0, VFORK_S0_OFFSET($sp)
	sw		$s1, VFORK_S1_OFFSET($sp)
	sw		$s2, VFORK_S2_OFFSET($sp)
	sw		$s3, VFORK_S3_OFFSET($sp)
	sw		$s4, VFORK_S4_OFFSET($sp)
	sw		$s5, VFORK_S5_OFFSET($sp)
	sw		$s6, VFORK_S6_OFFSET($sp)
	sw		$s7, VFORK_S7_OFFSET($sp)

#ifdef CONFIG_MIPS32_FRAMEPOINTER
	sw		$fp, VFORK_FP_OFFSET($sp)
#else
	sw		$s8, VFORK_S8_OFFSET($sp)
#endif

	/* Save the global pointer, stack pointer, and return address */

	sw		$t0, VFORK_SP_OFFSET($sp)
	sw		$ra, VFORK_RA_OFFSET($sp)
#ifdef MIPS32_SAVE_GP
	sw		$gp, VFORK_GP_OFFSET($sp)
#endif

	/* Floating point registers (not yet) */

	/* Then, call up_vfork(), passing it a pointer to the stack structure */

	move	$a0, $sp
	jal		up_vfork
	nop

	/* Release the stack data and return the value returned by up_vfork */

	lw		$ra, VFORK_RA_OFFSET($sp)
	addiu	$sp, $sp, VFORK_SIZEOF
	j		$ra

	.end	vfork
	.size	vfork, .-vfork
