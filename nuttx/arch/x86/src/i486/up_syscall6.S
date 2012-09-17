/****************************************************************************
 * arch/x86/src/i486/up_syscall6.S
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 *   Based on Bran's kernel development tutorials. Rewritten for JamesM's
 *   kernel development tutorials.
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

	.file	"up_syscall6.S"

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Globals
 ****************************************************************************/

/****************************************************************************
 * .text
 ****************************************************************************/

	.text

/****************************************************************************
 * Name: sys_call6
 *
 * C Prototype:
 *   uintptr_t sys_call6(unsigned int nbr, uintptr_t parm1,
 *                       uintptr_t parm2, uintptr_t parm3,
 *                       uintptr_t parm4, uintptr_t parm5,
 *                       uintptr_t parm6);
 *
 ****************************************************************************/

	.global	sys_call6
	.type	sys_call6, %function

sys_call6:
	pushl	%ebp			/* Save ebx, esi, edi, and ebp */
	pushl	%edi
	pushl	%esi
	pushl	%ebx

	movl	44(%esp),%ebp	/* Save parm6 in ebp */
	movl	40(%esp),%edi	/* Save parm5 in edi */
	movl	36(%esp),%esi	/* Save parm4 in esi */
	movl	32(%esp),%edx	/* Save parm3 in edx */
	movl	28(%esp),%ecx	/* Save parm2 in ecx */
	movl	24(%esp),%ebx	/* Save parm1 in ebx */
	movl	20(%esp),%eax	/* Save syscall number in eax */
	int		$0x80			/* Execute the trap */
							/* Return value is in %eax */
	popl	%ebx			/* Restore ebx, esi, edi, and ebp */
	popl	%esi
	popl	%edi
	popl	%ebp
	ret						/* And return with result in %eax */

	.size	sys_call6,.-sys_call6
	.end
