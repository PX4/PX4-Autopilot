/**************************************************************************
 * up_setjmp.S
 *
 *   Copyright (C) 2007, 2012 Gregory Nutt. All rights reserved.
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
 * Conditional Compilation Options
 **************************************************************************/

/**************************************************************************
 * Included Files
 **************************************************************************/

#include "up_internal.h"

/**************************************************************************
 * Private Definitions
 **************************************************************************/
 
#ifdef __CYGWIN__
# define SYMBOL(s) _##s
#else
# define SYMBOL(s) s
#endif

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

	.text
	.globl	SYMBOL(up_setjmp)
#ifndef __CYGWIN__
	.type	SYMBOL(up_setjmp), @function
#endif
SYMBOL(up_setjmp):

	/* %ebx, %esi, %edi, and %ebp must be preserved.
	 * save %ebx, $esi, and %edi now... */

	movl	4(%esp), %eax
	movl	%ebx, (JB_EBX)(%eax)
	movl	%esi, (JB_ESI)(%eax)
	movl	%edi, (JB_EDI)(%eax)

	/* Save the value of SP as will be after we return */

	leal	4(%esp), %ecx
	movl	%ecx, (JB_SP)(%eax)

	/* Save the return PC */

	movl	0(%esp), %ecx
	movl	%ecx, (JB_PC)(%eax)

	/* Save the framepointer */

	movl	%ebp, (JB_EBP)(%eax)

	/* And return 0 */

	xorl	%eax, %eax
	ret
#ifndef __CYGWIN__
	.size	SYMBOL(up_setjmp), . - SYMBOL(up_setjmp)
#endif
	.globl	SYMBOL(up_longjmp)
#ifndef __CYGWIN__
	.type	SYMBOL(up_longjmp), @function
#endif
SYMBOL(up_longjmp):
	movl	4(%esp), %ecx      /* U_pthread_jmpbuf in %ecx.  */
	movl	8(%esp), %eax      /* Second argument is return value.  */

	/* Save the return address now.  */

	movl	(JB_PC)(%ecx), %edx

	/* Restore registers.  */

	movl	(JB_EBX)(%ecx), %ebx
	movl	(JB_ESI)(%ecx), %esi
	movl	(JB_EDI)(%ecx), %edi
	movl	(JB_EBP)(%ecx), %ebp
	movl	(JB_SP)(%ecx), %esp

	/* Jump to saved PC.  */

	jmp		*%edx
#ifndef __CYGWIN__
	.size SYMBOL(up_longjmp), . - SYMBOL(up_longjmp)
#endif

