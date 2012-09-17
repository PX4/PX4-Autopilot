/**************************************************************************
 * arch/x86/src/qemu/up_saveusercontext.S
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
 **************************************************************************/

/**************************************************************************
 * Conditional Compilation Options
 **************************************************************************/

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <nuttx/config.h>
#include <arch/irq.h>
#include "up_internal.h"

	.file	"qemu_saveusercontext.S"

/**************************************************************************
 * Pre-processor Definitions
 **************************************************************************/

/****************************************************************************
 * Macros
 ****************************************************************************/

/* Trace macros, use like trace 'i' to print char to serial port. */

	.macro	chout, addr, ch
#ifdef CONFIG_DEBUG
	mov		$\addr, %dx
	mov		$\ch, %al
	out		%al, %dx
#endif
	.endm

	.macro	trace, ch
#ifdef CONFIG_DEBUG
	push	%eax
	push	%edx
	chout	0x3f8, \ch
	pop		%edx
	pop		%eax
#endif
	.endm

/**************************************************************************
 * .text
 **************************************************************************/

	.text

/**************************************************************************
 * Name: up_saveusercontext
 *
 * Full C prototype:
 *  int up_saveusercontext(uint32_t *regs);
 *
 * Description:
 *  Save the "user" context.  It is not necessary to save all of the
 *  registers because it is acceptable for certain registers to be
 *  modified upon return from a subroutine call.  On a context switch
 *  back to user mode, it will appear as a return from this function.
 *
 *  According to the Intel ABI, the EAX, EDX, and ECX are to be free for
 *  use within a procedure or function, and need not be preserved.  These
 *  are the so-called caller-saved registers are EAX, ECX, EDX.
 *
 *  On entry,
 *    sp points to the return address
 *    sp+4 points to register save array
 *
 **************************************************************************/

	.globl	up_saveusercontext
	.type	up_saveusercontext, @function
up_saveusercontext:
	/* Fetch the pointer to the register save array.  %eax is a available
	 * because it must be modified later to provide the return value.
	 */

	movl	4(%esp), %eax

	/* %ebx, %esi, %edi, and %ebp must be preserved.  We can freely used %eax
	 * because it will be the return value from this function.
	 */

	movl	%ebx, (4*REG_EBX)(%eax)
	movl	%esi, (4*REG_ESI)(%eax)
	movl	%edi, (4*REG_EDI)(%eax)

	/* Save the segment registers */

	mov		%ss, (4*REG_SS)(%eax)
	mov		%cs, (4*REG_CS)(%eax)
	mov		%ds, (4*REG_DS)(%eax)

	/* Save the value of SP as will be at the time of the IRET that will
	 * appear to be the return from this function.
	 *
	 *
	 * 		CURRENT STACK				IRET STACK
	 *							PRIO CHANGE			No PRIO CHANGE
	 *		---------------		---------------		-----------------
	 *							EIP
	 *							CS					...
	 *							EFLAGS				EIP
	 *						->	ESP					CS
	 * ESP->Return address		SS					EFLAGS
	 *		Argument			Argument			Argument
	 *
	 * NOTE:  We don't yet know the value for REG_ESP!  That depends upon
	 * if a priority change occurs or not.
	 */


	leal	-4(%esp), %ecx
	movl	%ecx, (4*REG_SP)(%eax)

	/* Fetch the PC from the stack and save it in the save block */

	movl	0(%esp), %ecx
	movl	%ecx, (4*REG_EIP)(%eax)

	/* Save the framepointer */

	movl	%ebp, (4*REG_EBP)(%eax)

	/* Save EAX=1.  This will be the "apparent" return value from this
	 * function when context is switch back to this thread.  The non-zero
	 * return value is the indication that we have been resumed.
	 */
 
	movl	$1, (4*REG_EAX)(%eax)

	/* Get and save the interrupt state */

	pushf
	pop		%ecx
	movl	%ecx, (4*REG_EFLAGS)(%eax)

	/* And return 0 -- The zero return value is the indication that that
	 * this is the original, "true" return from the function.
	 *
	 * 'ret' will remove the EIP from the top of the stack.
	 */

	xorl	%eax, %eax
	ret
	.size	up_saveusercontext, . - up_saveusercontext
	.end
