/**************************************************************************
 * arch/x86/src/qemu/qemu_fullcontextrestore.S
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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

	.file	"qemu_fullcontextrestore.S"

/**************************************************************************
 * Pre-processor Definitions
 **************************************************************************/

/**************************************************************************
 * Global Variables
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
 * Public Functions
 **************************************************************************/

	.text

/**************************************************************************
 * Name: up_fullcontextrestore
 *
 * Full C prototype:
 *  void up_fullcontextrestore(uint32_t *regs) noreturn_function;
 *
 **************************************************************************/

	.globl	up_fullcontextrestore
	.type	up_fullcontextrestore, @function
up_fullcontextrestore:
	/* Fetch the pointer to the register save array in EAX. */

	movl	4(%esp), %eax

	/* Disable interrupts now (the correct EFLAGS will be restored before we
	 * return
	 */

	cli

	/* Get the value of the stack pointer as it was when the pusha was
	 * executed the interrupt handler.
	 */

	movl	(4*REG_SP)(%eax), %esp

	/* Create an interrupt stack frame for the final iret.
	 *
	 *
	 *							IRET STACK
	 *				PRIO CHANGE			No PRIO CHANGE
	 *				---------------		-----------------
	 * SP Before 	->
	 * 				   SS				   EFLAGS
	 *				   ESP				   CS
	 *				   EFLAGS			-> EIP
	 *				   CS				   ...
	 * SP After		-> EIP
	 *
	 * So, first check for a priority change.
	 */

	movl	(4*REG_CS)(%eax), %edx
	andl	$3, %edx
	mov		%cs, %ebx
	andl	$3, %ebx
	cmpb	%bl, %dl
	je		.Lnopriochange

	/* The priority will change... put SS and ESP on the stack */

	mov		(4*REG_SS)(%eax), %ebx
	push	%ebx
	movl	(4*REG_SP)(%eax), %ebx
	push	%ebx

.Lnopriochange:
	movl	(4*REG_EFLAGS)(%eax), %ebx
	push	%ebx
	mov		(4*REG_CS)(%eax), %ebx
	push	%ebx
	movl	(4*REG_EIP)(%eax), %ebx
	push	%ebx

	/* Save the value of EAX on the stack too */

	movl	(4*REG_EAX)(%eax), %ebx
	push	%ebx

	/* Now restore the remaining registers */

	movl	(4*REG_ESI)(%eax), %esi
	movl	(4*REG_EDI)(%eax), %edi
	movl	(4*REG_EBP)(%eax), %ebp
	movl	(4*REG_EBX)(%eax), %ebx
	movl	(4*REG_EDX)(%eax), %edx
	movl	(4*REG_ECX)(%eax), %ecx

	/* Restore the data segment register.  I think there is an issue that will
	 * need to be address here at some time:  If the register save area is in
	 * one data segment and the stack is in another, then the above would not
	 * work (and, conversely, if they are in the same data segment, the
	 * following is unnecessary and redundant).
	 */

	mov		(4*REG_DS)(%eax), %ds

	/* Restore the correct value of EAX and then return */

	popl	%eax
	iret
	.size up_fullcontextrestore, . - up_fullcontextrestore
	.end
