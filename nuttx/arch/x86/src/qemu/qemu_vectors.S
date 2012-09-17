/****************************************************************************
 * arch/x86/src/qemu/qemu_head.S
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/irq.h>

	.file	"qemu_vectors.S"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define KSEG 0x10

/****************************************************************************
 * .text
 ****************************************************************************/

	.text

/****************************************************************************
 * Globals
 ****************************************************************************/

	.globl	irq_handler
	.globl	isr_handler

/****************************************************************************
 * Macros
 ****************************************************************************/

/* Trace macros, use like trace 'i' to print char to serial port. */

	.macro	trace, ch
	mov		$0x3f8, %dx
	mov		$\ch, %al
	out		%al, %dx
	.endm

/* This macro creates a stub for an ISR which does NOT pass it's own
 * error code (adds a dummy errcode byte).
 */

	.macro ISR_NOERRCODE, intno
	.globl	vector_isr\intno
vector_isr\intno:
	cli							/* Disable interrupts firstly. */
	push	$0					/* Push a dummy error code. */
	push	$\intno				/* Push the interrupt number. */
	jmp		isr_common			/* Go to the common ISR handler code. */
	.endm

/* This macro creates a stub for an ISR which passes it's own
 * error code.
 */

	.macro	ISR_ERRCODE, intno
	.globl	vector_isr\intno
vector_isr\intno:
	cli							/* Disable interrupts firstly. */
	push	$\intno				/* Push the interrupt number. */
	jmp		isr_common			/* Go to the common ISR handler code. */
	.endm

/* This macro creates a stub for an IRQ - the first parameter is
 * the IRQ number, the second is the ISR number it is remapped to.
 */

	.macro	IRQ, irqno, intno
	.globl	vector_irq\irqno
vector_irq\irqno:
	cli							/* Disable interrupts firstly. */
	push	$0					/* Push a dummy error code. */
	push	$\intno				/* Push the interrupt number. */
	jmp		irq_common			/* Go to the common IRQ handler code. */
	.endm

/****************************************************************************
 * IDT Vectors
 ****************************************************************************/
/* The following will be the vector addresses programmed into the IDT */

	ISR_NOERRCODE		ISR0
	ISR_NOERRCODE		ISR1
	ISR_NOERRCODE		ISR2
	ISR_NOERRCODE		ISR3
	ISR_NOERRCODE		ISR4
	ISR_NOERRCODE		ISR5
	ISR_NOERRCODE		ISR6
	ISR_NOERRCODE		ISR7
	ISR_ERRCODE			ISR8
	ISR_NOERRCODE		ISR9
	ISR_ERRCODE			ISR10
	ISR_ERRCODE			ISR11
	ISR_ERRCODE			ISR12
	ISR_ERRCODE			ISR13
	ISR_ERRCODE			ISR14
	ISR_NOERRCODE		ISR15
	ISR_NOERRCODE		ISR16
	ISR_NOERRCODE		ISR17
	ISR_NOERRCODE		ISR18
	ISR_NOERRCODE		ISR19
	ISR_NOERRCODE		ISR20
	ISR_NOERRCODE		ISR21
	ISR_NOERRCODE		ISR22
	ISR_NOERRCODE		ISR23
	ISR_NOERRCODE		ISR24
	ISR_NOERRCODE		ISR25
	ISR_NOERRCODE		ISR26
	ISR_NOERRCODE		ISR27
	ISR_NOERRCODE		ISR28
	ISR_NOERRCODE		ISR29
	ISR_NOERRCODE		ISR30
	ISR_NOERRCODE		ISR31
	IRQ				 0,	IRQ0
	IRQ				 1,	IRQ1
	IRQ 			 2,	IRQ2
	IRQ				 3,	IRQ3
	IRQ				 4,	IRQ4
	IRQ				 5,	IRQ5
	IRQ				 6,	IRQ6
	IRQ				 7,	IRQ7
	IRQ				 8,	IRQ8
	IRQ				 9,	IRQ9
	IRQ				10,	IRQ10
	IRQ				11,	IRQ11
	IRQ				12,	IRQ12
	IRQ				13,	IRQ13
	IRQ				14,	IRQ14
	IRQ				15,	IRQ15

/****************************************************************************
 * Name: isr_common
 *
 * Description:
 *   This is the common ISR logic. It saves the processor state, sets up for
 *   kernel mode segments, calls the C-level fault handler, and finally
 *   restores the stack frame.
 *
 ****************************************************************************/

	.type	isr_common, @function
isr_common:
/*	trace	'S' */
	pusha						/* Pushes edi,esi,ebp,esp,ebx,edx,ecx,eax */

	mov		%ds, %ax			/* Lower 16-bits of eax = ds. */
	pushl	%eax				/* Save the data segment descriptor */

	mov		$KSEG, %ax 			/* Load the kernel data segment descriptor */
	mov		%ax, %ds
	mov		%ax, %es
	mov		%ax, %fs
	mov		%ax, %gs

	/* The current value of the SP points to the beginning of the state save
	 * structure.  Save that on the stack as the input parameter to isr_handler.
	 */

	mov		%esp, %eax
	push	%eax
	call	isr_handler
	jmp		.Lreturn
	.size	isr_common, . - isr_common

/****************************************************************************
 * Name: irq_common
 *
 * Description:
 *   This is the common IRQ logic. It saves the processor state, sets up for
 *   kernel mode segments, calls the C-level fault handler, and finally
 *   restores the stack frame.
 *
 ****************************************************************************/

	.type	irq_common, @function
irq_common:
/*	trace	'R' */
	pusha						/* Pushes edi,esi,ebp,esp,ebx,edx,ecx,eax */

	mov		%ds, %ax			/* Lower 16-bits of eax = ds. */
	push	%eax				/* Save the data segment descriptor */

	mov		$KSEG, %ax 			/* Load the kernel data segment descriptor */
	mov		%ax, %ds
	mov		%ax, %es
	mov		%ax, %fs
	mov		%ax, %gs

	/* The current value of the SP points to the beginning of the state save
	 * structure.  Save that on the stack as the input parameter to irq_handler.
	 */

	mov		%esp, %eax
	push	%eax
	call	irq_handler

	/* The common return point for both isr_handler and irq_handler */

.Lreturn:
	add		$4, %esp

	/* EAX may possibly hold a pointer to a different regiser save area on
	 * return.  Are we switching to a new context?
	 */

	cmp		%eax, %esp
	je		.Lnoswitch

	/* A context swith will be performed. EAX holds the address of the new
	 * register save structure.
	 *
	 * Jump to up_fullcontextrestore().  We perform a call here, but that function
	 * never returns.  The address of the new register save block is the argument
	 * to the up_fullcontextrestore().
	 */

	push	%eax
	call	up_fullcontextrestore

.Lnoswitch:
	pop		%ebx				/* Reload the original data segment descriptor */
	mov		%bx, %ds
	mov		%bx, %es
	mov		%bx, %fs
	mov		%bx, %gs

	popa						/* Pops edi,esi,ebp... */
	add		$8, %esp			/* Cleans up the pushed error code and pushed ISR number */
	iret						/* Pops 3-5 things at once: CS, EIP, EFLAGS (and maybe SS and ESP) */
	.size	irq_common, . - irq_common
	.end
