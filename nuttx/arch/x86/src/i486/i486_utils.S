/****************************************************************************
 * arch/x86/src/i486/i486_utils.S
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

	.file	"i486_utils.S"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

 #define KSEG 0x10

/****************************************************************************
 * Globals
 ****************************************************************************/

	.globl	gdt_flush
	.globl	idt_flush

/****************************************************************************
 * .text
 ****************************************************************************/

	.text

/****************************************************************************
 * Name: gdt_flush
 ****************************************************************************/

	.type	gdt_flush, @function
gdt_flush:
    movl	%eax, 4(%esp)	/* Get the pointer to the GDT, passed as a parameter */
    lgdt	(%eax)			/* Load the new GDT pointer */

    mov		$KSEG, %ax 		/* KSEG is the offset in the GDT to our data segment */
    mov		%ax, %ds 		/* Load all data segment selectors */
    mov		%ax, %es
    mov		%ax, %fs
    mov		%ax, %gs
    mov		%ax, %ss
    jmp		$0x08, $.Lgflush		/* 0x08 is the offset to our code segment: Far jump! */
.Lgflush:
    ret
	.size	gdt_flush, . - gdt_flush

/****************************************************************************
 * Name: idt_flush
 ****************************************************************************/

	.type	idt_flush, @function
idt_flush:
    movl	%eax, 4(%esp)	/* Get the pointer to the IDT, passed as a parameter */
    lidt	(%eax)			/* Load the IDT pointer */
    ret
	.size	idt_flush, . - idt_flush
	.end
