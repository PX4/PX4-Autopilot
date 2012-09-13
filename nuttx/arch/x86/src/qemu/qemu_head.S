/****************************************************************************
 *  arch/x86/src/qemu/qemu_head.S
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

	.file	"qemu_head.S"

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/
 
/* Memory Map: _sbss is the start of the BSS region (see ld.script) _ebss is
 * the end of the BSS regsion (see ld.script). The idle task stack starts at
 * the end of BSS and is of size CONFIG_IDLETHREAD_STACKSIZE.  The IDLE thread
 * is the thread that the system boots on and, eventually, becomes the idle,
 * do nothing task that runs only when there is nothing else to run.  The
 * heap continues from there until the end of memory.  See g_heapbase below.
 */

#define STACKBASE	((_ebss + 0x1f) & 0xffffffe0)
#define IDLE_STACK	(STACKBASE+CONFIG_IDLETHREAD_STACKSIZE)
#define HEAP_BASE	(STACKBASE+CONFIG_IDLETHREAD_STACKSIZE)

/****************************************************************************
 * Macros
 ****************************************************************************/

/* Trace macros, use like trace 'i' to print char to serial port. */

	.macro	trace, ch
#ifdef CONFIG_DEBUG
	mov		$0x3f8, %dx
	mov		$\ch, %al
	out		%al, %dx
#endif
	.endm

/****************************************************************************
 * Global Symbols
 ****************************************************************************/

	.global	__start						/* Making entry point visible to linker */
	.global	os_start					/* os_start is defined elsewhere */
	.global	up_lowsetup					/* up_lowsetup is defined elsewhere */
	.global	g_heapbase					/* The start of the heap */

/****************************************************************************
 * .text
 ****************************************************************************/
/****************************************************************************
 * Multiboot Header
 ****************************************************************************/

	/* Setting up the Multiboot header - see GRUB docs for details */

	.set	ALIGN, 1<<0					/* Align loaded modules on page boundaries */
	.set	MEMINFO, 1<<1				/* Provide memory map */
	.set	FLAGS, ALIGN | MEMINFO		/* This is the Multiboot 'flag' field */
	.set	MAGIC, 0x1badb002			/* 'magic number' lets bootloader find the header */
	.set	CHECKSUM, -(MAGIC + FLAGS)	/* Checksum required */

	.text
	.align	4
	.long	MAGIC
	.long	FLAGS
	.long	CHECKSUM

/****************************************************************************
 * Name: Start
 ****************************************************************************/

	.type	__start, @function
__start:
	/* Set up the stack */

	mov		$(idle_stack + CONFIG_IDLETHREAD_STACKSIZE), %esp

	/* Multiboot setup */

	push	%eax						/* Multiboot magic number */
	push	%ebx						/* Multiboot data structure */

	/* Initialize and start NuttX */

	call	up_lowsetup					/* Low-level, pre-OS initialization */
	call	os_start					/* Start NuttX */

	/* NuttX will not return */

	cli
hang:
	hlt									/* Halt machine should NuttX return */
	jmp	hang
	.size	__start, . - __start

/****************************************************************************
 * .bss
 ****************************************************************************/

/* The stack for the IDLE task thread is declared in .bss.  NuttX boots and
 * initializes on the IDLE thread, then at the completion of OS startup, this
 * thread becomes the thread that executes when there is nothing else to
 * do in the system (see up_idle()).
 */

	.type	idle_stack, @object
	.comm   idle_stack, CONFIG_IDLETHREAD_STACKSIZE, 32
	.size	idle_stack, . - idle_stack

/****************************************************************************
 * .rodata
 ****************************************************************************/

	.section	.rodata, "a"

/* HEAP BASE: _sbss is the start of the BSS region (see ld.script) _ebss is
 * the end of the BSS region (see ld.script). The heap continues from there
 * until the end of memory.
 */

	.type	g_heapbase, @object
g_heapbase:
	.long	_ebss
	.size	g_heapbase, . - g_heapbase
	.end
