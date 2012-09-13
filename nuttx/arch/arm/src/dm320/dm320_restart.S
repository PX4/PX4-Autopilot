/********************************************************************
 * arch/arm/src/dm320/dm320_restart.S
 *
 *   Copyright (C) 2007, 2009 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 ********************************************************************/

/********************************************************************
 * Included Files
 ********************************************************************/

#include <nuttx/config.h>

#include "arm.h"
#include "up_internal.h"
#include "up_arch.h"

/********************************************************************
 * Definitions
 ********************************************************************/

/********************************************************************
 * Assembly Macros
 ********************************************************************/

/* Since the page table is closely related to the NuttX base
 * address, we can convert the page table base address to the
 * base address of the section containing both.
 */

	.macro	mksection, section, pgtable
	bic	\section, \pgtable, #0x000ff000
	.endm

/**************************************************************************
 * Name: up_restart
 **************************************************************************/

	.text
	.globl	up_restart
	.type	up_restart, %function
up_restart:
	/* Make sure that we are in SVC mode with all IRQs disabled */

	mov	r0, #(SVC_MODE | PSR_I_BIT | PSR_F_BIT)
	msr	cpsr_c, r0

	/* Create identity mapping for first MB section to support
	 * this re-start logic executing out of the physical address
	 * space.
	 */

	mksection r0, r4			/* r0=phys. base section */
	ldr	r1, .LCmmuflags		/* FLGS=MMU_MEMFLAGS */
	add	r3, r1, r0		/* r3=flags + base */
	str	r3, [r4, r0, lsr #18]	/* identity mapping */

	/* Jump into the physical address space */

	ldr	pc, .LCphysrestart
	nop
	nop

	/* We are now executing at our physical address, with the
	 * MMU disabled.
	 */

up_phyrestart:

	mov	r0, #0
	mcr	p15, 0, r0, c7, c7		/* Invalidate I,D caches */
	mcr	p15, 0, r0, c7, c10, 4	/* Drain write buffer */
	mcr	p15, 0, r0, c8, c7		/* Invalidate I,D TLBs */

	/* Clear bits in control register (see start.h): Disable,
	 * MMU, Data cache, alignment traps, write buffer, Instruction
	 * cache, exceptions at 0xffff0000, round robin)
	 */

	mrc	p15, 0, r0, c1, c0		/* Get control register */
	bic	r0, r0, #(CR_M|CR_C|CR_A|CR_W)
	bic	r0, r0, #(CR_S|CR_I|CR_V|CR_RR)
	mcr	p15, 0, r0, c1, c0, 0		/* Write control reg */

	/* We know that the bootloader entry point is at the
	 * beginning of flash.
	 */
#if 1
	ldr	pc, .LCbtldrentry		/* Restart bootloader */
#else
	b	__start				/* Restart Nuttx */
#endif

	.type	.LCphysrestart, %object
.LCphysrestart:
	.long	(up_phyrestart - CONFIG_DRAM_VSTART - CONFIG_DRAM_START)
.LCbtldrentry:
	.long	DM320_EXT_MEM_PADDR

/**************************************************************************
 * PC_Relative Data
 **************************************************************************/

	.type	.LCmmuflags, %object
.LCmmuflags:
	.long	MMU_MEMFLAGS
	.size	up_restart, .-up_restart

	.end

