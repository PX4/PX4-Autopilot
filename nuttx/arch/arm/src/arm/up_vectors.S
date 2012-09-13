/************************************************************************************
 * arch/arm/src/arm/up_vectors.S
 *
 *   Copyright (C) 2007-2010 Gregory Nutt. All rights reserved.
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
#include <nuttx/irq.h>

#include "arm.h"
#include "up_arch.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/************************************************************************************
 * Global Data
 ************************************************************************************/

	.data
g_irqtmp:
	.word	0		/* Saved lr */
	.word	0		/* Saved spsr */
g_undeftmp:
	.word	0		/* Saved lr */
	.word	0		/* Saved spsr */
g_aborttmp:
	.word	0		/* Saved lr */
	.word	0		/* Saved spsr */

/************************************************************************************
 * Assembly Macros
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

	.text

/************************************************************************************
 * Public Functions
 ************************************************************************************/

	.text

/************************************************************************************
 * Name: up_vectorirq
 *
 * Description:
 *   Interrupt excetpion. Entered in IRQ mode with spsr = SVC CPSR, lr = SVC PC
 *
 ************************************************************************************/

	.globl	up_vectorirq
	.type	up_vectorirq, %function
up_vectorirq:
	/* On entry, we are in IRQ mode.  We are free to use
	 * the IRQ mode r13 and r14.
	 */

	ldr	r13, .Lirqtmp
	sub	lr, lr, #4
	str	lr, [r13]			@ save lr_IRQ
	mrs	lr, spsr
	str	lr, [r13, #4]			@ save spsr_IRQ

	/* Then switch back to SVC mode */

	bic	lr, lr, #MODE_MASK	/* Keep F and T bits */
	orr	lr, lr, #(SVC_MODE | PSR_I_BIT)
	msr	cpsr_c, lr		/* Switch to SVC mode */

	/* Create a context structure.  First set aside a stack frame
	 * and store r0-r12 into the frame.
	 */

	sub	sp, sp, #XCPTCONTEXT_SIZE
	stmia	sp, {r0-r12}		/* Save the SVC mode regs */

	/* Get the correct values of r13(sp) and r14(lr) in r1 and r2 */

	add	r1, sp, #XCPTCONTEXT_SIZE
	mov	r2, r14

	/* Get the values for r15(pc) and CPSR in r3 and r4 */

	ldr	r0, .Lirqtmp		/* Points to temp storage */
	ldmia	r0, {r3, r4}		/* Recover r1=lr_IRQ, r2=spsr_IRQ */

	add	r0, sp, #(4*REG_SP)	/* Offset to pc, cpsr storage */
	stmia	r0, {r1-r4}

	/* Then call the IRQ handler with interrupts disabled. */

	mov	fp, #0			/* Init frame pointer */
	mov	r0, sp			/* Get r0=xcp */

#if CONFIG_ARCH_INTERRUPTSTACK > 3
	ldr	sp, .Lirqstackbase	/* SP = interrupt stack base */
	str	r0, [sp]		/* Save the user stack pointer */
	bl	up_decodeirq		/* Call the handler */
	ldr	sp, [sp]		/* Restore the user stack pointer */
#else
	bl	up_decodeirq		/* Call the handler */
#endif

	/* Restore the CPSR, SVC modr registers and return */
.Lnoirqset:
	ldr	r0, [sp, #(4*REG_CPSR)]	/* Setup the SVC mode SPSR */
	msr	spsr, r0
	ldmia	sp, {r0-r15}^		/* Return */

.Lirqtmp:
	.word	g_irqtmp
#if CONFIG_ARCH_INTERRUPTSTACK > 3
.Lirqstackbase:
	.word	up_stackbase
#endif
	.size	up_vectorirq, . - up_vectorirq
	.align	5

/************************************************************************************
 * Function: up_vectorswi
 * 
 * Description:
 *   SWI interrupt. We enter the SWI in SVC mode.
 *
 ************************************************************************************/

	.globl	up_vectorswi
	.type	up_vectorswi, %function
up_vectorswi:

	/* Create a context structure.  First set aside a stack frame
	 * and store r0-r12 into the frame.
	 */

	sub	sp, sp, #XCPTCONTEXT_SIZE
	stmia	sp, {r0-r12}		/* Save the SVC mode regs */

	/* Get the correct values of r13(sp), r14(lr), r15(pc)
	 * and CPSR in r1-r4 */

	add	r1, sp, #XCPTCONTEXT_SIZE
	mov	r2, r14			/* R14 is altered on return from SWI */
	mov	r3, r14			/* Save r14 as the PC as well */
	mrs	r4, spsr		/* Get the saved CPSR */

	add	r0, sp, #(4*REG_SP)	/* Offset to pc, cpsr storage */
	stmia	r0, {r1-r4}

	/* Then call the SWI handler with interrupts disabled.
	 * void up_syscall(struct xcptcontext *xcp)
	 */

	mov	fp, #0			/* Init frame pointer */
	mov	r0, sp			/* Get r0=xcp */
	bl	up_syscall		/* Call the handler */

	/* Restore the CPSR, SVC modr registers and return */

	ldr	r0, [sp, #(4*REG_CPSR)]	/* Setup the SVC mode SPSR */
	msr	spsr, r0
	ldmia	sp, {r0-r15}^		/* Return */
	.size	up_vectorswi, . - up_vectorswi

	.align	5

/************************************************************************************
 * Name: up_vectordata
 *
 * Description:
 *   This is the data abort exception dispatcher. The ARM data abort exception occurs
 *   when a memory fault is detected during a data transfer.  This handler saves the
 *   current processor state and gives control to data abort handler.  This function
 *   is entered in ABORT mode with spsr = SVC CPSR, lr = SVC PC
 *
 ************************************************************************************/
	
	.globl	up_vectordata
	.type	up_vectordata, %function
up_vectordata:
	/* On entry we are free to use the ABORT mode registers
	 * r13 and r14
	 */

	ldr	r13, .Ldaborttmp	/* Points to temp storage */
	sub	lr, lr, #8		/* Fixup return */
	str	lr, [r13]		/* Save in temp storage */
	mrs	lr, spsr		/* Get SPSR */
	str	lr, [r13, #4]		/* Save in temp storage */

	/* Then switch back to SVC mode */

	bic	lr, lr, #MODE_MASK	/* Keep F and T bits */
	orr	lr, lr, #(SVC_MODE | PSR_I_BIT)
	msr	cpsr_c, lr		/* Switch to SVC mode */

	/* Create a context structure.  First set aside a stack frame
	 * and store r0-r12 into the frame.
	 */

	sub	sp, sp, #XCPTCONTEXT_SIZE
	stmia	sp, {r0-r12}		/* Save the SVC mode regs */

	/* Get the correct values of r13(sp) and r14(lr) in r1 and r2 */

	add	r1, sp, #XCPTCONTEXT_SIZE
	mov	r2, r14

	/* Get the values for r15(pc) and CPSR in r3 and r4 */

	ldr	r0, .Ldaborttmp		/* Points to temp storage */
	ldmia	r0, {r3, r4}		/* Recover r1=lr_IRQ, r2=spsr_IRQ */

	add	r0, sp, #(4*REG_SP)	/* Offset to pc, cpsr storage */
	stmia	r0, {r1-r4}

	/* Then call the data abort handler with interrupts disabled.
	 * void up_dataabort(struct xcptcontext *xcp)
	 */

	mov	fp, #0			/* Init frame pointer */
	mov	r0, sp			/* Get r0=xcp */
#ifdef CONFIG_PAGING
	mrc	p15, 0, r2, c5, c0, 0	/* Get r2=FSR */
	mrc	p15, 0, r1, c6, c0, 0	/* Get R1=FAR */
#endif
	bl	up_dataabort		/* Call the handler */

	/* Restore the CPSR, SVC modr registers and return */

	ldr	r0, [sp, #(4*REG_CPSR)]	/* Setup the SVC mode SPSR */
	msr	spsr_cxsf, r0
	ldmia	sp, {r0-r15}^		/* Return */

.Ldaborttmp:
	.word	g_aborttmp
	.size	up_vectordata, . - up_vectordata

	.align	5

/************************************************************************************
 * Name: up_vectorprefetch
 *
 * Description:
 *   This is the prefetch abort exception dispatcher. The ARM prefetch abort exception
 *   occurs when a memory fault is detected during an an instruction fetch.  This
 *   handler saves the current processor state and gives control to prefetch abort
 *   handler.  This function is entered in ABT mode with spsr = SVC CPSR, lr = SVC PC.
 *
 ************************************************************************************/
	
	.globl	up_vectorprefetch
	.type	up_vectorprefetch, %function
up_vectorprefetch:
	/* On entry we are free to use the ABORT mode registers
	 * r13 and r14
	 */

	ldr	r13, .Lpaborttmp	/* Points to temp storage */
	sub	lr, lr, #4		/* Fixup return */
	str	lr, [r13]		/* Save in temp storage */
	mrs	lr, spsr		/* Get SPSR */
	str	lr, [r13, #4]		/* Save in temp storage */

	/* Then switch back to SVC mode */

	bic	lr, lr, #MODE_MASK	/* Keep F and T bits */
	orr	lr, lr, #(SVC_MODE | PSR_I_BIT)
	msr	cpsr_c, lr		/* Switch to SVC mode */

	/* Create a context structure.  First set aside a stack frame
	 * and store r0-r12 into the frame.
	 */

	sub	sp, sp, #XCPTCONTEXT_SIZE
	stmia	sp, {r0-r12}		/* Save the SVC mode regs */

	/* Get the correct values of r13(sp) and r14(lr) in r1 and r2 */

	add	r1, sp, #XCPTCONTEXT_SIZE
	mov	r2, r14

	/* Get the values for r15(pc) and CPSR in r3 and r4 */

	ldr	r0, .Lpaborttmp		/* Points to temp storage */
	ldmia	r0, {r3, r4}		/* Recover r1=lr_IRQ, r2=spsr_IRQ */

	add	r0, sp, #(4*REG_SP)	/* Offset to pc, cpsr storage */
	stmia	r0, {r1-r4}

	/* Then call the prefetch abort handler with interrupts disabled.
	 * void up_prefetchabort(struct xcptcontext *xcp)
	 */

	mov	fp, #0			/* Init frame pointer */
	mov	r0, sp			/* Get r0=xcp */
	bl	up_prefetchabort	/* Call the handler */

	/* Restore the CPSR, SVC modr registers and return */

	ldr	r0, [sp, #(4*REG_CPSR)]	/* Setup the SVC mode SPSR */
	msr	spsr_cxsf, r0
	ldmia	sp, {r0-r15}^		/* Return */

.Lpaborttmp:
	.word	g_aborttmp
	.size	up_vectorprefetch, . - up_vectorprefetch

	.align	5

/************************************************************************************
 * Name: up_vectorundefinsn
 *
 * Description:
 *   Undefined instruction entry exception.  Entered in UND mode, spsr = SVC  CPSR,
 *   lr = SVC PC
 *
 ************************************************************************************/
	
	.globl	up_vectorundefinsn
	.type	up_vectorundefinsn, %function
up_vectorundefinsn:
	/* On entry we are free to use the UND mode registers
	 * r13 and r14
	 */

	ldr	r13, .Lundeftmp		/* Points to temp storage */
	str	lr, [r13]		/* Save in temp storage */
	mrs	lr, spsr		/* Get SPSR */
	str	lr, [r13, #4]		/* Save in temp storage */

	/* Then switch back to SVC mode */

	bic	lr, lr, #MODE_MASK	/* Keep F and T bits */
	orr	lr, lr, #(SVC_MODE | PSR_I_BIT)
	msr	cpsr_c, lr		/* Switch to SVC mode */

	/* Create a context structure.  First set aside a stack frame
	 * and store r0-r12 into the frame.
	 */

	sub	sp, sp, #XCPTCONTEXT_SIZE
	stmia	sp, {r0-r12}		/* Save the SVC mode regs */

	/* Get the correct values of r13(sp) and r14(lr) in r1 and r2 */

	add	r1, sp, #XCPTCONTEXT_SIZE
	mov	r2, r14

	/* Get the values for r15(pc) and CPSR in r3 and r4 */

	ldr	r0, .Lundeftmp		/* Points to temp storage */
	ldmia	r0, {r3, r4}		/* Recover r1=lr_IRQ, r2=spsr_IRQ */

	add	r0, sp, #(4*REG_SP)	/* Offset to pc, cpsr storage */
	stmia	r0, {r1-r4}

	/* Then call the undef insn handler with interrupts disabled.
	 * void up_undefinedinsn(struct xcptcontext *xcp)
	 */

	mov	fp, #0			/* Init frame pointer */
	mov	r0, sp			/* Get r0=xcp */
	bl	up_undefinedinsn	/* Call the handler */

	/* Restore the CPSR, SVC modr registers and return */

	ldr	r0, [sp, #(4*REG_CPSR)]	/* Setup the SVC mode SPSR */
	msr	spsr_cxsf, r0
	ldmia	sp, {r0-r15}^		/* Return */

.Lundeftmp:
	.word	g_undeftmp
	.size	up_vectorundefinsn, . - up_vectorundefinsn

	.align	5

/************************************************************************************
 * Name: up_vectorfiq
 *
 * Description:
 *   Shouldn't happen
 *
 ************************************************************************************/

	.globl	up_vectorfiq
	.type	up_vectorfiq, %function
up_vectorfiq:
	subs	pc, lr, #4
	.size	up_vectorfiq, . - up_vectorfiq

/************************************************************************************
 *  Name: up_interruptstack/g_userstack
 ************************************************************************************/

#if CONFIG_ARCH_INTERRUPTSTACK > 3
	.bss
	.align	4
	.globl	g_userstack
	.type	g_userstack, object
up_interruptstack:
	.skip	((CONFIG_ARCH_INTERRUPTSTACK & ~3) - 4)
g_userstack:
up_stackbase:
	.skip	4
	.size	g_userstack, 4
	.size	up_interruptstack, (CONFIG_ARCH_INTERRUPTSTACK & ~3)
#endif
	.end
