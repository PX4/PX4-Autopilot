/************************************************************************************
 * arch/arm/src/stm32/up_exception.S
 * arch/arm/src/chip/up_exception.S
 *
 *   Copyright (C) 2009-2012 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2012 Michael Smith. All rights reserved.
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

#include <arch/irq.h>
#include "exc_return.h"

#include "chip.h"

/************************************************************************************
 * Global Symbols
 ************************************************************************************/

	.globl		exception_common

	.syntax		unified
	.thumb
	.file		"up_exception.S"

/************************************************************************************
 * .text
 ************************************************************************************/

/* Common exception handling logic.  On entry here, the return stack is on either
 * the PSP or the MSP and looks like the following:
 *
 *      REG_XPSR
 *      REG_R15
 *      REG_R14
 *      REG_R12
 *      REG_R3
 *      REG_R2
 *      REG_R1
 * MSP->REG_R0
 *
 * And
 *      IPSR contains the IRQ number
 *      R14 Contains the EXC_RETURN value
 *      We are in handler mode and the current SP is the MSP
 *
 * If CONFIG_ARCH_FPU is defined, the volatile FP registers and FPSCR are on the 
 * return stack immediately above REG_XPSR.
 */

	.text
	.type	exception_common, function
	.thumb_func
exception_common:

	mrs		r0, ipsr				/* R0=exception number */

	/* Complete the context save */

	/* The EXC_RETURN value tells us whether the context is on the MSP or PSP */

	tst		r14, #EXC_RETURN_PROCESS_STACK /* nonzero if context on process stack */
	ite		eq						/* next two instructions conditional */
	mrseq	r1, msp					/* R1=The main stack pointer */
	mrsne	r1, psp					/* R1=The process stack pointer */

	mov		r2, r1					/* R2=Copy of the main/process stack pointer */
	add		r2, #HW_XCPT_SIZE		/* R2=MSP/PSP before the interrupt was taken */
									/* (ignoring the xPSR[9] alignment bit) */
	mrs		r3, primask				/* R3=Current PRIMASK setting */

#ifdef CONFIG_ARCH_FPU

	/* Save the non-volatile FP registers here.
	 *
	 * This routine is the only point where we can save these registers; either before
	 * or after calling up_doirq.  The compiler is free to use them at any time as long
	 * as they are restored before returning, so we can't assume that we can get at the
	 * true values of these registers in any routine called from here.
	 *
	 * XXX we could do all this saving lazily on the context switch side if we knew where to put
	 *     the registers.
	 */

	vstmdb	r1!, {s16-s31}			/* Save the non-volatile FP context */

#endif

	stmdb	r1!, {r2-r11,r14}		/* Save the remaining registers plus the SP/PRIMASK values */

	/* Disable interrupts, select the stack to use for interrupt handling
	 * and call up_doirq to handle the interrupt
	 */

	cpsid	i						/* Disable further interrupts */

	/* If CONFIG_ARCH_INTERRUPTSTACK is defined, we will use a special interrupt
	 * stack pointer.  The way that this is done here prohibits nested interrupts!
	 * Otherwise, we will use the stack that was current when the interrupt was taken.
	 */

#if CONFIG_ARCH_INTERRUPTSTACK > 3
	ldr		sp, =g_intstackbase
	push	{r1}					/* Save the MSP on the interrupt stack */
	bl		up_doirq				/* R0=IRQ, R1=register save area on stack */
	pop		{r1}					/* Recover R1=main stack pointer */
#else
	msr	 msp, r1					/* We are using the main stack pointer */
	bl		up_doirq				/* R0=IRQ, R1=register save area on stack */
	mrs	 r1, msp					/* Recover R1=main stack pointer */
#endif

	/* On return from up_doirq, R0 will hold a pointer to register context
	 * array to use for the interrupt return.  If that return value is the same
	 * as current stack pointer, then things are relatively easy.
	 */

	cmp		r0, r1					/* Context switch? */
	beq		1f						/* Branch if no context switch */

	/* We are returning with a pending context switch.  This case is different
	 * because in this case, the register save structure does not lie on the
	 * stack but, rather within a TCB structure.  We'll have to copy some
	 * values to the stack.
	 */

	/* Copy the hardware-saved context to the stack, and restore the software
	 * saved context directly.
	 *
	 * XXX In the normal case, it appears that this entire operation is unnecessary;
	 *     context switch time would be improved if we could work out when the stack
	 *     is dirty and avoid the work...
	 */

	add		r1, r0, #SW_XCPT_SIZE 	/* R1=Address of HW save area in reg array */
	ldmia	r1!, {r4-r11}			/* Fetch eight registers in HW save area */
#ifdef CONFIG_ARCH_FPU
	vldmia	r1!, {s0-s15}			/* Fetch sixteen FP registers in HW save area */
	ldmia	r1, {r2-r3}				/* Fetch FPSCR and Reserved in HW save area */
#endif
	ldr		r1, [r0, #(4*REG_SP)]	/* R1=Value of SP before interrupt */
#ifdef CONFIG_ARCH_FPU
	stmdb	r1!, {r2-r3}			/* Store FPSCR and Reserved on the return stack */
	vstmdb	r1!, {s0-s15}			/* Store sixteen FP registers on the return stack */
#endif
	stmdb	r1!, {r4-r11}			/* Store eight registers on the return stack */
	ldmia	r0!, {r2-r11,r14}		/* Recover R4-R11, r14 + 2 temp values */
#ifdef CONFIG_ARCH_FPU
	vldmia	r0, {s16-s31}			/* Recover S16-S31 */
#endif

	b		2f						/* Re-join common logic */

1:
	/* We are returning with no context switch.  We simply need to "unwind"
	 * the same stack frame that we created at entry.
	 */

	ldmia	r1!, {r2-r11,r14}		/* Recover R4-R11, r14 + 2 temp values */
#ifdef CONFIG_ARCH_FPU
	vldmia  r1!, {s16-s31}			/* Recover S16-S31 */
#endif

2:
	/* The EXC_RETURN value tells us whether we are returning on the MSP or PSP 
	 */

	tst		r14, #EXC_RETURN_PROCESS_STACK /* nonzero if context on process stack */
	ite		eq						/* next two instructions conditional */
	msreq	msp, r1					/* R1=The main stack pointer */
	msrne	psp, r1					/* R1=The process stack pointer */

	/* Restore the interrupt state */

	msr		primask, r3				/* Restore interrupts */

	/* Always return with R14 containing the special value that will: (1)
	 * return to thread mode, and (2) select the correct stack.
	 */

	bx		r14						/* And return */

	.size	exception_common, .-exception_common

/************************************************************************************
 *  Name: up_interruptstack/g_intstackbase
 *
 * Description:
 *   Shouldn't happen
 *
 ************************************************************************************/

#if CONFIG_ARCH_INTERRUPTSTACK > 3
	.bss
	.global	g_intstackbase
	.align	4
up_interruptstack:
	.skip	(CONFIG_ARCH_INTERRUPTSTACK & ~3)
g_intstackbase:
	.size	up_interruptstack, .-up_interruptstack
#endif

	.end

