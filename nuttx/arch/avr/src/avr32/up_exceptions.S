/****************************************************************************
 * arch/avr/src/avr32/up_exceptions.S
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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

#include <arch/avr32/avr32.h>
#include <arch/irq.h>

/****************************************************************************
 * External Symbols
 ****************************************************************************/

	.global		avr32_int0irqno			/* Returns IRQ number of INT0 event */
	.global		avr32_int1irqno			/* Returns IRQ number of INT1 event */
	.global		avr32_int2irqno			/* Returns IRQ number of INT2 event */
	.global		avr32_int3irqno			/* Returns IRQ number of INT3 event */
	.global		up_doirq				/* Dispatch an IRQ                  */
	.global		up_fullcontextrestore	/* Restore new task contex          */

/****************************************************************************
 * Macros
 ****************************************************************************/

/* Exception entry logic.  On entry, thee context save area looks like:     */
/*     xx xx xx xx xx xx xx xx xx xx xx xx xx xx xx SR PC                   */
/*                                                  ^     ^+2*4             */
/* Upon joining common logic, the context save are will look like:          */
/*     xx xx xx xx xx xx xx xx xx xx xx xx xx xx 10 SR PC                   */
/*                                               ^        ^+3*4             */
/* and r10 will hold the exception's IRQ number                             */

	.macro  HANDLER, label, irqno
\label:
	st.w	--sp, r10
	mov		r10, \irqno
	rjmp	avr32_xcptcommon	/* FIXME!!! Need IRQ in a register          */
	.endm

/****************************************************************************
 * Exception Vector Table
 ****************************************************************************/

/* The Exception Vector Base Address (EVBA) register will contain "a        */
/* pointer to the exception routines. All exception routines start at this  */
/* address, or at a defined offset relative to the address. Special         */
/* alignment requirements may apply for EVBA, depending on the              */
/* implementation of the interrupt controller."                             */

/* REVISIT:  This alignment requirement may be different on other AVR32s    */

	.text
	.balign 0x200

	.global	vectortab
	.type	vectortab, @function
vectortab:
	lda.w	pc, avr32_unrec			/* EVBA+0x00 Unrecoverable exception    */
	lda.w	pc, avr32_tlbmult		/* EVBA+0x04 TLB multiple hit           */
	lda.w	pc, avr32_busdata		/* EVBA+0x08 Bus error data fetch       */
	lda.w	pc, avr32_businst		/* EVBA+0x0c Bus error instr fetch      */
	lda.w	pc, avr32_nmi			/* EVBA+0x10 NMI                        */
	lda.w	pc, avr32_instaddr		/* EVBA+0x14 Instruction Address        */
	lda.w	pc, avr32_itlbrot		/* EVBA+0x18 ITLB Protection            */
	lda.w	pc, avr32_bp			/* EVBA+0x1c Breakpoint                 */
	lda.w	pc, avr32_invinst		/* EVBA+0x20 Illegal Opcode             */
	lda.w	pc, avr32_unimpinst		/* EVBA+0x24 Unimplemented instruction  */
	lda.w	pc, avr32_priv			/* EVBA+0x28 Privilege violation        */
	lda.w	pc, avr32_fp			/* EVBA+0x2c Floating-point             */
	lda.w	pc, avr32_cop			/* EVBA+0x30 Coprocessor absent         */
	lda.w	pc, avr32_rddata		/* EVBA+0x34 Data Address (Read)        */
	lda.w	pc, avr32_wrdata		/* EVBA+0x38 Data Address (Write)       */
	lda.w	pc, avr32_tddtlbprot	/* EVBA+0x3c DTLB Protection (Read)     */
	lda.w	pc, avr32_wrdtlbprot	/* EVBA+0x40 DTLB Protection (Write)    */
	lda.w	pc, avr32_dltbmod		/* EVBA+0x44 DTLB Modified              */
	.rept	2
	lda.w	pc,	avr32_badvector		/* EVBA+0x48-0x4c No such vector        */
	.endr
	lda.w	pc, avr32_itlbmiss		/* EVBA+0x50 ITLB Miss                  */
	.rept	3
	lda.w	pc,	avr32_badvector		/* EVBA+0x54-0x5c No such vector        */
	.endr
	lda.w	pc, avr32_rddtlb		/* EVBA+0x60 DTLB Miss (Read)           */
	.rept	3
	lda.w	pc,	avr32_badvector		/* EVBA+0x64-0x6c No such vector        */
	.endr
	lda.w	pc, avr32_wrdtlb		/* EVBA+0x70 DTLB Miss (Write)          */
	.rept	(3+4*8)
	lda.w	pc,	avr32_badvector		/* EVBA+0x74-0xfc No such vector        */
	.endr
	lda.w	pc, avr32_super			/* EVBA+0x100 Supervisor call           */

/****************************************************************************
 * Interrupts
 ****************************************************************************/

/* The interrupt controller must provide an address that is relative to the */
/* the EVBA so it is natural to define these interrupt vectors just after   */
/* the exception table. On entry to each interrupt handler, R8-R12, LR, PC  */
/* and SR have already been pushed onto the system stack by the MCU.        */
/*                                                                          */
/* An interrupt may disappear while it is being fetched by the CPU and,     */
/* hence spurious interrupt may result.                                     */

	.balign 4
	.global	avr32_int0
avr32_int0:
	mov		r12, 0				/* r12=interrupt level 0                    */
	rjmp	avr32_intcommon		/* Jump to common interrupt handling logic  */

	.balign	4
	.global	avr32_int1
avr32_int1:
	mov		r12, 1				/* r12=interrupt level 1                    */
	rjmp	avr32_intcommon		/* Jump to common interrupt handling logic  */

	.balign	4
	.global	avr32_int2
avr32_int2:
	mov		r12, 2				/* r12=interrupt level 2                    */
	rjmp	avr32_intcommon		/* Jump to common interrupt handling logic  */

	.balign	4
	.global	avr32_int3
avr32_int3:
	mov		r12, 3				/* r12=interrupt level 3                    */

/* Common interrupt handling logic.  R12 holds the interrupt level index    */

avr32_intcommon:
	mcall	.Lavr32_intirqno	/* Get the IRQ number of the int0 event     */
	cp.w	r12, 0				/* Negative returned if spurious interrupt  */
	brge	avr32_common		/* Jump to the common xcptn handling logic  */
	rete						/* Ignore spurious interrupt                */

.Lavr32_intirqno:
	.word	avr32_intirqno

/****************************************************************************
 * Exception Vector Handlers
 ****************************************************************************/

/* Exception Handlers:                                                      */
/* On entry to each, the context save area looks like this:                 */
/*     xx xx xx xx xx xx xx xx xx xx xx xx xx xx xx SR PC                   */
/*                                                  ^     ^+2*4             */
 
	HANDLER	avr32_unrec, AVR32_IRQ_UNREC			/* Unrecoverable xcptn  */
	HANDLER	avr32_tlbmult, AVR32_IRQ_TLBMULT		/* TLB multiple hit     */
	HANDLER	avr32_busdata, AVR32_IRQ_BUSDATA		/* Bus error data fetch */
	HANDLER	avr32_businst, AVR32_IRQ_BUSINST		/* Bus err instr fetch  */
	HANDLER	avr32_nmi, AVR32_IRQ_NMI				/* NMI                  */
	HANDLER	avr32_instaddr, AVR32_IRQ_INSTADDR		/* Instruction Address  */
	HANDLER	avr32_itlbrot, AVR32_IRQ_ITLBPROT		/* ITLB Protection      */
	HANDLER	avr32_bp, AVR32_IRQ_BP					/* Breakpoint           */
	HANDLER	avr32_invinst, AVR32_IRQ_INVINST		/* Illegal Opcode       */
	HANDLER	avr32_unimpinst, AVR32_IRQ_UNIMPINST	/* Unimplemented intsr  */
	HANDLER	avr32_priv, AVR32_IRQ_PRIV 				/* Privilege violation  */
	HANDLER	avr32_fp, AVR32_IRQ_FP					/* Floating-point       */
	HANDLER	avr32_cop, AVR32_IRQ_COP				/* Coprocessor absent   */
	HANDLER	avr32_rddata, AVR32_IRQ_RDDATA			/* Data Address (RD)    */
	HANDLER	avr32_wrdata, AVR32_IRQ_WRDATA 			/* Data Address (WR)    */
	HANDLER	avr32_tddtlbprot, AVR32_IRQ_RDDTLBPROT	/* DTLB Protection (RD) */
	HANDLER	avr32_wrdtlbprot, AVR32_IRQ_WRDTLBPROT	/* DTLB Protection (WR) */
	HANDLER	avr32_dltbmod, AVR32_IRQ_DLTBMOD		/* DTLB Modified        */
	HANDLER	avr32_itlbmiss, AVR32_IRQ_ITLBMISS		/* ITLB Miss            */
	HANDLER	avr32_rddtlb, AVR32_IRQ_RDDTLB 			/* DTLB Miss (RD)       */
	HANDLER	avr32_wrdtlb, AVR32_IRQ_WRDTLB			/* DTLB Miss (WR)       */
	HANDLER	avr32_super, AVR32_IRQ_SUPER			/* Supervisor call      */
	HANDLER	avr32_badvector, AVR32_IRQ_BADVECTOR	/* No such vector       */

/* Common exception handling logic.  Unlike the interrupt handlers, the     */
/* exception handlers do not save R8-R12, and LR on the stack.  Only the PC */
/* and SR have been pushed onto the system stack by the MCU.  The following */
/* logic creates a common stack frame for exception handlers prior to       */
/* joining to the common interrupt/exception logic below.                   */
/*                                                                          */
/* The context save area looks like this on entry to the HANDLER above.     */
/*     xx xx xx xx xx xx xx xx xx xx xx xx xx xx xx SR PC                   */
/*                                                  ^     ^+2*4             */
/* Upon joining common logic here, the context save are will loke:          */
/*     xx xx xx xx xx xx xx xx xx xx xx xx xx xx 10 SR PC                   */
/*                                               ^        ^+3*4             */
/* and r10 will hold the exception's IRQ number                             */
/*                                                                          */
/* either (1) non-time-critical, or (2) fatal.  Obvious, that would not be  */
/* the case if TLB missing handling is required.  Such time-critical vector */
/* handling should be handled differently.                                  */

avr32_xcptcommon:
/* Save r10-r12, lr on the stack:                                           */
/*     xx xx xx xx xx xx xx xx xx xx xx LR 12 11 10 SR PC                   */
/*                                      ^           ^+4*4 ^+6*4             */

	stm		--sp, r11-r12, lr

/* Move SR and PC into the expected position:                               */
/*     xx xx xx xx xx xx xx xx xx SR PC LI 12 11 10 SR PC                   */
/*                                ^                       ^+8*4             */

	ld.w	r12, sp[4*4]
	ld.w	r11, sp[5*4]
	stm		--sp, r11-r12

/* Save r8 and r8:                                                          */
/*     xx xx xx xx xx xx xx xx xx SR PC LI 12 11 10 SR PC                   */
/*                                ^                 ^+6*4 ^+8*4             */
 
	st.w	sp[6*4], r9
	st.w	sp[7*4], r8

/* Move the IRQ number in r12 and fall through to the common event handling */
/* logic.                                                                   */

	mov		r12, r10

/****************************************************************************
 * Common Event Handling Logic
 ****************************************************************************/

/* After this point, logic to manage interrupts and exceptions is           */
/* equivalent. Here we have:                                                */
/*                                                                          */
/*   R8-R12, LR, SR, and the PC on the stack.                               */
/*   R12 holds the IRQ number to be dispatched.                             */
/*                                                                          */
/* The context save area looks like this:                                   */
/*     xx xx xx xx xx xx xx xx xx SR PC LR 12 11 10 09 08                   */
/*                                ^                       ^+8*4             */
/* This function will finish construction of the register save structure    */
/* and call the IRQ dispatching logic.                                      */

avr32_common:
    /* Disable interrupts in the current SR.  This is necessary because the */
	/* AVR32 permits nested interrupts (if they are of higher priority).    */
	/* We can support nested interrupts without some effort because:        */
	/* - The global variable current_regs permits only one interrupt,       */
	/* - If CONFIG_ARCH_INTERRUPTSTACK is defined, then there is a single   */
	/*   interrupt stack, and                                               */
	/* - Probably other things.                                             */

	ssrf	AVR32_SR_GM_SHIFT

	/* Save the SP (as it was before the interrupt) in the conext save      */
	/* structure.                                                           */
	/*     xx xx xx xx xx xx xx xx SP SR PC LR 12 11 10 09 08               */
	/*                             ^sp                                      */

	mov		r8, sp
	sub		r8, -8*4
	st.w	--sp, r8

	/* Saving R0-R7 is all that is left to complete the context save.       */
	/*     07 06 05 04 03 02 01 00 SP SR PC LR 12 11 10 09 08               */
	/*     ^sp                                                              */

	stm		--sp, r0-r7

	/* Now call up_doirq passing the IRQ number in r12 and the base address */
	/* of the register context save area in r11.                            */

	mov		r11, sp

	/* Switch to the interrrupt stack if so configured.  Move the current   */
	/* stack pointer into a preserved register (r7) and set the interrupt   */
	/* stack pointer.                                                       */

#if CONFIG_ARCH_INTERRUPTSTACK > 3
	mov		r7, sp
	lddpc	sp, .Linstackbaseptr
#endif

	/* Call up_doirq with r12=IRQ number and r11=register save area         */

	mcall	.Ldoirqptr

	/* Restore the user stack pointer.                                      */
	/*     07 06 05 04 03 02 01 00 SP SR PC LR 12 11 10 09 08               */
	/*     ^sp                                                              */

#if CONFIG_ARCH_INTERRUPTSTACK > 3
	mov		sp, r7
#endif

	/* On return, r12 will hold the new address of the register context     */
	/* save area.  On an interrupt contex switch, this will (1) not be the  */
	/* same as the value of r12 passed to up_doirq(), and (2) may not       */
	/* reside on a stack.                                                   */

	cp.w	sp, r12
	brne	1f

	/* No context switch... do the simple return.  First, restore r0-r7.    */
	/*     xx xx xx xx xx xx xx xx SP SR PC LR 12 11 10 09 08               */
	/*                             ^sp                                      */

	ldm		sp++, r0-r7

	/* Skip over the saved stack pointer and return from the interrupt.     */
	/*     xx xx xx xx xx xx xx xx xx SR PC LR 12 11 10 09 08               */
	/*                                ^sp                                   */

	sub		sp, -4
	rete

	/* Context switch... jump to up_fullcontestrestor with r12=address of   */
	/* the task context to restore.                                         */

1:
	lddpc	pc, .Lfullcontextrestoreptr

.Ldoirqptr:
	.word	up_doirq
.Lfullcontextrestoreptr:
	.word	up_fullcontextrestore

#if CONFIG_ARCH_INTERRUPTSTACK > 3
.Linstackbaseptr:
	.word	.Lintstackbase
#endif
	.size	vectortab, .-vectortab

/************************************************************************************
 *  Name: up_interruptstack
 ************************************************************************************/

#if CONFIG_ARCH_INTERRUPTSTACK > 3
	.bss
	.align	4
	.globl	up_interruptstack
	.type	up_interruptstack, object
up_interruptstack:
	.skip	(CONFIG_ARCH_INTERRUPTSTACK & ~3)
.Lintstackbase:
	.size	up_interruptstack, .-up_interruptstack
#endif
	.end

