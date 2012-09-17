/************************************************************
 * up_head.S
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
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
 ************************************************************/

/************************************************************
 * Included Files
 ************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>
#include "up_internal.h"

	.module	up_head
	.optsdcc -mmcs51 --model-large

/************************************************************
 * Private Data
 ************************************************************/

	.area REG_BANK_0	(REL,OVR,DATA)
	.ds 8

#ifndef CONFIG_8052_TIMER2
	.area	XSEG
_g_timer0tick:
	.ds 1
#endif

/************************************************************
 * Public Data
 ************************************************************/

	.globl	_g_irqtos
	.globl	_g_irqcontext
	.globl	_g_irqregs

/************************************************************
 * Public Functions
 ************************************************************/

	.globl	_irq_dispatch
	.globl	_up_restoreregisters

/************************************************************
 * Program entry points
 ************************************************************/

/* Program entry is through PROGRAM_BASE.  This is just a
 * branch to our start up logic.
 */

	.area	CODE1 (ABS)
	.org	PROGRAM_BASE
	ljmp	start

/* These are indirect interrupt vectors.  Logic in PAULMON2,
 * captures the interrupt vectors (near address 0x0000) and
 * re-routes them through the following entry points.
 *
 * Each of these saves acc and ie then passes the IRQ number
 * to higher level logic in a
 */

	.org	PM2_VECTOR_EXTINT0
	push	acc
	mov	a, #EXT_INT0_IRQ
	ljmp	_up_interrupt

	.org	PM2_VECTOR_TIMER0
	push	acc
#ifdef CONFIG_8052_TIMER2
	mov	a, #TIMER0_IRQ
	ljmp	_up_interrupt
#else
	ljmp	_up_timer0
#endif
	.org	PM2_VECTOR_EXTINT1
	push	acc
	mov	a, #EXT_INT1_IRQ
	ljmp	_up_interrupt

	.org	PM2_VECTOR_TIMER1
	push	acc
	mov	a, #TIMER1_IRQ
	ljmp	_up_interrupt

	.org	PM2_VECTOR_UART
	push	acc
	mov	a, #UART_IRQ
	ljmp	_up_interrupt

	.org	PM2_VECTOR_TIMER2
	push	acc
	mov	a, #TIMER2_IRQ
	ljmp	_up_interrupt

/************************************************************
 * Name: start
 *
 * Description:
 *   This is the initial entry point into NuttX
 *
 ************************************************************/

start:
	mov	sp, #(STACK_BASE-1)

#ifdef CONFIG_ARCH_LEDS
	lcall	_up_ledinit
#endif

	ljmp	_os_start

/************************************************************
 * Name: up_timer0
 *
 * Description:
 *   Timer 0, mode 0 can be used as a system timer.  In that
 *   mode, the 1.8432 MHz clock is divided by 32.  A single
 *   8-bit value is incremented at 57600 Hz, which results
 *   in 225 Timer 0 overflow interrupts per second.
 *
 *   The Timer0 interrupt vectors to this point which then
 *   does a software divide by 2 to get a system timer of
 *   112.5Hz.
 *
 *   On Entry:
 *
 *   (1) acc on the stack and
 *   (2) the IRQ number(TIMER0_IRQ) in the accumulator
 *
 ************************************************************/

#ifndef CONFIG_8052_TIMER2
_up_timer0:
	ar2 = 0x02
	ar3 = 0x03
	ar4 = 0x04
	ar5 = 0x05
	ar6 = 0x06
	ar7 = 0x07
	ar0 = 0x00
	ar1 = 0x01

	/* ACC already on the stack; push IE.  Then disable interrupts */

	push	ie
	clr	ea

	/* Save the remaining registers with interrupts disabled 
	 *
	 * a, ie, and dptr go on the stack.
	 */

	push	dpl
	push	dph

	/* Increment the tick counter */

	mov	dptr, #_g_timer0tick
        movx    a, @dptr
	inc	a
	movx	@dptr, a

	/* If bit 0 is '0', then just return from the interrupt */

	anl	a, #0x01
	jnz	00101$
	ljmp	_up_timer0exit

	/* If bit 0 is '1', then process the interrupt */

00101$:
	mov	a, #TIMER0_IRQ
	sjmp	_up_timer0join
#endif

/************************************************************
 * Name: up_interrupt
 *
 * Description:
 *   All interrupts vector to this point with:
 *
 *   (1) acc on the stack and
 *   (2) the IRQ number in the accumulator
 *
 ************************************************************/

_up_interrupt:
	ar2 = 0x02
	ar3 = 0x03
	ar4 = 0x04
	ar5 = 0x05
	ar6 = 0x06
	ar7 = 0x07
	ar0 = 0x00
	ar1 = 0x01

	/* ACC already on the stack; push IE.  Then disable interrupts */

	push	ie
	clr	ea

	/* Save the remaining registers with interrupts disabled 
	 *
	 * a, ie, and dptr go on the stack.
	 */

	push	dpl
	push	dph

_up_timer0join:
	/* Other registers go into the IRQ register save area */

	push	acc
	mov	dptr, #_g_irqregs
	lcall	_up_saveregisters

	/* Show interrupt status on the LEDs */

#ifdef CONFIG_ARCH_LEDS
	mov	dpl, #LED_INIRQ
	lcall	_up_ledon
#endif

	/* Save the IRQ number in r2 */

	pop	ar2

	/* Mark that we are in an interrupt and provide the top
	 * of stack pointer to the context switching logic.
	 */

	mov	dptr, #_g_irqtos
	mov	a, sp
        movx    @dptr, a

	/* Nullify the context pointer.  If a context switch is
	 * needed, this will be set to the address of the context
	 * structure.
	 */

	mov	dptr, #_g_irqcontext
	clr	a
	movx	@dptr,a
	inc	dptr
	movx	@dptr,a

	/* Now call void irq_dispatch(int irq, FAR void *context)
         *
	 * First, create the first argument as (int)irqno
	 */

        mov     dpl, r2
	mov	dph, #0

        /* Create the second argument (void *context) on the stack */

	push	sp
	clr	a
	push	acc

	/* Then dispatch the IRQ. */

	lcall	_irq_dispatch
	pop	acc
	pop	acc

	/* Indicate that we are no longer in an interrupt */

	mov	dptr, #_g_irqtos
	clr	a
        movx    @dptr, a

	/* Check if a context switch is pending */

	mov	dptr,#_g_irqcontext
	movx	a, @dptr
	mov     r2, a
	inc	dptr
	movx	a, @dptr
	mov	r3, a

	orl	a, r2
	jnz	00001$

	/* No context switch is pending.  Restore registers
	 * from the interrupt register save area.
	 */

	mov	dptr, #_g_irqregs
	sjmp	00004$

00001$: /****************************************************/

	/* A context switch is pending, clear g_irqcontext */

	mov	dpl, r2
	mov	dph, r3
	clr	a
	movx	@dptr, a
	inc	dptr
	movx	@dptr, a

#ifdef CONFIG_INTERRUPT_FRAME_DUMP
	mov	dpl, r2
	mov	dph, r3
	push	ar2
	push	ar3
	lcall	_up_dumpframe
	pop	ar3
	pop	ar2
#endif

	/* Register usage in the following:
	 *
	 * R0   - Holds working the 8-bit IRAM pointer
	 * R1   - Not used
	 * R2-3 - Holds the working 16-bit XRAM pointer
	 * R4   - Holds the working byte count
	 * R5   - Holds the new stack pointer
	 * R6-7 - Not used
	 */

	/* Fetch r4 = context->nbytes */

	mov	dpl, r2
	mov	dph, r3
	movx	a, @dptr
	mov	r4, a

	/* Save the new stack pointer in r5 */

	add	a, #(STACK_BASE-1)
	mov	r5, a

	/* Save r2-3 = &context->stack */

	inc	dptr
	push	dpl
	push	dph
	mov	r2, dpl
	mov	r3, dph

	/* Set r0 = stack base address */

	mov	r0, #STACK_BASE

	/* Top of the copy loop */
00002$:
	mov	a, r4	/* a = bytes left to transfer */
	dec	r4	/* (for next time through the loop) */
	jz	00003$	/* Jump if a = 0 (done) */

	/* Fetch the next byte from context->stack */

	mov	dpl, r2
	mov	dph, r3
	movx	a,@dptr

	/* Increment the XRAM pointer */

	inc	dptr
	mov	r2, dpl
	mov	r3, dph

	/* Save the next byte into IRAM */

	mov	@r0, a

	/* Increment the IRAM pointer */

	inc	r0
	sjmp	00002$

	/* The entire stack has been copied from XRAM into
	 * IRAM.  Set the new stack pointer
	 */

00003$:
	pop	dph
	pop	dpl
	mov	sp, r5

#ifdef CONFIG_INTERRUPT_FRAME_DUMP
	push	dpl
	push	dph
	lcall	_up_dumpstack
	pop	dph
	pop	dpl
#endif
	/* Get the pointer to the register save area */

	mov	a, #STACK_SIZE
	add	a, dpl
	mov	dpl, a
	clr	a
	addc	a, dph
	mov	dph, a
	
00004$: /****************************************************/

	/* Restore the context from the register save area
	 * and return from the interrupt.  At this point, dptr
	 * holds the pointer to the memory region that holds
	 * the register save area. This could be either
	 * g_irqregs (no context switch) or &g_irqcontext->regs
	 * (context switch).
	 */

#ifdef CONFIG_ARCH_LEDS
	push	dpl
	push	dph
	mov	dpl, #LED_INIRQ
	lcall	_up_ledoff
	pop	dph
	pop	dpl
#endif
	/* Restore registers from the register save area */

	lcall	_up_restoreregisters

_up_timer0exit:
	/* Restore registers from the stack and return */

	pop	dph
	pop	dpl

	/* Restore the interrupt state per the stored IE value */

	pop	acc
	jb	acc.7,00005$
	clr	ie.7
	sjmp	00006$
00005$:
	setb	ie.7

00006$:
	pop	acc
	reti
