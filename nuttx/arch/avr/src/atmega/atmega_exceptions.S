/********************************************************************************************
 * arch/avr/src/atmega/atmega_exceptions.S
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
 ********************************************************************************************/

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include <arch/irq.h>

#include "excptmacros.h"

/********************************************************************************************
 * External Symbols
 ********************************************************************************************/

	.file	"atmega_exceptions.S"
	.global	up_doirq
	.global	up_fullcontextrestore

/********************************************************************************************
 * Macros
 ********************************************************************************************/

/********************************************************************************************
 * Exception Vector Handlers
 ********************************************************************************************/

	.section .handlers, "ax", @progbits

	HANDLER	atmega_int0, ATMEGA_IRQ_INT0, excpt_common			/* External interrupt request 0 */
	HANDLER	atmega_int1, ATMEGA_IRQ_INT1, excpt_common			/* External interrupt request 1 */
	HANDLER	atmega_int2, ATMEGA_IRQ_INT2, excpt_common			/* External interrupt request 2 */
	HANDLER	atmega_int3, ATMEGA_IRQ_INT3, excpt_common			/* External interrupt request 3 */
	HANDLER	atmega_int4, ATMEGA_IRQ_INT4, excpt_common			/* External interrupt request 4 */
	HANDLER	atmega_int5, ATMEGA_IRQ_INT5, excpt_common			/* External interrupt request 5 */
	HANDLER	atmega_int6, ATMEGA_IRQ_INT6, excpt_common			/* External interrupt request 6 */
	HANDLER	atmega_int7, ATMEGA_IRQ_INT7, excpt_common			/* External interrupt request 7 */
	HANDLER	atmega_t2comp, ATMEGA_IRQ_T2COMP, excpt_common		/* TIMER2 COMP timer/counter2 compare match */
	HANDLER	atmega_t2ovf, ATMEGA_IRQ_T2OVF, excpt_common		/* TIMER2 OVF timer/counter2 overflow */
	HANDLER	atmega_t1capt, ATMEGA_IRQ_T1CAPT, excpt_common		/* TIMER1 CAPT timer/counter1 capture event */
	HANDLER	atmega_t1compa, ATMEGA_IRQ_T1COMPA, excpt_common	/* TIMER1 COMPA timer/counter1 compare match a */
	HANDLER	atmega_t1compb, ATMEGA_IRQ_T1COMPB, excpt_common	/* TIMER1 COMPB timer/counter1 compare match b */
	HANDLER	atmega_t1ovf, ATMEGA_IRQ_T1OVF, excpt_common		/* TIMER1 OVF timer/counter1 overflow */
	HANDLER	atmega_t0comp, ATMEGA_IRQ_T0COMP, excpt_common		/* TIMER0 COMP timer/counter0 compare match */
	HANDLER	atmega_t0ovf, ATMEGA_IRQ_T0OVF, excpt_common		/* TIMER0 OVF timer/counter0 overflow */
	HANDLER	atmega_spi, ATMEGA_IRQ_SPI, excpt_common			/* STC SPI serial transfer complete */
	HANDLER	atmega_u0rx, ATMEGA_IRQ_U0RX, excpt_common			/* USART0 RX complete */
	HANDLER	atmega_u0dre, ATMEGA_IRQ_U0DRE, excpt_common		/* USART0 data register empty */
	HANDLER	atmega_u0tx, ATMEGA_IRQ_U0TX, excpt_common			/* USART0 TX complete */
	HANDLER	atmega_adc, ATMEGA_IRQ_ADC, excpt_common			/* ADC conversion complete */
	HANDLER	atmega_ee, ATMEGA_IRQ_EE, excpt_common				/* EEPROM ready */
	HANDLER	atmega_anacomp, ATMEGA_IRQ_ANACOMP, excpt_common	/* ANALOG COMP analog comparator */
	HANDLER	atmega_t1compc, ATMEGA_IRQ_T1COMPC, excpt_common	/* TIMER1 COMPC timer/countre1 compare match c */
	HANDLER	atmega_t3capt, ATMEGA_IRQ_T3CAPT, excpt_common		/* TIMER3 CAPT timer/counter3 capture event */
	HANDLER	atmega_t3compa, ATMEGA_IRQ_T3COMPA, excpt_common	/* TIMER3 COMPA timer/counter3 compare match a */
	HANDLER	atmega_t3compb, ATMEGA_IRQ_T3COMPB, excpt_common	/* TIMER3 COMPB timer/counter3 compare match b */
	HANDLER	atmega_t3compc, ATMEGA_IRQ_T3COMPC, excpt_common	/* TIMER3 COMPC timer/counter3 compare match c */
	HANDLER	atmega_t3ovf, ATMEGA_IRQ_T3OVF, excpt_common		/* TIMER3 OVF timer/counter3 overflow */
	HANDLER	atmega_u1rx, ATMEGA_IRQ_U1RX, excpt_common			/* USART1 RX complete */
	HANDLER	atmega_u1dre, ATMEGA_IRQ_U1DRE, excpt_common		/* USART1 data register empty */
	HANDLER	atmega_u1tx, ATMEGA_IRQ_U1TX, excpt_common			/* USART1 TX complete */
	HANDLER	atmega_twi, ATMEGA_IRQ_TWI, excpt_common			/* TWI two-wire serial interface */
	HANDLER	atmega_spmrdy, ATMEGA_IRQ_SPMRDY, excpt_common		/* Store program memory ready */

/********************************************************************************************
 * Name: excpt_common
 *
 * Description:
 *   Exception Vector Handlers
 *
 * On Entry:
 *	The return PC and the saved r24 is on the stack, r24 now contains the IRQ number
 *
 *	  PCL
 *	  PCH
 *	  R0
 *	  --- <- SP
 *
 ********************************************************************************************/

excpt_common:
	/* Save the remaining registers on the stack, preserving the IRQ number in r14 */

	EXCPT_PROLOGUE

	/* Call up_doirq with r24 = IRQ number, r22-23 = Pointer to the save structure.  The stack
	 * pointer currently points to the save structure (or maybe the save struture -1 since
	 * the push operation post-decrements -- need to REVISIT this).
	 */

	in		r28, _SFR_IO_ADDR(SPL)	/* Get the save structure pointer in a Call-saved register pair */
	in		r29, _SFR_IO_ADDR(SPH)  /* Pointer can be obtained from the stack pointer */
	adiw	r28, 1					/* Remembering that push post-decrements */
	movw	r22, r28				/* Pass register save structure as the parameter 2 */
	USE_INTSTACK rx, ry, rz			/* Switch to the interrupt stack */
	call	up_doirq				/* Dispatch the interrupt */
	RESTORE_STACK rx, ry			/* Undo the operations of USE_INTSTACK */

	/* up_doiq returns with r24-r25 equal to the new save structure.  If no context
	 * switch occurred, this will be the same as the value passed to it in r22-23.
	 * But if a context switch occurs, then the returned value will point not at a
	 * stack frame, but at a register save area inside of the new task's TCB.
	 */

	 cp		r28, r24
	 cpc	r29, r25
	 breq	.Lnoswitch

	/* A context switch has occurred, jump to up_fullcontextrestore with r24, r25
	 * equal to the address of the new register save ared.
	 */

	jmp	up_fullcontextrestore

	/* No context switch occurred.. just return off the stack */

.Lnoswitch:
	EXCPT_EPILOGUE
	reti

/****************************************************************************************************
 *  Name: up_interruptstack
 ****************************************************************************************************/

#if CONFIG_ARCH_INTERRUPTSTACK > 0
	.bss
	.align	4
	.globl	up_interruptstack
	.type	up_interruptstack, object
up_interruptstack:
	.skip	CONFIG_ARCH_INTERRUPTSTACK
.Lintstackbase:
	.size	up_interruptstack, .-up_interruptstack
#endif
	.end

