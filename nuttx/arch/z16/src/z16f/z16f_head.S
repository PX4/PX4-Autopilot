/**************************************************************************
 * arch/z16/src/z16f/z16f_head.S
 * Z16F Reset Entry Point
 *
 *   Copyright (C) 2008, 2012 Gregory Nutt. All rights reserved.
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
 * Included Files
 **************************************************************************/

#include <nuttx/config.h>
#include <arch/irq.h>
#include "common/up_internal.h"

/**************************************************************************
 * Definitions
 **************************************************************************/

/**************************************************************************
 * External References / External Definitions
 **************************************************************************/

	xref	_z16f_clkinit:EROM
	xref	_z16f_lowinit:EROM
#ifdef CONFIG_ARCH_LEDS
	xref	_up_ledinit:EROM
#endif
#if defined(USE_LOWUARTINIT)
	xref	_z16f_lowuartinit:EROM
#elif defined(USE_EARLYSERIALINIT)
	xref	_up_earlyserialinit:EROM
#endif
	xref	_os_start:EROM
	xref	_up_doirq:EROM
	xref	_z16f_sysexec:EROM
	xdef	_z16f_reset

	xref	_low_nearbss:RAM
	xref	_len_nearbss
	xref	_low_farbss:ERAM
	xref	_len_farbss:ERAM
	xref	_low_neardata:RAM
	xref	_len_neardata
	xref	_low_near_romdata:EROM
	xref	_low_fardata:ERAM
	xref	_len_fardata:ERAM
	xref	_low_far_romdata:EROM
	xref	far_heapbot:ERAM
	xref	_far_stack:ERAM
	xref	_near_stack:RAM

/**************************************************************************
 * Flash Option Byte Setup
 **************************************************************************/

	define FOPTIONSEG, SPACE=ROM, ORG=0
	segment FOPTIONSEG
	db %FF
	db %FF
	db %FF
	db %FF

/**************************************************************************
 * vectors
 **************************************************************************/

	vector RESET=_z16f_reset
	vector SYSEXC=_sysexc_isr
	vector TIMER2=_timer2_isr
	vector TIMER1=_timer1_isr
	vector TIMER0=_timer0_isr
	vector UART0_RX=_uart0rx_isr
	vector UART0_TX=_uart0tx_isr
	vector I2C=_i2c_isr
	vector SPI=_spi_isr
	vector ADC=_adc_isr
	vector P7AD=_p7ad_isr
	vector P6AD=_p6ad_isr
	vector P5AD=_p5ad_isr
	vector P4AD=_p4ad_isr
	vector P3AD=_p3ad_isr
	vector P2AD=_p2ad_isr
	vector P1AD=_p1ad_isr
	vector P0AD=_p0ad_isr
	vector PWM_TIMER=_pwmtimer_isr
	vector UART1_RX=_uart1rx_isr
	vector UART1_TX=_uart1tx_isr
	vector PWM_FAULT=_pwmfault_isr
	vector C3=_c3_isr
	vector C2=_c3_isr
	vector C1=_c2_isr
	vector C0=_c0_isr
 
/**************************************************************************
 * Equates
 **************************************************************************/

/**************************************************************************
 * Data Allocation
 **************************************************************************/
 
/**************************************************************************
 * Code
 **************************************************************************/

	define CODESEG, SPACE=EROM
	segment CODESEG

/**************************************************************************
 * Name: _z16f_reset
 *
 * Description:
 *   Reset entry point
 *
 **************************************************************************/

_z16f_reset:
	/* Initialize the init/idle task stack */

	ld	sp, #(_near_stack+1)	/* Set Stack Pointer to the top of internal RAM */
	clr	fp

	/* Initialize clocking */

	call	_z16f_clkinit

	/* Initialize onboard LEDs */

#ifdef CONFIG_ARCH_LEDS
	call	_up_ledinit
#endif
	/* Perform VERY early UART initialization so that we can use it here */

#ifdef USE_LOWUARTINIT
	call	_z16f_lowuartinit	/* Initialize the UART for debugging */
#endif
	/* Initialize the hardware stack overflow register */

#ifdef CONFIG_Z16F_INITSPOV
	ld	r0, #(_near_stack_bot+1)
	ld	spov, r0
#endif
	/* Clear BSS */

	lea	r0, _low_nearbss
	ld	r1, #_len_nearbss+1
	jp	_z16f_reset2
_z16f_reset1:
	ld.b	(r0++), #0
_z16f_reset2:
	djnz	r1, _z16f_reset1

	lea	r0, _low_farbss
	ld	r1, #_len_farbss+1
	jp	_z16f_reset4
_z16f_reset3:
	ld.b	(r0++), #0
_z16f_reset4:
	djnz	r1, _z16f_reset3

	/* Copy ROM data into RAM */

	lea	r0, _low_near_romdata
	lea	r1, _low_neardata
	ld	r2, #_len_neardata+1
        jp      _z16f_reset6
_z16f_reset5:
        ld.b    r3, (r0++)
	ld.b	(r1++), r3
_z16f_reset6:
        djnz    r2, _z16f_reset5

       	lea	r0, _low_far_romdata
	lea	r1, _low_fardata
	ld	r2, #_len_fardata+1
	jp	_z16f_reset8
_z16f_reset7:
	ld.b	r3, (r0++)
	ld.b	(r1++), r3
_z16f_reset8:
	djnz	r2, _z16f_reset7

	/* Perform low-level hardware initialization */

	call	_z16f_lowinit		/* Perform low-level hardware initialization */

#ifdef USE_EARLYSERIALINIT
	/* Perform early serial initialization */

	call	_up_earlyserialinit
#endif
	/* Start NuttX */

	call	_os_start		/* Start the operating system */
_halt1:					/* _os_start() should not return */
	halt
	jp	_halt1

/**************************************************************************
 * Name: _sysexec_isr
 *
 * Description:
 *   System exception interrupt handler.  On entry, the stack looks like
 *   this:
 *
 *   TOS[0] = PC[31:24]
 *   TOS[1] = PC[23:16]
 *   TOS[2] = PC[15:8]
 *   TOS[3] = PC[7:0]
 *   TOS[4] = 0
 *   TOS[5] = flags
 *
 **************************************************************************/

_sysexc_isr:
	pushmlo	<r0-r7>		/* Save r0-r7 on the stack */
	
	/* Calculate the value of the SP BEFORE the interrupt occurred and
	 * push that as the saved value of r15=sp
	 */

	ld	r1, #-6		/* return(4) + flags(1) + padding(1) */
	add	r1, sp		/* r1 = Value of the SP before the interrupt */
	push	r1		/* Push r1 in the spot for the saved SP */

	/* Save all of the remaining registers */

	pushmhi	<r8-r14>

	/* SP now holds the address of the beginning of the save structure
	 * on the stack.  Now handle the system exception with arg1(r1)=address
	 * of the register save structure.
	 */

	ld	r1, sp
	call	_z16f_sysexec	/* Handle in C logic */

_halt2:				/* _z16f_sysexec() should not return */
	halt
	jp	_halt2

/**************************************************************************
 * Name: Interrupt handlers
 *
 * Description:
 *   All interrupts will be vectored to the following locations.
 *   On entry, the stack contains the following:
 *
 *   TOS[0] = PC[31:24]
 *   TOS[1] = PC[23:16]
 *   TOS[2] = PC[15:8]
 *   TOS[3] = PC[7:0]
 *   TOS[4] = 0
 *   TOS[5] = flags
 *
 **************************************************************************/

_timer2_isr:
	pushmlo	<r0-r7>			/* Save r0-r7 on the stack */
	ld	r1, #Z16F_IRQ_TIMER2	/* r1 = Timer 2 IRQ number */
	jp	_common_isr		/* Join common interrupt handling logic */

_timer1_isr:
	pushmlo	<r0-r7>			/* Save r0-r7 on the stack */
	ld	r1, #Z16F_IRQ_TIMER1	/* r1 = Timer 1 IRQ number */
	jp	_common_isr		/* Join common interrupt handling logic */

_timer0_isr:
	pushmlo	<r0-r7>			/* Save r0-r7 on the stack */
	ld	r1, #Z16F_IRQ_TIMER0	/* r1 = Timer 0 IRQ number */
	jp	_common_isr		/* Join common interrupt handling logic */

_uart0rx_isr:
	pushmlo	<r0-r7>			/* Save r0-r7 on the stack */
	ld	r1, #Z16F_IRQ_UART0RX	/* r1 = UART0 RX IRQ number */
	jp	_common_isr		/* Join common interrupt handling logic */

_uart0tx_isr:
	pushmlo	<r0-r7>			/* Save r0-r7 on the stack */
	ld	r1, #Z16F_IRQ_UART0TX	/* r1 = UART0 TX IRQ number */
	jp	_common_isr		/* Join common interrupt handling logic */

_i2c_isr:
	pushmlo	<r0-r7>			/* Save r0-r7 on the stack */
	ld	r1, #Z16F_IRQ_I2C	/* r1 = I2C IRQ number */
	jp	_common_isr		/* Join common interrupt handling logic */

_spi_isr:
	pushmlo	<r0-r7>			/* Save r0-r7 on the stack */
	ld	r1, #Z16F_IRQ_SPI	/* r1 = SPI IRQ number */
	jp	_common_isr		/* Join common interrupt handling logic */

_adc_isr:
	pushmlo	<r0-r7>			/* Save r0-r7 on the stack */
	ld	r1, #Z16F_IRQ_ADC	/* r1 = ADC IRQ number */
	jp	_common_isr		/* Join common interrupt handling logic */

_p7ad_isr:
	pushmlo	<r0-r7>			/* Save r0-r7 on the stack */
	ld	r1, #Z16F_IRQ_P7AD	/* r1 = Port A/D7, rising/falling edge IRQ number */
	jp	_common_isr		/* Join common interrupt handling logic */

_p6ad_isr:
	pushmlo	<r0-r7>			/* Save r0-r7 on the stack */
	ld	r1, #Z16F_IRQ_P6AD	/* r1 = Port A/D6, rising/falling edge IRQ number */
	jp	_common_isr		/* Join common interrupt handling logic */

_p5ad_isr:
	pushmlo	<r0-r7>			/* Save r0-r7 on the stack */
	ld	r1, #Z16F_IRQ_P5AD	/* r1 = Port A/D5, rising/falling edge IRQ number */
	jp	_common_isr		/* Join common interrupt handling logic */

_p4ad_isr:
	pushmlo	<r0-r7>			/* Save r0-r7 on the stack */
	ld	r1, #Z16F_IRQ_P4AD	/* r1 = Port A/D4, rising/falling edge IRQ number */
	jp	_common_isr		/* Join common interrupt handling logic */

_p3ad_isr:
	pushmlo	<r0-r7>			/* Save r0-r7 on the stack */
	ld	r1, #Z16F_IRQ_P3AD	/* r1 = Port A/D3, rising/falling edgeEXEC IRQ number */
	jp	_common_isr		/* Join common interrupt handling logic */

_p2ad_isr:
	pushmlo	<r0-r7>			/* Save r0-r7 on the stack */
	ld	r1, #Z16F_IRQ_P2AD	/* r1 = Port A/D2, rising/falling edge IRQ number */
	jp	_common_isr		/* Join common interrupt handling logic */

_p1ad_isr:
	pushmlo	<r0-r7>			/* Save r0-r7 on the stack */
	ld	r1, #Z16F_IRQ_P1AD	/* r1 = Port A/D1, rising/falling edge IRQ number */
	jp	_common_isr		/* Join common interrupt handling logic */

_p0ad_isr:
	pushmlo	<r0-r7>			/* Save r0-r7 on the stack */
	ld	r1, #Z16F_IRQ_P0AD	/* r1 = Port A/D0, rising/falling edge IRQ number */
	jp	_common_isr		/* Join common interrupt handling logic */

_pwmtimer_isr:
	pushmlo	<r0-r7>			/* Save r0-r7 on the stack */
	ld	r1, #Z16F_IRQ_PWMTIMER	/* r1 = PWM Timer IRQ number */
	jp	_common_isr		/* Join common interrupt handling logic */

_uart1rx_isr:
	pushmlo	<r0-r7>			/* Save r0-r7 on the stack */
	ld	r1, #Z16F_IRQ_UART1RX	/* r1 = UART1 RX IRQ number */
	jp	_common_isr		/* Join common interrupt handling logic */

_uart1tx_isr:
	pushmlo	<r0-r7>			/* Save r0-r7 on the stack */
	ld	r1, #Z16F_IRQ_UART1TX	/* r1 = UART1 TX IRQ number */
	jp	_common_isr		/* Join common interrupt handling logic */

_pwmfault_isr:
	pushmlo	<r0-r7>			/* Save r0-r7 on the stack */
	ld	r1, #Z16F_IRQ_PWMFAULT	/* r1 = PWM Fault IRQ number */
	jp	_common_isr		/* Join common interrupt handling logic */

_c3_isr:
	pushmlo	<r0-r7>			/* Save r0-r7 on the stack */
	ld	r1, #Z16F_IRQ_C3	/* r1 = Port C3, both edges DMA3 IRQ number */
	jp	_common_isr		/* Join common interrupt handling logic */

_c2_isr:
	pushmlo	<r0-r7>			/* Save r0-r7 on the stack */
	ld	r1, #Z16F_IRQ_C2	/* r1 = Port C2, both edges DMA2 IRQ number */
	jp	_common_isr		/* Join common interrupt handling logic */

_c1_isr:
	pushmlo	<r0-r7>			/* Save r0-r7 on the stack */
	ld	r1, #Z16F_IRQ_C1	/* r1 = Port C1, both edges DMA1 IRQ number */
	jp	_common_isr		/* Join common interrupt handling logic */

_c0_isr:
	pushmlo	<r0-r7>			/* Save r0-r7 on the stack */
	ld	r1, #Z16F_IRQ_C0	/* r1 = Port C0, both edges DMA0 IRQ number */
					/* Join common interrupt handling logic */


/**************************************************************************
 * Name: _common_isr
 *
 * Description:
 *   Common interrupt handling logic.
 *
 *   On entry:
 *
 *   r1 = IRQ number
 *
 *   And the stack contains the following:
 *
 *   TOS[ 0- 3] = r0
 *   TOS[ 4- 7] = r1
 *   TOS[ 8-11] = r2
 *   TOS[12-15] = r3
 *   TOS[16-19] = r4
 *   TOS[20-23] = r5
 *   TOS[24-27] = r6
 *   TOS[28-31] = r7
 *   TOS[32-35] = return PC
 *   TOS[36-37] = flags (with padding)
 *
 **************************************************************************/

_common_isr:
	/* Calculate the value of the SP BEFORE the interrupt occurred and
	 * push that as the saved value of r15=sp
	 */

	ld	r2, #(9*4+2)	/* See stack accounting above */
	add	r2, sp		/* r1 = Value of the SP before the interrupt */
	push	r2		/* Push r1 in the spot for the saved SP */

	/* Save all of the remaining registers */

	pushmhi	<r8-r14>

	/* SP now holds the address of the beginning of the save structure
	 * on the stack.  Now handle the interrupt with arg1(r1)=IRQ number and
	 * arg2(r2)=address of the register save structure.
	 */

	ld	r2, sp
	call	_up_doirq

	/* Upon return, _up_doirq will provide that address of the save structure
	 * to use to return from the interrupt in r0.  This may or may not be the
	 * same value as sp.
	 */

	cp	r0, sp			/* Check if we are performing a context switch */
	jp	nz, _common_switch	/* Jump if yes, else use faster return */
	popmhi	<r8-r14>		/* Restore r8-r14 */
	add	sp, #4			/* Skip over restore of r15=sp */
	popmlo	<r0-r7>			/* Restore r0-r7 */
	iret

	/* We are not returning to the same thread that was interrupted.  In this case,
	 * r0 is not in the stack but, instead, refers to a storage structure in the TCB
	 */

_common_switch:
	ld	sp, 2*REG_SP(r0)	/* sp=Value of SP on return from interrupt */
	ld.w	r1, 2*REG_FLAGS(r0)	/* r1=padded flags value */
	push.w	r1			/* Push padded flags value onto the stack */
	ld	r1, 2*REG_PC(r0)	/* r1=return address */
	push	r1			/* Push the return address onto the stack */
	ld	r7, 2*REG_R7(r0)	/* Recover saved r7 */
	push	r7			/* And save on the stack so that we can use r7 */
	ld	r7, sp			/* r7=saved sp */

	ld	sp, r0			/* sp=Pointer to register save structure */
	popmhi	<r8-r14>		/* Restore r8-r14 */
	add	sp, #4			/* Skip over restore of r15=sp */
	popmlo	<r0-r6>			/* Restore r0-r6 */
	ld	sp, r7			/* Switch back to the correct stack */
	pop	r7			/* Recover r7 from the stack */
	iret				/* Return from interrupt */

	end
