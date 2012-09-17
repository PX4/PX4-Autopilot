/************************************************************************************************
 * arch/arm/src/lpc17xx/lpc17_vectors.S
 * arch/arm/src/chip/lpc17_vectors.S
 *
 *   Copyright (C) 2010-2011 Gregory Nutt. All rights reserved.
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
 ************************************************************************************************/

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

#include <nuttx/config.h>
#include <arch/irq.h>

/************************************************************************************************
 * Preprocessor Definitions
 ************************************************************************************************/

/* Memory Map:
 *
 * 0x0000:0000 - Beginning of FLASH. Address of vectors
 * 0x0003:ffff - End of flash
 * 0x1000:0000 - Start of CPU SRAM and start of .data (_sdata)
 *             - End of .data (_edata) and start of .bss (_sbss)
 *             - End of .bss (_ebss) and bottom of idle stack
 *             - _ebss + CONFIG_IDLETHREAD_STACKSIZE = end of idle stack, start of heap. NOTE
 *               that the ARM uses a decrement before store stack so that the correct initial
 *               value is the end of the stack + 4;
 * 0x1000:7fff - End of CPU SRAM and end of heap (1st region)
 */

#define IDLE_STACK		(_ebss+CONFIG_IDLETHREAD_STACKSIZE)
#define HEAP_BASE		(_ebss+CONFIG_IDLETHREAD_STACKSIZE)

/* The Cortex-M3 return from interrupt is unusual.  We provide the following special
 * address to the BX instruction.  The particular value also forces a return to
 * thread mode and covers state from the main stack point, the MSP (vs. the MSP).
 */

#define EXC_RETURN		0xfffffff9

/************************************************************************************************
 * Global Symbols
 ************************************************************************************************/

	.globl		__start

	.syntax		unified
	.thumb
	.file		"lpc17_vectors.S"

/************************************************************************************************
 * Macros
 ************************************************************************************************/

/* On entry into an IRQ, the hardware automatically saves the xPSR, PC, LR, R12, R0-R3
 * registers on the stack, then branches to an instantantiation of the following
 * macro.  This macro simply loads the IRQ number into R0, then jumps to the common
 * IRQ handling logic.
 */

	.macro	HANDLER, label, irqno
	.thumb_func
\label:
	mov		r0, #\irqno
	b		lpc17_common
	.endm

/************************************************************************************************
 * Vectors
 ************************************************************************************************/

	.section	.vectors, "ax"
	.code		16
	.align		2
	.globl		lpc17_vectors
	.type		lpc17_vectors, function

lpc17_vectors:

/* Processor Exceptions */

	.word	IDLE_STACK			/* Vector  0: Reset stack pointer */
	.word	__start				/* Vector  1: Reset vector */
	.word	lpc17_nmi			/* Vector  2: Non-Maskable Interrupt (NMI) */
	.word	lpc17_hardfault		/* Vector  3: Hard fault */
	.word	lpc17_mpu			/* Vector  4: Memory management (MPU) */
	.word	lpc17_busfault		/* Vector  5: Bus fault */
	.word	lpc17_usagefault	/* Vector  6: Usage fault */
	.word	lpc17_reserved		/* Vector  7: Reserved */
	.word	lpc17_reserved		/* Vector  8: Reserved */
	.word	lpc17_reserved		/* Vector  9: Reserved */
	.word	lpc17_reserved		/* Vector 10: Reserved */
	.word	lpc17_svcall		/* Vector 11: SVC call */
	.word	lpc17_dbgmonitor	/* Vector 12: Debug monitor */
	.word	lpc17_reserved		/* Vector 13: Reserved */
	.word	lpc17_pendsv		/* Vector 14: Pendable system service request */
	.word	lpc17_systick		/* Vector 15: System tick */

/* External Interrupts */

	.word	lpc17_wdt			/* Vector 16+0:  Watchdog timer */
	.word	lpc17_tmr0			/* Vector 16+1:  Timer 0 */
	.word	lpc17_tmr1			/* Vector 16+2:  Timer 1 */
	.word	lpc17_tmr2			/* Vector 16+3:  Timer 2 */
	.word	lpc17_tmr3			/* Vector 16+4:  Timer 3 */
	.word	lpc17_uart0			/* Vector 16+5:  UART 0 */
	.word	lpc17_uart1			/* Vector 16+6:  UART 1 */
	.word	lpc17_uart2			/* Vector 16+7:  UART 2 */
	.word	lpc17_uart3			/* Vector 16+8:  UART 3 */
	.word	lpc17_pwm1			/* Vector 16+9:  PWM */
	.word	lpc17_i2c0			/* Vector 16+10: I2C 0 */
	.word	lpc17_i2c1			/* Vector 16+11: I2C 1 */
	.word	lpc17_i2c2			/* Vector 16+12: I2C 2 */
	.word	lpc17_spif			/* Vector 16+13: SPI */
	.word	lpc17_ssp0			/* Vector 16+14: SSP 0 */
	.word	lpc17_ssp1			/* Vector 16+15: SSP 1 */
	.word	lpc17_pll0			/* Vector 16+16: PLL 0 */
	.word	lpc17_rtc			/* Vector 16+17: Real time clock */
	.word	lpc17_eint0			/* Vector 16+18: External interrupt 0 */
	.word	lpc17_eint1			/* Vector 16+19: External interrupt 1 */
	.word	lpc17_eint2			/* Vector 16+20: External interrupt 2 */
	.word	lpc17_eint3			/* Vector 16+21: External interrupt 3 */
	.word	lpc17_adc			/* Vector 16+22: A/D Converter */
	.word	lpc17_bod			/* Vector 16+23: Brown Out detect */
	.word	lpc17_usb			/* Vector 16+24: USB */
	.word	lpc17_can			/* Vector 16+25: CAN */
	.word	lpc17_gpdma			/* Vector 16+26: GPDMA */
	.word	lpc17_i2s			/* Vector 16+27: I2S */
	.word	lpc17_eth			/* Vector 16+28: Ethernet */
	.word	lpc17_ritint		/* Vector 16+29: Repetitive Interrupt Timer */
	.word	lpc17_mcpwm			/* Vector 16+30: Motor Control PWM */
	.word	lpc17_qei			/* Vector 16+31: Quadrature Encoder */
	.word	lpc17_pll1			/* Vector 16+32: PLL 1 */
	.word	lpc17_usbact		/* Vector 16+33: USB Activity Interrupt */
	.word	lpc17_canact		/* Vector 16+34: CAN Activity Interrupt */
	.size	lpc17_vectors, .-lpc17_vectors

/************************************************************************************************
 * .text
 ************************************************************************************************/

	.text
	.type	handlers, function
	.thumb_func
handlers:
	HANDLER	lpc17_reserved, LPC17_IRQ_RESERVED		/* Unexpected/reserved vector */
	HANDLER	lpc17_nmi, LPC17_IRQ_NMI				/* Vector  2: Non-Maskable Interrupt (NMI) */
	HANDLER	lpc17_hardfault, LPC17_IRQ_HARDFAULT	/* Vector  3: Hard fault */
	HANDLER	lpc17_mpu, LPC17_IRQ_MEMFAULT			/* Vector  4: Memory management (MPU) */
	HANDLER	lpc17_busfault, LPC17_IRQ_BUSFAULT		/* Vector  5: Bus fault */
	HANDLER	lpc17_usagefault, LPC17_IRQ_USAGEFAULT	/* Vector  6: Usage fault */
	HANDLER	lpc17_svcall, LPC17_IRQ_SVCALL			/* Vector 11: SVC call */
	HANDLER	lpc17_dbgmonitor, LPC17_IRQ_DBGMONITOR	/* Vector 12: Debug Monitor */
	HANDLER	lpc17_pendsv, LPC17_IRQ_PENDSV			/* Vector 14: Penable system service request */
	HANDLER	lpc17_systick, LPC17_IRQ_SYSTICK		/* Vector 15: System tick */

	HANDLER	lpc17_wdt, LPC17_IRQ_WDT				/* Vector 16+0:  Watchdog timer */
	HANDLER	lpc17_tmr0, LPC17_IRQ_TMR0				/* Vector 16+1:  Timer 0 */
	HANDLER	lpc17_tmr1, LPC17_IRQ_TMR1 				/* Vector 16+2:  Timer 1 */
	HANDLER	lpc17_tmr2, LPC17_IRQ_TMR2 				/* Vector 16+3:  Timer 2 */
	HANDLER	lpc17_tmr3, LPC17_IRQ_TMR3 				/* Vector 16+4:  Timer 3 */
	HANDLER	lpc17_uart0, LPC17_IRQ_UART0 			/* Vector 16+5:  UART 0 */
	HANDLER	lpc17_uart1, LPC17_IRQ_UART1			/* Vector 16+6:  UART 1 */
	HANDLER	lpc17_uart2, LPC17_IRQ_UART2			/* Vector 16+7:  UART 2 */
	HANDLER	lpc17_uart3, LPC17_IRQ_UART3			/* Vector 16+8:  UART 3 */
	HANDLER	lpc17_pwm1, LPC17_IRQ_PWM1				/* Vector 16+9:  PWM 1 */
	HANDLER	lpc17_i2c0, LPC17_IRQ_I2C0				/* Vector 16+10: I2C 0 */
	HANDLER	lpc17_i2c1, LPC17_IRQ_I2C1				/* Vector 16+11: I2C 1 */
	HANDLER	lpc17_i2c2, LPC17_IRQ_I2C2 				/* Vector 16+12: I2C 2 */
	HANDLER	lpc17_spif, LPC17_IRQ_SPIF				/* Vector 16+13: SPI */
	HANDLER	lpc17_ssp0, LPC17_IRQ_SSP0				/* Vector 16+14: SSP 0 */
	HANDLER	lpc17_ssp1, LPC17_IRQ_SSP1 				/* Vector 16+15: SSP 1 */
	HANDLER	lpc17_pll0, LPC17_IRQ_PLL0				/* Vector 16+16: PLL 0 */
	HANDLER	lpc17_rtc, LPC17_IRQ_RTC				/* Vector 16+17: Real time clock */
	HANDLER	lpc17_eint0, LPC17_IRQ_EINT0			/* Vector 16+18: External interrupt 0 */
	HANDLER	lpc17_eint1, LPC17_IRQ_EINT1			/* Vector 16+19: External interrupt 1 */
	HANDLER	lpc17_eint2, LPC17_IRQ_EINT2 			/* Vector 16+20: External interrupt 2 */
	HANDLER	lpc17_eint3, LPC17_IRQ_EINT3			/* Vector 16+21: External interrupt 3 */
	HANDLER	lpc17_adc, LPC17_IRQ_ADC 				/* Vector 16+22: A/D Converter */
	HANDLER	lpc17_bod, LPC17_IRQ_BOD				/* Vector 16+23: Brown Out detect */
	HANDLER	lpc17_usb, LPC17_IRQ_USB				/* Vector 16+24: USB */
	HANDLER	lpc17_can, LPC17_IRQ_CAN				/* Vector 16+25: CAN */
	HANDLER	lpc17_gpdma, LPC17_IRQ_GPDMA			/* Vector 16+26: GPDMA */
	HANDLER	lpc17_i2s, LPC17_IRQ_I2S				/* Vector 16+27: I2S */
	HANDLER	lpc17_eth, LPC17_IRQ_ETH				/* Vector 16+28: Ethernet */
	HANDLER	lpc17_ritint, LPC17_IRQ_RITINT			/* Vector 16+29: Repetitive Interrupt Timer */
	HANDLER	lpc17_mcpwm, LPC17_IRQ_MCPWM 			/* Vector 16+30: Motor Control PWM */
	HANDLER	lpc17_qei, LPC17_IRQ_QEI 				/* Vector 16+31: Quadrature Encoder */
	HANDLER	lpc17_pll1, LPC17_IRQ_PLL1				/* Vector 16+32: PLL 1 */
	HANDLER	lpc17_usbact, LPC17_IRQ_USBACT			/* Vector 16+33: USB Activity Interrupt */
	HANDLER	lpc17_canact, LPC17_IRQ_CANACT			/* Vector 16+34: CAN Activity Interrupt */

/* Common IRQ handling logic.  On entry here, the return stack is on either
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
 *      R0 contains the IRQ number
 *      R14 Contains the EXC_RETURN value
 *      We are in handler mode and the current SP is the MSP
 */

lpc17_common:

	/* Complete the context save */

#ifdef CONFIG_NUTTX_KERNEL
	/* The EXC_RETURN value will be 0xfffffff9 (privileged thread) or 0xfffffff1
	 * (handler mode) if the state is on the MSP.  It can only be on the PSP if
	 * EXC_RETURN is 0xfffffffd (unprivileged thread)
	 */

	adds	r2, r14, #3				/* If R14=0xfffffffd, then r2 == 0 */
	ite		ne						/* Next two instructions are condition */
	mrsne	r1, msp					/* R1=The main stack pointer */
	mrseq	r1, psp					/* R1=The process stack pointer */
#else
	mrs		r1, msp					/* R1=The main stack pointer */
#endif

	mov		r2, r1					/* R2=Copy of the main/process stack pointer */
	add		r2, #HW_XCPT_SIZE		/* R2=MSP/PSP before the interrupt was taken */
	mrs		r3, primask				/* R3=Current PRIMASK setting */
#ifdef CONFIG_NUTTX_KERNEL 
	stmdb	r1!, {r2-r11,r14}		/* Save the remaining registers plus the SP value */
#else
	stmdb	r1!, {r2-r11}			/* Save the remaining registers plus the SP value */
#endif

	/* Disable interrupts, select the stack to use for interrupt handling
	 * and call up_doirq to handle the interrupt
	 */

	cpsid	i						/* Disable further interrupts */

	/* If CONFIG_ARCH_INTERRUPTSTACK is defined, we will use a special interrupt
	 * stack pointer.  The way that this is done here prohibits nested interrupts!
	 * Otherwise, we will re-use the main stack for interrupt level processing.
	 */

#if CONFIG_ARCH_INTERRUPTSTACK > 3
	ldr		sp, =g_intstackbase
	str		r1, [sp, #-4]!			/* Save the MSP on the interrupt stack */
	bl		up_doirq				/* R0=IRQ, R1=register save (msp) */
	ldr		r1, [sp, #+4]!			/* Recover R1=main stack pointer */
#else
	mov		sp, r1					/* We are using the main stack pointer */
	bl		up_doirq				/* R0=IRQ, R1=register save (msp) */
	mov		r1, sp					/* Recover R1=main stack pointer */
#endif

	/* On return from up_doirq, R0 will hold a pointer to register context
	 * array to use for the interrupt return.  If that return value is the same
	 * as current stack pointer, then things are relatively easy.
	 */

	cmp		r0, r1					/* Context switch? */
	beq		1f						/* Branch if no context switch */

	/* We are returning with a pending context switch.  This case is different
	 * because in this case, the register save structure does not lie on the
	 * stack but, rather, are within a TCB structure.  We'll have to copy some
	 * values to the stack.
	 */

	add		r1, r0, #SW_XCPT_SIZE	/* R1=Address of HW save area in reg array */
	ldmia	r1, {r4-r11}			/* Fetch eight registers in HW save area */
	ldr		r1, [r0, #(4*REG_SP)]	/* R1=Value of SP before interrupt */
	stmdb	r1!, {r4-r11}			/* Store eight registers in HW save area */
#ifdef CONFIG_NUTTX_KERNEL 
	ldmia	r0, {r2-r11,r14}		/* Recover R4-R11, r14 + 2 temp values */
#else
	ldmia	r0, {r2-r11}			/* Recover R4-R11 + 2 temp values */
#endif
	b		2f						/* Re-join common logic */

	/* We are returning with no context switch.  We simply need to "unwind"
	 * the same stack frame that we created
	 */
1:
#ifdef CONFIG_NUTTX_KERNEL 
	ldmia	r1!, {r2-r11,r14}		/* Recover R4-R11, r14 + 2 temp values */
#else
	ldmia	r1!, {r2-r11}			/* Recover R4-R11 + 2 temp values */
#endif
2:
#ifdef CONFIG_NUTTX_KERNEL
	/* The EXC_RETURN value will be 0xfffffff9 (privileged thread) or 0xfffffff1
	 * (handler mode) if the state is on the MSP.  It can only be on the PSP if
	 * EXC_RETURN is 0xfffffffd (unprivileged thread)
	 */

	adds	r0, r14, #3				/* If R14=0xfffffffd, then r0 == 0 */
	ite		ne						/* Next two instructions are condition */
	msrne	msp, r1					/* R1=The main stack pointer */
	msreq	psp, r1					/* R1=The process stack pointer */
#else
	msr		msp, r1					/* Recover the return MSP value */

	/* Preload r14 with the special return value first (so that the return
	 * actually occurs with interrupts still disabled).
	 */

	ldr		r14, =EXC_RETURN		/* Load the special value */
#endif

    /* Restore the interrupt state */

	msr		primask, r3				/* Restore interrupts */

	/* Always return with R14 containing the special value that will: (1)
	 * return to thread mode, and (2) continue to use the MSP
	 */

	bx		r14						/* And return */
	.size	handlers, .-handlers

/************************************************************************************************
 *  Name: up_interruptstack/g_intstackbase
 *
 * Description:
 *   Shouldn't happen
 *
 ************************************************************************************************/

#if CONFIG_ARCH_INTERRUPTSTACK > 3
	.bss
	.global	g_intstackbase
	.align	4
up_interruptstack:
	.skip	(CONFIG_ARCH_INTERRUPTSTACK & ~3)
g_intstackbase:
	.size	up_interruptstack, .-up_interruptstack
#endif

/************************************************************************************************
 * .rodata
 ************************************************************************************************/

	.section	.rodata, "a"

/* Variables: _sbss is the start of the BSS region (see ld.script) _ebss is the end
 * of the BSS regsion (see ld.script). The idle task stack starts at the end of BSS
 * and is of size CONFIG_IDLETHREAD_STACKSIZE.  The IDLE thread is the thread that
 * the system boots on and, eventually, becomes the idle, do nothing task that runs
 * only when there is nothing else to run.  The heap continues from there until the
 * end of memory.  See g_heapbase below.
 */

	.globl	g_heapbase
	.type	g_heapbase, object
g_heapbase:
	.word	HEAP_BASE
	.size	g_heapbase, .-g_heapbase

	.end
