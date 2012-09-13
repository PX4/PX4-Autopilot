/************************************************************************************
 * arch/sh/src/m16c/m16c_vectors.S
 *
 *	 Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *	 Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *		notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *		notice, this list of conditions and the following disclaimer in
 *		the documentation and/or other materials provided with the
 *		distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *		used to endorse or promote products derived from this software
 *		without specific prior written permission.
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
#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Macro Definitions
 ************************************************************************************/

/* Select register bank 1, and pass the IRQ number to _m16c_commonvector */

	.macro	m16c_vector, label, irqno
\label:
	fset	b
	mov.w	#\irqno, r1
	jmp.w	_m16c_commonvector
	.endm

/* Special page vectors. This macro puts the jump address of	
 * functions defined as special page into the special page vector table.
 * See example calls below and see the M16C Software Manual or NC30
 * manual for more information on special page vectors.
 */
#if 0
 	.macro	m16c_special, num
	.org	0x0ffffe-(\num*2)
	.globl	__SPECIAL_\num
	.word	__SPECIAL_\num	& 0x0ffff
	.endm
#endif

/************************************************************************************
 * Data
 ************************************************************************************/

/************************************************************************************
 * Interrupt Vectors
 ************************************************************************************/

/* Variable vector section */

	.section varvects			/* Variable vector table */
	.long	_m16c_brk_isr			/* ffd00: BRK instruction */
	.long	0xffffffff			/* ffd04 */
	.long	0xffffffff			/* ffd08 */
	.long	0xffffffff			/* ffd0c */
	.long	_m16c_int3_isr			/* ffd10: INT3 */
	.long	_m16c_contextsave		/* ffd14: Reserved -- SWINT5 used by NuttX */
#ifdef CONFIG_M16C_SWINTS
	.long	_m16c_swint6_isr		/* ffd18: S/W interrupt 7 */
	.long	_m16c_swint7_isr		/* ffd1c: S/W interrupt 7 */
#else
	.long	_m16c_unexpected_isr		/* ffd1c: Reserved */
	.long	_m16c_unexpected_isr		/* ffd1c: Reserved */
#endif
	.long	_m16c_int5_isr			/* ffd20: INT5 */
	.long	_m16c_int4_isr			/* ffd24: INT4 */
	.long	_m16c_uart2bcd_isr		/* ffd28: UART2 bus collision detection */
	.long	_m16c_dma0_isr			/* ffd2c: DMA0 */
	.long	_m16c_dma1_isr			/* ffd30: DMA1 */
	.long	_m16c_keyinp_isr		/* ffd34: Key input interrupt */
	.long	_m16c_adc_isr			/* ffd38: A-D */
	.long	_m16c_uart2xmitnack2_isr	/* ffd3c UART2 transmit/NACK2 */
	.long	_m16c_uart2rcvack2_isr		/* ffd40: UART2 receive/ACK2 */
	.long	_m16c_uart0xmit_isr		/* ffd44: UART0 transmit */
	.long	_m16c_uart0rcv_isr		/* ffd48: UART0 receive */
	.long	_m16c_uart1xmit_isr		/* ffd4c: UART1 transmit */
	.long	_m16c_uart1rcv_isr		/* ffd50: UART1 receive */
	.long	_m16c_tmra0_isr			/* ffd54: Timer A0 */
	.long	_m16c_tmra1_isr			/* ffd58: Timer A1 */
	.long	_m16c_tmra2_isr			/* ffd5c: Timer A2 */
	.long	_m16c_tmra3_isr			/* ffd60: Timer A3 */
	.long	_m16c_tmra4_isr			/* ffd64: Timer A4 */
	.long	_m16c_tmrb0_isr			/* ffd68: Timer B0 */
	.long	_m16c_tmrb1_isr			/* ffd6c: Timer B1 */
	.long	_m16c_tmrb2_isr			/* ffd70: Timer B2 */
	.long	_m16c_int0_isr			/* ffd74: INT0 */
	.long	_m16c_int1_isr			/* ffd78: INT1 */
#ifdef CONFIG_M16C_SWINTS
	.long	_m16c_swint31_isr		/* ffd7c: S/W interrupt 31 */
	.long	_m16c_contextrestore		/* ffd80: S/W interrupt 32 by NuttX */
	.long	_m16c_swint33_isr		/* ffd84: S/W interrupt 33 */
	.long	_m16c_swint34_isr		/* ffd88: S/W interrupt 34 */
	.long	_m16c_swint35_isr		/* ffd8c: S/W interrupt 35 */
	.long	_m16c_swint36_isr		/* ffd90: S/W interrupt 36 */
	.long	_m16c_swint37_isr		/* ffd94: S/W interrupt 37 */
	.long	_m16c_swint38_isr		/* ffd98: S/W interrupt 38 */
	.long	_m16c_swint39_isr		/* ffd9c: S/W interrupt 39 */
	.long	_m16c_swint40_isr		/* ffda0: S/W interrupt 40 */
	.long	_m16c_swint41_isr		/* ffda4: S/W interrupt 41 */
	.long	_m16c_swint42_isr		/* ffda8: S/W interrupt 42 */
	.long	_m16c_swint43_isr		/* ffdac: S/W interrupt 43 */
	.long	_m16c_swint44_isr		/* ffdb0: S/W interrupt 44 */
	.long	_m16c_swint45_isr		/* ffdb4: S/W interrupt 45 */
	.long	_m16c_swint46_isr		/* ffdb8: S/W interrupt 46 */
	.long	_m16c_swint47_isr		/* ffdbc: S/W interrupt 47 */
	.long	_m16c_swint48_isr		/* ffdc0: S/W interrupt 48 */
	.long	_m16c_swint49_isr		/* ffdc4: S/W interrupt 49 */
	.long	_m16c_swint50_isr		/* ffdc8: S/W interrupt 50 */
	.long	_m16c_swint51_isr		/* ffdcc: S/W interrupt 51 */
	.long	_m16c_swint52_isr		/* ffdd0: S/W interrupt 52 */
	.long	_m16c_swint53_isr		/* ffdd4: S/W interrupt 53 */
	.long	_m16c_swint54_isr		/* ffdd8: S/W interrupt 54 */
	.long	_m16c_swint55_isr		/* ffddc: S/W interrupt 55 */
	.long	_m16c_swint56_isr		/* ffde0: S/W interrupt 56 */
	.long	_m16c_swint57_isr		/* ffde4: S/W interrupt 57 */
	.long	_m16c_swint58_isr		/* ffde8: S/W interrupt 58 */
	.long	_m16c_swint59_isr		/* ffdec: S/W interrupt 59 */
	.long	_m16c_swint60_isr		/* ffdf0: S/W interrupt 60 */
	.long	_m16c_swint61_isr		/* ffdf4: S/W interrupt 61 */
	.long	_m16c_swint62_isr		/* ffdf8: S/W interrupt 62 */
	.long	_m16c_swint63_isr		/* ffdfc: S/W interrupt 63 */
#else
	.long	_m16c_unexpected_isr		/* ffd7c: Reserved */
	.long	_m16c_contextrestore		/* ffd80: S/W interrupt 32 by NuttX */
	.long	_m16c_unexpected_isr		/* ffd84: Not supported */
	.long	_m16c_unexpected_isr		/* ffd88: Not supported */
	.long	_m16c_unexpected_isr		/* ffd8c: Not supported */
	.long	_m16c_unexpected_isr		/* ffd90: Not supported */
	.long	_m16c_unexpected_isr		/* ffd94: Not supported */
	.long	_m16c_unexpected_isr		/* ffd98: Not supported */
	.long	_m16c_unexpected_isr		/* ffd9c: Not supported */
	.long	_m16c_unexpected_isr		/* ffda0: Not supported */
	.long	_m16c_unexpected_isr		/* ffda4: Not supported1 */
	.long	_m16c_unexpected_isr		/* ffda8: Not supported */
	.long	_m16c_unexpected_isr		/* ffdac: Not supported */
	.long	_m16c_unexpected_isr		/* ffdb0: Not supported */
	.long	_m16c_unexpected_isr		/* ffdb4: Not supported */
	.long	_m16c_unexpected_isr		/* ffdb8: Not supported */
	.long	_m16c_unexpected_isr		/* ffdbc: Not supported */
	.long	_m16c_unexpected_isr		/* ffdc0: Not supported */
	.long	_m16c_unexpected_isr		/* ffdc4: Not supported */
	.long	_m16c_unexpected_isr		/* ffdc8: Not supported */
	.long	_m16c_unexpected_isr		/* ffdcc: Not supported */
	.long	_m16c_unexpected_isr		/* ffdd0: Not supported */
	.long	_m16c_unexpected_isr		/* ffdd4: Not supported */
	.long	_m16c_unexpected_isr		/* ffdd8: Not supported */
	.long	_m16c_unexpected_isr		/* ffddc: Not supported */
	.long	_m16c_unexpected_isr		/* ffde0: Not supported */
	.long	_m16c_unexpected_isr		/* ffde4: Not supported */
	.long	_m16c_unexpected_isr		/* ffde8: Not supported */
	.long	_m16c_unexpected_isr		/* ffdec: Not supported */
	.long	_m16c_unexpected_isr		/* ffdf0: Not supported */
	.long	_m16c_unexpected_isr		/* ffdf4: Not supported */
	.long	_m16c_unexpected_isr		/* ffdf8: Not supported */
	.long	_m16c_unexpected_isr		/* ffdfc: Not supported */
#endif

/* Fixed vector section
 *
 * The fixed vector table begins at address ffe00.  The firt portion
 * of the fixed vector table is the special page table.  This table
 * is not currently used.
 */
 	.section specpg				/* Special page table */
 /* Nothing */
	.section fixvects			/* Fixed vector table */
	.long	_m16c_undefinst_irq		/* fffdc: Undefined instruction */
	.long	_m16c_overflow_irq		/* fffe0: Overflow */
	.long	_m16c_brkinst_irq		/* fffe4: BRK instruction */
	.long	_m16c_addrmatch_irq		/* fffe8: Address match */
#ifdef CONFIG_M16C_DEBUGGER
	.long	_m16c_sstep_irq			/* fffec: Single step */
#else
	.long	_m16c_unexpected_isr		/* fffec: Not supported */
#endif
	.long	_m16c_wdog_irq			/* ffff0: Watchdog timer */
#ifdef CONFIG_M16C_DEBUGGER
	.long	_m16c_dbc_irq			/* ffff4: DBC */
#else
	.long	_m16c_unexpected_isr		/* ffff4: Not supported */
#endif
	.long	_m16c_nmi_irq			/* ffff8: NMI */
	.long	__start				/* ffffc: Reset */

/************************************************************************************
 * Code
 ************************************************************************************/
/************************************************************************************
 * Name: m16c_*isr
 *
 * Description:
 *   Handler interrupt events. The CPU performs the following actions when an
 *   interrupt is taken:
 *
 *  - Save FLG register
 *  - Clear I, D, and U flags in FLG register
 *  - Builds stack frame like (on the push-down, interrupt stack):
 * 
 *    sp   -> PC bits 0-7
 *    sp+1 -> PC bits 8-15
 *    sp+2 -> FLG bits 0-7
 *    sp+3 -> FLG (Bits 12-14) + PC (bits 16-19)
 *
 *  - Sets IPL
 *  - Vectors to interrupt handler
 *
 ************************************************************************************/

 	.text
/* The unexpected interrupt vector */

 	m16c_vector	_m16c_unexpected_isr, NR_IRQS

 /* Variable vectors */
 
	m16c_vector	_m16c_brk_isr, M16C_BRK_IRQ
	m16c_vector	_m16c_int3_isr, M16C_INT3_IRQ
	m16c_vector	_m16c_int5_isr, M16C_INT5_IRQ
	m16c_vector	_m16c_int4_isr, M16C_INT4_IRQ
	m16c_vector	_m16c_uart2bcd_isr, M16C_UART2BCD_IRQ
	m16c_vector	_m16c_dma0_isr, M16C_DMA0_IRQ
	m16c_vector	_m16c_dma1_isr, M16C_DMA1_IRQ
	m16c_vector	_m16c_keyinp_isr, M16C_KEYINP_IRQ
	m16c_vector	_m16c_adc_isr, M16C_ADC_IRQ
	m16c_vector	_m16c_uart2xmitnack2_isr, M16C_UARTXNAK_IRQ
	m16c_vector	_m16c_uart2rcvack2_isr, M16C_UARTRACK_IRQ
	m16c_vector	_m16c_uart0xmit_isr, M16C_UART0XMT_IRQ
	m16c_vector	_m16c_uart0rcv_isr, M16C_UART0RCV_IRQ
	m16c_vector	_m16c_uart1xmit_isr, M16C_UART1XMT_IRQ
	m16c_vector	_m16c_uart1rcv_isr, M16C_UART1RCV_IRQ
	m16c_vector	_m16c_tmra0_isr, M16C_TMRA0_IRQ
	m16c_vector	_m16c_tmra1_isr, M16C_TMRA1_IRQ
	m16c_vector	_m16c_tmra2_isr, M16C_TMRA2_IRQ
	m16c_vector	_m16c_tmra3_isr, M16C_TMRA3_IRQ
	m16c_vector	_m16c_tmra4_isr, M16C_TMRA4_IRQ
	m16c_vector	_m16c_tmrb0_isr, M16C_TMRB0_IRQ
	m16c_vector	_m16c_tmrb1_isr, M16C_TMRB1_IRQ
	m16c_vector	_m16c_tmrb2_isr, M16C_TMRB2_IRQ
	m16c_vector	_m16c_int0_isr, M16C_INT0_IRQ
	m16c_vector	_m16c_int1_isr, M16C_INT1_IRQ

#ifdef CONFIG_M16C_SWINTS
	m16c_vector	_m16c_swint5_isr, M16C_SWINT5_IRQ
	m16c_vector	_m16c_swint6_isr, M16C_SWINT6_IRQ
	m16c_vector	_m16c_swint7_isr, M16C_SWINT7_IRQ
	m16c_vector	_m16c_swint31_isr, M16C_SWINT31_IRQ
	m16c_vector	_m16c_swint32_isr, M16C_SWINT32_IRQ
	m16c_vector	_m16c_swint33_isr, M16C_SWINT33_IRQ
	m16c_vector	_m16c_swint34_isr, M16C_SWINT34_IRQ
	m16c_vector	_m16c_swint35_isr, M16C_SWINT35_IRQ
	m16c_vector	_m16c_swint36_isr, M16C_SWINT36_IRQ
	m16c_vector	_m16c_swint37_isr, M16C_SWINT37_IRQ
	m16c_vector	_m16c_swint38_isr, M16C_SWINT38_IRQ
	m16c_vector	_m16c_swint39_isr, M16C_SWINT39_IRQ
	m16c_vector	_m16c_swint40_isr, M16C_SWINT40_IRQ
	m16c_vector	_m16c_swint41_isr, M16C_SWINT41_IRQ
	m16c_vector	_m16c_swint42_isr, M16C_SWINT42_IRQ
	m16c_vector	_m16c_swint43_isr, M16C_SWINT43_IRQ
	m16c_vector	_m16c_swint44_isr, M16C_SWINT44_IRQ
	m16c_vector	_m16c_swint45_isr, M16C_SWINT45_IRQ
	m16c_vector	_m16c_swint46_isr, M16C_SWINT46_IRQ
	m16c_vector	_m16c_swint47_isr, M16C_SWINT47_IRQ
	m16c_vector	_m16c_swint48_isr, M16C_SWINT48_IRQ
	m16c_vector	_m16c_swint49_isr, M16C_SWINT49_IRQ
	m16c_vector	_m16c_swint50_isr, M16C_SWINT50_IRQ
	m16c_vector	_m16c_swint51_isr, M16C_SWINT51_IRQ
	m16c_vector	_m16c_swint52_isr, M16C_SWINT52_IRQ
	m16c_vector	_m16c_swint53_isr, M16C_SWINT53_IRQ
	m16c_vector	_m16c_swint54_isr, M16C_SWINT54_IRQ
	m16c_vector	_m16c_swint55_isr, M16C_SWINT55_IRQ
	m16c_vector	_m16c_swint56_isr, M16C_SWINT56_IRQ
	m16c_vector	_m16c_swint57_isr, M16C_SWINT57_IRQ
	m16c_vector	_m16c_swint58_isr, M16C_SWINT58_IRQ
	m16c_vector	_m16c_swint59_isr, M16C_SWINT59_IRQ
	m16c_vector	_m16c_swint60_isr, M16C_SWINT60_IRQ
	m16c_vector	_m16c_swint61_isr, M16C_SWINT61_IRQ
	m16c_vector	_m16c_swint62_isr, M16C_SWINT62_IRQ
	m16c_vector	_m16c_swint63_isr, M16C_SWINT63_IRQ
#endif

/* Fixed vectors */

	m16c_vector	_m16c_undefinst_irq, M16C_UNDEFINST_IRQ
	m16c_vector	_m16c_overflow_irq, M16C_OVERFLOW_IRQ
	m16c_vector	_m16c_brkinst_irq, M16C_BRK_IRQ
	m16c_vector	_m16c_addrmatch_irq, M16C_ADDRMATCH_IRQ
	m16c_vector	_m16c_wdog_irq, M16C_WDOG_IRQ
	m16c_vector	_m16c_nmi_irq, M16C_NMI_IRQ
#ifdef CONFIG_M16C_DEBUGGER
	m16c_vector	_m16c_sstep_irq, M16C_SSTEP_IRQ
	m16c_vector	_m16c_dbc_irq, M16C_DBC_IRQ
#endif

/* At this point, the stack remains as it was on interrupt.  The interrupt
 * stack is selected, register bank 1 is selected and r1 holds the IRQ
 * number.
 */

_m16c_commonvector:

/* At this point, the interrupt stack has 4 bytes of info on it. */

/* Save all registers on the interrupt stack */

	fclr	b				/* Switch back to Bank 0 */
	pushm	fb,sb,a1,a0,r3,r2,r1,r0		/* Save on interrupt stack */
	fset	b				/* Bank to bank 1 */

/* Save the user stack pointer on the interrupt stack */

	fset	u				/* Switch to User stack */
	stc	sp, r0				/* R0 = user stack pointer */
	fclr	u				/* Back to interrupt stack */
	push.w	r0				/* Save user sp on isp */

/* Allow nested interrupts */

#ifdef M16C_INTERRUPT_IPL
	fset	i				/* Enable interrupts */
	ldipl	#M16C_INTERRUPT_IPL		/* Set interrupt level */
#endif

/* Then call _up_doirq with r1=IRQ number, r2=address of context info.  At this
 * point, the interrupt stack holds the address of the last byte of the context
 * info array
 */

 	stc	isp, r2				/* R2 = address of base of context info */
	jsr.a	_up_doirq

#ifdef M16C_INTERRUPT_IPL
	fclr	i				/* Disable interrupts */
#endif

/* Upon return, r0 will hold address of the base of the new context info structure
 * use for return.  Most of the time this will be the same as the address passed to
 * to _up_doirg above, but will differ if a context switch occurs during interrupt
 * processing.
 *
 * Of the ISP to the end of the context array:
 */
 
 	ldc	r0, isp				/* ISP = address of base of new context info */
 
 /* Restore the user stack pointer */
 
 	pop.w	r0				/* R0 = saved user stack pointer */
	fset	u				/* Switch to User stack */
	ldc	r0, sp				/* Restore the user stack pointer */
	fclr	u				/* Back to interrupt stack */
 	
/* Recover all registers and return */

	fclr	b				/* Switch back to Bank 0 */
	popm	fb,sb,a1,a0,r3,r2,r1,r0		/* Recover registers */
	reit					/* Return from interrupt */

/************************************************************************************
 * Name: int up_saveusercontext(uint32_t *regs)
 *
 * Description:
 *   Save the context of the calling function at the point of the return from the
 *   call.  This basically a setjmp.
 *
 * Input Parameters:
 *   R1 = Address of register save array
 *
 * Returned Value:
 *   R0 = 0 on normal call; 1 on context switch
 *
 ************************************************************************************/

	.globl	_up_saveusercontext
	.type	_up_saveusercontext, #function

_up_saveusercontext:
	int	#5		/* Execute S/W interrupt 5 */
	rts			/* Then return with the correct value in r0 */

/* This the logic executes in response to S/W interrupt 5.  The 'int 5' instruction
 * will cause the following actions:
 *
 *  - Save FLG register
 *  - Clear I, D, and U flags in FLG register
 *  - Builds stack frame like (on the push-down, interrupt stack):
 * 
 *    sp   -> PC bits 0-7
 *    sp+1 -> PC bits 8-15
 *    sp+2 -> FLG bits 0-7
 *    sp+3 -> FLG (Bits 12-14) + PC (bits 16-19)
 *
 *  - Set IPL
 *  - Vectors to this S/W interrupt handler:
 */

_m16c_contextsave:

/* Save all registers on the interrupt stack */

	mov.w	#1, r0				/* Return 1 on context switch */
	pushm	fb,sb,a1,a0,r3,r2,r1,r0		/* Save on interrupt stack */

/* Save the user stack pointer on the interrupt stack */

	fset	u				/* Switch to User stack */
	stc	sp, r0				/* R0 = user stack pointer */
	fclr	u				/* Back to interrupt stack */
	push.w	r0				/* Save user sp on isp */

/* Then copy the stack content to the register context array pointed to by r1 */

 	stc	isp, a0				/* A0 = address of base of context info */
	mov.w	r1, a1				/* A1 = address of register context array */
	mov.b	#0, r1h				/* R1H = 4 MS of 20-bit source address */
	mov.w	#XCPTCONTEXT_SIZE, r3		/* R3 = Number of bytes to transfer */
	smovf.b					/* Copy ISP to near context array */

/* Then return zero to indicate a normal function call return */

	add.w	#(XCPTCONTEXT_SIZE - 4), sp	/* Remove stuff from stack */
	mov.w	#0, r0				/* Return zero */
	reit					/* Return from interrupt */
	.size	_up_saveusercontext, .-_up_saveusercontext
	
/************************************************************************************
 * Name: void up_fullcontextrestore(uint32_t *regs)
 *
 * Description:
 *   Restore the context of the using the provided register save array.
 *
 * Input Parameters:
 *   R1 = Address of register save array
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

	.globl	_up_fullcontextrestore
	.type	_up_fullcontextrestore, #function

_up_fullcontextrestore:
	int	#32		/* Execute S/W interrupt 32 */
	rts			/* and return */

/* This the logic executes in response to S/W interrupt 32.  The 'int 32' instruction
 * will cause the following actions:
 *
 *  - Save FLG register
 *  - Clear I and D flags in FLG register (U is preserved)
 *  - Builds stack frame like (on the push-down, interrupt stack):
 * 
 *    sp   -> PC bits 0-7
 *    sp+1 -> PC bits 8-15
 *    sp+2 -> FLG bits 0-7
 *    sp+3 -> FLG (Bits 12-14) + PC (bits 16-19)
 *
 *  - Set IPL
 *  - Vectors to this S/W interrupt handler:
 */

_m16c_contextrestore:

/* Set the USP to the beginning of the context save area */

 	ldc	r1, sp				/* USP = address of base of new context info */
 
 /* Restore the user stack pointer */
 
 	fset	b				/* Switch to bank 1 */
 	pop.w	r0				/* R0 = saved user stack pointer */
	fclr	b				/* Back to bank 0 */
 	
/* Recover all registers */

	popm	fb,sb,a1,a0,r3,r2,r1,r0		/* Restore from registers from user stack */

/* Set the USP and return */

	fset	b				/* Switch to bank 1 */
	popm	r1,r0				/* Recover the PC and flags */
	stc	sp, r0				/* Set the correct USP */
	pushm	r1,r0				/* Put the PC and flags */
	fclr	b				/* Back to bank 0 */

/* The return from interrupt */

	reit					/* Return from interrupt */
	.size	_up_fullcontextrestore, .-_up_fullcontextrestore
	.end
