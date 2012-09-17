/****************************************************************************
 * arch/lpc2378/irq.h
 *
 *   Copyright (C) 2010 Rommel Marcelo. All rights reserved.
 *   Author: Rommel Marcelo
 *
 * This file is part of the NuttX RTOS and based on the lpc2148 port:
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

/* This file should never be included directed but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_LPC2378_IRQ_H
#define __ARCH_LPC2378_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Definitions
 ****************************************************************************/
 
/* LPC2378 Interrupts */

#define WDT_IRQ        0 /* Watchdog */
#define RESERVED_IRQ   1 /* SWI only */
#define DBGCOMMRX_IRQ  2 /* Embedded debug */
#define DBGCOMMTX_IRQ  3 /* Embedded debug */
#define TIMER0_IRQ     4 /* Timer 0 */
#define TIMER1_IRQ     5 /* Timer 1 */
#define UART0_IRQ      6 /* UART 0 */
#define UART1_IRQ      7 /* UART 1 */
#define PWM0_IRQ       8 /* PWM 0 */
#define I2C0_IRQ       9 /* I2C 0 */
#define SPI0_IRQ      10 /* SPI 0 */
#define SSP0_IRQ      10 /* SSP 0 */
#define SSP1_IRQ      11 /* SSP 1 */
#define PLL_IRQ       12 /* PLL Lock IRQ */
#define RTC_IRQ       13 /* Real Time Clock */
#define EINT0_IRQ     14 /* External interrupt 0 */
#define EINT1_IRQ     15 /* External interrupt 1 */
#define EINT2_IRQ     16 /* External interrupt 2 */
#define EINT3_IRQ     17 /* External interrupt 3 */
#define ADC0_IRQ      18 /* ADC 0 */
#define I2C1_IRQ      19 /* I2C 1 */
#define BOD_IRQ       20 /* Brown Out Detect */
#define EMAC_IRQ      21 /* Ethernet */
#define USB_IRQ       22 /* USB */
#define CAN_IRQ       23 /* CAN */
#define MCI_IRQ       24 /* SD/MMC Interface */
#define GPDMA_IRQ     25 /* General Purpose DMA */
#define TIMER2_IRQ    26 /* Timer 2 */
#define TIMER3_IRQ    27 /* Timer 3 */
#define UART2_IRQ     28 /* Uart 2 */
#define UART3_IRQ     29 /* Uart 3 */
#define I2C2_IRQ      30 /* I2C 2 */
#define I2S_IRQ       31 /* I2S */


#define IRQ_SYSTIMER  TIMER0_IRQ

#define NR_IRQS             32

/* There are 32 vectored interrupts.  If vectored interrupts are enabled, the
 * following will be used by the system.
 */
#define SYSTIMER_VEC  0 /* System timer */

#define CLASS_IRQ			0
#define CLASS_FIQ			1
#define PRIORITY_LOWEST		15
#define PRIORITY_HIGHEST	0	/* System timer */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__
typedef void (*vic_vector_t)(uint32_t *regs);
#endif

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#ifndef CONFIG_VECTORED_INTERRUPTS
EXTERN void up_attach_vector(int irq, int priority, vic_vector_t handler);
EXTERN void up_detach_vector(int vector);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_LPC2378_IRQ_H */

