/****************************************************************************
 * arch/lpc214x/irq.h
 *
 *   Copyright (C) 2007, 2008 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_LPC214X_IRQ_H
#define __ARCH_LPC214X_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* LPC214X Interrupts */

#define LPC214X_WDT_IRQ        0 /* Watchdog */
#define LPC214X_RESERVED_IRQ   1 /* SWI only */
#define LPC214X_DBGCOMMRX_IRQ  2 /* Embedded debug */
#define LPC214X_DBGCOMMTX_IRQ  3 /* Embedded debug */
#define LPC214X_TIMER0_IRQ     4 /* Timer 0 */
#define LPC214X_TIMER1_IRQ     5 /* Timer 1 */
#define LPC214X_UART0_IRQ      6 /* UART 0 */
#define LPC214X_UART1_IRQ      7 /* UART 1 */
#define LPC214X_PWM0_IRQ       8 /* PWM 0 */
#define LPC214X_I2C0_IRQ       9 /* I2C 0 */
#define LPC214X_SPI0_IRQ      10 /* SPI 0 */
#define LPC214X_SPI1_IRQ      11 /* SPI 1 */
#define LPC214X_PLL_IRQ       12 /* PLL Lock IRQ */
#define LPC214X_RTC_IRQ       13 /* Real Time Clock */
#define LPC214X_EINT0_IRQ     14 /* External interrupt 0 */
#define LPC214X_EINT1_IRQ     15 /* External interrupt 1 */
#define LPC214X_EINT2_IRQ     16 /* External interrupt 2 */
#define LPC214X_EINT3_IRQ     17 /* External interrupt 3 */
#define LPC214X_ADC0_IRQ      18 /* ADC 0 */
#define LPC214X_I2C1_IRQ      19 /* I2C 1 */
#define LPC214X_BOD_IRQ       20 /* Brown Out Detect */
#define LPC214X_ADC1_IRQ      21 /* ADC 1 */
#define LPC214X_USB_IRQ       22 /* USB */

#define LPC214X_IRQ_SYSTIMER  LPC214X_TIMER0_IRQ
#define NR_IRQS               23

/* There are 16 vectored interrupts.  If vectored interrupts are enabled, the
 * following will be used by the system.
 */

#define LPC214X_SYSTIMER_VEC  0 /* System timer */

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
EXTERN void up_attach_vector(int irq, int vector, vic_vector_t handler);
EXTERN void up_detach_vector(int vector);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_LPC214X_IRQ_H */

