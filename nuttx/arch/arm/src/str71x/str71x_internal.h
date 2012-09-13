/************************************************************************************
 * arch/arm/src/str71x/str71x_internal.h
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_STR71X_STR71X_INTERNAL_H
#define __ARCH_ARM_SRC_STR71X_STR71X_INTERNAL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include <arch/board/board.h>

/************************************************************************************
 * Pre-procesor Definitions
 ************************************************************************************/

/* Calculate the values of PCLK1 and PCLK2 from settings in board.h.
 *
 * Example:
 *  STR71X_RCCU_MAIN_OSC = 4MHz (not divided by 2)
 *  STR71X_CLK2 = 4MHz
 *  STR71X_PLL1OUT = 16 * STR71X_CLK2 / 2 = 32MHz
 *  CLK3 = 32MHz
 *  RCLK = 32MHz
 *  PCLK1 = 32MHz / 1 = 32MHz
 */

/* PLL1OUT derives from Main OSC->CLK2 */

#ifdef STR71X_PLL1IN_DIV2                              /* CLK2 is input to PLL1 */
#  define STR71X_CLK2  (STR71X_RCCU_MAIN_OSC/2)        /* CLK2 is OSC/2 */
#else
#  define STR71X_CLK2  STR71X_RCCU_MAIN_OSC            /* CLK2 is OSC */
#endif

#define STR71X_PLL1OUT ((STR71X_PLL1OUT_MUL * STR71X_CLK2) / STR71X_PLL1OUT_DIV)

/* PLL2 OUT derives from HCLK */

#define STR71X_PLL2OUT ((STR71X_PLL2OUT_MUL * STR71X_HCLK) / STR71X_PLL2OUT_DIV)

/* Peripheral clocks derive from PLL1OUT->CLK3->RCLK->PCLK1/2 */

#define STR71X_CLK3    STR71X_PLL1OUT                  /* CLK3 hard coded to be PLL1OUT */
#define STR71X_RCLK    STR71X_CLK3                     /* RCLK hard coded to be CLK3 */
#define STR71X_PCLK1   (STR71X_RCLK / STR71X_APB1_DIV) /* PCLK1 derives from RCLK */
#define STR71X_PCLK2   (STR71X_RCLK / STR71X_APB2_DIV) /* PCLK2 derives from RCLK */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/********************************************************************************
 * Name: str7x_xtiinitialize
 *
 * Description:
 *   Configure XTI for operation.  Note that the lines are not used as wake-up
 *   sources in this implementation.  Some extensions would be required for that
 *   capability.
 *
 ********************************************************************************/

#ifdef CONFIG_STR71X_XTI
extern int str71x_xtiinitialize(void);
#else
#  define str71x_xtiinitialize()
#endif /* CONFIG_STR71X_XTI */

/********************************************************************************
 * Name: str7x_xticonfig
 *
 * Description:
 *   Configure an external line to provide interrupts.  Interrupt is configured,
 *   but disabled on return.
 *
 ********************************************************************************/

#ifdef CONFIG_STR71X_XTI
extern int str71x_xticonfig(int irq, bool rising);
#else
#  define str71x_xticonfig(irq,rising)
#endif /* CONFIG_STR71X_XTI */

/****************************************************************************
 * Name: str71x_enable_xtiirq
 *
 * Description:
 *   Enable an external interrupt.
 *
 ****************************************************************************/

#ifdef CONFIG_STR71X_XTI
extern void str71x_enable_xtiirq(int irq);
#else
#  define str71x_enable_xtiirq(irq)
#endif /* CONFIG_STR71X_XTI */

/****************************************************************************
 * Name: str71x_disable_xtiirq
 *
 * Description:
 *   Disable an external interrupt.
 *
 ****************************************************************************/

#ifdef CONFIG_STR71X_XTI
extern void str71x_disable_xtiirq(int irq);
#else
#  define str71x_disable_xtiirq(irq)
#endif /* CONFIG_STR71X_XTI */

#endif /* __ARCH_ARM_SRC_STR71X_STR71X_INTERNAL_H */
