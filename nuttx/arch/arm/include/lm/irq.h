/************************************************************************************
 * arch/arm/include/lm/irq.h
 *
 *   Copyright (C) 2009-2011 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_INCLUDE_LM_IRQ_H
#define __ARCH_ARM_INCLUDE_LM_IRQ_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

#if defined(CONFIG_ARCH_CHIP_LM3S)
#  include <arch/lm/lm3s_irq.h>
#elif defined(CONFIG_ARCH_CHIP_LM4F)
#  include <arch/lm/lm4f_irq.h>
#else
#  error "Unsupported Stellaris IRQ file"
#endif

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* GPIO IRQs -- Note that support for individual GPIO ports can
 * be disabled in order to reduce the size of the implemenation.
 */

#ifndef CONFIG_LM_DISABLE_GPIOA_IRQS
#  define LM_IRQ_GPIOA_0 (NR_IRQS + 0)
#  define LM_IRQ_GPIOA_1 (NR_IRQS + 1)
#  define LM_IRQ_GPIOA_2 (NR_IRQS + 2)
#  define LM_IRQ_GPIOA_3 (NR_IRQS + 3)
#  define LM_IRQ_GPIOA_4 (NR_IRQS + 4)
#  define LM_IRQ_GPIOA_5 (NR_IRQS + 5)
#  define LM_IRQ_GPIOA_6 (NR_IRQS + 6)
#  define LM_IRQ_GPIOA_7 (NR_IRQS + 7)
#  define _NGPIOAIRQS    (NR_IRQS + 8)
#else
#  define _NGPIOAIRQS    NR_IRQS
#endif

#ifndef CONFIG_LM_DISABLE_GPIOB_IRQS
#  define LM_IRQ_GPIOB_0 (_NGPIOAIRQS + 0)
#  define LM_IRQ_GPIOB_1 (_NGPIOAIRQS + 1)
#  define LM_IRQ_GPIOB_2 (_NGPIOAIRQS + 2)
#  define LM_IRQ_GPIOB_3 (_NGPIOAIRQS + 3)
#  define LM_IRQ_GPIOB_4 (_NGPIOAIRQS + 4)
#  define LM_IRQ_GPIOB_5 (_NGPIOAIRQS + 5)
#  define LM_IRQ_GPIOB_6 (_NGPIOAIRQS + 6)
#  define LM_IRQ_GPIOB_7 (_NGPIOAIRQS + 7)
#  define _NGPIOBIRQS    (_NGPIOAIRQS + 8)
#else
#  define _NGPIOBIRQS    _NGPIOAIRQS
#endif

#ifndef CONFIG_LM_DISABLE_GPIOC_IRQS
#  define LM_IRQ_GPIOC_0 (_NGPIOBIRQS + 0)
#  define LM_IRQ_GPIOC_1 (_NGPIOBIRQS + 1)
#  define LM_IRQ_GPIOC_2 (_NGPIOBIRQS + 2)
#  define LM_IRQ_GPIOC_3 (_NGPIOBIRQS + 3)
#  define LM_IRQ_GPIOC_4 (_NGPIOBIRQS + 4)
#  define LM_IRQ_GPIOC_5 (_NGPIOBIRQS + 5)
#  define LM_IRQ_GPIOC_6 (_NGPIOBIRQS + 6)
#  define LM_IRQ_GPIOC_7 (_NGPIOBIRQS + 7)
#  define _NGPIOCIRQS    (_NGPIOBIRQS + 8)
#else
#  define _NGPIOCIRQS    _NGPIOBIRQS
#endif

#ifndef CONFIG_LM_DISABLE_GPIOD_IRQS
#  define LM_IRQ_GPIOD_0 (_NGPIOCIRQS + 0)
#  define LM_IRQ_GPIOD_1 (_NGPIOCIRQS + 1)
#  define LM_IRQ_GPIOD_2 (_NGPIOCIRQS + 2)
#  define LM_IRQ_GPIOD_3 (_NGPIOCIRQS + 3)
#  define LM_IRQ_GPIOD_4 (_NGPIOCIRQS + 4)
#  define LM_IRQ_GPIOD_5 (_NGPIOCIRQS + 5)
#  define LM_IRQ_GPIOD_6 (_NGPIOCIRQS + 6)
#  define LM_IRQ_GPIOD_7 (_NGPIOCIRQS + 7)
#  define _NGPIODIRQS    (_NGPIOCIRQS + 8)
#else
#  define _NGPIODIRQS    _NGPIOCIRQS
#endif

#ifndef CONFIG_LM_DISABLE_GPIOE_IRQS
#  define LM_IRQ_GPIOE_0 (_NGPIODIRQS + 0)
#  define LM_IRQ_GPIOE_1 (_NGPIODIRQS + 1)
#  define LM_IRQ_GPIOE_2 (_NGPIODIRQS + 2)
#  define LM_IRQ_GPIOE_3 (_NGPIODIRQS + 3)
#  define LM_IRQ_GPIOE_4 (_NGPIODIRQS + 4)
#  define LM_IRQ_GPIOE_5 (_NGPIODIRQS + 5)
#  define LM_IRQ_GPIOE_6 (_NGPIODIRQS + 6)
#  define LM_IRQ_GPIOE_7 (_NGPIODIRQS + 7)
#  define _NGPIOEIRQS    (_NGPIODIRQS + 8)
#else
#  define _NGPIOEIRQS    _NGPIODIRQS
#endif

#ifndef CONFIG_LM_DISABLE_GPIOF_IRQS
#  define LM_IRQ_GPIOF_0 (_NGPIOEIRQS + 0)
#  define LM_IRQ_GPIOF_1 (_NGPIOEIRQS + 1)
#  define LM_IRQ_GPIOF_2 (_NGPIOEIRQS + 2)
#  define LM_IRQ_GPIOF_3 (_NGPIOEIRQS + 3)
#  define LM_IRQ_GPIOF_4 (_NGPIOEIRQS + 4)
#  define LM_IRQ_GPIOF_5 (_NGPIOEIRQS + 5)
#  define LM_IRQ_GPIOF_6 (_NGPIOEIRQS + 6)
#  define LM_IRQ_GPIOF_7 (_NGPIOEIRQS + 7)
#  define _NGPIOFIRQS    (_NGPIOEIRQS + 8)
#else
#  define _NGPIOFIRQS    _NGPIOEIRQS
#endif

#ifndef CONFIG_LM_DISABLE_GPIOG_IRQS
#  define LM_IRQ_GPIOG_0 (_NGPIOFIRQS + 0)
#  define LM_IRQ_GPIOG_1 (_NGPIOFIRQS + 1)
#  define LM_IRQ_GPIOG_2 (_NGPIOFIRQS + 2)
#  define LM_IRQ_GPIOG_3 (_NGPIOFIRQS + 3)
#  define LM_IRQ_GPIOG_4 (_NGPIOFIRQS + 4)
#  define LM_IRQ_GPIOG_5 (_NGPIOFIRQS + 5)
#  define LM_IRQ_GPIOG_6 (_NGPIOFIRQS + 6)
#  define LM_IRQ_GPIOG_7 (_NGPIOFIRQS + 7)
#  define _NGPIOGIRQS    (_NGPIOFIRQS + 8)
#else
#  define _NGPIOGIRQS    _NGPIOFIRQS
#endif

#ifndef CONFIG_LM_DISABLE_GPIOH_IRQS
#  define LM_IRQ_GPIOH_0 (_NGPIOGIRQS + 0)
#  define LM_IRQ_GPIOH_1 (_NGPIOGIRQS + 1)
#  define LM_IRQ_GPIOH_2 (_NGPIOGIRQS + 2)
#  define LM_IRQ_GPIOH_3 (_NGPIOGIRQS + 3)
#  define LM_IRQ_GPIOH_4 (_NGPIOGIRQS + 4)
#  define LM_IRQ_GPIOH_5 (_NGPIOGIRQS + 5)
#  define LM_IRQ_GPIOH_6 (_NGPIOGIRQS + 6)
#  define LM_IRQ_GPIOH_7 (_NGPIOGIRQS + 7)
#  define _NGPIOHIRQS    (_NGPIOGIRQS + 8)
#else
#  define _NGPIOHIRQS    _NGPIOGIRQS
#endif

#ifndef CONFIG_LM_DISABLE_GPIOJ_IRQS
#  define LM_IRQ_GPIOJ_0 (_NGPIOHIRQS + 0)
#  define LM_IRQ_GPIOJ_1 (_NGPIOHIRQS + 1)
#  define LM_IRQ_GPIOJ_2 (_NGPIOHIRQS + 2)
#  define LM_IRQ_GPIOJ_3 (_NGPIOHIRQS + 3)
#  define LM_IRQ_GPIOJ_4 (_NGPIOHIRQS + 4)
#  define LM_IRQ_GPIOJ_5 (_NGPIOHIRQS + 5)
#  define LM_IRQ_GPIOJ_6 (_NGPIOHIRQS + 6)
#  define LM_IRQ_GPIOJ_7 (_NGPIOHIRQS + 7)
#  define _NGPIOJIRQS    (_NGPIOHIRQS + 8)
#else
#  define _NGPIOJIRQS    _NGPIOHIRQS
#endif

#define NR_GPIO_IRQS     (_NGPIOJIRQS - NR_IRQS)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
extern "C"
{
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/****************************************************************************
 * Name: gpio_irqattach
 *
 * Description:
 *   Attach the interrupt handler 'isr' to the GPIO IRQ 'irq'
 *
 ****************************************************************************/

int gpio_irqattach(int irq, xcpt_t isr);
#define gpio_irqdetach(isr) gpio_irqattach(isr, NULL)

/****************************************************************************
 * Name: gpio_irqenable
 *
 * Description:
 *   Enable the GPIO IRQ specified by 'irq'
 *
 ****************************************************************************/

void gpio_irqenable(int irq);

/****************************************************************************
 * Name: gpio_irqdisable
 *
 * Description:
 *   Disable the GPIO IRQ specified by 'irq'
 *
 ****************************************************************************/

void gpio_irqdisable(int irq);

#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_LM_IRQ_H */
