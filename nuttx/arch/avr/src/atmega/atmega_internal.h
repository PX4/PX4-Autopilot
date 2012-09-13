/****************************************************************************
 * arch/avr/src/atmega/atmega_internal.h
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
 ****************************************************************************/

#ifndef __ARCH_AVR_SRC_ATMEGA_ATMEGA_INTERNAL_H
#define __ARCH_AVR_SRC_ATMEGA_ATMEGA_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "atmega_config.h"

#ifdef CONFIG_AVR_GPIOIRQ
#  include <nuttx/irq.h>
#endif

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: up_clkinit
 *
 * Description:
 *   Initialiaze clock/PLL settings per the definitions in the board.h file.
 *
 ****************************************************************************/

EXTERN void up_clkinitialize(void);

/****************************************************************************
 * Name: usart0_reset and usart1_reset
 *
 * Description:
 *   Reset USART0 or USART1.
 *
 ****************************************************************************/

EXTERN void usart0_reset(void);
EXTERN void usart1_reset(void);

/****************************************************************************
 * Name: usart0_configure and usart1_configure
 *
 * Description:
 *   Configure USART0 or USART1.
 *
 ****************************************************************************/

EXTERN void usart0_configure(void);
EXTERN void usart1_configure(void);

/****************************************************************************
 * Name: up_consoleinit
 *
 * Description:
 *   Initialize a console for debug output.  This function is called very
 *   early in the intialization sequence to configure the serial console
 *   uart (only).
 *
 ****************************************************************************/

EXTERN void up_consoleinit(void);

/****************************************************************************
 * Name: up_boardinit
 *
 * Description:
 *   This function must be provided by the board-specific logic in the
 *   directory configs/<board-name>/up_boot.c.
 *
 ****************************************************************************/

EXTERN void up_boardinitialize(void);

/****************************************************************************
 * Name: gpio_irqinitialize
 *
 * Description:
 *   Initialize all vectors to the unexpected interrupt handler
 *
 * Configuration Notes:
 *   Configuration CONFIG_AVR_GPIOIRQ must be selected to enable the
 *   overall GPIO IRQ feature.
 *
 * Assumptions:
 *   Called during the early boot sequence before global interrupts have
 *   been enabled.
 *
 ****************************************************************************/

#ifdef CONFIG_AVR_GPIOIRQ
EXTERN void weak_function gpio_irqinitialize(void);
#endif

/****************************************************************************
 * Name: gpio_irqattach
 *
 * Description:
 *   Attach in GPIO interrupt to the provide 'isr'
 *
 * Configuration Notes:
 *   Configuration CONFIG_AVR_GPIOIRQ must be selected to enable the
 *   overall GPIO IRQ feature.
 *
 ****************************************************************************/

#ifdef CONFIG_AVR_GPIOIRQ
EXTERN int gpio_irqattach(int irq, xcpt_t newisr, xcpt_t *oldisr);
#endif

/****************************************************************************
 * Name: gpio_irqenable
 *
 * Description:
 *   Enable the GPIO IRQ specified by 'irq'
 *
 * Configuration Notes:
 *   Configuration CONFIG_AVR_GPIOIRQ must be selected to enable the
 *   overall GPIO IRQ feature.
 *
 ****************************************************************************/

#ifdef CONFIG_AVR_GPIOIRQ
EXTERN void gpio_irqenable(int irq);
#endif

/*****************************************************************************
 * Name: gpio_irqdisable
 *
 * Description:
 *   Disable the GPIO IRQ specified by 'irq'
 *
 * Configuration Notes:
 *   Configuration CONFIG_AVR_GPIOIRQ must be selected to enable the
 *   overall GPIO IRQ feature.
 *
 ****************************************************************************/

#ifdef CONFIG_AVR_GPIOIRQ
EXTERN void gpio_irqdisable(int irq);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_AVR_SRC_ATMEGA_ATMEGA_INTERNAL_H */

