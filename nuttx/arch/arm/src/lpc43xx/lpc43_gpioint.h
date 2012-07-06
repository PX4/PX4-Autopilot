/************************************************************************************
 * arch/arm/src/lpc43xx/lpc43_gpioint.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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
/* GPIO pin interrupts
 *
 * From all available GPIO pins, up to eight pins can be selected in the system
 * control block to serve as external interrupt pins. The external interrupt pins
 * are connected to eight individual interrupts in the NVIC and are created based
 * on rising or falling edges or on the input level on the pin.
 *
 * GPIO group interrupt
 *
 * For each port/pin connected to one of the two the GPIO Grouped Interrupt blocks
 * (GROUP0 and GROUP1), the GPIO grouped interrupt registers determine which pins are
 * enabled to generate interrupts and what the active polarities of each of those
 * inputs are. The GPIO grouped interrupt registers also select whether the interrupt
 * output will be level or edge triggered and whether it will be based on the OR or
 * the AND of all of the enabled inputs.
 */

#ifndef __ARCH_ARM_SRC_LPC43XX_LPC43_GPIOINT_H
#define __ARCH_ARM_SRC_LPC43XX_LPC43_GPIOINT_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip.h"
#include "chip/lpc43_gpio.h"

#ifdef CONFIG_GPIO_IRQ

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/****************************************************************************
 * Name: lpc43_gpioint_grpinitialize
 *
 * Description:
 *   Initialize the properties of a GPIO group.  The properties of the group
 *   should be configured before any pins are added to the group by
 *   lpc32_gpioint_grpconfig().  As side effects, this call also removes
 *   all pins from the group and disables the group interrupt.  On return,
 *   this is a properly configured, empty GPIO interrupt group.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 * Assumptions:
 *   Interrupts are disabled so that read-modify-write operations are safe.
 *
 ****************************************************************************/

EXTERN int lpc43_gpioint_grpinitialize(int group, bool anded, bool level);

/****************************************************************************
 * Name: lpc43_gpioint_pinconfig
 *
 * Description:
 *   Configure a GPIO pin as an GPIO pin interrupt source (after it has been
 *   configured as an input).  This function should *not* be called directly
 *   from user application code; user code should call this function only
 *   indirectly through lpc32_gpio_config().
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 * Assumptions:
 *   Interrupts are disabled so that read-modify-write operations are safe.
 *
 ****************************************************************************/

EXTERN int lpc43_gpioint_pinconfig(uint16_t gpiocfg);

/****************************************************************************
 * Name: lpc43_gpioint_grpconfig
 *
 * Description:
 *   Configure a GPIO pin as an GPIO group interrupt member (after it has been
 *   configured as an input).  This function should *not* be called directly
 *   from user application code; user code should call this function only
 *   indirectly through lpc32_gpio_config().
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 * Assumptions:
 *   Interrupts are disabled so that read-modify-write operations are safe.
 *
 ****************************************************************************/

EXTERN int lpc43_gpioint_grpconfig(uint16_t gpiocfg);

#endif /* CONFIG_GPIO_IRQ */
#endif /* __ARCH_ARM_SRC_LPC43XX_LPC43_GPIOINT_H */
