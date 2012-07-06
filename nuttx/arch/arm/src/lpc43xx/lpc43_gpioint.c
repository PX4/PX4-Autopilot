/****************************************************************************
 *  arch/arm/src/lpc43/lpc43_gpioint.c
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
 ****************************************************************************/
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/board/board.h>
#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <errno.h>

#include "lpc43_gpioint.h"

#ifdef CONFIG_GPIO_IRQ

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_gpioint_initialize
 *
 * Description:
 *   Initialize logic to interrupting GPIO pins GPIO pins
 *
 ****************************************************************************/

void lpc43_gpioint_initialize(void)
{
#warning "Missing logic"
}

/****************************************************************************
 * Name: lpc43_gpioint_config
 *
 * Description:
 *   Configure a GPIO pin as an interrupt source (after it has been
 *   configured as an input).
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lpc43_gpioint_config(uint16_t gpiocfg)
{
#warning "Missing logic"
  return -ENOSYS;
}

/****************************************************************************
 * Name: lpc43_gpiointconfig
 *
 * Description:
 *   Un-configure a GPIO pin as an interrupt source.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lpc43_gpioint_unconfig(uint16_t gpiocfg)
{
#warning "Missing logic"
  return -ENOSYS;
}

/****************************************************************************
 * Name: lpc43_gpioint_enable
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

void lpc43_gpioint_enable(int irq)
{
#warning "Missing logic"
}

/****************************************************************************
 * Name: lpc43_gpioint_disable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

void lpc43_gpioint_disable(int irq)
{
#warning "Missing logic"
}

#endif /* CONFIG_GPIO_IRQ */

