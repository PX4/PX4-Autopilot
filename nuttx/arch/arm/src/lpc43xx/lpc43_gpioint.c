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

#include "chip.h"
#include "chip/lpc43_scu.h"
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

int lpc43_gpioint_grpinitialize(int group, bool anded, bool level)
{
  irqstate_t flags;
  uintptr_t grpbase;
  uint32_t regval;
  int i;

  DEBUGASSERT(group >= 0 && group < NUM_GPIO_NGROUPS);

  /* Select the group register base address and disable the group interrupt */

  flags = irqsave();
  if (group == 0)
    {
      grpbase = LPC43_GRP0INT_BASE;
      up_disable_irq(LPC43M4_IRQ_GINT0);
    }
  else
    {
      grpbase = LPC43_GRP1INT_BASE;
      up_disable_irq(LPC43M4_IRQ_GINT1);
    }

  /* Clear all group polarity and membership settings */

  for (i = 0; i < NUM_GPIO_PORTS; i++)
    {
      putreg32(0, grpbase + LPC43_GRPINT_POL_OFFSET(i));
      putreg32(0, grpbase + LPC43_GRPINT_ENA_OFFSET(i));
    }

  /* Configure the group.  Note that writing "1" to the status bit will also
   * clear any pending group interrupts.
   */

  regval = GRPINT_CTRL_INT;
  if (anded)
    {
      regval |= GRPINT_CTRL_COMB;
    }

  if (level)
    {
      regval |= GRPINT_CTRL_TRIG;
    }
  putreg32(regbal, grpbase + LPC43_GRP1INT_CTRL_OFFSET);

  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Name: lpc43_gpioint_pinconfig
 *
 * Description:
 *   Configure a GPIO pin as an GPIO pin interrupt source (after it has been
 *   configured as an input).
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 * Assumptions:
 *   Interrupts are disabled so that read-modify-write operations are safe.
 *
 ****************************************************************************/

int lpc43_gpioint_pinconfig(uint16_t gpiocfg)
{
  unsigned int port    = ((gpiocfg & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT);
  unsigned int pin     = ((gpiocfg & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT);
  unsigned int pinint  = ((gpiocfg & GPIO_PININT_MASK) >> GPIO_PININT_SHIFT);
  uint32_t     bitmask = (1 << pinint);
  uint32_t     regval;
  int          ret     = OK;

  DEBUGASSERT(port < NUM_GPIO_PORTS && pin < NUM_GPIO_PINS && GPIO_IS_PININT(gpiocfg));

  /* Make sure that pin interrupts are initially disabled at the NVIC.
   * After the pin is configured, the caller will need to manually enable
   * the pin interrupt.
   */

  up_disable_irq(LPC43M4_IRQ_PININT0 + pinint);

  /* Select the pin as the input in the SCU PINTSELn register (overwriting any
   * previous selection).
   */

  if (pinint < 4)
    {
      regval = getreg32(LPC43_SCU_PINTSEL0);
      regval &= ~SCU_PINTSEL0_MASK(pinint);
      regval |= ((pin  << SCU_PINTSEL0_INTPIN_SHIFT(pinint)) |
                 (port << SCU_PINTSEL0_PORTSEL_SHIFT(pinint)));
      putreg32(regval, LPC43_SCU_PINTSEL0);
    }
  else
    {
      regval = getreg32(LPC43_SCU_PINTSEL1);
      regval &= ~SCU_PINTSEL1_MASK(pinint);
      regval |= ((pin  << SCU_PINTSEL1_INTPIN_SHIFT(pinint)) |
                 (port << SCU_PINTSEL1_PORTSEL_SHIFT(pinint)));
      putreg32(regval, LPC43_SCU_PINTSEL1);
    }

  /* Set level or edge sensitive */

  regval = getreg32(LPC43_GPIOINT_ISEL);
  if (GPIO_IS_LEVEL(gpiocfg))
    {
      regval |= bitmask;
    }
  else
    {
      regval &= ~bitmask;
    }
  putreg32(regval, LPC43_GPIOINT_ISEL);

  /* Configure the active high level or rising edge */

  regval = getreg32(LPC43_GPIOINT_IENR);
  if (GPIO_IS_ACTIVE_HI(gpiocfg))
    {
      regval |= bitmask;
    }
  else
    {
      regval &= ~bitmask;
    }
  putreg32(regval, LPC43_GPIOINT_IENR);

  /* Configure the active high low or falling edge */

  regval = getreg32(LPC43_GPIOINT_IENF);
  if (GPIO_IS_ACTIVE_LOW(gpiocfg))
    {
      regval |= bitmask;
    }
  else
    {
      regval &= ~bitmask;
    }
  putreg32(regval, LPC43_GPIOINT_IENF);

  return OK;
}

/****************************************************************************
 * Name: lpc43_gpioint_grpconfig
 *
 * Description:
 *   Configure a GPIO pin as an GPIO group interrupt member (after it has been
 *   configured as an input).
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 * Assumptions:
 *   Interrupts are disabled so that read-modify-write operations are safe.
 *
 ****************************************************************************/

int lpc43_gpioint_grpconfig(uint16_t gpiocfg)
{
  unsigned int port    = ((gpiocfg & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT);
  unsigned int pin     = ((gpiocfg & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT);
  irqstate_t   flags;
  uintptr_t    grpbase;
  uintptr_t    regaddr;
  uint32_t     regval;
  uint32_t     bitmask = (1 << pin);
  int          ret     = OK;

  /* Select the group register base address */

  flags = irqsave();
  if (GPIO_IS_GROUP0(gpiocfg))
    {
      grpbase = LPC43_GRP0INT_BASE;
    }
  else
    {
      grpbase = LPC43_GRP1INT_BASE;
    }

  /* Set/clear the polarity for this pin */

  regaddr = grpbase + LPC43_GRPINT_POL_OFFSET(port);
  regval  = getreg32(regaddr);

  if (GPIO_IS_POLARITY_HI(gpiocfg))
    {
      regval |= bitmask;
    }
  else
    {
      regval &= ~bitmask;
    }

  putreg32(regval, regaddr);

  /* Set the corresponding bit in the port enable register so that this pin
   * will contribute to the group interrupt.
   */

  regaddr = grpbase + LPC43_GRPINT_ENA_OFFSET(port);
  regval  = getreg32(regaddr);
  regval |= bitmask;
  putreg32(regval, regaddr);

  irqrestore(flags);
  return OK;
}

#endif /* CONFIG_GPIO_IRQ */

