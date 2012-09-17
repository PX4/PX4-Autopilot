/****************************************************************************
 * arch/z80/src/z8/z8_irq.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <ez8.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "chip/switch.h"
#include "up_internal.h"

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* This structure holds information about the current interrupt processing state */

struct z8_irqstate_s g_z8irqstate;

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
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  /* Clear and disable all interrupts.  Set all to priority 0. */

  putreg8(0xff, IRQ0);
  putreg8(0xff, IRQ1);
  putreg8(0xff, IRQ2);

  putreg16(0x0000, IRQ0EN);
  putreg16(0x0000, IRQ1EN);
  putreg16(0x0000, IRQ2EN);

  /* And finally, enable interrupts */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  EI();
#endif
}

/****************************************************************************
 * Name: irqsave
 *
 * Description:
 *   Disable all interrupts; return previous interrupt state
 *
 ****************************************************************************/

irqstate_t irqsave(void)
{
  /* Bit 7 (IRQE) of the IRQCTL register determines if interrupts were
   * enabled when this function was called.
   */

  register irqstate_t retval = getreg8(IRQCTL);

  /* Disable interrupts */

  DI();

  /* Return the previous interrupt state */

  return retval;
}

/****************************************************************************
 * Name: irqrestore
 *
 * Description:
 *   Restore previous interrupt state
 *
 ****************************************************************************/

void irqrestore(irqstate_t flags)
{
  /* Bit 7 (IRQE) of the IRQCTL register determines if interrupts were
   * enabled when irqsave() was called.
   */

  if ((flags & 0x80) != 0)
    {
      /* The IRQE bit was set, re-enable interrupts */

      EI();
    }
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
  /* System exceptions cannot be disabled */

  if (irq >= Z8_IRQ0_MIN)
    {
      /* Disable the interrupt by clearing the corresponding bit in the
       * appropriate IRQ enable high register.  The enable low
       * register is assumed to be zero, resulting interrupt disabled.
       */

      if (irq <= Z8_IRQ0_MAX)
        {
           putreg8((getreg8(IRQ0ENH) & ~Z8_IRQ0_BIT(irq)), IRQ0ENH);
        }
      else if (irq <= Z8_IRQ1_MAX)
        {
           putreg8((getreg8(IRQ1ENH) & ~Z8_IRQ1_BIT(irq)), IRQ1ENH);
        }
      else if (irq < NR_IRQS)
        {
           putreg8((getreg8(IRQ2ENH) & ~Z8_IRQ2_BIT(irq)), IRQ2ENH);
        }
    }
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
  /* System exceptions cannot be disabled */

  if (irq >= Z8_IRQ0_MIN)
    {
      /* Enable the interrupt by setting the corresponding bit in the
       * appropriate IRQ enable high register.  The enable low
       * register is assumed to be zero, resulting in "nomimal" interrupt
       * priority.
       */

      if (irq <= Z8_IRQ0_MAX)
        {
           putreg8((getreg8(IRQ0ENH) | Z8_IRQ0_BIT(irq)), IRQ0ENH);
        }
      else if (irq <= Z8_IRQ1_MAX)
        {
           putreg8((getreg8(IRQ1ENH) | Z8_IRQ1_BIT(irq)), IRQ1ENH);
        }
      else if (irq < NR_IRQS)
        {
           putreg8((getreg8(IRQ2ENH) | Z8_IRQ2_BIT(irq)), IRQ2ENH);
        }
    }
}

/****************************************************************************
 * Name: up_maskack_irq
 *
 * Description:
 *   Mask the IRQ and acknowledge it
 *
 ****************************************************************************/

void up_maskack_irq(int irq)
{
  /* System exceptions cannot be disabled or acknowledged */

  if (irq >= Z8_IRQ0_MIN)
    {
      /* Disable the interrupt by clearing the corresponding bit in the
       * appropriate IRQ enable register and acknowledge it by setting the
       * corresponding bit in the IRQ status register.
       */

      if (irq <= Z8_IRQ0_MAX)
        {
           putreg8((getreg8(IRQ0ENH) & ~Z8_IRQ0_BIT(irq)), IRQ0ENH);
           putreg8(Z8_IRQ0_BIT(irq), IRQ0);
        }
      else if (irq <= Z8_IRQ1_MAX)
        {
           putreg8((getreg8(IRQ1ENH) & ~Z8_IRQ1_BIT(irq)), IRQ1ENH);
           putreg8(Z8_IRQ1_BIT(irq), IRQ2);
        }
      else if (irq < NR_IRQS)
        {
           putreg8((getreg8(IRQ2ENH) & ~Z8_IRQ2_BIT(irq)), IRQ2ENH);
           putreg8(Z8_IRQ2_BIT(irq), IRQ2);
        }
    }
}
