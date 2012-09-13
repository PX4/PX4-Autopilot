/****************************************************************************
 * z16f/z16f_irq.c
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

#include <nuttx/irq.h>
#include <arch/irq.h>

#include "chip/chip.h"
#include "os_internal.h"
#include "up_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
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
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  /* Clear and disable all interrupts.  Set all to priority 0. */

  putreg8(0xff, Z16F_IRQ0);
  putreg8(0xff, Z16F_IRQ1);
  putreg8(0xff, Z16F_IRQ2);

  putreg16(0x0000, Z16F_IRQ0_EN);
  putreg16(0x0000, Z16F_IRQ1_EN);
  putreg16(0x0000, Z16F_IRQ2_EN);

  /* currents_regs is non-NULL only while processing an interrupt */

  current_regs = NULL;

  /* And finally, enable interrupts */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  EI();
#endif
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

  if (irq >= Z16F_IRQ_IRQ0)
    {
      /* Disable the interrupt by clearing the corresponding bit in the
       * appropriate IRQ enable high register.  The enable low
       * register is assumed to be zero, resulting interrupt disabled.
       */

      if (irq < Z16F_IRQ_IRQ1)
        {
           putreg8((getreg8(Z16F_IRQ0_ENH) & ~Z16F_IRQ0_BIT(irq)), Z16F_IRQ0_ENH);
        }
      else if (irq < Z16F_IRQ_IRQ2)
        {
           putreg8((getreg8(Z16F_IRQ1_ENH) & ~Z16F_IRQ1_BIT(irq)), Z16F_IRQ1_ENH);
        }
      else if (irq < NR_IRQS)
        {
           putreg8((getreg8(Z16F_IRQ2_ENH) & ~Z16F_IRQ2_BIT(irq)), Z16F_IRQ2_ENH);
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

  if (irq >= Z16F_IRQ_IRQ0)
    {
      /* Enable the interrupt by setting the corresponding bit in the
       * appropriate IRQ enable register.  The enable low
       * register is assumed to be zero, resulting in "nomimal" interrupt
       * priority.
       */

      if (irq < Z16F_IRQ_IRQ1)
        {
           putreg8((getreg8(Z16F_IRQ0_ENH) | Z16F_IRQ0_BIT(irq)), Z16F_IRQ0_ENH);
        }
      else if (irq < Z16F_IRQ_IRQ2)
        {
           putreg8((getreg8(Z16F_IRQ1_ENH) | Z16F_IRQ1_BIT(irq)), Z16F_IRQ1_ENH);
        }
      else if (irq < NR_IRQS)
        {
           putreg8((getreg8(Z16F_IRQ2_ENH) | Z16F_IRQ2_BIT(irq)), Z16F_IRQ2_ENH);
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

  if (irq >= Z16F_IRQ_IRQ0)
    {
      /* Disable the interrupt by clearing the corresponding bit in the
       * appropriate IRQ enable register and acknowledge it by setting the
       * corresponding bit in the IRQ status register.
       */

      if (irq < Z16F_IRQ_IRQ1)
        {
           putreg8((getreg8(Z16F_IRQ0_ENH) & ~Z16F_IRQ0_BIT(irq)), Z16F_IRQ0_ENH);
           putreg8(Z16F_IRQ0_BIT(irq), Z16F_IRQ0);
        }
      else if (irq < Z16F_IRQ_IRQ2)
        {
           putreg8((getreg8(Z16F_IRQ1_ENH) & ~Z16F_IRQ1_BIT(irq)), Z16F_IRQ1_ENH);
           putreg8(Z16F_IRQ1_BIT(irq), Z16F_IRQ2);
        }
      else if (irq < NR_IRQS)
        {
           putreg8((getreg8(Z16F_IRQ2_ENH) & ~Z16F_IRQ2_BIT(irq)), Z16F_IRQ2_ENH);
           putreg8(Z16F_IRQ2_BIT(irq), Z16F_IRQ2);
        }
    }
}

/****************************************************************************
 * Name: up_prioritize_irq
 *
 * Description:
 *   Set interrupt priority
 *
 ****************************************************************************/

void up_prioritize_irq(int irq, int priority)
{
  /* To be provided */
}

