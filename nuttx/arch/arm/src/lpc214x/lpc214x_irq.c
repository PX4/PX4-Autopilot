/****************************************************************************
 * arch/arm/src/lpc214x/lpc214x_irq.c
 *
 *   Copyright (C) 2007-2009, 2011 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <debug.h>
#include <nuttx/irq.h>

#include "arm.h"
#include "chip.h"
#include "up_arch.h"
#include "os_internal.h"
#include "up_internal.h"

#include "lpc214x_vic.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile uint32_t *current_regs;

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
  int reg;

  /* Disable all interrupts.  We do this by writing zero to the IntEnable
   * register.  This is equivalent to writing all ones to the IntClearEnable
   * register.
   */

  vic_putreg(0, LPC214X_VIC_INTENABLE_OFFSET);

  /* Select all IRQs, no FIQs */

  vic_putreg(0, LPC214X_VIC_INTSELECT_OFFSET);

  /* Set the default vector */

  vic_putreg((uint32_t)up_decodeirq, LPC214X_VIC_DEFVECTADDR_OFFSET);

  /* Disable all vectored interrupts */

  for (reg = LPC214X_VIC_VECTCNTL0_OFFSET;
       reg <= LPC214X_VIC_VECTCNTL15_OFFSET;
       reg += 4)
    {
      vic_putreg(0, reg);
    }

  /* currents_regs is non-NULL only while processing an interrupt */

  current_regs = NULL;

  /* And finally, enable interrupts */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  irqrestore(SVC_MODE | PSR_F_BIT);
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
  /* Verify that the IRQ number is within range */

  if (irq < NR_IRQS)
    {
      /* Disable the irq by setting the corresponding bit in the VIC
       * Interrupt Enable Clear register.
       */

      vic_putreg((1 << irq), LPC214X_VIC_INTENCLEAR_OFFSET);
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
  /* Verify that the IRQ number is within range */

  if (irq < NR_IRQS)
    {
      /* Disable all interrupts */

      irqstate_t flags = irqsave();

      /* Enable the irq by setting the corresponding bit in the VIC
       * Interrupt Enable register.
       */

      uint32_t val = vic_getreg(LPC214X_VIC_INTENABLE_OFFSET);
      vic_putreg(val | (1 << irq), LPC214X_VIC_INTENABLE_OFFSET);
      irqrestore(flags);
    }
}

/****************************************************************************
 * Name: up_attach_vector
 *
 * Description:
 *   Attach a user-supplied handler to a vectored interrupt
 *
 ****************************************************************************/

#ifndef CONFIG_VECTORED_INTERRUPTS
void up_attach_vector(int irq, int vector, vic_vector_t handler)
{
  /* Verify that the IRQ number and vector number are within range */

  if (irq < NR_IRQS && vector < 16 && handler)
    {
      int offset = vector << 2;

      /* Disable all interrupts */

      irqstate_t flags = irqsave();

      /* Save the vector address */

      vic_putreg((uint32_t)handler, LPC214X_VIC_VECTADDR0_OFFSET + offset);

      /* Enable the vectored interrupt */

      vic_putreg(((irq << LPC214X_VECTCNTL_IRQSHIFT) | LPC214X_VECTCNTL_ENABLE),
		 LPC214X_VIC_VECTCNTL0_OFFSET + offset);
      irqrestore(flags);
    }
}
#endif

/****************************************************************************
 * Name: up_detach_vector
 *
 * Description:
 *   Detach a user-supplied handler from a vectored interrupt
 *
 ****************************************************************************/

#ifndef CONFIG_VECTORED_INTERRUPTS
void up_detach_vector(int vector)
{
  /* Verify that the vector number is within range */

  if (vector < 16)
    {
      /* Disable the vectored interrupt */

      int offset = vector << 2;
      vic_putreg(0, LPC214X_VIC_VECTCNTL0_OFFSET + offset);
    }
}
#endif
