/****************************************************************************
 * arch/arm/src/lpc2378/lpc23xx_irq.c
 *
 *   Copyright (C) 2010 Rommel Marcelo. All rights reserved.
 *   Author: Rommel Marcelo
 *
 * This file is part of the NuttX RTOS and based on the lpc2148 port:
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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
#include <errno.h>
#include <debug.h>
#include <nuttx/irq.h>

#include "arm.h"
#include "chip.h"
#include "up_arch.h"
#include "os_internal.h"

#include "internal.h"
#include "lpc23xx_vic.h"

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

  /* Disable all interrupts.  We do this by writing ones to the IntClearEnable
   * register.
   */

  vic_putreg(0xffffffff, VIC_INTENCLEAR_OFFSET);

  /* Select all IRQs, FIQs are not used */

  vic_putreg(0, VIC_INTSELECT_OFFSET);

  /* Clear priority interrupts */

  for (reg = 0; reg < NR_IRQS; reg++)
    {
      vic_putreg(0, VIC_VECTADDR0_OFFSET + (reg << 2));
      vic_putreg(0x0F, VIC_VECTPRIORITY0_OFFSET + (reg << 2));
    }

  /* currents_regs is non-NULL only while processing an interrupt */

  current_regs = NULL;

  /* Enable global ARM interrupts */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  irqrestore(SVC_MODE | PSR_F_BIT);
#endif
}

/***********************************************************************
 * Name: up_enable_irq_protect
 * VIC registers can be accessed in User or privileged mode
 ***********************************************************************/

static void up_enable_irq_protect(void)
{
  // ~ uint32_t reg32 = vic_getreg(VIC_PROTECTION_OFFSET);
  // ~ reg32 &= ~(0xFFFFFFFF);
  vic_putreg(0x01, VIC_PROTECTION_OFFSET);
}

/***********************************************************************
 * Name: up_disable_irq_protect
 * VIC registers can only be accessed in privileged mode
 ***********************************************************************/

static void up_disable_irq_protect(void)
{
  vic_putreg(0, VIC_PROTECTION_OFFSET);
}

/***********************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the IRQ specified by 'irq'
 *
 ***********************************************************************/

void up_disable_irq(int irq)
{
  /* Verify that the IRQ number is within range */

  if (irq < NR_IRQS)
    {
      /* Disable the irq by setting the corresponding bit in the VIC Interrupt
       * Enable Clear register.
       */

      vic_putreg((1 << irq), VIC_INTENCLEAR_OFFSET);
    }
}

/***********************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the IRQ specified by 'irq'
 *
 ***********************************************************************/

void up_enable_irq(int irq)
{
  /* Verify that the IRQ number is within range */

  if (irq < NR_IRQS)
    {
      /* Disable all interrupts */

      irqstate_t flags = irqsave();

      /* Enable the irq by setting the corresponding bit in the VIC Interrupt
       * Enable register.
       */

      uint32_t val = vic_getreg(VIC_INTENABLE_OFFSET);
      vic_putreg(val | (1 << irq), VIC_INTENABLE_OFFSET);
      irqrestore(flags);
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
  uint32_t reg32;

  if ((unsigned)irq < NR_IRQS)
    {
      /* Mask the IRQ by clearing the associated bit in Software Priority Mask
       * register
       */

      reg32 = vic_getreg(VIC_PRIORITY_MASK_OFFSET);
      reg32 &= ~(1 << irq);
      vic_putreg(reg32, VIC_PRIORITY_MASK_OFFSET);
    }

  /* Clear interrupt */

  vic_putreg((1 << irq), VIC_SOFTINTCLEAR_OFFSET);
#ifdef CONFIG_VECTORED_INTERRUPTS
  vic_putreg(0, VIC_ADDRESS_OFFSET);    /* dummy write to clear VICADDRESS */
#endif
}

/****************************************************************************
 * Name: up_prioritize_irq
 *
 * Description:
 *   set interrupt priority
 * MOD
 ****************************************************************************/

int up_prioritize_irq(int irq, int priority)
{
  /* The default priority on reset is 16 */
  if (irq < NR_IRQS && priority > 0 && priority < 16)
    {
      int offset = irq << 2;
      vic_putreg(priority, VIC_VECTPRIORITY0_OFFSET + offset);
      return OK;
    }
  return -EINVAL;
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

  if (irq < NR_IRQS && vector < 32 && handler)
    {
      int offset = vector << 2;

      /* Disable all interrupts */

      irqstate_t flags = irqsave();

      /* Save the vector address */

      vic_putreg((uint32_t) handler, VIC_VECTADDR0_OFFSET + offset);

      /* Set the interrupt priority */

      up_prioritize_irq(irq, vector);

      /* Enable the vectored interrupt */

      uint32_t val = vic_getreg(VIC_INTENABLE_OFFSET);
      vic_putreg(val | (1 << irq), VIC_INTENABLE_OFFSET);

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

#ifdef CONFIG_VECTORED_INTERRUPTS
void up_detach_vector(int vector)
{
  /* Verify that the vector number is within range */

  if (vector < 32)
    {
      /* Disable the vectored interrupt */

      int offset = vector << 2;
      vic_putreg(0, (VIC_VECTADDR0_OFFSET + offset));
    }
}
#endif
