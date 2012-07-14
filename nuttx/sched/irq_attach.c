/****************************************************************************
 * sched/irq_attach.c
 *
 *   Copyright (C) 2007-2008, 2010, 2012 Gregory Nutt. All rights reserved.
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

#include "irq_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: irq_attach
 *
 * Description:
 *   Configure the IRQ subsystem so that IRQ number 'irq' is dispatched to
 *   'isr'
 *
 ****************************************************************************/

int irq_attach(int irq, xcpt_t isr)
{
#if NR_IRQS > 0
  int ret = ERROR;

  if ((unsigned)irq < NR_IRQS)
    {
      irqstate_t state;

      /* If the new ISR is NULL, then the ISR is being detached.
       * In this case, disable the ISR and direct any interrupts
       * to the unexpected interrupt handler.
       */

      state = irqsave();
      if (isr == NULL)
        {
          /* Disable the interrupt if we can before detaching it.  We might
           * not be able to do this if:  (1) the device does not have a 
           * centralized interrupt controller (so up_disable_irq() is not
           * supported).  Or (2) if the device has different number for vector
           * numbers and IRQ numbers (in that case, we don't know the correct
           * IRQ number to use to disable the interrupt).  In those cases, the
           * code will just need to be careful that it disables all interrupt
           * sources before detaching from the interrupt vector.
           */

#if !defined(CONFIG_ARCH_NOINTC) && !defined(CONFIG_ARCH_VECNOTIRQ)
          up_disable_irq(irq);
#endif
          /* Detaching the ISR really means re-attaching it to the
           * unexpected exception handler.
           */

           isr = irq_unexpected_isr;
        }

      /* Save the new ISR in the table. */

      g_irqvector[irq] = isr;
      irqrestore(state);
      ret = OK;
    }

  return ret;
#else
  return OK;
#endif
}
