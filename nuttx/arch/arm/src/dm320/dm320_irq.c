/************************************************************************
 * arch/arm/src/dm320/dm320_irq.c
 *
 *   Copyright (C) 2007, 2009, 2012 Gregory Nutt. All rights reserved.
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
 ************************************************************************/

/************************************************************************
 * Included Files
 ************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <nuttx/irq.h>

#include "arm.h"
#include "chip.h"

#include "up_arch.h"
#include "os_internal.h"
#include "up_internal.h"

/************************************************************************
 * Pre-processor Definitions
 ************************************************************************/

/************************************************************************
 * Public Data
 ************************************************************************/

volatile uint32_t *current_regs;

/************************************************************************
 * Private Data
 ************************************************************************/

/* The value of _svectors is defined in ld.script.  It could be hard-
 * coded because we know that correct IRAM area is 0xffc00000.
 */

extern int _svectors; /* Type does not matter */

/************************************************************************
 * Private Functions
 ************************************************************************/

/************************************************************************
 * Public Functions
 ************************************************************************/

/************************************************************************
 * Name: up_irqinitialize
 ************************************************************************/

void up_irqinitialize(void)
{
  /* Clear, disable and configure all interrupts. */

  putreg16(0, DM320_INTC_EINT0);      /* Mask all IRQs/FIQs */
  putreg16(0, DM320_INTC_EINT1);
  putreg16(0, DM320_INTC_EINT2);

  putreg16(0, DM320_INTC_INTRAW);     /* No masked interrupts in status */

  putreg16(0, DM320_INTC_FISEL0);     /* No FIQs */
  putreg16(0, DM320_INTC_FISEL1);
  putreg16(0, DM320_INTC_FISEL2);

  putreg16(0xffff, DM320_INTC_FIQ0);  /* Clear all pending FIQs */
  putreg16(0xffff, DM320_INTC_FIQ1);
  putreg16(0xffff, DM320_INTC_FIQ2);

  putreg16(0xffff, DM320_INTC_IRQ0);  /* Clear all pending IRQs */
  putreg16(0xffff, DM320_INTC_IRQ1);
  putreg16(0xffff, DM320_INTC_IRQ2);

 /* Make sure that the base addresses are zero and that
  * the table increment is 4 bytes.
  */

  putreg16(0, DM320_INTC_EABASE0);
  putreg16(0, DM320_INTC_EABASE1);

  /* currents_regs is non-NULL only while processing an interrupt */

  current_regs = NULL;

  /* And finally, enable interrupts */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  irqrestore(SVC_MODE | PSR_F_BIT);
#endif
}

/************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the IRQ specified by 'irq'
 *
 ************************************************************************/

void up_disable_irq(int irq)
{
  /* Disable the interrupt by clearing the corresponding bit in
   * the IRQ enable register.
   */

  if (irq < 16)
    {
      /* IRQs0-15 are controlled by the IRQ0 enable register
       * Clear the associated bit to disable the interrupt
       */

      putreg16((getreg16(DM320_INTC_EINT0) & ~(1 << irq)), DM320_INTC_EINT0);
    }
  else if (irq < 32)
    {
      /* IRQs16-31 are controlled by the IRQ1 enable register
       * Clear the associated bit to disable the interrupt
       */

      putreg16((getreg16(DM320_INTC_EINT1) & ~(1 << (irq-16))), DM320_INTC_EINT1);
    }
  else
    {
      /* IRQs32- are controlled by the IRQ2 enable register
       * Clear the associated bit to disable the interrupt
       */

      putreg16((getreg16(DM320_INTC_EINT2) & ~(1 << (irq-32))), DM320_INTC_EINT2);
    }
}

/************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the IRQ specified by 'irq'
 *
 ************************************************************************/

void up_enable_irq(int irq)
{
  /* Enable the interrupt by setting the corresponding bit in
   * the IRQ enable register.
   */

  if (irq < 16)
    {
      /* IRQs0-15 are controlled by the IRQ0 enable register
       * Set the associated bit to enable the interrupt
       */

      putreg16((getreg16(DM320_INTC_EINT0) | (1 << irq)), DM320_INTC_EINT0);
    }
  else if (irq < 32)
    {
      /* IRQs16-31 are controlled by the IRQ1 enable register
       * Set the associated bit to enable the interrupt
       */

      putreg16((getreg16(DM320_INTC_EINT1) | (1 << (irq-16))), DM320_INTC_EINT1);
    }
  else
    {
      /* IRQs32- are controlled by the IRQ2 enable register
       * Set the associated bit to enable the interrupt
       */

      putreg16((getreg16(DM320_INTC_EINT2) | (1 << (irq-32))), DM320_INTC_EINT2);
    }
}

/************************************************************************
 * Name: up_maskack_irq
 *
 * Description:
 *   Mask the IRQ and acknowledge it
 *
 ************************************************************************/

void up_maskack_irq(int irq)
{
  /* Disable the interrupt by clearing the corresponding bit in
   * the IRQ enable register.  And acknowlege it by setting the
   * corresponding bit in the IRQ status register.
   */

  if (irq < 16)
    {
      /* IRQs0-15 are controlled by the IRQ0 enable register
       * Clear the associated enable bit to disable the interrupt
       * Set the associated status bit to clear the interrupt
       */

      putreg16((getreg16(DM320_INTC_EINT0) & ~(1<< irq)), DM320_INTC_EINT0);
      putreg16((1 << irq), DM320_INTC_IRQ0);
    }
  else if (irq < 32)
    {
      /* IRQs16-31 are controlled by the IRQ1 enable register
       * Clear the associated enable bit to disable the interrupt
       * Set the associated status bit to clear the interrupt
       */

      putreg16((getreg16(DM320_INTC_EINT1) & ~(1<< (irq-16))), DM320_INTC_EINT1);
      putreg16((1 << (irq-16)), DM320_INTC_IRQ1);
    }
  else
    {
      /* IRQs32- are controlled by the IRQ2 enable register
       * Clear the associated enable bit to disable the interrupt
       * Set the associated status bit to clear the interrupt
       */

      putreg16((getreg16(DM320_INTC_EINT2) & ~(1<< (irq-32))), DM320_INTC_EINT2);
      putreg16((1 << (irq-32)), DM320_INTC_IRQ2);
    }
}
