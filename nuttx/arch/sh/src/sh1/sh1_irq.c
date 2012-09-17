/****************************************************************************
 * arch/sh/src/sh1/sh1_irq.c
 *
 *   Copyright (C) 2008-2009, 2011 Gregory Nutt. All rights reserved.
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

#include "up_arch.h"
#include "up_internal.h"
#include "chip.h"

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
 * Public Funtions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  /* Currents_regs is non-NULL only while processing an interrupt */

  current_regs = NULL;

  /* Enable interrupts */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  irqenable();
#endif
}

/****************************************************************************
 * Name: up_prioritize_irq
 *
 * Description:
 *   set interrupt priority
 *
 ****************************************************************************/

void up_prioritize_irq(int irq, int priority)
{
  uint16_t mask;
  uint16_t reg16;
  uint32_t reg;
  int      shift;

#ifdef CONFIG_DEBUG
  if ((unsigned) irq > NR_IRQS || (unsigned)priority > 15)
    {
      dbg("Invalid parameters\n");
      return;
    }
#endif

  /* Get the register to set, the mask for the bits to be set, and
   * the shift value to position the bits in the register.
   */

  switch (irq)
    {
#if 0 /* Not yet supported */
    case SH1_IRQ0_IRQ:
      mask  = SH1_IPRA_IRQ0MASK;
      shift = SH1_IPRA_IRQ0SHIFT;
      goto ipra;

    case SH1_IRQ1_IRQ:
      mask  = SH1_IPRA_IRQ1MASK;
      shift = SH1_IPRA_IRQ1SHIFT;
      goto ipra;

    case SH1_IRQ2_IRQ:
      mask  = SH1_IPRA_IRQ2MASK;
      shift = SH1_IPRA_IRQ2SHIFT;
      goto ipra;

    case SH1_IRQ3_IRQ:
      mask  = SH1_IPRA_IRQ3MASK;
      shift = SH1_IPRA_IRQ3SHIFT;
      goto ipra;
    ipra:
      reg   = SH1_INTC_IPRA;
      break;

    case SH1_IRQ4_IRQ:
      mask  = SH1_IPRB_IRQ4MASK;
      shift = SH1_IPRB_IRQ4SHIFT;
      goto iprb;

    case SH1_IRQ5_IRQ:
      mask  = SH1_IPRB_IRQ5MASK;
      shift = SH1_IPRB_IRQ5SHIFT;
      goto iprb;

    case SH1_IRQ6_IRQ:
      mask  = SH1_IPRB_IRQ6MASK;
      shift = SH1_IPRB_IRQ6SHIFT;
      goto iprb;

    case SH1_IRQ7_IRQ:
      mask  = SH1_IPRB_IRQ7MASK;
      shift = SH1_IPRB_IRQ7SHIFT;
      goto iprb;
    iprb:
      reg   = SH1_INTC_IPRB;
      break;
#endif

#if defined(CONFIG_SH1_DMAC0) || defined(CONFIG_SH1_DMAC1)
    case SH1_DEI0_IRQ:
    case SH1_DEI1_IRQ:
      mask  = SH1_IPRC_DM01MASK;
      shift = SH1_IPRC_DM01SHIFT;
      goto iprc;
#endif

#if defined(CONFIG_SH1_DMAC2) || defined(CONFIG_SH1_DMAC3)
    case SH1_DEI2_IRQ:
    case SH1_DEI3_IRQ:
      mask  = SH1_IPRC_DM23MASK;
      shift = SH1_IPRC_DM23SHIFT;
      goto iprc;
#endif

#ifdef CONFIG_SH1_ITU1
    case SH1_IMIA1_IRQ:
    case SH1_IMIB1_IRQ:
    case SH1_OVI1_IRQ:
      mask  = SH1_IPRC_ITU1MASK;
      shift = SH1_IPRC_ITU1SHIFT;
      goto iprc;
#endif

    case SH1_IMIA0_IRQ:
    case SH1_IMIB0_IRQ:
    case SH1_OVI0_IRQ:
      mask  = SH1_IPRC_ITU0MASK;
      shift = SH1_IPRC_ITU0SHIFT;
      goto iprc;
    iprc:
      reg   = SH1_INTC_IPRC;
      break;

#if defined(CONFIG_SH1_ITU2) || defined(CONFIG_SH1_ITU3) || \
    defined(CONFIG_SH1_ITU4) || defined(CONFIG_SH1_SCI0)
#ifdef CONFIG_SH1_ITU2
    case SH1_IMIA2_IRQ:
    case SH1_IMIB2_IRQ:
    case SH1_OVI2_IRQ:
      mask  = SH1_IPRD_ITU2MASK;
      shift = SH1_IPRD_ITU2SHIFT;
      goto iprd;
#endif
#ifdef CONFIG_SH1_ITU3
    case SH1_IMIA3_IRQ:
    case SH1_IMIB3_IRQ:
    case SH1_OVI3_IRQ:
      mask  = SH1_IPRD_ITU3MASK;
      shift = SH1_IPRD_ITU3SHIFT;
      goto iprd;
#endif
#ifdef CONFIG_SH1_ITU4
    case SH1_IMIA4_IRQ:
    case SH1_IMIB4_IRQ:
    case SH1_OVI4_IRQ:
      mask  = SH1_IPRD_ITU4MASK;
      shift = SH1_IPRD_ITU4SHIFT;
      goto iprd;
#endif
#ifdef CONFIG_SH1_SCI0
    case SH1_ERI0_IRQ:
    case SH1_RXI0_IRQ:
    case SH1_TXI0_IRQ:
    case SH1_TEI0_IRQ:
      mask  = SH1_IPRD_SCI0MASK;
      shift = SH1_IPRD_SCI0SHIFT;
      goto iprd;
#endif
    iprd:
      reg   = SH1_INTC_IPRD;
      break;
#endif

#if defined(CONFIG_SH1_SCI1) || defined(CONFIG_SH1_PCU) || \
    defined(CONFIG_SH1_AD) || defined(CONFIG_SH1_WDT) || defined(CONFIG_SH1_CMI)
#ifdef CONFIG_SH1_SCI1
    case SH1_ERI1_IRQ:
    case SH1_RXI1_IRQ:
    case SH1_TXI1_IRQ:
    case SH1_TEI1_IRQ:
      mask  = SH1_IPRE_SCI1MASK;
      shift = SH1_IPRE_SCI1SHIFT;
      goto ipre;
#endif
#if defined(CONFIG_SH1_PCU) || defined(CONFIG_SH1_AD)
    case SH1_PEI_IRQ:
    case SH1_ADITI_IRQ:
      mask  = SH1_IPRE_PRADMASK;
      shift = SH1_IPRE_PRADSHIFT;
      goto ipre;
#endif
#if defined(CONFIG_SH1_WDT) || defined(CONFIG_SH1_CMI)
    case SH1_WDTITI_IRQ:
    case SH1_CMI_IRQ:
      mask  = SH1_IPRE_WDRFMASK;
      shift = SH1_IPRE_WDRFSHIFT;
      goto ipre;
#endif
    ipre:
      reg   = SH1_INTC_IPRE;
      break;
#endif

    default:
      dbg("Invalid irq=%d\n", irq);
      return;
    }

  /* Then set the priority bits */

  reg16  = getreg16(reg);
  reg16 &= ~mask;
  reg16 |= (priority << shift);
  putreg16(reg16, reg);
}

