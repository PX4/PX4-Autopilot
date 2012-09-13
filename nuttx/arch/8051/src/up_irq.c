/************************************************************************
 * up_irq.c
 *
 *   Copyright (C) 2007, 2009 Gregory Nutt. All rights reserved.
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

#include <nuttx/irq.h>
#include <debug.h>

#include <8052.h>

#include "up_internal.h"

extern int g_nints;

/************************************************************************
 * Definitions
 ************************************************************************/

/************************************************************************
 * Public Data
 ************************************************************************/

/************************************************************************
 * Private Data
 ************************************************************************/

/************************************************************************
 * Private Functions
 ************************************************************************/

/************************************************************************
 * Public Funtions
 ************************************************************************/

/************************************************************************
 * Name: irq_initialize
 ************************************************************************/

void up_irqinitialize(void)
{
  /* Set interrupt priorities (all low) */

  IP = 0;

#ifdef CONFIG_SUPPRESS_INTERRUPTS
  /* Disable all interrupts */

  IE = 0;
#else
  /* Enable interrupts globally, but disable all interrupt
   * sources.
   */

  IE = 0x80;
#endif
}

/************************************************************************
 * Name: irqsave
 *
 * Description:
 *   Disable all IRQs
 *
 ************************************************************************/

irqstate_t irqsave(void)
{
  irqstate_t ret = IE;
  EA = 0;
  return ret;
}

/************************************************************************
 * Name: irqrestore
 *
 * Description:
 *   Restore a previous interrupt state
 *
 ************************************************************************/

void irqrestore(irqstate_t flags)
{
  IE = flags;
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
  if ((unsigned)irq < NR_IRQS)
    {
      IE &= ~(g_ntobit[irq]);
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
  if ((unsigned)irq < NR_IRQS)
    {
      IE |= g_ntobit[irq];
    }
}
