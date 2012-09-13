/****************************************************************************
 * arch/sh/src/m16c/m16c_irq.c
 *
 *   Copyright (C) 2009, 2011 Gregory Nutt. All rights reserved.
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
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "up_internal.h"

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* This holds a references to the current interrupt level register storage
 * structure.  If is non-NULL only during interrupt processing.
 */

volatile uint32_t *current_regs; /* Actually a pointer to the beginning of a uint8_t array */

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
  current_regs = NULL;
  
  /* And finally, enable interrupts */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  asm("fset i");
#endif
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   On many architectures, there are three levels of interrupt enabling: (1)
 *   at the global level, (2) at the level of the interrupt controller,
 *   and (3) at the device level.  In order to receive interrupts, they
 *   must be enabled at all three levels.
 *
 *   This function implements disabling of the device specified by 'irq'
 *   at the interrupt controller level if supported by the architecture
 *   (irqsave() supports the global level, the device level is hardware
 *   specific).
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_NOINTC
void up_disable_irq(int irq)
{
  /* There are no ez80 interrupt controller settings to disable IRQs */
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   This function implements enabling of the device specified by 'irq'
 *   at the interrupt controller level if supported by the architecture
 *   (irqsave() supports the global level, the device level is hardware
 *   specific).
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
  /* There are no ez80 interrupt controller settings to enable IRQs */
}

#endif /* CONFIG_ARCH_NOINTC */
