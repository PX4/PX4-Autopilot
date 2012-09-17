/****************************************************************************
 * arch/arm/src/kinetis/kinetis_clrpend.c
 * arch/arm/src/chip/kinetis_clrpend.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

#include <arch/irq.h>

#include "nvic.h"
#include "up_arch.h"
#include "kinetis_internal.h"

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
 * Name: kinetis_clrpend
 *
 * Description:
 *   Clear a pending interrupt at the NVIC.  This does not seem to be required
 *   for most interrupts.  Don't know why... 
 *
 *   I keep it in a separate file so that it will not increase the footprint
 *   on Kinetis platforms that do not need this function.
 *
 ****************************************************************************/

void kinetis_clrpend(int irq)
{
  /* Check for external interrupt */

  if (irq >= KINETIS_IRQ_EXTINT)
    {
      if (irq < (KINETIS_IRQ_EXTINT+32))
        {
          putreg32(1 << (irq - KINETIS_IRQ_EXTINT), NVIC_IRQ0_31_CLRPEND);
        }
      else if (irq < (KINETIS_IRQ_EXTINT+64))
        {
          putreg32(1 << (irq - KINETIS_IRQ_EXTINT - 32), NVIC_IRQ32_63_CLRPEND);
        }
      else if (irq < (KINETIS_IRQ_EXTINT+96))
        {
          putreg32(1 << (irq - KINETIS_IRQ_EXTINT - 64), NVIC_IRQ64_95_CLRPEND);
        }
      else if (irq < NR_IRQS)
        {
          putreg32(1 << (irq - KINETIS_IRQ_EXTINT - 96), NVIC_IRQ96_127_CLRPEND);
        }
    }
}
