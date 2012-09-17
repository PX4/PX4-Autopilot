/****************************************************************************
 * arch/arm/src/c5471/c5471_irq.c
 *
 *   Copyright (C) 2007, 2009, 2011-2012 Gregory Nutt. All rights reserved.
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
#include <nuttx/irq.h>

#include "arm.h"
#include "chip.h"
#include "up_arch.h"
#include "os_internal.h"
#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ILR_EDGESENSITIVE 0x00000020
#define ILR_PRIORITY      0x0000001E

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile uint32_t *current_regs;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The value of _svectors is defined in ld.script.  It could be hard-coded
 * because we know that correct IRAM area is 0xffc00000.
 */

extern int _svectors; /* Type does not matter */

/* The C5471 has FLASH at the low end of memory.  The rrload bootloaer will
 * catch all interrupts and re-vector them to vectors stored in IRAM.  The
 * following table is used to initialize those vectors.
 */

static up_vector_t g_vectorinittab[] =
{
  (up_vector_t)NULL,
  up_vectorundefinsn,
  up_vectorswi,
  up_vectorprefetch,
  up_vectordata,
  up_vectoraddrexcptn,
  up_vectorirq,
  up_vectorfiq
};
#define NVECTORS ((sizeof(g_vectorinittab)) / sizeof(up_vector_t))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_ackirq
 *
 * Description:
 *   Acknowlede the IRQ.Bit 0 of the Interrupt Control
 *   Register ==  New IRQ agreement (NEW_IRQ_AGR). Reset IRQ
 *   output. Clear source IRQ register. Enables a new IRQ
 *   generation. Reset by internal logic.
 *
 ****************************************************************************/

static inline void up_ackirq(unsigned int irq)
{
  uint32_t reg;
  reg = getreg32(SRC_IRQ_REG);              /* Insure appropriate IT_REG bit clears */
  putreg32(reg | 0x00000001, INT_CTRL_REG); /* write the NEW_IRQ_AGR bit. */
}

/****************************************************************************
 * Name: up_ackfiq
 *
 * Description:
 *   Acknowledge the FIQ.  Bit 1 of the Interrupt Control
 *   Register ==  New FIQ agreement (NEW_FIQ_AGR). Reset FIQ
 *   output. Clear source FIQ register. Enables a new FIQ
 *   generation. Reset by internal logic.
 *
 ****************************************************************************/

static inline void up_ackfiq(unsigned int irq)
{
  uint32_t reg;
  reg = getreg32(SRC_FIQ_REG);              /* Insure appropriate IT_REG bit clears */
  putreg32(reg | 0x00000002, INT_CTRL_REG); /* write the NEW_FIQ_AGR bit. */
}

/****************************************************************************
 * Name: up_vectorinitialize
 ****************************************************************************/

static inline void up_vectorinitialize(void)
{
  up_vector_t *src  = g_vectorinittab;
  up_vector_t *dest = (up_vector_t*)&_svectors;
  int i;

  for (i = 0; i < NVECTORS; i++)
    {
      *dest++ = *src++;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  /* Disable all interrupts. */

  putreg32(0x0000ffff, MASK_IT_REG);

  /* Clear any pending interrupts */

  up_ackirq(0);
  up_ackfiq(0);
  putreg32(0x00000000, IT_REG);

  /* Override hardware defaults */

  putreg32(ILR_EDGESENSITIVE | ILR_PRIORITY, ILR_IRQ2_REG);
  putreg32(ILR_EDGESENSITIVE | ILR_PRIORITY, ILR_IRQ4_REG);
  putreg32(ILR_PRIORITY,                     ILR_IRQ6_REG);
  putreg32(ILR_EDGESENSITIVE | ILR_PRIORITY, ILR_IRQ15_REG);

  /* Initialize hardware interrupt vectors */

  up_vectorinitialize();
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
  if ((unsigned)irq < NR_IRQS)
    {
      uint32_t reg = getreg32(MASK_IT_REG);
      putreg32(reg | (1 << irq), MASK_IT_REG);
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
  if ((unsigned)irq < NR_IRQS)
    {
      uint32_t reg = getreg32(MASK_IT_REG);
      putreg32(reg & ~(1 << irq), MASK_IT_REG);
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
  uint32_t reg = getreg32(INT_CTRL_REG);

  /* Mask the interrupt */

  reg = getreg32(MASK_IT_REG);
  putreg32(reg | (1 << irq), MASK_IT_REG);

  /* Set the NEW_IRQ_AGR bit.  This clears the IRQ src register
   * enables generation of a new IRQ.
   */

  reg = getreg32(INT_CTRL_REG);
  putreg32(reg | 0x00000001, INT_CTRL_REG); /* write the NEW_IRQ_AGR bit. */
}
