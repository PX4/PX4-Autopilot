/****************************************************************************
 * configs/ez80f910200zco/src/ez80_leds.c
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

#include <stdint.h>

#include <nuttx/irq.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_PB/1/2interrupt
 *
 * Description:
 *   These could be exteneded to provide interrupt driven button input
 *
 ****************************************************************************/

#if 0
void up_PBinterrupt(void)
{
  uint8_t regval;

  regval = inp(EZ80_PB_DR); /* Clear interrupt flag for eZ80F91 date codes before 0611 */
  regval |= 7;
  outp(EZ80_PB_DR, regval);

  regval = inp(EZ80_PB_ALT0); /* Clear interrupt flag for eZ80F91 date codes 0611 and after */
  regval |= 1;
  outp(EZ80_PB_ALT0, regval);
}

void up_pb1interrupt(void)
{
  uint8_t regval;

  regval = inp(EZ80_PB_DR); /* Clear interrupt flag for eZ80F91 date codes before 0611 */
  regval |= 7;
  outp(EZ80_PB_DR, regval);

  regval = inp(EZ80_PB_ALT0); /* Clear interrupt flag for eZ80F91 date codes 0611 and after */
  regval |= 2;
  outp(EZ80_PB_ALT0, regval);
}

void up_pb2interrupt(void)
{
  uint8_t regval;

  regval = inp(EZ80_PB_DR); /* Clear interrupt flag for eZ80F91 date codes before 0611 */
  regval |= 7;
  outp(EZ80_PB_DR, regval);

  regval = inp(EZ80_PB_ALT0);
  regval |= 4;
  outp(EZ80_PB_ALT0, regval); /* Clear interrupt flag for eZ80F91 date codes 0611 and after */
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_buttoninit
 ****************************************************************************/

#ifdef CONFIG_ARCH_BUTTONS
void up_buttoninit(void)
{
  uint8_t regval;

#if 0 /* Interrupts are not used */

  /* Attach GIO interrupts */
  
  irq_attach(EZ80_PB_IRQ, up_PBinterrupt);
  irq_attach(EZ80_PB1_IRQ, up_pb1interrupt);
  irq_attach(EZ80_PB2_IRQ, up_pb2interrupt);

  /* Configure PB0,1,2 as interrupt, rising edge */

  regval = inp(EZ80_PB_DR);
  regval |= 7;
  outp(EZ80_PB_DR, regval);

  regval = inp(EZ80_PB_DDR);
  regval |= 7;
  outp(EZ80_PB_DDR, regval);

  regval = inp(EZ80_PB_ALT1);
  regval |= 7;
  outp(EZ80_PB_ALT1, regval);

  regval = inp(EZ80_PB_ALT2);
  regval |= 7;
  outp(EZ80_PB_ALT2, regval);
#else
  /* Configure PB0,1,2 as inputs */

  regval = inp(EZ80_PB_DDR);
  regval |= 7;
  outp(EZ80_PB_DDR, regval);

  regval = inp(EZ80_PB_ALT1);
  regval &= ~7;
  outp(EZ80_PB_ALT1, regval);

  regval = inp(EZ80_PB_ALT2);
  regval &= ~7;
  outp(EZ80_PB_ALT2, regval);
#endif
}

/****************************************************************************
 * Name: up_buttons
 ****************************************************************************/

uint8_t up_buttons(void)
{
  return inp(EZ80_PB_DDR) & 7;
}
#endif /* CONFIG_ARCH_BUTTONS */
