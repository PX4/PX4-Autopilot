/****************************************************************************
 * configs/us7032evb1/src/up_leds.c
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

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* The SH1_LPEVB only a single LED controlled by either port A, pin 15, or
 * port B, pin 15 (selectable via JP8).  In this file, we assume the portB
 * setup.
 */

#define SH1_PBDR_LED  0x8000
#define SH1_PBIOR_LED 0x8000
#define SH1_PBCR2_LED 0xc000

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
 * Name: up_ledinit
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void up_ledinit(void)
{
  uint16_t reg16;

  /* Setup port B, pin 15 as an output */

  reg16  = getreg16(SH1_PFC_PBIOR);
  reg16 |= SH1_PBIOR_LED;
  putreg16(reg16, SH1_PFC_PBIOR);

  /* Setup port B, pin 15 as a normal I/O register */

  reg16  = getreg16(SH1_PFC_PBCR1);
  reg16 &= ~SH1_PBCR2_LED;
  putreg16(reg16, SH1_PFC_PBCR1);

  /* Turn the LED off */

  reg16  = getreg16(SH1_PORTB_DR);
  reg16 &= ~SH1_PBDR_LED;
  putreg16(reg16, SH1_PORTB_DR);
}

/****************************************************************************
 * Name: up_ledon
 ****************************************************************************/

void up_ledon(int led)
{
  uint16_t reg16;

  if (led)
    {
      /* Turn the LED on */

      reg16  = getreg16(SH1_PORTB_DR);
      reg16 |= SH1_PBDR_LED;
      putreg16(reg16, SH1_PORTB_DR);
    }
}

/****************************************************************************
 * Name: up_ledoff
 ****************************************************************************/

void up_ledoff(int led)
{
  uint16_t reg16;

  if (led)
    {
      /* Turn the LED off */

      reg16  = getreg16(SH1_PORTB_DR);
      reg16 &= ~SH1_PBDR_LED;
      putreg16(reg16, SH1_PORTB_DR);
    }
}
#endif /* CONFIG_ARCH_LEDS */
