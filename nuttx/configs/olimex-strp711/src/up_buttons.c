/****************************************************************************
 * configs/olimex-strp711/src/up_leds.c
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

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* The Olimex board has two buttons, one labled "BUT" and the other "WAKEUP"
 *
 * P0.15: WAKEUP button
 * P1.13: BUT button
 */

#define STR71X_BUTBUTTON_GPIO1    (0x2000)
#define STR71X_WAKEUPBUTTON_GPIO0 (0x8000)

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
 * Name: up_buttoninit
 ****************************************************************************/

#ifdef CONFIG_ARCH_BUTTONS
void up_buttoninit(void)
{
  uint16_t reg16;

  /* Configure the GPIO0 & 1 pins as inputs */

  reg16  = getreg16(STR71X_GPIO0_PC0);
  reg16 |= STR71X_WAKEUPBUTTON_GPIO0;
  putreg16(reg16, STR71X_GPIO0_PC0);

  reg16  = getreg16(STR71X_GPIO0_PC1);
  reg16 &= ~STR71X_WAKEUPBUTTON_GPIO0;
  putreg16(reg16, STR71X_GPIO0_PC1);

  reg16  = getreg16(STR71X_GPIO0_PC2);
  reg16 &= ~STR71X_WAKEUPBUTTON_GPIO0;
  putreg16(reg16, STR71X_GPIO0_PC2);

  reg16  = getreg16(STR71X_GPIO1_PC0);
  reg16 |= STR71X_BUTBUTTON_GPIO1;
  putreg16(reg16, STR71X_GPIO1_PC0);

  reg16  = getreg16(STR71X_GPIO1_PC1);
  reg16 &= ~STR71X_BUTBUTTON_GPIO1;
  putreg16(reg16, STR71X_GPIO1_PC1);

  reg16  = getreg16(STR71X_GPIO1_PC2);
  reg16 &= ~STR71X_BUTBUTTON_GPIO1;
  putreg16(reg16, STR71X_GPIO1_PC2);
}

/****************************************************************************
 * Name: up_buttons
 ****************************************************************************/

uint8_t up_buttons(void)
{
  uint8_t ret    = 0;

  if ((getreg16(STR71X_GPIO0_PD) & STR71X_WAKEUPBUTTON_GPIO0) != 0)
    {
      ret |= WAKEUP_BUTTON;
    }

  if ((getreg16(STR71X_GPIO1_PD) & STR71X_BUTBUTTON_GPIO1) != 0)
    {
      ret |= BUT_BUTTON;
    }

  return ret;
}
#endif /* CONFIG_ARCH_BUTTONS */
