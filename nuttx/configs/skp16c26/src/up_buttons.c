/****************************************************************************
 * configs/skp16c26/src/up_buttons.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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

/* The SKP62C26 has 3 buttons control by bits 1, 2, and 3 in port 8. */

#define SW1_BIT             (1 << 3)   /* Bit 3, port 8 */
#define SW2_BIT             (1 << 2)   /* Bit 2, port 8 */
#define SW3_BIT             (1 << 1)   /* Bit 1, port 8 */

#define SW_PRESSED(p,b)     (((p) & (b)) == 0)

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
  uint8_t regval;

  regval  = getreg8(M16C_PD8);
  regval |= (SW1_BIT | SW2_BIT | SW3_BIT);
  putreg8(regval, M16C_PD8);
}

/****************************************************************************
 * Name: up_buttons
 ****************************************************************************/

uint8_t up_buttons(void)
{
  uint8_t swset  = 0;
  uint8_t regval = getreg8(M16C_P8);

  if (SW_PRESSED(regval, SW1_BIT))
    {
      swset |= SW1_PRESSED;
    }

  if (SW_PRESSED(regval, SW2_BIT))
    {
      swset |= SW2_PRESSED;
    }

  if (SW_PRESSED(regval, SW3_BIT))
    {
      swset |= SW3_PRESSED;
    }

  return swset;
}
#endif /* CONFIG_ARCH_BUTTONS */
