/****************************************************************************
 *  arch/arm/src/lpc43/lpc43_pin_config.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

#include <arch/board/board.h>
#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <errno.h>

#include "up_arch.h"
#include "lpc43_pinconfig.h"

/****************************************************************************
 * Pre-processor Definitions
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
 * Name: lpc43_pin_config
 *
 * Description:
 *   Configure a pin based on bit-encoded description of the pin.
 *
 * Input Value:
 *   20-bit encoded value describing the pin.
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ****************************************************************************/

int lpc43_pin_config(uint32_t pinconf)
{
  unsigned int pinset = ((pinconf & PINCONF_PINS_MASK) >> PINCONF_PINS_SHIFT);
  unsigned int pin    = ((pinconf & PINCONF_PIN_MASK) >> PINCONF_PIN_SHIFT);
  unsigned int func   = ((pinconf & PINCONF_FUNC_MASK) >> PINCONF_FUNC_SHIFT);
  uintptr_t regaddr;
  uint32_t  regval;

  /* Set up common pin configurations */

  regval = (func << SCU_PIN_MODE_SHIFT);

  /* Enable/disable pull-down resistor */

  if (PINCONF_IS_PULLDOWN(pinconf))
    {
      regval |= SCU_PIN_EPD;  /* Set bit to enable */
    }

  if (!PINCONF_IS_PULLUP(pinconf))
    {
      regval |= SCU_PIN_EPUN; /* Set bit to disable */
    }

  /* Enable/disable input buffering */

  if (PINCONF_INBUFFER_ENABLED(pinconf))
   {
     regval |= SCU_PIN_EZI; /* Set bit to enable */
   }

  /* Enable/disable glitch filtering */

  if (!PINCONF_GLITCH_ENABLE(pinconf))
   {
     regval |= SCU_PIN_ZIF; /* Set bit to disable */
   }

  /* Only normal and high speed pins support the slew rate setting */

  if (PINCONF_IS_SLEW_FAST(pinconf))
   {
     regval |= SCU_NDPIN_EHS; /* 0=slow; 1=fast */
   }
 
  /* Only high drive pins suppose drive strength */

  switch (pinconf & PINCONF_DRIVE_MASK)
    {
      default:
      case PINCONF_DRIVE_NORMAL: /* Normal-drive: 4 mA drive strength (or not high drive pin) */
        regval |= SCU_HDPIN_EHD_NORMAL;
        break;

      case PINCONF_DRIVE_MEDIUM: /* Medium-drive: 8 mA drive strength */
        regval |= SCU_HDPIN_EHD_MEDIUM;
        break;

      case PINCONF_DRIVE_HIGH: /* High-drive: 14 mA drive strength */
        regval |= SCU_HDPIN_EHD_HIGH;
        break;

      case PINCONF_DRIVE_ULTRA: /* Ultra high-drive: 20 mA drive strength */
        regval |= SCU_HDPIN_EHD_ULTRA;
        break;
    }

  /* Get the address of the pin configuration register and save the new
   * pin configuration.
   */

  regaddr =  LPC43_SCU_SFSP(pinset, pin);
  putreg32(regval, regaddr);

  return OK;
}
