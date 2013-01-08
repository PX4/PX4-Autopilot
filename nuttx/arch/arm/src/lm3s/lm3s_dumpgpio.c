/****************************************************************************
 * arch/arm/src/lm3s/lm3s_dumpgpio.c
 *
 *   Copyright (C) 2009-2010 Gregory Nutt. All rights reserved.
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
#include <stdbool.h>
#include <debug.h>

#include <nuttx/arch.h>

#include "up_arch.h"

#include "chip.h"
#include "lm_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* NOTE: this is duplicated in lm3s_gpio.c */

#ifdef LM3S_GPIOH_BASE
static const uint32_t g_gpiobase[8] =
{
  LM3S_GPIOA_BASE, LM3S_GPIOB_BASE, LM3S_GPIOC_BASE, LM3S_GPIOD_BASE,
  LM3S_GPIOE_BASE, LM3S_GPIOF_BASE, LM3S_GPIOG_BASE, LM3S_GPIOH_BASE,
};

static const char g_portchar[8]   = { 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H' };
#else
static const uint32_t g_gpiobase[8] =
{
  LM3S_GPIOA_BASE, LM3S_GPIOB_BASE, LM3S_GPIOC_BASE, LM3S_GPIOD_BASE,
  LM3S_GPIOE_BASE, LM3S_GPIOF_BASE, LM3S_GPIOG_BASE, 0,
};

static const char g_portchar[8]   = { 'A', 'B', 'C', 'D', 'E', 'F', 'G', '?' };
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lm3s_gpiobaseaddress
 *
 * Description:
 *   Given a GPIO enumeration value, return the base address of the
 *   associated GPIO registers.
 *
 ****************************************************************************/

static inline uint32_t lm3s_gpiobaseaddress(int port)
{
  return g_gpiobase[port & 7];
}

/****************************************************************************
 * Name: lm3s_gpioport
 *
 * Description:
 *   Given a GPIO enumeration value, return the base address of the
 *   associated GPIO registers.
 *
 ****************************************************************************/

static inline uint8_t lm3s_gpioport(int port)
{
  return g_portchar[port & 7];
}

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  lm3s_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the provided base address
 *
 ****************************************************************************/

int lm3s_dumpgpio(uint32_t pinset, const char *msg)
{
  irqstate_t   flags;
  unsigned int port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  uint32_t     base;
  uint32_t     rcgc2;
  bool         enabled;

  /* Get the base address associated with the GPIO port */

  base = lm3s_gpiobaseaddress(port);
  DEBUGASSERT(base != 0);

  /* The following requires exclusive access to the GPIO registers */

  flags   = irqsave();
  rcgc2   = getreg32(LM3S_SYSCON_RCGC2);
  enabled = ((rcgc2 & SYSCON_RCGC2_GPIO(port)) != 0);

  lldbg("GPIO%c pinset: %08x base: %08x -- %s\n",
        lm3s_gpioport(port), pinset, base, msg);
  lldbg("  RCGC2: %08x (%s)\n",
        rcgc2, enabled ? "enabled" : "disabled" );

  /* Don't bother with the rest unless the port is enabled */

  if (enabled)
    {
      lldbg("  AFSEL: %02x DEN: %02x DIR: %02x DATA: %02x\n",
            getreg32(base + LM3S_GPIO_AFSEL_OFFSET), getreg32(base + LM3S_GPIO_DEN_OFFSET),
            getreg32(base + LM3S_GPIO_DIR_OFFSET), getreg32(base + LM3S_GPIO_DATA_OFFSET + 0x3fc));
      lldbg("  IS:    %02x IBE: %02x IEV: %02x IM:  %02x RIS: %08x MIS: %08x\n",
            getreg32(base + LM3S_GPIO_IEV_OFFSET), getreg32(base + LM3S_GPIO_IM_OFFSET),
            getreg32(base + LM3S_GPIO_RIS_OFFSET), getreg32(base + LM3S_GPIO_MIS_OFFSET));
      lldbg("  2MA:   %02x 4MA: %02x 8MA: %02x ODR: %02x PUR %02x PDR: %02x SLR: %02x\n",
            getreg32(base + LM3S_GPIO_DR2R_OFFSET), getreg32(base + LM3S_GPIO_DR4R_OFFSET),
            getreg32(base + LM3S_GPIO_DR8R_OFFSET), getreg32(base + LM3S_GPIO_ODR_OFFSET),
            getreg32(base + LM3S_GPIO_PUR_OFFSET), getreg32(base + LM3S_GPIO_PDR_OFFSET),
            getreg32(base + LM3S_GPIO_SLR_OFFSET));
    }
  irqrestore(flags);
  return OK;
}
