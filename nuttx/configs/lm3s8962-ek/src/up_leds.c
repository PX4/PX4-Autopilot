/****************************************************************************
 * configs/lm3s8962-ek/src/up_leds.c
 * arch/arm/src/board/up_leds.c
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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

#include <stdbool.h>
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "lm_gpio.h"
#include "lm3s8962ek_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Enables debug output from this file (needs CONFIG_DEBUG with
 * CONFIG_DEBUG_VERBOSE too)
 */

#undef LED_DEBUG  /* Define to enable debug */

#ifdef LED_DEBUG
#  define leddbg  lldbg
#  define ledvdbg llvdbg
#else
#  define leddbg(x...)
#  define ledvdbg(x...)
#endif

/* Dump GPIO registers */

#ifdef LED_DEBUG
#  define led_dumpgpio(m) lm_dumpgpio(LED_GPIO, m)
#else
#  define led_dumpgpio(m)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

static uint8_t g_nest;

/****************************************************************************
 * Name: up_ledinit
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void up_ledinit(void)
{
  leddbg("Initializing\n");

  /* Configure Port E, Bit 1 as an output, initial value=OFF */

  led_dumpgpio("up_ledinit before lm_configgpio()");
  lm_configgpio(LED_GPIO);
  led_dumpgpio("up_ledinit after lm_configgpio()");
  g_nest = 0;
}

/****************************************************************************
 * Name: up_ledon
 ****************************************************************************/

void up_ledon(int led)
{
  switch (led)
    {
      case LED_STARTED:
      case LED_HEAPALLOCATE:
      default:
        break;

      case LED_INIRQ:
      case LED_SIGNAL:
      case LED_ASSERTION:
      case LED_PANIC:
        g_nest++;
      case LED_IRQSENABLED:
      case LED_STACKCREATED:
        led_dumpgpio("up_ledon: before lm_gpiowrite()");
        lm_gpiowrite(LED_GPIO, false);
        led_dumpgpio("up_ledon: after lm_gpiowrite()");
        break;
    }
}

/****************************************************************************
 * Name: up_ledoff
 ****************************************************************************/

void up_ledoff(int led)
{
  switch (led)
    {
      case LED_IRQSENABLED:
      case LED_STACKCREATED:
      case LED_STARTED:
      case LED_HEAPALLOCATE:
      default:
        break;

      case LED_INIRQ:
      case LED_SIGNAL:
      case LED_ASSERTION:
      case LED_PANIC:
        if (--g_nest <= 0)
          {
            led_dumpgpio("up_ledoff: before lm_gpiowrite()");
            lm_gpiowrite(LED_GPIO, true);
            led_dumpgpio("up_ledoff: after lm_gpiowrite()");
          }
        break;
    }
}

#endif /* CONFIG_ARCH_LEDS */
