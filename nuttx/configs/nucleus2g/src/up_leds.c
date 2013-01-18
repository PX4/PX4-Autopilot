/****************************************************************************
 * configs/nucleus2g/src/up_leds.c
 * arch/arm/src/board/up_leds.c
 *
 *   Copyright (C) 2010, 2013 Gregory Nutt. All rights reserved.
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

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

#include "lpc17_gpio.h"

#include "nucleus2g_internal.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Enables debug output from this file (needs CONFIG_DEBUG with
 * CONFIG_DEBUG_VERBOSE too)
 */

#undef LED_DEBUG   /* Define to enable debug */
#undef LED_VERBOSE /* Define to enable verbose debug */

#ifdef LED_DEBUG
#  define leddbg  lldbg
#  ifdef LED_VERBOSE
#    define ledvdbg lldbg
#  else
#    define ledvdbg(x...)
#  endif
#else
#  undef LED_VERBOSE
#  define leddbg(x...)
#  define ledvdbg(x...)
#endif

/* Dump GPIO registers */

#ifdef LED_VERBOSE
#  define led_dumpgpio(m) lpc17_dumpgpio(NUCLEUS2G_LED1_A, m)
#else
#  define led_dumpgpio(m)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The Nucleus2G has 3 LEDs... two on the Babel CAN board and a "heartbeat" LED."
 * The LEDs on the Babel CAN board are capabl of OFF/GREEN/RED/AMBER status.
 * In normal usage, the two LEDs on the Babel CAN board would show CAN status, but if
 * CONFIG_ARCH_LEDS is defined, these LEDs will be controlled as follows for NuttX
 * debug functionality (where NC means "No Change").
 * 
 *                      LED1   LED2   HEARTBEAT
 *                    +------- ------ -----------------------
 *   LED_STARTED      | OFF    OFF    OFF
 *   LED_HEAPALLOCATE | GREEN  OFF    OFF
 *   LED_IRQSENABLED  | OFF    GREEN  OFF
 *   LED_STACKCREATED | OFF    OFF    OFF
 *   LED_INIRQ        | NC     NC     ON  (momentary)
 *   LED_SIGNAL       | NC     NC     ON  (momentary)
 *   LED_ASSERTION    | NC     NC     ON  (momentary)
 *   LED_PANIC        | NC     NC     ON  (1Hz flashing)
 */

static bool g_initialized;
static int  g_nestcount;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_ledinit
 ****************************************************************************/

void up_ledinit(void)
{
  /* Configure all LED GPIO lines */

  led_dumpgpio("up_ledinit() Entry)");

  lpc17_configgpio(NUCLEUS2G_LED1_A);
  lpc17_configgpio(NUCLEUS2G_LED1_B);
  lpc17_configgpio(NUCLEUS2G_LED2_A);
  lpc17_configgpio(NUCLEUS2G_LED2_B);
  lpc17_configgpio(NUCLEUS2G_HEARTBEAT);
  lpc17_configgpio(NUCLEUS2G_EXTRA_LED);

  led_dumpgpio("up_ledinit() Exit");
}

/****************************************************************************
 * Name: up_ledon
 ****************************************************************************/

void up_ledon(int led)
{
  /* We will control LED1 and LED2 not yet completed the boot sequence. */

  if (!g_initialized)
    {
      enum lpc17_ledstate_e led1 = LPC17_LEDSTATE_OFF;
      enum lpc17_ledstate_e led2 = LPC17_LEDSTATE_OFF;
      switch (led)
        {
        case LED_STACKCREATED:
          g_initialized = true;
        case LED_STARTED:
        default:
          break;

        case LED_HEAPALLOCATE:
          led1 = LPC17_LEDSTATE_GREEN;
          break;

        case LED_IRQSENABLED:
          led2 = LPC17_LEDSTATE_GREEN;
        }
      lpc17_led1(led1);
      lpc17_led2(led2);
    }

  /* We will always control the HB LED */

  switch (led)
    {
    default:
      break;

    case LED_INIRQ:
    case LED_SIGNAL:
    case LED_ASSERTION:
    case LED_PANIC:
      lpc17_gpiowrite(NUCLEUS2G_HEARTBEAT, false);
      g_nestcount++;
    }
}

/****************************************************************************
 * Name: up_ledoff
 ****************************************************************************/

void up_ledoff(int led)
{
  /* In all states, OFF can only mean turning off the HB LED */

  if (g_nestcount <= 1)
    {
      lpc17_gpiowrite(NUCLEUS2G_HEARTBEAT, true);
      g_nestcount = 0;
    }
  else
    {
      g_nestcount--;
    }
}

/************************************************************************************
 * Name: lpc17_led1 and 2
 *
 * Description:
 *   Once the system has booted, these functions can be used to control LEDs 1 and 2
 *
 ************************************************************************************/

void lpc17_led1(enum lpc17_ledstate_e state)
{
  bool red   = (((unsigned int)state & LPC17_LEDSTATE_RED) != 0);
  bool green = (((unsigned int)state & LPC17_LEDSTATE_GREEN) != 0);

  lpc17_gpiowrite(NUCLEUS2G_LED1_A, red);
  lpc17_gpiowrite(NUCLEUS2G_LED1_B, green);
}

void lpc17_led2(enum lpc17_ledstate_e state)
{
  bool red   = (((unsigned int)state & LPC17_LEDSTATE_RED) != 0);
  bool green = (((unsigned int)state & LPC17_LEDSTATE_GREEN) != 0);

  lpc17_gpiowrite(NUCLEUS2G_LED2_A, red);
  lpc17_gpiowrite(NUCLEUS2G_LED2_B, green);
}
#endif /* CONFIG_ARCH_LEDS */
