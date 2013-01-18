/****************************************************************************
 * configs/lincoln60/src/up_leds.c
 * arch/arm/src/board/up_leds.c
 *
 *   Copyright (C) 2012-2013 Gregory Nutt. All rights reserved.
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

#include "lincoln60_internal.h"

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
#  define led_dumpgpio(m) lpc17_dumpgpio(LINCOLN60_LED2, m)
#else
#  define led_dumpgpio(m)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* LED definitions ******************************************************************

The Lincoln 60 has 2 LEDs along the bottom of the board. Green or off.
If CONFIG_ARCH_LEDS is defined, the LEDs will be controlled as follows for NuttX
debug functionality (where NC means "No Change").

During the boot phases.  LED1 and LED2 will show boot status.

               LED1   LED2   
STARTED         OFF    OFF   
HEAPALLOCATE   BLUE    OFF    
IRQSENABLED     OFF   BLUE
STACKCREATED    OFF    OFF 

After the system is booted, this logic will no longer use LEDs 1 & 2.  They
are available for use by applications using lpc17_led (prototyped below)
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

  lpc17_configgpio(LINCOLN60_LED1);
  lpc17_configgpio(LINCOLN60_LED2);

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
      int led1 = 0;
      int led2 = 0;
      switch (led)
        {
        case LED_STACKCREATED:
          g_initialized = true;
        case LED_STARTED:
        default:
          break;

        case LED_HEAPALLOCATE:
          led1 = 1;
          break;

        case LED_IRQSENABLED:
          led2 = 1;
        }
      lpc17_led(LINCOLN60_LED1,led1);
      lpc17_led(LINCOLN60_LED2,led2);
    }

  /* We will always control the HB LED */

  switch (led)
    {
    case LED_INIRQ:
    case LED_SIGNAL:
    case LED_ASSERTION:
    case LED_PANIC:
      lpc17_gpiowrite(LINCOLN60_HEARTBEAT, false);
      g_nestcount++;

    default:
      break;
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
      lpc17_led(LINCOLN60_HEARTBEAT, true);
      g_nestcount = 0;
    }
  else
    {
      g_nestcount--;
    }
}

/************************************************************************************
 * Name: lpc17_led
 *
 * Description:
 *   Once the system has booted, these functions can be used to control the LEDs
 *
 ************************************************************************************/

void lpc17_led(int lednum, int state)

{
  lpc17_gpiowrite(lednum, state);
}
#endif /* CONFIG_ARCH_LEDS */
