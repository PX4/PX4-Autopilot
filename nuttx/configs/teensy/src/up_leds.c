/****************************************************************************
 * configs/teensy/src/up_leds.c
 * arch/arm/src/board/up_leds.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
#include <avr/io.h>

#include "up_arch.h"
#include "up_internal.h"

#include "at90usb_internal.h"
#include "teensy_internal.h"

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

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_ncoff;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: at90usb_ledinit
 ****************************************************************************/

void at90usb_ledinit(void)
{
  /* The Teensy's single LED is on Port D, Pin 6.  Configur this pin as an
   * output and turn it OFF.  The "other" side of the LED is onnected to
   * ground through a resistor.  Therefore, a logic value of 0 should turn
   * the LED off.
   */

  DDRD   |= (1 << 6);
  PORTD  &= ~(1 << 6);
  g_ncoff = true;
}

/****************************************************************************
 * Name: up_ledon
 ****************************************************************************/

void up_ledon(int led)
{
 /*                         ON      OFF
  * LED_STARTED        0    OFF     ON  (never happens)
  * LED_HEAPALLOCATE   0    OFF     ON  (never happens)
  * LED_IRQSENABLED    0    OFF     ON  (never happens)
  * LED_STACKCREATED   1    ON      ON  (never happens)
  * LED_INIRQ          2    OFF     NC  (momentary)
  * LED_SIGNAL         2    OFF     NC  (momentary)
  * LED_ASSERTION      2    OFF     NC  (momentary)
  * LED_PANIC          0    OFF     ON  (1Hz flashing)
  */

  switch (led)
    {
    case 0:
      /* The steady state is OFF */

      g_ncoff = true;

    case 2:
      /* Turn the LED off */

      PORTD &= ~(1 << 6);
      break;

    case 1:
      /* The steady state is ON */

      PORTD |= (1 << 6);
      g_ncoff = false;
      break;

    default:
      return;
    }
}

/****************************************************************************
 * Name: up_ledoff
 ****************************************************************************/

void up_ledoff(int led)
{
 /*                         ON      OFF
  * LED_STARTED        0    OFF     ON  (never happens)
  * LED_HEAPALLOCATE   0    OFF     ON  (never happens)
  * LED_IRQSENABLED    0    OFF     ON  (never happens)
  * LED_STACKCREATED   1    ON      ON  (never happens)
  * LED_INIRQ          2    OFF     NC  (momentary)
  * LED_SIGNAL         2    OFF     NC  (momentary)
  * LED_ASSERTION      2    OFF     NC  (momentary)
  * LED_PANIC          0    OFF     ON  (1Hz flashing)
  */

  switch (led)
    {
    case 2:
      /* If the "no-change" state is OFF, then turn the LED off */

      if (g_ncoff)
        {
          PORTD &= ~(1 << 6);
          break;
        }

      /* Otherwise, fall through to turn the LED ON */

    case 0:
    case 1:
      /* Turn the LED on */

      PORTD |= (1 << 6);
      break;

    default:
      return;
    }
}

#endif /* CONFIG_ARCH_LEDS */
