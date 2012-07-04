/****************************************************************************
 * configs/lpc4330-xplorer/src/up_autoleds.c
 * arch/arm/src/board/up_autoleds.c
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

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

#include "xplorer_internal.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Definitions
 ****************************************************************************/
/* LED definitions **********************************************************/
/* The LPC4330-Xplorer has 2 user-controllable LEDs labeled D2 an D3 in the
 * schematic and on but referred to has LED1 and LED2 here, respectively.
 *
 *  LED1   D2  GPIO1[12]
 *  LED2   D3  GPIO1[11]
 *
 * LEDs are pulled high to a low output illuminates the LED.
 *
 * If CONFIG_ARCH_LEDS is defined, the LEDs will be controlled as follows
 * for NuttX debug functionality (where NC means "No Change").
 * 
 *                                      ON            OFF
 *                                  LED1   LED2   LED1   LED2
 *   LED_STARTED                0   OFF    OFF     -      -
 *   LED_HEAPALLOCATE           1   ON     OFF     -      -
 *   LED_IRQSENABLED            1   ON     OFF     -      -
 *   LED_STACKCREATED           1   ON     OFF     -      -
 *   LED_INIRQ                  2   NC     ON      NC     OFF
 *   LED_SIGNAL                 2   NC     ON      NC     OFF
 *   LED_ASSERTION              2   NC     ON      NC     OFF
 *   LED_PANIC                  2   NC     ON      NC     OFF
 *
 * If CONFIG_ARCH_LEDS is not defined, then the LEDs are completely under
 * control of the application.  The following interfaces are then available
 * for application control of the LEDs:
 *
 *  void lpc43_ledinit(void);
 *  void lpc43_setled(int led, bool ledon);
 *  void lpc43_setleds(uint8_t ledset);
 */

/* Debug definitions ********************************************************/
/* Enables debug output from this file (needs CONFIG_DEBUG with
 * CONFIG_DEBUG_VERBOSE too)
 */

#ifdef CONFIG_DEBUG_LED
#  define leddbg  lldbg
#  ifdef CONFIG_DEBUG_VERBOSE
#    define LED_VERBOSE 1
#    define ledvdbg lldbg
#  else
#    undef LED_VERBOSE
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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: led_dumppins
 ****************************************************************************/

#ifdef LED_VERBOSE
static void led_dumppins(FAR const char *msg)
{
  lpc43_dumppinconfig(PINCONFIG_LED1, msg);
  lpc43_dumpgpio(GPIO_LED2, msg);
}
#else
#  define led_dumppins(m)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_ledinit
 ****************************************************************************/

void up_ledinit(void)
{
  /* Configure all LED pins as GPIO outputs */

  led_dumppins("up_ledinit() Entry)");

  /* Configure LED pins as GPIOs, then configure GPIOs as outputs */

  lpc43_pinconfig(PINCONFIG_LED1);
  lpc43_gpioconfig(GPIO_LED1);

  lpc43_pinconfig(PINCONFIG_LED2);
  lpc43_gpioconfig(GPIO_LED2);

  led_dumppins("up_ledinit() Exit");
}

/****************************************************************************
 * Name: up_ledon
 ****************************************************************************/

void up_ledon(int led)
{
  switch (led)
    {
      default:
      case 0:
        lpc43_gpiowrite(GPIO_LED1, true);   /* LED1 OFF */
        lpc43_gpiowrite(GPIO_LED2, true);   /* LED2 OFF */
        break;

      case 1:
        lpc43_gpiowrite(GPIO_LED1, false);  /* LED1 ON */
        lpc43_gpiowrite(GPIO_LED2, true);   /* LED2 OFF */
        break;

      case 2:
        lpc43_gpiowrite(GPIO_LED2, false);  /* LED2 ON */
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
      default:
      case 0:
      case 1:
        break;

      case 2:
        lpc43_gpiowrite(GPIO_LED2, true);  /* LED2 OFF */
        break;
    }
}

#endif /* CONFIG_ARCH_LEDS */
