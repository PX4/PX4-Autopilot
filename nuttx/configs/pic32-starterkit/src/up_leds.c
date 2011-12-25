/****************************************************************************
 * configs/pic32-starterkit/src/up_leds.c
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

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

#include "pic32mx-internal.h"
#include "pic32mx-ioport.h"
#include "starterkit_internal.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Definitions
 ****************************************************************************/
/* LED Configuration ********************************************************/
/* The PIC32MX Ethernet Starter kit has 3 user LEDs labeled LED1-3 on the
 * board graphics (but referred to as LED4-6 in the schematic):
 *
 * PIN User's Guide  Board Stencil  Notes
 * --- ------------- -------------- -------------------
 * RD0 "User LED D4" "LED1 (RD0")   High illuminates
 * RD2 "User LED D5" "LED3 (RD2)"   High illuminates
 * RD1 "User LED D6" "LED2 (RD1)"   High illuminates
 *
 * We will use the labels on the board to identify LEDs
 *
 *                           ON                  OFF
 * ------------------------- ---- ---- ---- ---- ---- ----
 *                           LED1 LED2 LED3 LED1 LED2 LED3
 * ------------------------- ---- ---- ---- ---- ---- ----
 * LED_STARTED            0  OFF  OFF  OFF  ---  ---  ---
 * LED_HEAPALLOCATE       1  ON   OFF  N/C  ---  ---  ---
 * LED_IRQSENABLED        2  OFF  ON   N/C  ---  ---  ---
 * LED_STACKCREATED       3  ON   ON   N/C  ---  ---  ---
 * LED_INIRQ              4  N/C  N/C  ON   N/C  N/C  OFF
 * LED_SIGNAL             4  N/C  N/C  ON   N/C  N/C  OFF
 * LED_ASSERTION          4  N/C  N/C  ON   N/C  N/C  OFF
 * LED_PANIC              4  N/C  N/C  ON   N/C  N/C  OFF
 */

#define GPIO_LED_1   (GPIO_OUTPUT|GPIO_VALUE_ZERO|GPIO_PORTD|GPIO_PIN0)
#define GPIO_LED_2   (GPIO_OUTPUT|GPIO_VALUE_ZERO|GPIO_PORTD|GPIO_PIN1)
#define GPIO_LED_3   (GPIO_OUTPUT|GPIO_VALUE_ZERO|GPIO_PORTD|GPIO_PIN2)

/* LED Management Definitions ***********************************************/

#define LED_OFF 0
#define LED_ON  1
#define LED_NC  2

/* Debug ********************************************************************/

#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_LEDS)
#  define leddbg  lldbg
#  ifdef CONFIG_DEBUG_VERBOSE
#    define ledvdbg lldbg
#  else
#    define ledvdbg(x...)
#  endif
#else
#  undef CONFIG_DEBUG_LEDS
#  undef CONFIG_DEBUG_VERBOSE
#  define leddbg(x...)
#  define ledvdbg(x...)
#endif

/****************************************************************************
 * Private types
 ****************************************************************************/

struct led_setting_s
{
  uint8_t led1   : 2;
  uint8_t led2   : 2;
  uint8_t led3   : 2;
  uint8_t unused : 2;
};

 /****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct led_setting_s g_ledonvalues[LED_NVALUES] =
{
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF},
  {LED_ON,  LED_OFF, LED_NC,  LED_OFF},
  {LED_OFF, LED_ON,  LED_NC,  LED_OFF},
  {LED_ON,  LED_ON,  LED_NC,  LED_OFF},
  {LED_NC,  LED_NC,  LED_ON,  LED_OFF},
};

static const struct led_setting_s g_ledoffvalues[LED_NVALUES] =
{
  {LED_NC,  LED_NC,  LED_NC,  LED_OFF},
  {LED_NC,  LED_NC,  LED_NC,  LED_OFF},
  {LED_NC,  LED_NC,  LED_NC,  LED_OFF},
  {LED_NC,  LED_NC,  LED_NC,  LED_OFF}, 
  {LED_NC,  LED_NC,  LED_OFF, LED_OFF},
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_setleds
 ****************************************************************************/

void up_setleds(FAR const struct led_setting_s *setting)
{
  if (setting->led1 != LED_NC)
    {
      pic32mx_gpiowrite(GPIO_LED_1, setting->led1 == LED_ON);
    }

  if (setting->led2 != LED_NC)
    {
      pic32mx_gpiowrite(GPIO_LED_2, setting->led2 == LED_ON);
    }

  if (setting->led3 != LED_NC)
    {
      pic32mx_gpiowrite(GPIO_LED_3, setting->led3 == LED_ON);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mx_ledinit
 ****************************************************************************/

void pic32mx_ledinit(void)
{
  /* Configure output pins */

  pic32mx_configgpio(GPIO_LED_1);
  pic32mx_configgpio(GPIO_LED_2);
  pic32mx_configgpio(GPIO_LED_3);
}

/****************************************************************************
 * Name: up_ledon
 ****************************************************************************/

void up_ledon(int led)
{
  if (led < LED_NVALUES)
    {
      up_setleds(&g_ledonvalues[led]);
    }
}

/****************************************************************************
 * Name: up_ledoff
 ****************************************************************************/

void up_ledoff(int led)
{
  if (led < LED_NVALUES)
    {
      up_setleds(&g_ledoffvalues[led]);
    }
}
#endif /* CONFIG_ARCH_LEDS */
