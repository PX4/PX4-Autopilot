/****************************************************************************
 * configs/mirtoo/src/up_leds.c
 * arch/arm/src/board/up_leds.c
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

#include "pic32mx-internal.h"
#include "mirtoo-internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/
/* LED Configuration ********************************************************/
/* The Mirtoo module has 2 user LEDs labeled LED0 and LED1 in the schematics:
 *
 * ---  ----- --------------------------------------------------------------
 * PIN  Board Notes
 * ---  ----- --------------------------------------------------------------
 * RC8  LED0  Grounded, high value illuminates
 * RC9  LED1  Grounded, high value illuminates
 *
 * The Dimitech DTX1-4000L EV-kit1 supports 3 more LEDs, but there are not
 * controllable from software.
 *
 * If CONFIG_ARCH_LEDS is defined, then NuttX will control these LEDs as
 * follows:
 *                              ON        OFF
 * ------------------------- ---- ---- ---- ----
 *                           LED0 LED1 LED0 LED1
 * ------------------------- ---- ---- ---- ----
 * LED_STARTED            0  OFF  OFF  ---  ---
 * LED_HEAPALLOCATE       1  ON   OFF  ---  ---
 * LED_IRQSENABLED        2  OFF  ON   ---  ---
 * LED_STACKCREATED       3  ON   ON   ---  ---
 * LED_INIRQ              4  ON   N/C  OFF  N/C
 * LED_SIGNAL             4  ON   N/C  OFF  N/C
 * LED_ASSERTION          4  ON   N/C  OFF  N/C
 * LED_PANIC              4  ON   N/C  OFF  N/C
 */

#define GPIO_LED_0   (GPIO_OUTPUT|GPIO_VALUE_ZERO|GPIO_PORTC|GPIO_PIN8)
#define GPIO_LED_1   (GPIO_OUTPUT|GPIO_VALUE_ZERO|GPIO_PORTC|GPIO_PIN9)

/* LED Management Definitions ***********************************************/

#ifdef CONFIG_ARCH_LEDS
#  define LED_OFF 0
#  define LED_ON  1
#  define LED_NC  2
#endif

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

#ifdef CONFIG_ARCH_LEDS
struct led_setting_s
{
  uint8_t led0   : 2;
  uint8_t led1   : 2;
  uint8_t unused : 4;
};
#endif

 /****************************************************************************
 * Private Data
 ****************************************************************************/
/* If CONFIG_ARCH_LEDS is defined then NuttX will control the LEDs.  The
 * following structures identified the LED settings for each NuttX LED state.
 */

#ifdef CONFIG_ARCH_LEDS
static const struct led_setting_s g_ledonvalues[LED_NVALUES] =
{
  {LED_OFF, LED_OFF, 0},
  {LED_ON,  LED_OFF, 0},
  {LED_OFF, LED_ON,  0},
  {LED_ON,  LED_ON,  0},
  {LED_ON,  LED_NC,  0},
};

static const struct led_setting_s g_ledoffvalues[LED_NVALUES] =
{
  {LED_NC,  LED_NC,  0},
  {LED_NC,  LED_NC,  0},
  {LED_NC,  LED_NC,  0},
  {LED_NC,  LED_NC,  0}, 
  {LED_OFF, LED_NC,  0},
};

/* If CONFIG_ARCH_LEDS is not defined, the the user can control the LEDs in
 * any way.  The following array simply maps the PIC32MX_MIRTOO_LEDn
 * index values to the correct LED pin configuration.
 */

#else
static const uint16_t g_ledpincfg[PIC32MX_MIRTOO_NLEDS] =
{
  GPIO_LED_0, GPIO_LED_1
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_setleds
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void up_setleds(FAR const struct led_setting_s *setting)
{
  /* LEDs are pulled up so writing a low value (false) illuminates them */

  if (setting->led0 != LED_NC)
    {
      pic32mx_gpiowrite(GPIO_LED_0, setting->led0 == LED_ON);
    }

  if (setting->led1 != LED_NC)
    {
      pic32mx_gpiowrite(GPIO_LED_1, setting->led1 == LED_ON);
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mx_ledinit
 ****************************************************************************/

void pic32mx_ledinit(void)
{
  /* Configure output pins */

  pic32mx_configgpio(GPIO_LED_0);
  pic32mx_configgpio(GPIO_LED_1);
}

/****************************************************************************
 * Name: pic32mx_setled
 ****************************************************************************/

#ifndef CONFIG_ARCH_LEDS
void pic32mx_setled(int led, bool ledon)
{
  /* LEDs are pulled up so writing a low value (false) illuminates them */

  if ((unsigned)led < PIC32MX_MIRTOO_NLEDS)
    {
      pic32mx_gpiowrite(g_ledpincfg[led], ledon);
    }
}
#endif

/****************************************************************************
 * Name: pic32mx_setleds
 ****************************************************************************/

#ifndef CONFIG_ARCH_LEDS
void pic32mx_setleds(uint8_t ledset)
{
  /* Call pic32mx_setled() with ledon == true to illuminated the LED */

  pic32mx_setled(PIC32MX_MIRTOO_LED0, (ledset & PIC32MX_MIRTOO_LED0_BIT) != 0);
  pic32mx_setled(PIC32MX_MIRTOO_LED1, (ledset & PIC32MX_MIRTOO_LED1_BIT) != 0);
}
#endif

/****************************************************************************
 * Name: up_ledon
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void up_ledon(int led)
{
  if ((unsigned)led < LED_NVALUES)
    {
      up_setleds(&g_ledonvalues[led]);
    }
}
#endif

/****************************************************************************
 * Name: up_ledoff
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void up_ledoff(int led)
{
  if ((unsigned)led < LED_NVALUES)
    {
      up_setleds(&g_ledoffvalues[led]);
    }
}
#endif
