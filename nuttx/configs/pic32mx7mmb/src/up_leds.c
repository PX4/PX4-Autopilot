/****************************************************************************
 * configs/pic32mx7mmb/src/up_leds.c
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
#include "pic32mx7mmb_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/
/* LED Configuration ********************************************************/
/* The Mikroelektronika PIC32MX7 MMB has 3 user LEDs labeled LED0-2 in the
 * schematics:
 *
 * ---  ----- --------------------------------------------------------------
 * PIN  Board Notes
 * ---  ----- --------------------------------------------------------------
 * RA0  LED0  Pulled-up, low value illuminates
 * RA1  LED1  Pulled-up, low value illuminates
 * RD9  LED2  Pulled-up, low value illuminates
 * RA9  LED4  Not available for general use*, indicates MMC/SD activity
 * ---  LED5  Not controllable by software, indicates power-on
 *
 * * RA9 is also the SD chip select.  It will illuminate whenever the SD card
 *   is selected.  If SD is not used, then LED4 could also be used as a user-
 *   controlled LED.
 *
 * If CONFIG_ARCH_LEDS is defined, then NuttX will control these LEDs as
 * follows:
 *                           ON                  OFF
 * ------------------------- ---- ---- ---- ---- ---- ----
 *                           LED0 LED1 LED2 LED0 LED1 LED2
 * ------------------------- ---- ---- ---- ---- ---- ----
 * LED_STARTED            0  OFF  OFF  OFF  ---  ---  ---
 * LED_HEAPALLOCATE       1  ON   OFF  N/C  ---  ---  ---
 * LED_IRQSENABLED        2  OFF  ON   N/C  ---  ---  ---
 * LED_STACKCREATED       3  ON   ON   N/C  ---  ---  ---
 * LED_INIRQ              4  N/C  N/C  ON   N/C  N/C  OFF
 * LED_SIGNAL             4  N/C  N/C  ON   N/C  N/C  OFF
 * LED_ASSERTION          4  N/C  N/C  ON   N/C  N/C  OFF
 * LED_PANIC              5  ON   N/C  N/C  OFF  N/C  N/C
 */

#define GPIO_LED_0   (GPIO_OUTPUT|GPIO_VALUE_ONE|GPIO_PORTA|GPIO_PIN0)
#define GPIO_LED_1   (GPIO_OUTPUT|GPIO_VALUE_ONE|GPIO_PORTA|GPIO_PIN1)
#define GPIO_LED_2   (GPIO_OUTPUT|GPIO_VALUE_ONE|GPIO_PORTD|GPIO_PIN9)
#define GPIO_LED_4   (GPIO_OUTPUT|GPIO_VALUE_ONE|GPIO_PORTA|GPIO_PIN9)

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
  uint8_t led2   : 2;
  uint8_t unused : 2;
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
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF},
  {LED_ON,  LED_OFF, LED_NC,  LED_OFF},
  {LED_OFF, LED_ON,  LED_NC,  LED_OFF},
  {LED_ON,  LED_ON,  LED_NC,  LED_OFF},
  {LED_NC,  LED_NC,  LED_ON,  LED_OFF},
  {LED_ON,  LED_NC,  LED_NC,  LED_OFF},
};

static const struct led_setting_s g_ledoffvalues[LED_NVALUES] =
{
  {LED_NC,  LED_NC,  LED_NC,  LED_OFF},
  {LED_NC,  LED_NC,  LED_NC,  LED_OFF},
  {LED_NC,  LED_NC,  LED_NC,  LED_OFF},
  {LED_NC,  LED_NC,  LED_NC,  LED_OFF}, 
  {LED_NC,  LED_NC,  LED_OFF, LED_OFF},
  {LED_OFF, LED_NC,  LED_NC,  LED_OFF},
};

/* If CONFIG_ARCH_LEDS is not defined, the the user can control the LEDs in
 * any way.  The following array simply maps the PIC32MX_PIC32MX7MMB_LEDn
 * index values to the correct LED pin configuration.
 */

#else
static const uint16_t g_ledpincfg[PIC32MX_PIC32MX7MMB_NLEDS] =
{
  GPIO_LED_0, GPIO_LED_1, GPIO_LED_2
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
      pic32mx_gpiowrite(GPIO_LED_0, setting->led0 != LED_ON);
    }

  if (setting->led1 != LED_NC)
    {
      pic32mx_gpiowrite(GPIO_LED_1, setting->led1 != LED_ON);
    }

  if (setting->led2 != LED_NC)
    {
      pic32mx_gpiowrite(GPIO_LED_2, setting->led2 != LED_ON);
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
  pic32mx_configgpio(GPIO_LED_2);
}

/****************************************************************************
 * Name: pic32mx_setled
 ****************************************************************************/

#ifndef CONFIG_ARCH_LEDS
void pic32mx_setled(int led, bool ledon)
{
  /* LEDs are pulled up so writing a low value (false) illuminates them */

  if ((unsigned)led < PIC32MX_PIC32MX7MMB_NLEDS)
    {
      pic32mx_gpiowrite(g_ledpincfg[led], !ledon);
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

  pic32mx_setled(PIC32MX_PIC32MX7MMB_LED0, (ledset & PIC32MX_PIC32MX7MMB_LED0_BIT) != 0);
  pic32mx_setled(PIC32MX_PIC32MX7MMB_LED1, (ledset & PIC32MX_PIC32MX7MMB_LED1_BIT) != 0);
  pic32mx_setled(PIC32MX_PIC32MX7MMB_LED2, (ledset & PIC32MX_PIC32MX7MMB_LED2_BIT) != 0);
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
