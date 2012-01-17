/****************************************************************************
 * configs/sure-pic32mx/src/up_leds.c
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
#include "sure-internal.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Definitions
 ****************************************************************************/
/* LED Configuration ********************************************************/
/* The Sure PIC32MX board has five LEDs.  One (D4, lablel "Power") is not
 * controllable by software.  Four are controllable by software:
 *
 * D7  "USB"    Yellow  RD7 Low illuminates
 * D8  "SD"     Yellow  RD6 Low illuminates
 * D9  "Flash"  Yellow  RF0 Low illuminates
 * D10 "Error"  Red     RF1 Low illuminates
 *
 *                           ON                  OFF
 *                           USB SD  FLASH ERROR USB SD  FLASH ERROR
 * LED_STARTED            0  OFF OFF OFF   OFF   --- --- ---   ---
 * LED_HEAPALLOCATE       1  ON  OFF N/C   N/C   --- --- ---   ---
 * LED_IRQSENABLED        2  OFF ON  N/C   N/C   --- --- ---   ---
 * LED_STACKCREATED       3  ON  ON  N/C   N/C   --- --- ---   ---
 * LED_INIRQ              4  N/C N/C ON    N/C   N/C N/C OFF   N/C
 * LED_SIGNAL             4  N/C N/C ON    N/C   N/C N/C OFF   N/C
 * LED_ASSERTION          4  N/C N/C ON    N/C   N/C N/C OFF   N/C
 * LED_PANIC              5  N/C N/C N/C   ON    N/C N/C N/C   OFF
 */

#define GPIO_USB_LED   (GPIO_OUTPUT|GPIO_VALUE_ONE|GPIO_PORTD|GPIO_PIN7)
#define GPIO_SD_LED    (GPIO_OUTPUT|GPIO_VALUE_ONE|GPIO_PORTD|GPIO_PIN6)
#define GPIO_FLASH_LED (GPIO_OUTPUT|GPIO_VALUE_ONE|GPIO_PORTF|GPIO_PIN0)
#define GPIO_ERROR_LED (GPIO_OUTPUT|GPIO_VALUE_ONE|GPIO_PORTF|GPIO_PIN1)

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
  uint8_t usb    : 2;
  uint8_t sd     : 2;
  uint8_t flash  : 2;
  uint8_t error  : 2;
};

 /****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct led_setting_s g_ledonvalues[LED_NVALUES] =
{
  {LED_OFF, LED_OFF, LED_OFF, LED_OFF},
  {LED_ON,  LED_OFF, LED_NC,  LED_NC},
  {LED_OFF, LED_ON,  LED_NC,  LED_NC},
  {LED_ON,  LED_ON,  LED_NC,  LED_NC},
  {LED_NC,  LED_NC,  LED_ON,  LED_NC},
  {LED_NC,  LED_NC,  LED_NC,  LED_ON},
};

static const struct led_setting_s g_ledoffvalues[LED_NVALUES] =
{
  {LED_NC,  LED_NC,  LED_NC,  LED_NC},
  {LED_NC,  LED_NC,  LED_NC,  LED_NC},
  {LED_NC,  LED_NC,  LED_NC,  LED_NC},
  {LED_NC,  LED_NC,  LED_NC,  LED_NC}, 
  {LED_NC,  LED_NC,  LED_OFF, LED_NC},
  {LED_NC,  LED_NC,  LED_NC,  LED_OFF}
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_setleds
 ****************************************************************************/

void up_setleds(FAR const struct led_setting_s *setting)
{
  if (setting->usb != LED_NC)
    {
      pic32mx_gpiowrite(GPIO_USB_LED, setting->usb != LED_ON);
    }

  if (setting->sd != LED_NC)
    {
      pic32mx_gpiowrite(GPIO_SD_LED, setting->sd != LED_ON);
    }

  if (setting->flash != LED_NC)
    {
      pic32mx_gpiowrite(GPIO_FLASH_LED, setting->flash != LED_ON);
    }

  if (setting->error != LED_NC)
    {
      pic32mx_gpiowrite(GPIO_ERROR_LED, setting->error != LED_ON);
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

  pic32mx_configgpio(GPIO_USB_LED);
  pic32mx_configgpio(GPIO_SD_LED);
  pic32mx_configgpio(GPIO_FLASH_LED);
  pic32mx_configgpio(GPIO_ERROR_LED);
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
