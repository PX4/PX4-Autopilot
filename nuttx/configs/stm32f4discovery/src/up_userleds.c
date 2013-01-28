/****************************************************************************
 * configs/stm32f4discovery/src/up_leds.c
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
#include <nuttx/power/pm.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "stm32_internal.h"
#include "stm32f4discovery-internal.h"

#ifndef CONFIG_ARCH_LEDS

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

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* This array maps an LED number to GPIO pin configuration */

static uint32_t g_ledcfg[BOARD_NLEDS] = 
{
  GPIO_LED1, GPIO_LED2, GPIO_LED3, GPIO_LED4
};

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

/* LED Power Management */

#ifdef CONFIG_PM
static void led_pm_notify(struct pm_callback_s *cb, enum pm_state_e pmstate);
static int led_pm_prepare(struct pm_callback_s *cb, enum pm_state_e pmstate);
#endif


/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_PM
static struct pm_callback_s g_ledscb =
{
  .notify  = led_pm_notify,
  .prepare = led_pm_prepare,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: led_pm_notify
 *
 * Description:
 *   Notify the driver of new power state. This callback is called after
 *   all drivers have had the opportunity to prepare for the new power state.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static void led_pm_notify(struct pm_callback_s *cb , enum pm_state_e pmstate)
{
  switch (pmstate)
    {
      case(PM_NORMAL):
        {
          /* Restore normal LEDs operation */

        }
        break;

      case(PM_IDLE):
        {
          /* Entering IDLE mode - Turn leds off */

        }
        break;

      case(PM_STANDBY):
        {
          /* Entering STANDBY mode - Logic for PM_STANDBY goes here */

        }
        break;

      case(PM_SLEEP):
        {
          /* Entering SLEEP mode - Logic for PM_SLEEP goes here */

        }
        break;

      default:
        {
          /* Should not get here */

        }
        break;
    }
}
#endif

/****************************************************************************
 * Name: led_pm_prepare
 *
 * Description:
 *   Request the driver to prepare for a new power state. This is a warning
 *   that the system is about to enter into a new power state. The driver
 *   should begin whatever operations that may be required to enter power
 *   state. The driver may abort the state change mode by returning a
 *   non-zero value from the callback function.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static int led_pm_prepare(struct pm_callback_s *cb , enum pm_state_e pmstate)
{
  /* No preparation to change power modes is required by the LEDs driver.
   * We always accept the state change by returning OK.
   */

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_ledinit
 ****************************************************************************/

void stm32_ledinit(void)
{
   /* Configure LED1-4 GPIOs for output */

   stm32_configgpio(GPIO_LED1);
   stm32_configgpio(GPIO_LED2);
   stm32_configgpio(GPIO_LED3);
   stm32_configgpio(GPIO_LED4);
}

/****************************************************************************
 * Name: stm32_setled
 ****************************************************************************/

void stm32_setled(int led, bool ledon)
{
  if ((unsigned)led < BOARD_NLEDS)
    {
      stm32_gpiowrite(g_ledcfg[led], ledon);
    }
}

/****************************************************************************
 * Name: stm32_setleds
 ****************************************************************************/

void stm32_setleds(uint8_t ledset)
{
  stm32_gpiowrite(GPIO_LED1, (ledset & BOARD_LED1_BIT) == 0);
  stm32_gpiowrite(GPIO_LED2, (ledset & BOARD_LED2_BIT) == 0);
  stm32_gpiowrite(GPIO_LED3, (ledset & BOARD_LED3_BIT) == 0);
  stm32_gpiowrite(GPIO_LED4, (ledset & BOARD_LED4_BIT) == 0);
}

/****************************************************************************
 * Name: up_ledpminitialize
 ****************************************************************************/

#ifdef CONFIG_PM
void up_ledpminitialize(void)
{
  /* Register to receive power management callbacks */

  int ret = pm_register(&g_ledscb);
  if (ret != OK)
  {
      up_ledon(LED_ASSERTION);
    }
}
#endif /* CONFIG_PM */

#endif /* !CONFIG_ARCH_LEDS */
