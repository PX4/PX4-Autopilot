/****************************************************************************
 * configs/cloudctrl/src/up_autoleds.c
 * arch/arm/src/board/up_autoleds.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Darcy Gong <darcy.gong@gmail.com>
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
#include "cloudctrl-internal.h"

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

/* The following definitions map the encoded LED setting to GPIO settings */

#define CLOUDCTRL_LED1     (1 << 0)
#define CLOUDCTRL_LED2     (1 << 1)
#define CLOUDCTRL_LED3     (1 << 2)
#define CLOUDCTRL_LED4     (1 << 3)

#define ON_SETBITS_SHIFT  (0)
#define ON_CLRBITS_SHIFT  (4)
#define OFF_SETBITS_SHIFT (8)
#define OFF_CLRBITS_SHIFT (12)

#define ON_BITS(v)        ((v) & 0xff)
#define OFF_BITS(v)       (((v) >> 8) & 0x0ff)
#define SETBITS(b)        ((b) & 0x0f)
#define CLRBITS(b)        (((b) >> 4) & 0x0f)

#define ON_SETBITS(v)     (SETBITS(ON_BITS(v))
#define ON_CLRBITS(v)     (CLRBITS(ON_BITS(v))
#define OFF_SETBITS(v)    (SETBITS(OFF_BITS(v))
#define OFF_CLRBITS(v)    (CLRBITS(OFF_BITS(v))

#define LED_STARTED_ON_SETBITS       ((CLOUDCTRL_LED1) << ON_SETBITS_SHIFT)
#define LED_STARTED_ON_CLRBITS       ((CLOUDCTRL_LED2|CLOUDCTRL_LED3|CLOUDCTRL_LED4) << ON_CLRBITS_SHIFT)
#define LED_STARTED_OFF_SETBITS      (0 << OFF_SETBITS_SHIFT)
#define LED_STARTED_OFF_CLRBITS      ((CLOUDCTRL_LED1|CLOUDCTRL_LED2|CLOUDCTRL_LED3|CLOUDCTRL_LED4) << OFF_CLRBITS_SHIFT)

#define LED_HEAPALLOCATE_ON_SETBITS  ((CLOUDCTRL_LED2) << ON_SETBITS_SHIFT)
#define LED_HEAPALLOCATE_ON_CLRBITS  ((CLOUDCTRL_LED1|CLOUDCTRL_LED3|CLOUDCTRL_LED4) << ON_CLRBITS_SHIFT)
#define LED_HEAPALLOCATE_OFF_SETBITS ((CLOUDCTRL_LED1) << OFF_SETBITS_SHIFT)
#define LED_HEAPALLOCATE_OFF_CLRBITS ((CLOUDCTRL_LED2|CLOUDCTRL_LED3|CLOUDCTRL_LED4) << OFF_CLRBITS_SHIFT)

#define LED_IRQSENABLED_ON_SETBITS   ((CLOUDCTRL_LED1|CLOUDCTRL_LED2) << ON_SETBITS_SHIFT)
#define LED_IRQSENABLED_ON_CLRBITS   ((CLOUDCTRL_LED3|CLOUDCTRL_LED4) << ON_CLRBITS_SHIFT)
#define LED_IRQSENABLED_OFF_SETBITS  ((CLOUDCTRL_LED2) << OFF_SETBITS_SHIFT)
#define LED_IRQSENABLED_OFF_CLRBITS  ((CLOUDCTRL_LED1|CLOUDCTRL_LED3|CLOUDCTRL_LED4) << OFF_CLRBITS_SHIFT)

#define LED_STACKCREATED_ON_SETBITS  ((CLOUDCTRL_LED3) << ON_SETBITS_SHIFT)
#define LED_STACKCREATED_ON_CLRBITS  ((CLOUDCTRL_LED1|CLOUDCTRL_LED2|CLOUDCTRL_LED4) << ON_CLRBITS_SHIFT)
#define LED_STACKCREATED_OFF_SETBITS ((CLOUDCTRL_LED1|CLOUDCTRL_LED2) << OFF_SETBITS_SHIFT)
#define LED_STACKCREATED_OFF_CLRBITS ((CLOUDCTRL_LED3|CLOUDCTRL_LED4) << OFF_CLRBITS_SHIFT)

#define LED_INIRQ_ON_SETBITS         ((CLOUDCTRL_LED1) << ON_SETBITS_SHIFT)
#define LED_INIRQ_ON_CLRBITS         ((0) << ON_CLRBITS_SHIFT)
#define LED_INIRQ_OFF_SETBITS        ((0) << OFF_SETBITS_SHIFT)
#define LED_INIRQ_OFF_CLRBITS        ((CLOUDCTRL_LED1) << OFF_CLRBITS_SHIFT)

#define LED_SIGNAL_ON_SETBITS        ((CLOUDCTRL_LED2) << ON_SETBITS_SHIFT)
#define LED_SIGNAL_ON_CLRBITS        ((0) << ON_CLRBITS_SHIFT)
#define LED_SIGNAL_OFF_SETBITS       ((0) << OFF_SETBITS_SHIFT)
#define LED_SIGNAL_OFF_CLRBITS       ((CLOUDCTRL_LED2) << OFF_CLRBITS_SHIFT)

#define LED_ASSERTION_ON_SETBITS     ((CLOUDCTRL_LED3) << ON_SETBITS_SHIFT)
#define LED_ASSERTION_ON_CLRBITS     ((0) << ON_CLRBITS_SHIFT)
#define LED_ASSERTION_OFF_SETBITS    ((0) << OFF_SETBITS_SHIFT)
#define LED_ASSERTION_OFF_CLRBITS    ((CLOUDCTRL_LED3) << OFF_CLRBITS_SHIFT)

#define LED_PANIC_ON_SETBITS         ((CLOUDCTRL_LED3) << ON_SETBITS_SHIFT)
#define LED_PANIC_ON_CLRBITS         ((0) << ON_CLRBITS_SHIFT)
#define LED_PANIC_OFF_SETBITS        ((0) << OFF_SETBITS_SHIFT)
#define LED_PANIC_OFF_CLRBITS        ((CLOUDCTRL_LED3) << OFF_CLRBITS_SHIFT)

/**************************************************************************************
 * Private Function Protototypes
 **************************************************************************************/

/* LED State Controls */

static inline void led_clrbits(unsigned int clrbits);
static inline void led_setbits(unsigned int setbits);
static void led_setonoff(unsigned int bits);

/* LED Power Management */

#ifdef CONFIG_PM
static void led_pm_notify(struct pm_callback_s *cb, enum pm_state_e pmstate);
static int led_pm_prepare(struct pm_callback_s *cb, enum pm_state_e pmstate);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint16_t g_ledbits[8] =
{
  (LED_STARTED_ON_SETBITS       | LED_STARTED_ON_CLRBITS |
   LED_STARTED_OFF_SETBITS      | LED_STARTED_OFF_CLRBITS),

  (LED_HEAPALLOCATE_ON_SETBITS  | LED_HEAPALLOCATE_ON_CLRBITS |
   LED_HEAPALLOCATE_OFF_SETBITS | LED_HEAPALLOCATE_OFF_CLRBITS),

  (LED_IRQSENABLED_ON_SETBITS   | LED_IRQSENABLED_ON_CLRBITS |
   LED_IRQSENABLED_OFF_SETBITS  | LED_IRQSENABLED_OFF_CLRBITS),

  (LED_STACKCREATED_ON_SETBITS  | LED_STACKCREATED_ON_CLRBITS |
   LED_STACKCREATED_OFF_SETBITS | LED_STACKCREATED_OFF_CLRBITS),

  (LED_INIRQ_ON_SETBITS         | LED_INIRQ_ON_CLRBITS |
   LED_INIRQ_OFF_SETBITS        | LED_INIRQ_OFF_CLRBITS),

  (LED_SIGNAL_ON_SETBITS        | LED_SIGNAL_ON_CLRBITS |
   LED_SIGNAL_OFF_SETBITS       | LED_SIGNAL_OFF_CLRBITS),

  (LED_ASSERTION_ON_SETBITS     | LED_ASSERTION_ON_CLRBITS |
   LED_ASSERTION_OFF_SETBITS    | LED_ASSERTION_OFF_CLRBITS),

  (LED_PANIC_ON_SETBITS         | LED_PANIC_ON_CLRBITS |
   LED_PANIC_OFF_SETBITS        | LED_PANIC_OFF_CLRBITS)
};

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
 * Name: led_clrbits
 *
 * Description:
 *   Clear all LEDs to the bit encoded state
 *
 ****************************************************************************/

static inline void led_clrbits(unsigned int clrbits)
{
  /* All LEDs are pulled up and, hence, active low */

  if ((clrbits & CLOUDCTRL_LED1) != 0)
    {
      stm32_gpiowrite(GPIO_LED1, true);
    }

  if ((clrbits & CLOUDCTRL_LED2) != 0)
    {
      stm32_gpiowrite(GPIO_LED2, true);
    }

  if ((clrbits & CLOUDCTRL_LED3) != 0)
    {
      stm32_gpiowrite(GPIO_LED3, true);
    }

  if ((clrbits & CLOUDCTRL_LED4) != 0)
    {
      stm32_gpiowrite(GPIO_LED4, true);
    }

}

/****************************************************************************
 * Name: led_setbits
 *
 * Description:
 *   Set all LEDs to the bit encoded state
 *
 ****************************************************************************/

static inline void led_setbits(unsigned int setbits)
{
  /* All LEDs are pulled up and, hence, active low */

  if ((setbits & CLOUDCTRL_LED1) != 0)
    {
      stm32_gpiowrite(GPIO_LED1, false);
    }

  if ((setbits & CLOUDCTRL_LED2) != 0)
    {
      stm32_gpiowrite(GPIO_LED2, false);
    }

  if ((setbits & CLOUDCTRL_LED3) != 0)
    {
      stm32_gpiowrite(GPIO_LED3, false);
    }

  if ((setbits & CLOUDCTRL_LED4) != 0)
    {
      stm32_gpiowrite(GPIO_LED4, false);
    }

}

/****************************************************************************
 * Name: led_setonoff
 *
 * Description:
 *   Set/clear all LEDs to the bit encoded state
 *
 ****************************************************************************/

static void led_setonoff(unsigned int bits)
{
  led_clrbits(CLRBITS(bits));
  led_setbits(SETBITS(bits));
}

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
 * Name: up_ledinit
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void up_ledinit(void)
{
   /* Configure LED1-4 GPIOs for output */

   stm32_configgpio(GPIO_LED1);
   stm32_configgpio(GPIO_LED2);
   stm32_configgpio(GPIO_LED3);
   stm32_configgpio(GPIO_LED4);
}

/****************************************************************************
 * Name: up_ledon
 ****************************************************************************/

void up_ledon(int led)
{
  led_setonoff(ON_BITS(g_ledbits[led]));
}

/****************************************************************************
 * Name: up_ledoff
 ****************************************************************************/

void up_ledoff(int led)
{
  led_setonoff(OFF_BITS(g_ledbits[led]));
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

#endif /* CONFIG_ARCH_LEDS */
