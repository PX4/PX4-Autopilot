/****************************************************************************
 * configs/stm3210e-eval/src/up_pmbuttons.c
 * arch/arm/src/board/up_pmbuttons.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Diego Sanchez <dsanchez@nx-engineering.com>
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

#include <arch/board/board.h>
#include <nuttx/config.h>

#include <nuttx/power/pm.h>
#include <arch/irq.h>
#include <stdbool.h>
#include <debug.h>

#include "up_arch.h"
#include "nvic.h"
#include "stm32_pwr.h"
#include "stm32_pm.h"
#include "stm3210e-internal.h"

#if defined(CONFIG_PM) && defined(CONFIG_IDLE_CUSTOM) && defined(CONFIG_PM_BUTTONS)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_ARCH_BUTTONS
#  error "CONFIG_ARCH_BUTTONS is not defined in the configuration"
#endif

#define BUTTON_MIN 0
#define BUTTON_MAX 7

#ifndef CONFIG_PM_BUTTONS_MIN
#  define CONFIG_PM_BUTTONS_MIN BUTTON_MIN
#endif
#ifndef CONFIG_PM_BUTTONS_MAX
#  define CONFIG_PM_BUTTONS_MAX BUTTON_MAX
#endif

#if CONFIG_PM_BUTTONS_MIN > CONFIG_PM_BUTTONS_MAX
#  error "CONFIG_PM_BUTTONS_MIN > CONFIG_PM_BUTTONS_MAX"
#endif
#if CONFIG_PM_BUTTONS_MAX > 7
#  error "CONFIG_PM_BUTTONS_MAX > 7"
#endif

#ifndef CONFIG_ARCH_IRQBUTTONS
#  warning "CONFIG_ARCH_IRQBUTTONS is not defined in the configuration"
#endif

#ifndef CONFIG_PM_IRQBUTTONS_MIN
#  define CONFIG_PM_IRQBUTTONS_MIN CONFIG_PM_BUTTONS_MIN
#endif
#ifndef CONFIG_PM_IRQBUTTONS_MAX
#  define CONFIG_PM_IRQBUTTONS_MAX CONFIG_PM_BUTTONS_MAX
#endif

#if CONFIG_PM_IRQBUTTONS_MIN > CONFIG_PM_IRQBUTTONS_MAX
#  error "CONFIG_PM_IRQBUTTONS_MIN > CONFIG_PM_IRQBUTTONS_MAX"
#endif
#if CONFIG_PM_IRQBUTTONS_MAX > 7
#  error "CONFIG_PM_IRQBUTTONS_MAX > 7"
#endif

#ifndef CONFIG_PM_BUTTON_ACTIVITY
#  define CONFIG_PM_BUTTON_ACTIVITY 10
#endif

/* Miscellaneous Definitions ************************************************/

#ifndef MIN
#  define MIN(a,b) (a < b ? a : b)
#endif
#ifndef MAX
#  define MAX(a,b) (a > b ? a : b)
#endif

#define MIN_BUTTON MIN(CONFIG_PM_BUTTONS_MIN, CONFIG_PM_IRQBUTTONS_MIN)
#define MAX_BUTTON MAX(CONFIG_PM_BUTTONS_MAX, CONFIG_PM_IRQBUTTONS_MAX)

#define NUM_PMBUTTONS   (MAX_BUTTON - MIN_BUTTON + 1)
#define BUTTON_INDEX(b) ((b)-MIN_BUTTON)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
static void button_handler(int id, int irq);

#if MIN_BUTTON < 1
static int button0_handler(int irq, FAR void *context);
#endif
#if MIN_BUTTON < 2 && MAX_BUTTON > 0
static int button1_handler(int irq, FAR void *context);
#endif
#if MIN_BUTTON < 3 && MAX_BUTTON > 1
static int button2_handler(int irq, FAR void *context);
#endif
#if MIN_BUTTON < 4 && MAX_BUTTON > 2
static int button3_handler(int irq, FAR void *context);
#endif
#if MIN_BUTTON < 5 && MAX_BUTTON > 3
static int button4_handler(int irq, FAR void *context);
#endif
#if MIN_BUTTON < 6 && MAX_BUTTON > 4
static int button5_handler(int irq, FAR void *context);
#endif
#if MIN_BUTTON < 7 && MAX_BUTTON > 5
static int button6_handler(int irq, FAR void *context);
#endif
#if MAX_BUTTON > 6
static int button7_handler(int irq, FAR void *context);
#endif
#endif /* CONFIG_ARCH_IRQBUTTONS */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Button interrupt handlers */

#ifdef CONFIG_ARCH_IRQBUTTONS
static const xcpt_t g_buttonhandlers[NUM_PMBUTTONS] =
{
#if MIN_BUTTON < 1
    button0_handler,
#endif
#if MIN_BUTTON < 2 && MAX_BUTTON > 0
    button1_handler,
#endif
#if MIN_BUTTON < 3 && MAX_BUTTON > 1
    button2_handler,
#endif
#if MIN_BUTTON < 4 && MAX_BUTTON > 2
    button3_handler,
#endif
#if MIN_BUTTON < 5 && MAX_BUTTON > 3
    button4_handler,
#endif
#if MIN_BUTTON < 6 && MAX_BUTTON > 4
    button5_handler,
#endif
#if MIN_BUTTON < 7 && MAX_BUTTON > 5
    button6_handler,
#endif
#if MAX_BUTTON > 6
    button7_handler,
#endif
};
#endif /* CONFIG_ARCH_IRQBUTTONS */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: button_handler
 *
 * Description:
 *   Handle a button wake-up interrupt
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
static void button_handler(int id, int irq)
{
  /* At this point the MCU should have already awakened.  The state
   * change will be handled in the IDLE loop when the system is re-awakened
   * The button interrupt handler should be totally ignorant of the PM
   * activities and should report button activity as if nothing
   * special happened.
   */

  pm_activity(CONFIG_PM_BUTTON_ACTIVITY);
}

#if MIN_BUTTON < 1
static int button0_handler(int irq, FAR void *context)
{
  button_handler(0, irq);
  return OK;
}
#endif

#if MIN_BUTTON < 2 && MAX_BUTTON > 0
static int button1_handler(int irq, FAR void *context)
{
  button_handler(1, irq);
  return OK;
}
#endif

#if MIN_BUTTON < 3 && MAX_BUTTON > 1
static int button2_handler(int irq, FAR void *context)
{
  button_handler(2, irq);
  return OK;
}
#endif

#if MIN_BUTTON < 4 && MAX_BUTTON > 2
static int button3_handler(int irq, FAR void *context)
{
  button_handler(3, irq);
  return OK;
}
#endif

#if MIN_BUTTON < 5 && MAX_BUTTON > 3
static int button4_handler(int irq, FAR void *context)
{
  button_handler(4, irq);
  return OK;
}
#endif

#if MIN_BUTTON < 6 && MAX_BUTTON > 4
static int button5_handler(int irq, FAR void *context)
{
  button_handler(5, irq);
  return OK;
}
#endif

#if MIN_BUTTON < 7 && MAX_BUTTON > 5
static int button6_handler(int irq, FAR void *context)
{
  button_handler(6, irq);
  return OK;
}
#endif

#if MAX_BUTTON > 6
static int button7_handler(int irq, FAR void *context)
{
  button_handler(7, irq);
  return OK;
}
#endif
#endif /* CONFIG_ARCH_IRQBUTTONS */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_pmbuttons
 *
 * Description:
 *   Configure all the buttons of the STM3210e-eval board as EXTI,
 *   so any button is able to wakeup the MCU from the PM_STANDBY mode
 *
 ****************************************************************************/

void up_pmbuttons(void)
{
  /* Initialize the button GPIOs */

  up_buttoninit();

#ifdef CONFIG_ARCH_IRQBUTTONS
  int i;
  for (i = CONFIG_PM_IRQBUTTONS_MIN; i <= CONFIG_PM_IRQBUTTONS_MAX; i++)
    {
      xcpt_t oldhandler = up_irqbutton(i, g_buttonhandlers[BUTTON_INDEX(i)]);

      if (oldhandler != NULL)
        {
          lowsyslog("WARNING: oldhandler:%p is not NULL!  "
                    "Button events may be lost or aliased!\n",
                    oldhandler);
        }
    }
#endif
}

#endif /* defined(CONFIG_PM) && defined(CONFIG_IDLE_CUSTOM) && defined(CONFIG_PM_BUTTONS) */
