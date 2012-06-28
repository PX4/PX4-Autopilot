/****************************************************************************
 * configs/stm3210e-eval/src/up_idle.c
 * arch/arm/src/board/up_idle.c
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
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/board/board.h>
#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <nuttx/power/pm.h>

#include <debug.h>
#include <nuttx/rtc.h>
#include <arch/irq.h>

#include "stm32_pm.h"
#include "stm32_rcc.h"
#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Does the board support an IDLE LED to indicate that the board is in the
 * IDLE state?
 */

#if defined(CONFIG_ARCH_LEDS) && defined(LED_IDLE)
#  define BEGIN_IDLE() up_ledon(LED_IDLE)
#  define END_IDLE()   up_ledoff(LED_IDLE)
#else
#  define BEGIN_IDLE()
#  define END_IDLE()
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_idlepm
 *
 * Description:
 *   Perform IDLE state power management.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static void up_idlepm(void)
{
  static enum pm_state_e oldstate = PM_NORMAL;
  enum pm_state_e newstate;
  irqstate_t flags;
  int ret;
  
  /* Decide, which power saving level can be obtained */

  newstate = pm_checkstate();

  /* Check for state changes */

  if (newstate != oldstate)
    {
      lldbg("newstate= %d oldstate=%d\n", newstate, oldstate);

      flags = irqsave();

      /* Force the global state change */

      ret = pm_changestate(newstate);
      if (ret < 0)
        {
          /* The new state change failed, revert to the preceding state */

          (void)pm_changestate(oldstate);
        }
      else
        {
          /* Save the new state */

          oldstate = newstate;
        }

      /* Then perform board-specific, state-dependent logic here */

      switch (newstate)
        {
        case PM_NORMAL:
          break;

        case PM_IDLE:
        break;

        case PM_STANDBY:
          /* Configure all the buttons as wakeup EXTI */

          up_pmbuttons();

          /* Call the MCU stop mode */

          stm32_pmstop(true);
          break;

        case PM_SLEEP:
          (void)stm32_pmstandby();
          break;

        default:
          break;
        }

      irqrestore(flags);
    }
}
#else
#  define up_idlepm()
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_idle
 *
 * Description:
 *   up_idle() is the logic that will be executed when their is no other
 *   ready-to-run task.  This is processor idle time and will continue until
 *   some interrupt occurs to cause a context switch from the idle task.
 *
 *   Processing in this state may be processor-specific. e.g., this is where
 *   power management operations might be performed.
 *
 ****************************************************************************/

void up_idle(void)
{
#if defined(CONFIG_SUPPRESS_INTERRUPTS) || defined(CONFIG_SUPPRESS_TIMER_INTS)
  /* If the system is idle and there are no timer interrupts, then process
   * "fake" timer interrupts. Hopefully, something will wake up.
   */

  sched_process_timer();
#else

  /* Perform IDLE mode power management */

  up_idlepm();

  /* Sleep until an interrupt occurs to save power */

  BEGIN_IDLE();
  asm("WFI");
  END_IDLE();
#endif
}

