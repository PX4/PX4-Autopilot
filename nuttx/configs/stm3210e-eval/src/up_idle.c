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
#include <nuttx/clock.h>
#include <nuttx/power/pm.h>

#include <debug.h>
#include <nuttx/rtc.h>
#include <arch/irq.h>

#include "up_internal.h"
#include "stm32_pm.h"
#include "stm32_rcc.h"
#include "stm32_exti.h"

#include "stm3210e-internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
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

/* Values for the RTC Alarm to wake up from the PM_STANDBY mode
 * (which corresponds to STM32 stop mode).  If this alarm expires,
 * the logic in this file will wakeup from PM_STANDBY mode and
 * transition to PM_SLEEP mode (STM32 standby mode).
 */

#ifndef CONFIG_PM_ALARM_SEC
#  define CONFIG_PM_ALARM_SEC 15
#endif

#ifndef CONFIG_PM_ALARM_NSEC
#  define CONFIG_PM_ALARM_NSEC 0
#endif

/* Values for the RTC Alarm to reset from the PM_SLEEP mode (STM32
 * standby mode).  If CONFIG_PM_SLEEP_WAKEUP is defined in the
 * configuration, then the logic in this file will program the RTC
 * alarm to wakeup the processor after an a delay.
 *
 * This feature might be useful, for example, in a system that needs to
 * use minimal power but awake up to perform some task at periodic
 * intervals.
 */

#ifdef CONFIG_PM_SLEEP_WAKEUP

#  ifndef CONFIG_RTC_ALARM
#    error "CONFIG_RTC_ALARM should be enabled to use CONFIG_PM_SLEEP_WAKEUP"
#  endif

   /* If CONFIG_PM_SLEEP_WAKEUP is defined, then ifndefCONFIG_PM_SLEEP_WAKEUP_SEC
    * and ifndefCONFIG_PM_SLEEP_WAKEUP_NSEC define the delay until the STM32
    * awakens from PM_SLEEP mode.
    */

#  ifndefCONFIG_PM_SLEEP_WAKEUP_SEC
#    defineCONFIG_PM_SLEEP_WAKEUP_SEC 10
#  endif

#  ifndefCONFIG_PM_SLEEP_WAKEUP_NSEC
#    defineCONFIG_PM_SLEEP_WAKEUP_NSEC 0
#  endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_PM) && defined(CONFIG_RTC_ALARM)
static void up_alarmcb(void);
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
#ifdef CONFIG_RTC_ALARM
  struct timespec alarmtime;
#endif
  static enum pm_state_e oldstate = PM_NORMAL;
  enum pm_state_e newstate;
  int ret;

  /* Decide, which power saving level can be obtained */

  newstate = pm_checkstate();

  /* Check for state changes */

  if (newstate != oldstate)
    {
      lldbg("newstate= %d oldstate=%d\n", newstate, oldstate);

      sched_lock();

      /* Force the global state change */

      ret = pm_changestate(newstate);
      if (ret < 0)
        {
          /* The new state change failed, revert to the preceding state */

          (void)pm_changestate(oldstate);

          /* No state change... */

          return;
        }

      /* Then perform board-specific, state-dependent logic here */

      switch (newstate)
        {
        case PM_NORMAL:
          {
            /* If we just awakened from PM_STANDBY mode, then reconfigure
             * clocking.
             */

            if (oldstate == PM_STANDBY)
              {              
                stm32_clockconfig();
              }
          }
          break;

        case PM_IDLE:
          {
          }
          break;

        case PM_STANDBY:
          {
#ifdef CONFIG_RTC_ALARM
            /* Disable RTC Alarm interrupt */

            stm32_exti_alarm(true, true, true, NULL);

            /* Configure the RTC alarm to Auto Wake the system */

            (void)up_rtc_gettime(&alarmtime);

            alarmtime.tv_sec  += CONFIG_PM_ALARM_SEC;
            alarmtime.tv_nsec += CONFIG_PM_ALARM_NSEC;

            /* The tv_nsec value must not exceed 1,000,000,000. That
             * would be an invalid time.
             */

            if (alarmtime.tv_nsec >= NSEC_PER_SEC)
              {
                /* Carry to the seconds */

                alarmtime.tv_sec++;
                alarmtime.tv_nsec -= NSEC_PER_SEC;
              }

            /* Set the alarm */

            ret = up_rtc_setalarm(&alarmtime, &up_alarmcb);
            if (ret < 0)
              {
                lldbg("Warning: The alarm is already set\n");
              }
#endif
            /* Enter the STM32 stop mode */

            (void)stm32_pmstop(false);

            /* We have been re-awakened by some even:  A button press?
             * An alarm?  Cancel any pending alarm and resume the normal
             * operation.
             */

#ifdef CONFIG_RTC_ALARM
            ret = up_rtc_cancelalarm();
            if (ret < 0)
              {
                lldbg("Warning: Cancel alarm failed\n");
              }
#endif
            /* Resume normal operation */

            pm_changestate(PM_NORMAL);
          }
          break;

        case PM_SLEEP:
          {
            /* We should not return from standby mode.  The only way out
             * of standby is via the reset path.
             */

#ifdef CONFIG_PM_SLEEP_WAKEUP
            /* Configure the RTC alarm to Auto Reset the system */

            (void)up_rtc_gettime(&alarmtime);

            alarmtime.tv_sec  +=CONFIG_PM_SLEEP_WAKEUP_SEC;
            alarmtime.tv_nsec +=CONFIG_PM_SLEEP_WAKEUP_NSEC;

            /* The tv_nsec value must not exceed 1,000,000,000. That
             * would be an invalid time.
             */

            if (alarmtime.tv_nsec >= NSEC_PER_SEC)
              {
                /* Carry to the seconds */

                alarmtime.tv_sec++;
                alarmtime.tv_nsec -= NSEC_PER_SEC;
              }

            /* Set the alarm */

            ret = up_rtc_setalarm(&alarmtime, &up_alarmcb);
            if (ret < 0)
              {
                lldbg("Warning: The alarm is already set\n");
              }
#endif
            /* Enter the STM32 standby mode */

            up_mdelay(10);
            (void)stm32_pmstandby();
          }
          break;

        default:
          break;
        }

      /* Save the new state */

      oldstate = newstate;

      sched_unlock();
    }
}
#else
#  define up_idlepm()
#endif

/************************************************************************************
 * Name: up_alarmcb
 *
 * Description:
 *    RTC alarm service routine
 *
 ************************************************************************************/

#if defined(CONFIG_PM) && defined(CONFIG_RTC_ALARM)
static void up_alarmcb(void)
{
  /* This alarm occurs because there wasn't any EXTI interrupt during the
   * PM_STANDBY period. So just go to sleep.
   */

  pm_changestate(PM_SLEEP);
}
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

  BEGIN_IDLE();
  up_idlepm();
  END_IDLE();
#endif
}

