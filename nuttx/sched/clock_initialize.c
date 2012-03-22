/****************************************************************************
 * sched/clock_initialize.c
 *
 *   Copyright (C) 2007, 2009, 2011-2012 Gregory Nutt. All rights reserved.
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
#include <nuttx/compiler.h>

#include <stdint.h>
#include <time.h>
#include <errno.h>
#include <debug.h>

#ifdef CONFIG_RTC
#  include <arch/irq.h>
#endif

#include <nuttx/clock.h>
#include <nuttx/time.h>
#include <nuttx/rtc.h>

#include "clock_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/
/* Standard time definitions (in units of seconds) */

#define SEC_PER_MIN  ((time_t)60)
#define SEC_PER_HOUR ((time_t)60 * SEC_PER_MIN)
#define SEC_PER_DAY  ((time_t)24 * SEC_PER_HOUR)

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/**************************************************************************
 * Public Constant Data
 **************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

#ifdef CONFIG_SYSTEM_TIME64
volatile uint64_t g_system_timer;
uint64_t          g_tickbias;
#else
volatile uint32_t g_system_timer;
uint32_t          g_tickbias;
#endif

struct timespec   g_basetime;

/**************************************************************************
 * Private Variables
 **************************************************************************/

/**************************************************************************
 * Private Functions
 **************************************************************************/

/****************************************************************************
 * Function: clock_basetime
 *
 * Description:
 *   Get the initial time value from the best source available.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC
#if defined(CONFIG_RTC_DATETIME)
/* Initialize the system time using a broken out date/time structure */

static inline void clock_basetime(FAR struct timespec *tp)
{
  struct tm rtctime;

  /* Get the broken-out time from the date/time RTC. */

  (void)up_rtc_getdatetime(&rtctime);

  /* And use the broken-out time to initialize the system time */

  tp->tv_sec  = mktime(&rtctime);
  tp->tv_nsec = 0;
}

#elif defined(CONFIG_RTC_HIRES)

/* Initialize the system time using a high-resolution structure */

static inline void clock_basetime(FAR struct timespec *tp)
{
  /* Get the complete time from the hi-res RTC. */

  (void)up_rtc_gettime(tp);
}

#else

/* Initialize the system time using seconds only */

static inline void clock_basetime(FAR struct timespec *tp)
{
  /* Get the seconds (only) from the lo-resolution RTC */

  tp->tv_sec  = up_rtc_time();
  tp->tv_nsec = 0;
}

#endif /* CONFIG_RTC_HIRES */
#else /* CONFIG_RTC */

static inline void clock_basetime(FAR struct timespec *tp)
{
  time_t jdn = 0;

  /* Get the EPOCH-relative julian date from the calendar year,
   * month, and date
   */

  jdn = clock_calendar2utc(CONFIG_START_YEAR, CONFIG_START_MONTH,
                           CONFIG_START_DAY);

  /* Set the base time as seconds into this julian day. */

  tp->tv_sec  = jdn * SEC_PER_DAY;
  tp->tv_nsec = 0;
}

#endif /* CONFIG_RTC */

/****************************************************************************
 * Function: clock_inittime
 *
 * Description:
 *   Get the initial time value from the best source available.
 *
 ****************************************************************************/

static void clock_inittime(void)
{
  /* (Re-)initialize the time value to match the RTC*/

  clock_basetime(&g_basetime);
  g_system_timer = 0;
  g_tickbias     = 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: clock_initialize
 *
 * Description:
 *   Perform one-time initialization of the timing facilities.
 *
 ****************************************************************************/

void clock_initialize(void)
{
  /* Initialize the RTC hardware */

#ifdef CONFIG_RTC
  up_rtcinitialize();
#endif

  /* Initialize the time value to match the RTC */

  clock_inittime();
}

/****************************************************************************
 * Function:  clock_synchronize
 *
 * Description:
 *   Synchronize the system timer to a hardware RTC.  This operation is
 *   normally performed automatically by the system during clock
 *   initialization.  However, the user may also need to explicitly re-
 *   synchronize the system timer to the RTC under certain conditions where
 *   the system timer is known to be in error.  For example, in certain low-
 *   power states, the system timer may be stopped but the RTC will continue
 *   keep correct time.  After recovering from such low-power state, this
 *   function should be called to restore the correct system time.
 *
 *   Calling this function could result in system time going "backward" in
 *   time, especially with certain lower resolution RTC implementations.
 *   Time going backward could have bad consequences if there are ongoing
 *   timers and delays.  So use this interface with care.
 *
 * Parameters:
 *   None
 *
 * Return Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_RTC
void clock_synchronize(void)
{
  irqstate_t flags;

  /* Re-initialize the time value to match the RTC */

  flags = irqsave();
  clock_inittime();
  irqrestore(flags);
}
#endif

/****************************************************************************
 * Function: clock_timer
 *
 * Description:
 *   This function must be called once every time the real
 *   time clock interrupt occurs.  The interval of this
 *   clock interrupt must be MSEC_PER_TICK
 *
 ****************************************************************************/

void clock_timer(void)
{
  /* Increment the per-tick system counter */

  g_system_timer++;
}
