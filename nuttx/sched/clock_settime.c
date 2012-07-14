/************************************************************************
 * sched/clock_settime.c
 *
 *   Copyright (C) 2007, 2009, 2011 Gregory Nutt. All rights reserved.
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
 ************************************************************************/

/************************************************************************
 * Included Files
 ************************************************************************/

#include <nuttx/config.h>
#include <nuttx/rtc.h>

#include <time.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>
#include "clock_internal.h"

/************************************************************************
 * Definitions
 ************************************************************************/

/************************************************************************
 * Private Type Declarations
 ************************************************************************/

/************************************************************************
 * Private Function Prototypes
 ************************************************************************/

/**********************************************************************
 * Public Constant Data
 **********************************************************************/

/************************************************************************
 * Public Variables
 ************************************************************************/

/**********************************************************************
 * Private Variables
 **********************************************************************/

/************************************************************************
 * Private Functions
 ************************************************************************/

/************************************************************************
 * Public Functions
 ************************************************************************/

/************************************************************************
 * Name: clock_settime
 *
 * Description:
 *   Clock Functions based on POSIX APIs
 *
 ************************************************************************/

int clock_settime(clockid_t clock_id, FAR const struct timespec *tp)
{
  irqstate_t flags;
  int ret = OK;

  sdbg("clock_id=%d\n", clock_id);
  DEBUGASSERT(tp != NULL);

  /* CLOCK_REALTIME - POSIX demands this to be present. This is the wall
   * time clock.
   */

#ifdef CONFIG_RTC
  if (clock_id == CLOCK_REALTIME || clock_id == CLOCK_ACTIVETIME)
#else
  if (clock_id == CLOCK_REALTIME)
#endif
    {
      /* Interrupts are disabled here so that the in-memory time
       * representation and the RTC setting will be as close as
       * possible.
       */

      flags = irqsave();

      /* Save the new base time. */

      g_basetime.tv_sec  = tp->tv_sec;
      g_basetime.tv_nsec = tp->tv_nsec;

      /* Get the elapsed time since power up (in milliseconds) biased
       * as appropriate.
       */

      g_tickbias = g_system_timer;

      /* Setup the RTC (lo- or high-res) */

#ifdef CONFIG_RTC
      if (g_rtc_enabled && clock_id != CLOCK_ACTIVETIME)
        {
          up_rtc_settime(tp);
        } 
#endif
      irqrestore(flags);

      sdbg("basetime=(%d,%d) tickbias=%d\n",
          (int)g_basetime.tv_sec, (int)g_basetime.tv_nsec,
          (int)g_tickbias);
    }
  else 
    {
      sdbg("Returning ERROR\n");
      set_errno(EINVAL);
      ret = ERROR;
    }

  return ret;
}
