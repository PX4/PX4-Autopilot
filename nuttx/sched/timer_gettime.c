/********************************************************************************
 * timer_gettime.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
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
 ********************************************************************************/

/********************************************************************************
 * Included Files
 ********************************************************************************/

#include <nuttx/config.h>

#include <time.h>
#include <errno.h>

#include "clock_internal.h"
#include "timer_internal.h"

#ifndef CONFIG_DISABLE_POSIX_TIMERS

/********************************************************************************
 * Definitions
 ********************************************************************************/

/********************************************************************************
 * Private Data
 ********************************************************************************/

/********************************************************************************
 * Public Data
 ********************************************************************************/

/********************************************************************************
 * Private Functions
 ********************************************************************************/

/********************************************************************************
 * Public Functions
 ********************************************************************************/

/********************************************************************************
 * Name: timer_gettime
 *
 * Description:
 *  The timer_gettime() function will store the amount of time until the
 *  specified timer, timerid, expires and the reload value of the timer into the
 *  space pointed to by the value argument. The it_value member of this structure
 *  will contain the amount of time before the timer expires, or zero if the timer
 *  is disarmed. This value is returned as the interval until timer expiration,
 *  even if the timer was armed with absolute time. The it_interval member of
 *  value will contain the reload value last set by timer_settime().
 *
 * Parameters:
 *   timerid - The pre-thread timer, previously created by the call to
 *   timer_create(), whose remaining time count will be returned..
 *
 * Return Value:
 *   If the timer_gettime() succeeds, a value of 0 (OK) will be returned.
 *   If an error occurs, the value -1 (ERROR) will be returned, and errno set to
 *   indicate the error.
 *
 *   EINVAL - The timerid argument does not correspond to an ID returned by
 *     timer_create() but not yet deleted by timer_delete().
 *
 * Assumptions/Limitations:
 *   Due to the asynchronous operation of this function, the time reported
 *   by this function could be significantly more than that actual time
 *   remaining on the timer at any time.
 *
 ********************************************************************************/

int timer_gettime(timer_t timerid, FAR struct itimerspec *value)
{
  FAR struct posix_timer_s *timer = (FAR struct posix_timer_s *)timerid;
  int ticks;

  if (!timer || !value)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  /* Get the number of ticks before the underlying watchdog expires */

  ticks = wd_gettime(timer->pt_wdog);

  /* Convert that to a struct timespec and return it */

  (void)clock_ticks2time(ticks, &value->it_value);
  (void)clock_ticks2time(timer->pt_last, &value->it_interval);
  return OK;
}

#endif /* CONFIG_DISABLE_POSIX_TIMERS */
