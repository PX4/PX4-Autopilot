/********************************************************************************
 * sched/clock_time2ticks.c
 *
 *   Copyright (C) 2007, 2009 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <time.h>

#include "clock_internal.h"

/********************************************************************************
 * Definitions
 ********************************************************************************/

/********************************************************************************
 * Private Type Declarations
 ********************************************************************************/

/********************************************************************************
 * Global Variables
 ********************************************************************************/

/********************************************************************************
 * Private Variables
 ********************************************************************************/

/********************************************************************************
 * Private Functions
 ********************************************************************************/

/********************************************************************************
 * Public Functions
 ********************************************************************************/

/********************************************************************************
 * Name: clock_time2ticks
 *
 * Description:
 *   Convert a timespec delay to system timer ticks.  This function is suitable
 *   for calculating relative time delays and does not depend on the other
 *   clock_* logic.
 *
 * Parameters:
 *   reltime - Convert this relative time to system clock ticks.
 *   ticks - Return the converted number of ticks here.
 *
 * Return Value:
 *   Always returns OK
 *
 * Assumptions:
 *
 ********************************************************************************/

int clock_time2ticks(FAR const struct timespec *reltime, FAR int *ticks)
{
#ifdef CONFIG_HAVE_LONG_LONG
  int64_t relnsec;

  /* Convert the relative time into nanoseconds.  The range of the int64_t is
   * sufficiently large that there is no real need for range checking.
   */

  relnsec = (int64_t)reltime->tv_sec * NSEC_PER_SEC +
            (int64_t)reltime->tv_nsec;
  
  /* Convert nanoseconds to clock ticks, rounding up to the smallest integer
   * that is greater than or equal to the exact number of tick.
   */

  *ticks = (int)((relnsec + NSEC_PER_TICK - 1) / NSEC_PER_TICK);
  return OK;
#else
  int32_t relusec;

  /* This function uses an int32_t to only the relative time in microseconds.
   * that means that the maximum supported relative time is 2,147,487.647
   * seconds
   */

#if 0 // overkill
  DEBUGASSERT(reltime->tv_sec  <  2147487 ||
              reltime->tv_sec  == 2147487 &&
              reltime->tv_nsec <= 647 * NSEC_PER_MSEC);
#endif

  /* Convert the relative time into microseconds, rounding up to the smallest
   * value that is greater than or equal to the exact number of microseconds.
   */

  relusec = reltime->tv_sec * USEC_PER_SEC +
            (reltime->tv_nsec + NSEC_PER_USEC - 1) / NSEC_PER_USEC;

  /* Convert microseconds to clock ticks, rounding up to the smallest integer
   * that is greater than or equal to the exact number of tick.
   */

  *ticks = (int)((relusec + USEC_PER_TICK - 1) / USEC_PER_TICK);
  return OK;
#endif
}
