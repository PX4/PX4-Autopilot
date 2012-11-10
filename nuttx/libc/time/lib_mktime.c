/****************************************************************************
 * libc/time/lib_mktime.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <time.h>
#include <debug.h>

#include <nuttx/time.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Constant Data
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  mktime
 *
 * Description:
 *  Time conversion (based on the POSIX API)
 *
 ****************************************************************************/

#ifdef CONFIG_GREGORIAN_TIME
time_t mktime(const struct tm *tp)
{
  time_t ret;
  time_t jdn;

  /* Get the EPOCH-relative julian date from the calendar year,
   * month, and date
   */

  jdn = clock_calendar2utc(tp->tm_year+1900, tp->tm_mon+1, tp->tm_mday);
  sdbg("jdn=%d tm_year=%d tm_mon=%d tm_mday=%d\n",
       (int)jdn, tp->tm_year, tp->tm_mon, tp->tm_mday);

  /* Return the seconds into the julian day. */

  ret = ((jdn*24 + tp->tm_hour)*60 + tp->tm_min)*60 + tp->tm_sec;
  sdbg("ret=%d tm_hour=%d tm_min=%d tm_sec=%d\n",
       (int)ret, tp->tm_hour, tp->tm_min, tp->tm_sec);

  return ret;
}
#else

/* Simple version that only works for dates within a (relatively) small range
 * from the epoch.  It does not handle earlier days or longer days where leap
 * seconds, etc. apply.
 */

time_t mktime(const struct tm *tp)
{
  unsigned int days;

  /* Years since epoch in units of days (ignoring leap years). */

  days = (tp->tm_year - 70) * 365;

  /* Add in the extra days for the leap years prior to the current year. */

  days += (tp->tm_year - 69) >> 2;

  /* Add in the days up to the beginning of this month. */

  days += (time_t)clock_daysbeforemonth(tp->tm_mon, clock_isleapyear(tp->tm_year + 1900));

  /* Add in the days since the beginning of this month (days are 1-based). */

  days += tp->tm_mday - 1;

  /* Then convert the seconds and add in hours, minutes, and seconds */

  return ((days * 24 + tp->tm_hour) * 60 + tp->tm_min) * 60 + tp->tm_sec;
}
#endif /* CONFIG_GREGORIAN_TIME */

