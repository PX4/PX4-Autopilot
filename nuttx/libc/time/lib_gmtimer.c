/****************************************************************************
 * libc/time/lib_gmtimer.c
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

#include <stdbool.h>
#include <time.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/time.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define SEC_PER_MIN  ((time_t)60)
#define SEC_PER_HOUR ((time_t)60 * SEC_PER_MIN)
#define SEC_PER_DAY  ((time_t)24 * SEC_PER_HOUR)

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Calendar/UTC conversion routines */

static void   clock_utc2calendar(time_t utc, int *year, int *month, int *day);
#ifdef CONFIG_GREGORIAN_TIME
static void   clock_utc2gregorian (time_t jdn, int *year, int *month, int *day);

#ifdef CONFIG_JULIAN_TIME
static void   clock_utc2julian(time_t jdn, int *year, int *month, int *day);
#endif /* CONFIG_JULIAN_TIME */
#endif /* CONFIG_GREGORIAN_TIME */

/**************************************************************************
 * Public Constant Data
 **************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/**************************************************************************
 * Private Variables
 **************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  clock_calendar2utc, clock_gregorian2utc,
 *            and clock_julian2utc
 *
 * Description:
 *    Calendar to UTC conversion routines.  These conversions
 *    are based on algorithms from p. 604 of Seidelman, P. K.
 *    1992.  Explanatory Supplement to the Astronomical
 *    Almanac.  University Science Books, Mill Valley. 
 *
 ****************************************************************************/

#ifdef CONFIG_GREGORIAN_TIME
static void clock_utc2calendar(time_t utc, int *year, int *month, int *day)
{
#ifdef CONFIG_JULIAN_TIME

  if (utc >= GREG_DUTC)
    {
      clock_utc2gregorian(utc + JD_OF_EPOCH, year, month, day);
    }
  else
    {
      clock_utc2julian (utc + JD_OF_EPOCH, year, month, day);
    }

#else /* CONFIG_JULIAN_TIME */

  clock_utc2gregorian(utc + JD_OF_EPOCH, year, month, day);

#endif /* CONFIG_JULIAN_TIME */
}

static void clock_utc2gregorian(time_t jd, int *year, int *month, int *day)
{
  long	l, n, i, j, d, m, y;

  l = jd + 68569;
  n = (4*l) / 146097;
  l = l - (146097*n + 3)/4;
  i = (4000*(l+1))/1461001;
  l = l - (1461*i)/4 + 31;
  j = (80*l)/2447;
  d = l - (2447*j)/80;
  l = j/11;
  m = j + 2 - 12*l;
  y = 100*(n-49) + i + l;

  *year  = y;
  *month = m;
  *day   = d;
}

#ifdef CONFIG_JULIAN_TIME

static void clock_utc2julian(time_t jd, int *year, int *month, int *day)
{
  long	j, k, l, n, d, i, m, y;

  j = jd + 1402;
  k = (j-1)/1461;
  l = j - 1461*k;
  n = (l-1)/365 - l/1461;
  i = l - 365*n + 30;
  j = (80*i)/2447;
  d = i - (2447*j)/80;
  i = j/11;
  m = j + 2 - 12*i;
  y = 4*k + n + i - 4716;

  *year  = y;
  *month = m;
  *day   = d;
}

#endif /* CONFIG_JULIAN_TIME */
#else/* CONFIG_GREGORIAN_TIME */

/* Only handles dates since Jan 1, 1970 */

static void clock_utc2calendar(time_t days, int *year, int *month, int *day)
{
  int  value;
  int  min;
  int  max;
  int  tmp;
  bool leapyear;

  /* There is one leap year every four years, so we can get close with the
   * following:
   */

  value   = days  / (4*365 + 1);  /* Number of 4-years periods since the epoch*/
  days   -= value * (4*365 + 1);  /* Remaining days */
  value <<= 2;                    /* Years since the epoch */

  /* Then we will brute force the next 0-3 years */

  for (;;)
    {
      /* Is this year a leap year (we'll need this later too) */

      leapyear = clock_isleapyear(value + 1970);

      /* Get the number of days in the year */

      tmp = (leapyear ? 366 : 365);

      /* Do we have that many days? */

      if (days >= tmp)
        {
           /* Yes.. bump up the year */

           value++;
           days -= tmp;
        }
      else
        {
           /* Nope... then go handle months */

           break;
        }
    }

  /* At this point, value has the year and days has number days into this year */

  *year = 1970 + value;

  /* Handle the month (zero based) */

  min = 0;
  max = 11;

  do
    {
      /* Get the midpoint */

      value = (min + max) >> 1;

      /* Get the number of days that occurred before the beginning of the month
       * following the midpoint.
      */

      tmp = clock_daysbeforemonth(value + 1, leapyear);

      /* Does the number of days before this month that equal or exceed the
       * number of days we have remaining?
       */

      if (tmp > days)
        {
          /* Yes.. then the month we want is somewhere from 'min' and to the
           * midpoint, 'value'.  Could it be the midpoint?
           */

          tmp = clock_daysbeforemonth(value, leapyear);
          if (tmp > days)
            {
              /* No... The one we want is somewhere between min and value-1 */

              max = value - 1;
            }
          else
            {
              /* Yes.. 'value' contains the month that we want */

              break;
            }
        }
      else
        {
           /* No... The one we want is somwhere between value+1 and max */

           min = value + 1;
        }

      /* If we break out of the loop because min == max, then we want value
       * to be equal to min == max.
       */

      value = min;
    }
  while (min < max);

  /* The selected month number is in value. Subtract the number of days in the
   * selected month
   */

  days -= clock_daysbeforemonth(value, leapyear);

  /* At this point, value has the month into this year (zero based) and days has
   * number of days into this month (zero based)
   */

  *month = value + 1; /* 1-based */
  *day   = days + 1;  /* 1-based */
}

#endif /* CONFIG_GREGORIAN_TIME */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  gmtime_r
 *
 * Description:
 *  Time conversion (based on the POSIX API)
 *
 ****************************************************************************/

FAR struct tm *gmtime_r(FAR const time_t *timer, FAR struct tm *result)
{
  time_t epoch;
  time_t jdn;
  int    year;
  int    month;
  int    day;
  int    hour;
  int    min;
  int    sec;

  /* Get the seconds since the EPOCH */

  epoch = *timer;
  sdbg("timer=%d\n", (int)epoch);

  /* Convert to days, hours, minutes, and seconds since the EPOCH */

  jdn    = epoch / SEC_PER_DAY;
  epoch -= SEC_PER_DAY * jdn;

  hour   = epoch / SEC_PER_HOUR;
  epoch -= SEC_PER_HOUR * hour;

  min    = epoch / SEC_PER_MIN;
  epoch -= SEC_PER_MIN * min;

  sec    = epoch;

  sdbg("hour=%d min=%d sec=%d\n",
       (int)hour, (int)min, (int)sec);

  /* Convert the days since the EPOCH to calendar day */

  clock_utc2calendar(jdn, &year, &month, &day);

  sdbg("jdn=%d year=%d month=%d day=%d\n",
       (int)jdn, (int)year, (int)month, (int)day);

  /* Then return the struct tm contents */

  result->tm_year = (int)year - 1900; /* Relative to 1900 */
  result->tm_mon  = (int)month - 1;   /* zero-based */
  result->tm_mday = (int)day;         /* one-based */
  result->tm_hour = (int)hour;
  result->tm_min  = (int)min;
  result->tm_sec  = (int)sec;

  return result;
}

