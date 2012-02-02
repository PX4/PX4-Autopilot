/****************************************************************************
 * apps/nshlib/dbg_timcmds.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

#include "nsh.h"
#include "nsh_console.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define MAX_TIME_STRING 80

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined (CONFIG_RTC) && !defined(CONFIG_DISABLE_CLOCK) && !defined(CONFIG_NSH_DISABLE_DATE)
static FAR const char * const g_datemontab[] =
{
  "jan", "feb", "mar", "apr", "may", "jun",
  "jul", "aug", "sep", "oct", "nov", "dec"
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: date_month
 ****************************************************************************/

#if defined (CONFIG_RTC) && !defined(CONFIG_DISABLE_CLOCK) && !defined(CONFIG_NSH_DISABLE_DATE)
static inline int date_month(FAR const char *abbrev)
{
  int i;

  for (i = 0; i < 12; i++)
    {
      if (strncasecmp(g_datemontab[i], abbrev, 3) == 0)
        {
          return i;
        }
    }
  return ERROR;
}
#endif

/****************************************************************************
 * Name: date_gettime
 ****************************************************************************/

#if defined (CONFIG_RTC) && !defined(CONFIG_DISABLE_CLOCK) && !defined(CONFIG_NSH_DISABLE_DATE)
static inline int date_showtime(FAR struct nsh_vtbl_s *vtbl, FAR const char *name)
{
  static const char format[] = "%b %d %H:%M:%S %Y";
  struct timespec ts;
  struct tm tm;
  char timbuf[MAX_TIME_STRING];
  int ret;

  /* Get the current time */

  ret = clock_gettime(CLOCK_REALTIME, &ts);
  if (ret < 0)
    {
      nsh_output(vtbl, g_fmtcmdfailed, name, "clock_gettime", NSH_ERRNO);
      return ERROR;
    }

  /* Break the current time up into the format needed by strftime */

  (void)gmtime_r((FAR const time_t*)&ts.tv_sec, &tm);

  /* Show the current time in the requested format */

  (void)strftime(timbuf, MAX_TIME_STRING, format, &tm);
  nsh_output(vtbl, "%s\n", timbuf);
  return OK;
}
#endif

/****************************************************************************
 * Name: date_settime
 ****************************************************************************/

#if defined (CONFIG_RTC) && !defined(CONFIG_DISABLE_CLOCK) && !defined(CONFIG_NSH_DISABLE_DATE)
static inline int date_settime(FAR struct nsh_vtbl_s *vtbl, FAR const char *name,
                               FAR char *newtime)
{
  struct timespec ts;
  struct tm tm;
  FAR char *token;
  FAR char *saveptr;
  long result;
  int ret;

  /* Only this date format is supported: MMM DD HH:MM:SS YYYY */
  /* Get the month abbreviation */

  token = strtok_r(newtime, " \t",&saveptr);
  if (token == NULL)
    {
      goto errout_bad_parm;
    }

  tm.tm_mon = date_month(token);
  if (tm.tm_mon < 0)
    {
      goto errout_bad_parm;
    }

  /* Get the day of the month.  NOTE: Accepts day-of-month up to 31 for all months */

  token = strtok_r(NULL, " \t",&saveptr);
  if (token == NULL)
    {
      goto errout_bad_parm;
    }

  result = strtol(token, NULL, 10);
  if (result < 1 || result > 31)
    {
      goto errout_bad_parm;
    }
  tm.tm_mday = (int)result;

  /* Get the hours */

  token = strtok_r(NULL, " \t:", &saveptr);
  if (token == NULL)
    {
      goto errout_bad_parm;
    }

  result = strtol(token, NULL, 10);
  if (result < 0 || result > 23)
    {
      goto errout_bad_parm;
    }
  tm.tm_hour = (int)result;

  /* Get the minutes */

  token = strtok_r(NULL, " \t:", &saveptr);
  if (token == NULL)
    {
      goto errout_bad_parm;
    }

  result = strtol(token, NULL, 10);
  if (result < 0 || result > 59)
    {
      goto errout_bad_parm;
    }
  tm.tm_min = (int)result;

  /* Get the seconds */

  token = strtok_r(NULL, " \t:", &saveptr);
  if (token == NULL)
    {
      goto errout_bad_parm;
    }

  result = strtol(token, NULL, 10);
  if (result < 0 || result > 61)
    {
      goto errout_bad_parm;
    }
  tm.tm_sec = (int)result;

  /* And finally the year */

  token = strtok_r(NULL, " \t", &saveptr);
  if (token == NULL)
    {
      goto errout_bad_parm;
    }

  result = strtol(token, NULL, 10);
  if (result < 1900 || result > 2100)
    {
      goto errout_bad_parm;
    }
  tm.tm_year = (int)result - 1900;

  /* Convert this to the right form, then set the timer */

  ts.tv_sec  = mktime(&tm);
  ts.tv_nsec = 0;

  ret = clock_settime(CLOCK_REALTIME, &ts);
  if (ret < 0)
    {
      nsh_output(vtbl, g_fmtcmdfailed, name, "clock_settime", NSH_ERRNO);
      return ERROR;
    }
  return OK;

errout_bad_parm:
  nsh_output(vtbl, g_fmtarginvalid, name);
  return ERROR;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cmd_date
 ****************************************************************************/

#if defined (CONFIG_RTC) && !defined(CONFIG_DISABLE_CLOCK) && !defined(CONFIG_NSH_DISABLE_DATE)
int cmd_date(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  FAR char *newtime = NULL;
  FAR const char *errfmt;
  bool badarg = false;
  int option;
  int ret;

  /* Get the date options:  date [-s time] [+FORMAT] */

  while ((option = getopt(argc, argv, "s:")) != ERROR)
    {
      if (option == 's')
        {
          /* We will be setting the time */

          newtime = optarg;
        }
      else /* option = '?' */
        {
          /* We need to parse to the end anyway so that getopt stays healthy */

          badarg = true;
        }
   }

  /* If a bad argument was encountered then exit with an error */

  if (badarg)
    {
      errfmt = g_fmtarginvalid;
      goto errout;
    }

  /* optind < argc-1 means that there are additional, unexpected arguments on
   * th command-line
   */

  if (optind < argc)
    {
      errfmt = g_fmttoomanyargs;
      goto errout;
    }

 /* Display or set the time */

 if (newtime)
   {
     ret = date_settime(vtbl, argv[0], newtime);
   }
 else
   {
     ret = date_showtime(vtbl, argv[0]);
   }
 return ret;

errout:
  nsh_output(vtbl, errfmt, argv[0]);
  return ERROR;
}
#endif
