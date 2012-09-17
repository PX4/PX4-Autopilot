/****************************************************************************
 * netutils/thttpd/timers.c
 * Parse string dates into internal form, stripped-down version
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derived from the file of the same name in the original THTTPD package:
 *
 *   Copyright © 1995 by Jef Poskanzer <jef@mail.acme.com>.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <time.h>
#include <debug.h>

#include "tdate_parse.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

#undef TDATE_PARSE_WORKS /* tdate_parse() doesn't work */
#undef HAVE_DAY_OF_WEEK  /* Day of week not yet supported by NuttX */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct strlong
{
  char *s;
  long l;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef HAVE_DAY_OF_WEEK /* Day of week not yet supported by NuttX */
static void pound_case(char *str)
{
  for (; *str != '\0'; ++str)
    {
      if (isupper((int)*str))
        {
          *str = tolower((int)*str);
        }
    }
}
#endif

#ifdef HAVE_DAY_OF_WEEK /* Day of week not yet supported by NuttX */
static int strlong_compare(const void *v1, const void *v2)
{
  return strcmp(((struct strlong *)v1)->s, ((struct strlong *)v2)->s);
}
#endif

#ifdef HAVE_DAY_OF_WEEK /* Day of week not yet supported by NuttX */
static int strlong_search(char *str, struct strlong *tab, int n, long *lP)
{
  int i, h, l, r;

  l = 0;
  h = n - 1;
  for (;;)
    {
      i = (h + l) / 2;
      r = strcmp(str, tab[i].s);
      if (r < 0)
        {
          h = i - 1;
        }
      else if (r > 0)
        {
          l = i + 1;
        }
      else
        {
          *lP = tab[i].l;
          return 1;
        }

      if (h < l)
        {
          return 0;
        }
    }
}
#endif

#ifdef HAVE_DAY_OF_WEEK /* Day of week not yet supported by NuttX */
static int scan_wday(char *str_wday, long *tm_wdayP)
{
  static struct strlong wday_tab[] = {
    {"sun", 0}, {"sunday", 0},
    {"mon", 1}, {"monday", 1},
    {"tue", 2}, {"tuesday", 2},
    {"wed", 3}, {"wednesday", 3},
    {"thu", 4}, {"thursday", 4},
    {"fri", 5}, {"friday", 5},
    {"sat", 6}, {"saturday", 6},
  };
  static int sorted = 0;

  if (!sorted)
    {
      (void)qsort(wday_tab, sizeof(wday_tab) / sizeof(struct strlong),
                  sizeof(struct strlong), strlong_compare);
      sorted = 1;
    }
  pound_case(str_wday);
  return strlong_search(str_wday, wday_tab,
                        sizeof(wday_tab) / sizeof(struct strlong), tm_wdayP);
}
#endif /* Day of week not yet supported by NuttX */

#ifdef TDATE_PARSE_WORKS
static int scan_mon(char *str_mon, long *tm_monP)
{
  static struct strlong mon_tab[] = {
    {"jan", 0}, {"january", 0},
    {"feb", 1}, {"february", 1},
    {"mar", 2}, {"march", 2},
    {"apr", 3}, {"april", 3},
    {"may", 4},
    {"jun", 5}, {"june", 5},
    {"jul", 6}, {"july", 6},
    {"aug", 7}, {"august", 7},
    {"sep", 8}, {"september", 8},
    {"oct", 9}, {"october", 9},
    {"nov", 10}, {"november", 10},
    {"dec", 11}, {"december", 11},
  };
  static int sorted = 0;

  if (!sorted)
    {
      (void)qsort(mon_tab, sizeof(mon_tab) / sizeof(struct strlong),
                  sizeof(struct strlong), strlong_compare);
      sorted = 1;
    }
  pound_case(str_mon);
  return strlong_search(str_mon, mon_tab,
                        sizeof(mon_tab) / sizeof(struct strlong), tm_monP);
}
#endif
/****************************************************************************
 * Public Functions
 ****************************************************************************/

time_t tdate_parse(char *str)
{
#ifdef TDATE_PARSE_WORKS /* REVISIT -- doesn't work */
  struct tm tm;
  char *cp;
  char str_mon[32];
  int tm_year;
  int tm_mday;
  int tm_hour;
  int tm_min;
  int tm_sec;
  long tm_mon;
#ifdef HAVE_DAY_OF_WEEK /* Day of week not yet supported by NuttX */
  char str_wday[32];
  long tm_wday;
#endif

  nvdbg("str: \"%s\"\n", str);

  /* Initialize. */

  (void)memset((char *)&tm, 0, sizeof(struct tm));

  /* Skip initial whitespace ourselves - sscanf is clumsy at this. */

  for (cp = str; *cp == ' ' || *cp == '\t'; ++cp)
    {
      continue;
    }

  /* And do the sscanfs.  WARNING: you can add more formats here, but be
   * careful! You can easily screw up the parsing of existing formats when
   * you add new ones.  The order is important. */

  /* DD-mth-YY HH:MM:SS GMT */
  if (sscanf(cp, "%d-%400[a-zA-Z]-%d %d:%d:%d GMT",
             &tm_mday, str_mon, &tm_year, &tm_hour, &tm_min,
             &tm_sec) == 6 && scan_mon(str_mon, &tm_mon))
    {
      tm.tm_mday = tm_mday;
      tm.tm_mon  = tm_mon;
      tm.tm_year = tm_year;
      tm.tm_hour = tm_hour;
      tm.tm_min  = tm_min;
      tm.tm_sec  = tm_sec;
    }

  /* DD mth YY HH:MM:SS GMT */
  else if (sscanf(cp, "%d %400[a-zA-Z] %d %d:%d:%d GMT",
                  &tm_mday, str_mon, &tm_year, &tm_hour, &tm_min,
                  &tm_sec) == 6 && scan_mon(str_mon, &tm_mon))
    {
      tm.tm_mday = tm_mday;
      tm.tm_mon  = tm_mon;
      tm.tm_year = tm_year;
      tm.tm_hour = tm_hour;
      tm.tm_min  = tm_min;
      tm.tm_sec  = tm_sec;
    }

  /* HH:MM:SS GMT DD-mth-YY */
  else if (sscanf(cp, "%d:%d:%d GMT %d-%400[a-zA-Z]-%d",
                  &tm_hour, &tm_min, &tm_sec, &tm_mday, str_mon,
                  &tm_year) == 6 && scan_mon(str_mon, &tm_mon))
    {
      tm.tm_hour = tm_hour;
      tm.tm_min  = tm_min;
      tm.tm_sec  = tm_sec;
      tm.tm_mday = tm_mday;
      tm.tm_mon  = tm_mon;
      tm.tm_year = tm_year;
    }

  /* HH:MM:SS GMT DD mth YY */
  else if (sscanf(cp, "%d:%d:%d GMT %d %400[a-zA-Z] %d",
                  &tm_hour, &tm_min, &tm_sec, &tm_mday, str_mon,
                  &tm_year) == 6 && scan_mon(str_mon, &tm_mon))
    {
      tm.tm_hour = tm_hour;
      tm.tm_min  = tm_min;
      tm.tm_sec  = tm_sec;
      tm.tm_mday = tm_mday;
      tm.tm_mon  = tm_mon;
      tm.tm_year = tm_year;
    }

#ifdef HAVE_DAY_OF_WEEK /* Day of week not yet supported by NuttX */
  /* wdy, DD-mth-YY HH:MM:SS GMT */
  else if (sscanf(cp, "%400[a-zA-Z], %d-%400[a-zA-Z]-%d %d:%d:%d GMT",
                  str_wday, &tm_mday, str_mon, &tm_year, &tm_hour, &tm_min,
                  &tm_sec) == 7 &&
           scan_wday(str_wday, &tm_wday) && scan_mon(str_mon, &tm_mon))
    {
      tm.tm_wday = tm_wday;
      tm.tm_mday = tm_mday;
      tm.tm_mon  = tm_mon;
      tm.tm_year = tm_year;
      tm.tm_hour = tm_hour;
      tm.tm_min  = tm_min;
      tm.tm_sec  = tm_sec;
    }
#endif /* Day of week not yet supported by NuttX */

#ifdef HAVE_DAY_OF_WEEK /* Day of week not yet supported by NuttX */
  /* wdy, DD mth YY HH:MM:SS GMT */
  else if (sscanf(cp, "%400[a-zA-Z], %d %400[a-zA-Z] %d %d:%d:%d GMT",
                  str_wday, &tm_mday, str_mon, &tm_year, &tm_hour, &tm_min,
                  &tm_sec) == 7 &&
           scan_wday(str_wday, &tm_wday) && scan_mon(str_mon, &tm_mon))
    {
      tm.tm_wday = tm_wday;
      tm.tm_mday = tm_mday;
      tm.tm_mon  = tm_mon;
      tm.tm_year = tm_year;
      tm.tm_hour = tm_hour;
      tm.tm_min  = tm_min;
      tm.tm_sec  = tm_sec;
    }
#endif /* Day of week not yet supported by NuttX */

#ifdef HAVE_DAY_OF_WEEK /* Day of week not yet supported by NuttX */
  /* wdy mth DD HH:MM:SS GMT YY */
  else if (sscanf(cp, "%400[a-zA-Z] %400[a-zA-Z] %d %d:%d:%d GMT %d",
                  str_wday, str_mon, &tm_mday, &tm_hour, &tm_min, &tm_sec,
                  &tm_year) == 7 &&
           scan_wday(str_wday, &tm_wday) && scan_mon(str_mon, &tm_mon))
    {
      tm.tm_wday = tm_wday;
      tm.tm_mon  = tm_mon;
      tm.tm_mday = tm_mday;
      tm.tm_hour = tm_hour;
      tm.tm_min  = tm_min;
      tm.tm_sec  = tm_sec;
      tm.tm_year = tm_year;
    }
#endif /* Day of week not yet supported by NuttX */
  else
    {
      return (time_t) - 1;
    }

  if (tm.tm_year > 1900)
    {
      tm.tm_year -= 1900;
    }
  else if (tm.tm_year < 70)
    {
      tm.tm_year += 100;
    }

  return mktime(&tm);
#else
  return 0; // for now
#endif
}

