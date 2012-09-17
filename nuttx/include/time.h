/********************************************************************************
 * include/time.h
 *
 *   Copyright (C) 2007-2011 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_TIME_H
#define __INCLUDE_TIME_H

/********************************************************************************
 * Included Files
 ********************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>

/********************************************************************************
 * Pre-processor Definitions
 ********************************************************************************/

/* Clock tick of the system (frequency Hz).  
 *
 * NOTE: This symbolic name CLK_TCK has been removed from the standard.  It is
 * replaced with CLOCKS_PER_SEC.  Both are defined here.
 *
 * The default value is 100Hz, but this default setting can be overridden by
 * defining the clock interval in milliseconds as CONFIG_MSEC_PER_TICK in the
 * board configuration file.
 */

#ifdef CONFIG_MSEC_PER_TICK
# define CLK_TCK           (1000/CONFIG_MSEC_PER_TICK)
# define CLOCKS_PER_SEC    (1000/CONFIG_MSEC_PER_TICK)
#else
# define CLK_TCK           (100)
# define CLOCKS_PER_SEC    (100)
#endif

/* CLOCK_REALTIME refers to the standard time source.  For most implementations,
 * the standard time source is the system timer interrupt.  However, if the
 * platform supports an RTC, then the standard time source will be the RTC
 * for the clock_gettime() and clock_settime() interfaces (the system timer
 * is still the time source for all of the interfaces).
 */

#define CLOCK_REALTIME     0

/* If an RTC is supported, then the non-standard CLOCK_ACTIVETIME is also
 * supported to manage time based on the system timer interrupt separately from
 * the RTC.  This may be necessary, for example, in certain cases where the
 * system timer interrupt has been stopped in low power modes.
 *
 * CLOCK_ACTIVETIME is only recognized by clock_gettime() and clock_settime().
 */

#ifdef CONFIG_RTC
#  define CLOCK_ACTIVETIME 1
#else
#  define CLOCK_ACTIVETIME CLOCK_REALTIME
#endif

/* This is a flag that may be passed to the timer_settime() function */

#define TIMER_ABSTIME      1

/* Local time is the same as gmtime in this implementation */

#define localtime(c)       gmtime(c)
#define localtime_r(c,r)   gmtime_r(c,r)

/********************************************************************************
 * Global Type Declarations
 ********************************************************************************/

typedef uint32_t  time_t;         /* Holds time in seconds */
typedef uint8_t   clockid_t;      /* Identifies one time base source */
typedef FAR void *timer_t;        /* Represents one POSIX timer */

struct timespec
{
  time_t tv_sec;                   /* Seconds */
  long   tv_nsec;                  /* Nanoseconds */
};

struct timeval
{
  time_t tv_sec;                   /* Seconds */
  long tv_usec;                    /* Microseconds */
};

struct tm
{
  int tm_sec;     /* second (0-61, allows for leap seconds) */
  int tm_min;     /* minute (0-59) */
  int tm_hour;    /* hour (0-23) */
  int tm_mday;    /* day of the month (1-31) */
  int tm_mon;     /* month (0-11) */
  int tm_year;    /* years since 1900 */
#if 0 /* not supported */
  int tm_wday;    /* day of the week (0-6) */
  int tm_yday;    /* day of the year (0-365) */
  int tm_isdst;   /* non-0 if daylight savings time is in effect */
#endif
};

/* Struct itimerspec is used to define settings for an interval timer */

struct itimerspec
{
  struct timespec it_value;    /* First time */
  struct timespec it_interval; /* and thereafter */
};

/* forward reference (defined in signal.h) */

struct sigevent;

/********************************************************************************
 * Global Variables
 ********************************************************************************/

/* extern char *tznames[]; not supported */

/********************************************************************************
 * Global Function Prototypes
 ********************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

EXTERN clock_t clock(void);

EXTERN int clock_settime(clockid_t clockid, const struct timespec *tp);
EXTERN int clock_gettime(clockid_t clockid, struct timespec *tp);
EXTERN int clock_getres(clockid_t clockid, struct timespec *res);

EXTERN time_t mktime(const struct tm *tp);
EXTERN FAR struct tm *gmtime(FAR const time_t *timer);
EXTERN FAR struct tm *gmtime_r(FAR const time_t *timer, FAR struct tm *result);
EXTERN size_t strftime(char *s, size_t max, const char *format, const struct tm *tm);

EXTERN time_t time(time_t *tloc);

EXTERN int timer_create(clockid_t clockid, FAR struct sigevent *evp, FAR timer_t *timerid);
EXTERN int timer_delete(timer_t timerid);
EXTERN int timer_settime(timer_t timerid, int flags, FAR const struct itimerspec *value,
                         FAR struct itimerspec *ovalue);
EXTERN int timer_gettime(timer_t timerid, FAR struct itimerspec *value);
EXTERN int timer_getoverrun(timer_t timerid);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif  /* __INCLUDE_TIME_H */
