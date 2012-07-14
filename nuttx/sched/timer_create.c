/********************************************************************************
 * sched/timer_create.c
 *
 *   Copyright (C) 2007-2009, 2011 Gregory Nutt. All rights reserved.
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
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <wdog.h>
#include <errno.h>

#include <nuttx/kmalloc.h>

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
 * Name: timer_allocate
 *
 * Description:
 *   Allocate one POSIX timer and place it into the allocated timer list.
 *
 ********************************************************************************/

static struct posix_timer_s *timer_allocate(void)
{
  struct posix_timer_s *ret;
  irqstate_t            flags;
  uint8_t               pt_flags;

  /* Try to get a preallocated timer from the free list */

#if CONFIG_PREALLOC_TIMERS > 0
  flags = irqsave();
  ret   = (struct posix_timer_s*)sq_remfirst((sq_queue_t*)&g_freetimers);
  irqrestore(flags);

  /* Did we get one? */
  
  if (ret)
    {
      pt_flags = PT_FLAGS_PREALLOCATED;
    }
  else
#endif
    {
      /* Allocate a new timer from the heap */

      ret      = (struct posix_timer_s*)kmalloc(sizeof(struct posix_timer_s));
      pt_flags = 0;
    }

  /* If we have a timer, then put it into the allocated timer list */

  if (ret)
    {
      /* Initialize the timer structure */

      memset(ret, 0, sizeof(struct posix_timer_s));
      ret->pt_flags = pt_flags;

      /* And add it to the end of the list of allocated timers */

      flags = irqsave();
      sq_addlast((sq_entry_t*)ret, (sq_queue_t*)&g_alloctimers);
      irqrestore(flags);
    }

  return ret;
}

/********************************************************************************
 * Public Functions
 ********************************************************************************/

/********************************************************************************
 * Name: timer_create
 *
 * Description:
 *   The  timer_create() function creates per-thread timer using the specified
 *   clock, clock_id, as the timing base. The timer_create() function returns, in
 *   the location referenced by timerid, a timer ID of type timer_t used to identify
 *   the timer in timer requests. This timer ID is unique until the timer is
 *   deleted. The particular clock, clock_id, is defined in <time.h>. The timer
 *   whose ID is returned will be in a disarmed state upon return from
 *   timer_create().
 *
 *   The evp argument, if non-NULL, points to a sigevent structure. This structure
 *   is allocated by the called and defines the asynchronous notification to occur.
 *   If the evp argument is NULL, the effect is as if the evp argument pointed to
 *   a sigevent structure with the sigev_notify member having the value SIGEV_SIGNAL,
 *   the sigev_signo having a default signal number, and the sigev_value member
 *   having the value of the timer ID.
 *
 *   Each implementation defines a set of clocks that can be used as timing bases
 *   for per-thread timers. All implementations shall support a clock_id of
 *   CLOCK_REALTIME.
 *
 * Parameters:
 *   clockid - Specifies the clock to use as the timing base.
 *   evp - Refers to a user allocated sigevent structure that defines the
 *     asynchronous notification.  evp may be NULL (see above).
 *   timerid - The pre-thread timer created by the call to timer_create().
 *
 * Return Value:
 *   If the call succeeds, timer_create() will return 0 (OK) and update the
 *   location referenced by timerid to a timer_t, which can be passed to the
 *   other per-thread timer calls.  If an error occurs, the function will return
 *   a value of -1 (ERROR) and set errno to indicate the error.
 *
 *   EAGAIN - The system lacks sufficient signal queuing resources to honor the
 *    request.
 *   EAGAIN - The calling process has already created all of the timers it is
 *     allowed by this implementation.
 *   EINVAL - The specified clock ID is not defined.
 *   ENOTSUP - The implementation does not support the creation of a timer attached
 *     to the CPU-time clock that is specified by clock_id and associated with a
 *     thread different thread invoking timer_create().
 *
 * Assumptions:
 *
 ********************************************************************************/

int timer_create(clockid_t clockid, FAR struct sigevent *evp, FAR timer_t *timerid)
{
  struct posix_timer_s *ret;
  WDOG_ID               wdog;

  /* Sanity checks.  Also, we support only CLOCK_REALTIME */

  if (!timerid || clockid != CLOCK_REALTIME)
    {
      errno = EINVAL;
      return ERROR;
    }

  /* Allocate a watchdog to provide the underling CLOCK_REALTIME timer */

  wdog = wd_create();
  if (!wdog)
    {
      errno = EAGAIN;
      return ERROR;
    }

  /* Allocate a timer instance to contain the watchdog */

  ret = timer_allocate();
  if (!ret)
    {
      errno = EAGAIN;
      return ERROR;
    }

  /* Initialize the timer instance */

  ret->pt_crefs = 1;
  ret->pt_owner = getpid();
  ret->pt_delay = 0;
  ret->pt_wdog  = wdog;

  if (evp)
    {
      ret->pt_signo           = evp->sigev_signo;
#ifdef CONFIG_CAN_PASS_STRUCTS
      ret->pt_value           = evp->sigev_value;
#else
      ret->pt_value.sival_ptr = evp->sigev_value.sival_ptr;
#endif
    }
  else
    {
      ret->pt_signo           = SIGALRM;
      ret->pt_value.sival_ptr = ret;
    }

  /* Return the timer */

  *timerid = ret;
  return OK;
}

#endif /* CONFIG_DISABLE_POSIX_TIMERS */
