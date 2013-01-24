/********************************************************************************
 * sched/timer_settime.c
 *
 *   Copyright (C) 2007-2010 Gregory Nutt. All rights reserved.
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
#include <string.h>
#include <errno.h>

#include "os_internal.h"
#include "clock_internal.h"
#include "sig_internal.h"
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
 * Private Function Prototypes
 ********************************************************************************/

static void inline timer_sigqueue(FAR struct posix_timer_s *timer);
static void inline timer_restart(FAR struct posix_timer_s *timer, uint32_t itimer);
static void timer_timeout(int argc, uint32_t itimer);

/********************************************************************************
 * Private Functions
 ********************************************************************************/

/********************************************************************************
 * Name: timer_sigqueue
 *
 * Description:
 *   This function basically reimplements sigqueue() so that the si_code can
 *   be correctly set to SI_TIMER
 *
 * Parameters:
 *   timer - A reference to the POSIX timer that just timed out
 *
 * Return Value:
 *   None
 *
 * Assumptions:
 *   This function executes in the context of the watchod timer interrupt.
 *
 ********************************************************************************/

static void inline timer_sigqueue(FAR struct posix_timer_s *timer)
{
  FAR _TCB *tcb;

  /* Get the TCB of the receiving task */

  tcb = sched_gettcb(timer->pt_owner);
  if (tcb)
    {
       siginfo_t info;

       /* Create the siginfo structure */

       info.si_signo           = timer->pt_signo;
       info.si_code            = SI_TIMER;
#ifdef CONFIG_CAN_PASS_STRUCTS
       info.si_value           = timer->pt_value;
#else
       info.si_value.sival_ptr = timer->pt_value.sival_ptr;
#endif
#ifdef CONFIG_SCHED_HAVE_PARENT
       info.si_pid             = 0;  /* Not applicable */
       info.si_status          = OK;
#endif

       /* Send the signal */

       (void)sig_received(tcb, &info);
    }
}

/********************************************************************************
 * Name: timer_restart
 *
 * Description:
 *   If a periodic timer has been selected, then restart the watchdog.
 *
 * Parameters:
 *   timer - A reference to the POSIX timer that just timed out
 *
 * Return Value:
 *   None
 *
 * Assumptions:
 *   This function executes in the context of the watchod timer interrupt.
 *
 ********************************************************************************/

static void inline timer_restart(FAR struct posix_timer_s *timer, uint32_t itimer)
{
  /* If this is a repetitive timer, then restart the watchdog */

  if (timer->pt_delay)
    {
      timer->pt_last = timer->pt_delay;
      (void)wd_start(timer->pt_wdog, timer->pt_delay, (wdentry_t)timer_timeout,
                     1, itimer);
    }
}

/********************************************************************************
 * Name: timer_timeout
 *
 * Description:
 *   This function is called if the timeout elapses before the condition is
 *   signaled.
 *
 * Parameters:
 *   argc   - the number of arguments (should be 1)
 *   itimer - A reference to the POSIX timer that just timed out
 *   signo  - The signal to use to wake up the task
 *
 * Return Value:
 *   None
 *
 * Assumptions:
 *   This function executes in the context of the watchod timer interrupt.
 *
 ********************************************************************************/

static void timer_timeout(int argc, uint32_t itimer)
{
#ifndef CONFIG_CAN_PASS_STRUCTS
  /* On many small machines, pointers are encoded and cannot be simply cast from
   * uint32_t to _TCB*.  The following union works around this (see wdogparm_t).
   */

  union
  {
    FAR struct posix_timer_s *timer;
    uint32_t                  itimer;
  } u;

  u.itimer = itimer;

  /* Send the specified signal to the specified task.   Increment the reference
   * count on the timer first so that will not be deleted until after the
   * signal handler returns.
   */

  u.timer->pt_crefs++;
  timer_sigqueue(u.timer);

  /* Release the reference.  timer_release will return nonzero if the timer
   * was not deleted.
   */

  if (timer_release(u.timer))
    {
      /* If this is a repetitive timer, the restart the watchdog */

      timer_restart(u.timer, itimer);
    }
#else
  /* (casting to uintptr_t first eliminates complaints on some architectures
   *  where the sizeof uint32_t is different from the size of a pointer).
   */

  FAR struct posix_timer_s *timer = (FAR struct posix_timer_s *)((uintptr_t)itimer);

  /* Send the specified signal to the specified task.   Increment the reference
   * count on the timer first so that will not be deleted until after the
   * signal handler returns.
   */

  timer->pt_crefs++;
  timer_sigqueue(timer);

  /* Release the reference.  timer_release will return nonzero if the timer
   * was not deleted.
   */

  if (timer_release(timer))
    {
      /* If this is a repetitive timer, the restart the watchdog */

      timer_restart(timer, itimer);
    }
#endif
}

/********************************************************************************
 * Public Functions
 ********************************************************************************/

/********************************************************************************
 * Name: timer_settime
 *
 * Description:
 *   The timer_settime() function sets the time until the next expiration of the
 *   timer specified by timerid from the it_value member of the value argument
 *   and arm the timer if the it_value member of value is non-zero. If the
 *   specified timer was already armed when timer_settime() is called, this call
 *   will reset the time until next expiration to the value specified. If the
 *   it_value member of value is zero, the timer will be disarmed. The effect
 *   of disarming or resetting a timer with pending expiration notifications is
 *   unspecified.
 *
 *   If the flag TIMER_ABSTIME is not set in the argument flags, timer_settime()
 *   will behave as if the time until next expiration is set to be equal to the
 *   interval specified by the it_value member of value. That is, the timer will
 *   expire in it_value nanoseconds from when the call is made. If the flag
 *   TIMER_ABSTIME is set in the argument flags, timer_settime() will behave as
 *   if the time until next expiration is set to be equal to the difference between
 *   the absolute time specified by the it_value member of value and the current
 *   value of the clock associated with timerid.  That is, the timer will expire
 *   when the clock reaches the value specified by the it_value member of value.
 *   If the specified time has already passed, the function will succeed and the
 *   expiration notification will be made.
 *
 *   The reload value of the timer will be set to the value specified by the
 *   it_interval member of value.  When a timer is armed with a non-zero
 *   it_interval, a periodic (or repetitive) timer is specified.
 *
 *   Time values that are between two consecutive non-negative integer multiples
 *   of the resolution of the specified timer will be rounded up to the larger
 *   multiple of the resolution. Quantization error will not cause the timer to
 *   expire earlier than the rounded time value.
 *
 *   If the argument ovalue is not NULL, the timer_settime() function will store,
 *   in the location referenced by ovalue, a value representing the previous
 *   amount of time before the timer would have expired, or zero if the timer was
 *   disarmed, together with the previous timer reload value. Timers will not
 *   expire before their scheduled time.
 *
 * Parameters:
 *   timerid - The pre-thread timer, previously created by the call to
 *     timer_create(), to be be set.
 *   flags - Specifie characteristics of the timer (see above)
 *   value - Specifies the timer value to set
 *   ovalue - A location in which to return the time remaining from the previous
 *     timer setting. (ignored)
 *
 * Return Value:
 *   If the timer_settime() succeeds, a value of 0 (OK) will be returned.
 *   If an error occurs, the value -1 (ERROR) will be returned, and errno set to
 *   indicate the error.
 *
 *   EINVAL - The timerid argument does not correspond to an ID returned by
 *     timer_create() but not yet deleted by timer_delete().
 *   EINVAL - A value structure specified a nanosecond value less than zero or
 *     greater than or equal to 1000 million, and the it_value member of that
 *     structure did not specify zero seconds and nanoseconds.
 *
 * Assumptions:
 *
 ********************************************************************************/

int timer_settime(timer_t timerid, int flags, FAR const struct itimerspec *value,
                  FAR struct itimerspec *ovalue)
{
  FAR struct posix_timer_s *timer = (FAR struct posix_timer_s *)timerid;
  irqstate_t state;
  int delay;
  int ret = OK;

  /* Some sanity checks */

  if (!timer || !value)
    {
      errno = EINVAL;
      return ERROR;
    }

  /* Disarm the timer (in case the timer was already armed when timer_settime()
   * is called).
   */

  (void)wd_cancel(timer->pt_wdog);

  /* If the it_value member of value is zero, the timer will not be re-armed */

  if (value->it_value.tv_sec <= 0 && value->it_value.tv_nsec <= 0)
    {
      return OK;
    }

  /* Setup up any repititive timer */

  if (value->it_interval.tv_sec > 0 || value->it_interval.tv_nsec > 0)
    {
       (void)clock_time2ticks(&value->it_interval, &timer->pt_delay);
    }
  else
    {
       timer->pt_delay = 0;
    }

  /* We need to disable timer interrupts through the following section so
   * that the system timer is stable.
   */

  state = irqsave();

  /* Check if abstime is selected */

  if ((flags & TIMER_ABSTIME) != 0)
    {
#ifdef CONFIG_DISABLE_CLOCK
      /* Absolute timing depends upon having access to clock functionality */

      errno = ENOSYS;
      return ERROR;
#else
      /* Calculate a delay corresponding to the absolute time in 'value'.
       * NOTE:  We have internal knowledge the clock_abstime2ticks only
       * returns an error if clockid != CLOCK_REALTIME.
       */

      (void)clock_abstime2ticks(CLOCK_REALTIME, &value->it_value, &delay);
#endif
    }
  else
    {
      /* Calculate a delay assuming that 'value' holds the relative time
       * to wait.  We have internal knowledge that clock_time2ticks always
       * returns success.
       */

      (void)clock_time2ticks(&value->it_value, &delay);
    }

  /* If the time is in the past or now, then set up the next interval
   * instead (assuming a repititive timer).
   */

  if (delay <= 0)
    {
      delay = timer->pt_delay;
    }

  /* Then start the watchdog */


  if (delay > 0)
    {
      timer->pt_last = delay;
      ret = wd_start(timer->pt_wdog, delay, (wdentry_t)timer_timeout,
                     1, (uint32_t)((uintptr_t)timer));
    }

  irqrestore(state);
  return ret;
}

#endif /* CONFIG_DISABLE_POSIX_TIMERS */
