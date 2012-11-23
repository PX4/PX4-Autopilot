/****************************************************************************
 * sched/usleep.c
 *
 *   Copyright (C) 2007, 2009, 2012 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <unistd.h>
#include <signal.h>
#include <assert.h>
#include <errno.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usleep
 *
 * Description:
 *   The usleep() function will cause the calling thread to be suspended
 *   from execution until either the number of real-time microseconds
 *   specified by the argument 'usec' has elapsed or a signal is delivered
 *   to the calling thread. The suspension time may be longer than requested
 *   due to the scheduling of other activity by the system.
 *
 *   The 'usec' argument must be less than 1,000,000. If the value of
 *   'usec' is 0, then the call has no effect.
 *
 *   If a SIGALRM signal is generated for the calling process during
 *   execution of usleep() and if the SIGALRM signal is being ignored or
 *   blocked from delivery, it is unspecified whether usleep() returns
 *   when the SIGALRM signal is scheduled. If the signal is being blocked, it
 *   is also unspecified whether it remains pending after usleep() returns or
 *   it is discarded.
 *
 *   If a SIGALRM signal is generated for the calling process during
 *   execution of usleep(), except as a result of a prior call to alarm(),
 *   and if the SIGALRM signal is not being ignored or blocked from delivery,
 *   it is unspecified whether that signal has any effect other than causing
 *   usleep() to return.
 *
 *   If a signal-catching function interrupts usleep() and examines or changes
 *   either the time a SIGALRM is scheduled to be generated, the action
 *   associated with the SIGALRM signal, or whether the SIGALRM signal is
 *   blocked from delivery, the results are unspecified.
 *
 *   If a signal-catching function interrupts usleep() and calls siglongjmp()
 *   or longjmp() to restore an environment saved prior to the usleep() call,
 *   the action associated with the SIGALRM signal and the time at which a
 *   SIGALRM signal is scheduled to be generated are unspecified. It is also
 *   unspecified whether the SIGALRM signal is blocked, unless the process'
 *   signal mask is restored as part of the environment.
 *
 *   Implementations may place limitations on the granularity of timer values.
 *   For each interval timer, if the requested timer value requires a finer
 *   granularity than the implementation supports, the actual timer value will
 *   be rounded up to the next supported value. 
 *
 *   Interactions between usleep() and any of the following are unspecified:
 *
 *   nanosleep(), setitimer(), timer_create(), timer_delete(), timer_getoverrun(),
 *   timer_gettime(), timer_settime(), ualarm(), sleep()

 * Parameters:
 *   usec - the number of microseconds to wait.
 *
 * Returned Value:
 *   On successful completion, usleep() returns 0. Otherwise, it returns -1
 *   and sets errno to indicate the error. 
 *
 * Assumptions:
 *
 ****************************************************************************/

int usleep(useconds_t usec)
{
  sigset_t set;
  struct timespec ts;
  struct siginfo value;
  int errval;
  int ret = 0;

  if (usec)
    {
      /* Set up for the sleep.  Using the empty set means that we are not
       * waiting for any particular signal.  However, any unmasked signal
       * can still awaken sigtimedwait().
       */

      (void)sigemptyset(&set);
      ts.tv_sec  = usec / 1000000;
      ts.tv_nsec = (usec % 1000000) * 1000;

      /* usleep is a simple application of sigtimedwait. */

      ret = sigtimedwait(&set, &value, &ts);

      /* sigtimedwait() cannot succeed.  It should always return error with
       * either (1) EAGAIN meaning that the timeout occurred, or (2) EINTR
       * meaning that some other unblocked signal was caught.
       */

      errval = errno;
      DEBUGASSERT(ret < 0 && (errval == EAGAIN || errval == EINTR));
      if (errval == EAGAIN)
        {
          /* The timeout "error" is the normal, successful result */

          ret = 0;
        }
    }

  return ret;
}
