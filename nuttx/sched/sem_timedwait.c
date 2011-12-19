/****************************************************************************
 * sched/sem_timedwait.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include <stdint.h>
#include <unistd.h>
#include <semaphore.h>
#include <time.h>
#include <errno.h>
#include <wdog.h>
#include <debug.h>

#include <nuttx/arch.h>

#include "os_internal.h"
#include "clock_internal.h"
#include "sem_internal.h"

/****************************************************************************
 * Pre-processor Definitions
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
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  sem_timeout
 *
 * Description:
 *   This function is called if the timeout elapses before the message queue
 *   becomes non-empty.
 *
 * Parameters:
 *   argc  - the number of arguments (should be 1)
 *   pid   - the task ID of the task to wakeup
 *
 * Return Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void sem_timeout(int argc, uint32_t pid)
{
  FAR _TCB *wtcb;
  irqstate_t flags;

  /* Disable interrupts to avoid race conditions */

  flags = irqsave();

  /* Get the TCB associated with this pid.  It is possible that
   * task may no longer be active when this watchdog goes off.
   */

  wtcb = sched_gettcb((pid_t)pid);

  /* It is also possible that an interrupt/context switch beat us to the
   * punch and already changed the task's state.
   */

  if (wtcb && wtcb->task_state == TSTATE_WAIT_SEM)
    {
      /* Cancel the semaphore wait */
 
      sem_waitirq(wtcb, ETIMEDOUT);
    }

  /* Interrupts may now be enabled. */

  irqrestore(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  sem_timedwait
 *
 * Description:
 *   This function will lock the semaphore referenced by sem as in the
 *   sem_wait() function. However, if the semaphore cannot be locked without
 *   waiting for another process or thread to unlock the semaphore by
 *   performing a sem_post() function, this wait will be terminated when the
 *   specified timeout expires.
 *
 *   The timeout will expire when the absolute time specified by abstime
 *   passes, as measured by the clock on which timeouts are based (that is,
 *   when the value of that clock equals or exceeds abstime), or if the
 *   absolute time specified by abstime has already been passed at the
 *   time of the call.
 *
 * Parameters:
 *   sem - Semaphore object
 *   abstime - The absolute time to wait until a timeout is declared.
 *
 * Return Value:
 *   One success, the length of the selected message in bytes.is
 *   returned.  On failure, -1 (ERROR) is returned and the errno
 *   is set appropriately:
 *
 *   EINVAL    The sem argument does not refer to a valid semaphore.  Or the
 *             thread would have blocked, and the abstime parameter specified
 *             a nanoseconds field value less than zero or greater than or
 *             equal to 1000 million.
 *   ETIMEDOUT The semaphore could not be locked before the specified timeout
 *             expired.
 *   EDEADLK   A deadlock condition was detected.
 *   EINTR     A signal interrupted this function.
 *
 ****************************************************************************/

int sem_timedwait(FAR sem_t *sem, FAR const struct timespec *abstime)
{
  WDOG_ID    wdog;
  irqstate_t flags;
  int        ticks;
  int        err;
  int        ret = ERROR;

  DEBUGASSERT(up_interrupt_context() == false);

  /* Verify the input parameters and, in case of an error, set
   * errno appropriately.
   */

#ifdef CONFIG_DEBUG
  if (!abstime || !sem)
    {
      err = EINVAL;
      goto errout;
    }
#endif

  /* Create a watchdog.  We will not actually need this watchdog
   * unless the the semaphore is unavailable, but we will reserve it up
   * front before we enter the following critical section.
   */

  wdog = wd_create();
  if (!wdog)
    {
      err = ENOMEM;
      goto errout;
    }

  /* We will disable interrupts until we have completed the semaphore
   * wait.  We need to do this (as opposed to just disabling pre-emption)
   * because there could be interrupt handlers that are asynchronoulsy
   * posting semaphores and to prevent race conditions with watchdog
   * timeout.  This is not too bad because interrupts will be re-
   * enabled while we are blocked waiting for the semaphore.
   */

  flags = irqsave();

  /* Try to take the semaphore without waiting. */

  ret = sem_trywait(sem);
  if (ret == 0)
    {
      /* We got it! */

      irqrestore(flags);
      wd_delete(wdog);
      return OK;
    }

  /* We will have to wait for the semphore.  Make sure that we were provided
   * with a valid timeout.
   */

  if (abstime->tv_sec < 0 || abstime->tv_nsec > 1000000000)
    {
      err = EINVAL;
      goto errout_disabled;
    }

  /* Convert the timespec to clock ticks.  We must have interrupts
   * disabled here so that this time stays valid until the wait begins.
   */

  err = clock_abstime2ticks(CLOCK_REALTIME, abstime, &ticks);

  /* If the time has already expired return immediately. */

  if (err == OK && ticks <= 0)
    {
      err = ETIMEDOUT;
      goto errout_disabled;
    }

  /* Handle any time-related errors */

  if (err != OK)
    {
      goto errout_disabled;
    }

  /* Start the watchdog */

  err = OK; 
  wd_start(wdog, ticks, (wdentry_t)sem_timeout, 1, getpid());

  /* Now perform the blocking wait */

  ret = sem_wait(sem);

  /* Stop the watchdog timer */

  wd_cancel(wdog);

  /* We can now restore interrupts and delete the watchdog */

  irqrestore(flags);
  wd_delete(wdog);

  /* We are either returning success or an error detected by sem_wait()
   * or the timeout detected by sem_timeout().  The 'errno' value has
   * been set appropriately by sem_wait() or sem_timeout() in those
   * cases.
   */

  return ret;

/* Error exits */

errout_disabled:
  irqrestore(flags);
  wd_delete(wdog);
errout:
  set_errno(err);
  return ERROR;
}
