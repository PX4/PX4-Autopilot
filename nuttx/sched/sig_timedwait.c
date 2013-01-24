/****************************************************************************
 * sched/sig_timedwait.c
 *
 *   Copyright (C) 2007-2009, 2012 Gregory Nutt. All rights reserved.
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
#include <nuttx/compiler.h>

#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <time.h>
#include <wdog.h>
#include <assert.h>
#include <debug.h>
#include <sched.h>
#include <errno.h>

#include <nuttx/arch.h>

#include "os_internal.h"
#include "sig_internal.h"
#include "clock_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This is a special value of si_signo that means that it was the timeout
 * that awakened the wait... not the receipt of a signal.
 */

#define SIG_WAIT_TIMEOUT 0xff

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
 * Private Functionss
 ****************************************************************************/

/****************************************************************************
 * Name: sig_timeout
 *
 * Description:
 *  A timeout elapsed while waiting for signals to be queued.
 *
 ****************************************************************************/

static void sig_timeout(int argc, uint32_t itcb)
{
  /* On many small machines, pointers are encoded and cannot be simply cast
   * from uint32_t to _TCB*.  The following union works around this
   * (see wdogparm_t).  This odd logic could be conditioned on
   * CONFIG_CAN_CAST_POINTERS, but it is not too bad in any case.
   */

  union
    {
      FAR _TCB *wtcb;
      uint32_t  itcb;
    } u;

   u.itcb = itcb;

  if (!u.wtcb)
    {
      PANIC(OSERR_TIMEOUTNOTCB);
    }

  /* There may be a race condition -- make sure the task is
   * still waiting for a signal
   */

  if (u.wtcb->task_state == TSTATE_WAIT_SIG)
    {
      u.wtcb->sigunbinfo.si_signo           = SIG_WAIT_TIMEOUT;
      u.wtcb->sigunbinfo.si_code            = SI_TIMER;
      u.wtcb->sigunbinfo.si_value.sival_int = 0;
#ifdef CONFIG_SCHED_HAVE_PARENT
      u.wtcb->sigunbinfo.si_pid             = 0;  /* Not applicable */
      u.wtcb->sigunbinfo.si_status          = OK;
#endif
      up_unblock_task(u.wtcb);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sigtimedwait
 *
 * Description:
 *   This function selects the pending signal set specified by the argument
 *   set.  If multiple signals are pending in set, it will remove and return
 *   the lowest numbered one.  If no signals in set are pending at the time
 *   of the call, the calling process will be suspended until one of the
 *   signals in set becomes pending, OR until the process is interrupted by
 *   an unblocked signal, OR until the time interval specified by timeout
 *   (if any), has expired. If timeout is NULL, then the timeout interval
 *   is forever.
 *
 *   If the info argument is non-NULL, the selected signal number is stored
 *   in the si_signo member and the cause of the signal is store in the
 *   si_code emember.  The content of si_value is only meaningful if the
 *   signal was generated by sigqueue().
 *
 *   The following values for si_code are defined in signal.h:
 *     SI_USER    - Signal sent from kill, raise, or abort
 *     SI_QUEUE   - Signal sent from sigqueue
 *     SI_TIMER   - Signal is result of timer expiration
 *     SI_ASYNCIO - Signal is the result of asynch IO completion 
 *     SI_MESGQ   - Signal generated by arrival of a message on an
 *                  empty message queue.
 *
 * Parameters:
 *   set - The pending signal set.
 *   info - The returned value
 *   timeout - The amount of time to wait
 *
 * Return Value:
 *   Signal number that cause the wait to be terminated, otherwise -1 (ERROR)
 *   is returned with errno set to either:
 *
 *   EAGAIN - No signal specified by set was generated within the specified
 *            timeout period.
 *   EINTR  - The wait was interrupted by an unblocked, caught signal.
 *
 * Assumptions:
 *
 ****************************************************************************/

int sigtimedwait(FAR const sigset_t *set, FAR struct siginfo *info,
                 FAR const struct timespec *timeout)
{
  FAR _TCB       *rtcb = (FAR _TCB*)g_readytorun.head;
  sigset_t        intersection;
  FAR sigpendq_t *sigpend;
  WDOG_ID         wdog;
  irqstate_t      saved_state;
  int32_t         waitticks;
  int             ret = ERROR;

  sched_lock();  /* Not necessary */

  /* Several operations must be performed below:  We must determine if any
   * signal is pending and, if not, wait for the signal.  Since signals can
   * be posted from the interrupt level, there is a race condition that
   * can only be eliminated by disabling interrupts!
   */

  saved_state = irqsave();

  /* Check if there is a pending signal corresponding to one of the
   * signals in the pending signal set argument.
   */

  intersection = *set & sig_pendingset(rtcb);
  if (intersection != NULL_SIGNAL_SET)
    {
      /* One or more of the signals in intersections is sufficient to cause
       * us to not wait.  Pick the lowest numbered signal and mark it not
       * pending.
       */

      sigpend = sig_removependingsignal(rtcb, sig_lowest(&intersection));
      if (!sigpend)
        {
          PANIC(OSERR_NOPENDINGSIGNAL);
        }

      /* Return the signal info to the caller if so requested */

      if (info)
        {
          memcpy(info, &sigpend->info, sizeof(struct siginfo));
        }

      /* Then dispose of the pending signal structure properly */

      sig_releasependingsignal(sigpend);
      irqrestore(saved_state);

      /* The return value is the number of the signal that awakened us */

      ret = sigpend->info.si_signo;
    }

  /* We will have to wait for a signal to be posted to this task. */

  else
    {
      /* Save the set of pending signals to wait for */

      rtcb->sigwaitmask = *set;

      /* Check if we should wait for the timeout */

      if (timeout)
        {
          /* Convert the timespec to system clock ticks, making sure that
           * the resultint delay is greater than or equal to the requested
           * time in nanoseconds.
           */

#ifdef CONFIG_HAVE_LONG_LONG
          uint64_t waitticks64 = ((uint64_t)timeout->tv_sec * NSEC_PER_SEC +
                                  (uint64_t)timeout->tv_nsec + NSEC_PER_TICK - 1) /
                                  NSEC_PER_TICK;
          DEBUGASSERT(waitticks64 <= UINT32_MAX);
          waitticks = (uint32_t)waitticks64;
#else
          uint32_t waitmsec;

          DEBUGASSERT(timeout->tv_sec < UINT32_MAX / MSEC_PER_SEC);
          waitmsec = timeout->tv_sec * MSEC_PER_SEC +
                    (timeout->tv_nsec + NSEC_PER_MSEC - 1) / NSEC_PER_MSEC;
          waitticks = (waitmsec + MSEC_PER_TICK - 1) / MSEC_PER_TICK;
#endif

          /* Create a watchdog */

          wdog = wd_create();
          if (wdog)
            {
              /* This little of nonsense is necessary for some
               * processors where sizeof(pointer) < sizeof(uint32_t).
               * see wdog.h.
               */

              wdparm_t wdparm;
              wdparm.pvarg = (FAR void*)rtcb;

              /* Start the watchdog */

              wd_start(wdog, waitticks, (wdentry_t)sig_timeout, 1, wdparm.dwarg);

              /* Now wait for either the signal or the watchdog */

              up_block_task(rtcb, TSTATE_WAIT_SIG);

              /* We no longer need the watchdog */

              wd_delete(wdog);
            }
        }

      /* No timeout, just wait */

      else
        {
          /* And wait until one of the unblocked signals is posted */

          up_block_task(rtcb, TSTATE_WAIT_SIG);
        }

      /* We are running again, clear the sigwaitmask */

      rtcb->sigwaitmask = NULL_SIGNAL_SET;

      /* When we awaken, the cause will be in the TCB.  Get the signal number
       * or timeout) that awakened us.
       */

      if (GOOD_SIGNO(rtcb->sigunbinfo.si_signo))
        {
          /* We were awakened by a signal... but is it one of the signals that
           * we were waiting for?
           */
 
          if (sigismember(set, rtcb->sigunbinfo.si_signo))
            {
              /* Yes.. the return value is the number of the signal that
               * awakened us.
               */

              ret = rtcb->sigunbinfo.si_signo;
            }
          else
            {
              /* No... then set EINTR and report an error */

              set_errno(EINTR);
              ret = ERROR;
            }
        }
      else
        {
          /* Otherwise, we must have been awakened by the timeout.  Set EGAIN
           * and return an error.
           */

          DEBUGASSERT(rtcb->sigunbinfo.si_signo == SIG_WAIT_TIMEOUT);
          set_errno(EAGAIN);
          ret = ERROR;
        }
      
      /* Return the signal info to the caller if so requested */

      if (info)
        {
          memcpy(info, &rtcb->sigunbinfo, sizeof(struct siginfo));
        }

      irqrestore(saved_state);
   }

   sched_unlock();
   return ret;
}
