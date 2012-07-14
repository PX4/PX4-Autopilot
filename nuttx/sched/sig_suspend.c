/****************************************************************************
 * sched/sig_suspend.c
 *
 *   Copyright (C) 2007-2009 Gregory Nutt. All rights reserved.
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

#include <signal.h>
#include <assert.h>
#include <debug.h>
#include <sched.h>

#include <nuttx/arch.h>

#include "os_internal.h"
#include "sig_internal.h"

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
 * Name: sigsuspend
 *
 * Description:
 *
 *   The sigsuspend() function replaces the signal mask of the task with the
 *   set of signals pointed to by the argument 'set' and then suspends the
 *   process until delivery of a signal to the task.
 *
 *   If the effect of the set argument is to unblock a pending signal, then
 *   no wait is performed.
 *
 *   The original signal mask is restored when this function returns.
 *
 *   Waiting for an empty signal set stops a task without freeing any
 *   resources.
 *
 * Parameters:
 *   set - signal mask to use while suspended.
 *
 * Return Value:
 *   -1 (ERROR) always
 *
 * Assumptions:
 *
 * POSIX Compatibility:
 *   int sigsuspend(const sigset_t *set);
 *
 *   POSIX states that sigsuspend() "suspends the process until delivery of
 *   a signal whose action is either to execute a signal-catching function
 *   or to terminate the process."  Only the deliver of a signal is required
 *   in the present implementation (even if the signal is ignored).
 *
 ****************************************************************************/

int sigsuspend(FAR const sigset_t *set)
{
  FAR _TCB       *rtcb = (FAR _TCB*)g_readytorun.head;
  sigset_t        intersection;
  sigset_t        saved_sigprocmask;
  FAR sigpendq_t *sigpend;
  irqstate_t      saved_state;
  int             unblocksigno;

  /* Several operations must be performed below:  We must determine if any
   * signal is pending and, if not, wait for the signal.  Since signals can
   * be posted from the interrupt level, there is a race condition that
   * can only be eliminated by disabling interrupts!
   */

  sched_lock();  /* Not necessary */
  saved_state = irqsave();

  /* Check if there is a pending signal corresponding to one of the
   * signals that will be unblocked by the new sigprocmask.
   */

  intersection = ~(*set) & sig_pendingset(rtcb);
  if (intersection != NULL_SIGNAL_SET)
    {
      /* One or more of the signals in intersections is sufficient to cause
       * us to not wait.  Pick the lowest numbered signal and mark it not
       * pending.
       */

      unblocksigno = sig_lowest(&intersection);
      sigpend = sig_removependingsignal(rtcb, unblocksigno);
      if (!sigpend)
        {
          PANIC(OSERR_FAILEDTOREMOVESIGNAL);
        }

      sig_releasependingsignal(sigpend);
      irqrestore(saved_state);
    }
  else
    {
      /* Its time to wait. Save a copy of the old sigprocmask and install
       * the new (temporary) sigprocmask 
       */

      saved_sigprocmask = rtcb->sigprocmask;
      rtcb->sigprocmask = *set;
      rtcb->sigwaitmask = NULL_SIGNAL_SET;

      /* And wait until one of the unblocked signals is posted */

      up_block_task(rtcb, TSTATE_WAIT_SIG);

      /* We are running again, restore the original sigprocmask */

      rtcb->sigprocmask = saved_sigprocmask;
      irqrestore(saved_state);

      /* Now, handle the (rare?) case where (a) a blocked signal was received
       * while the task was suspended but (b) restoring the original
       * sigprocmask will unblock the signal.
       */

      sig_unmaskpendingsignal();
    }

  sched_unlock();
  return ERROR;
}
