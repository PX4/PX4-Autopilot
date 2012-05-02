/****************************************************************************
 * sched/sig_deliver.c
 *
 *   Copyright (C) 2007, 2008, 2012 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <signal.h>
#include <unistd.h>
#include <sched.h>
#include <debug.h>
#include <nuttx/arch.h>

#include "os_internal.h"
#include "sem_internal.h"
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
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: sig_deliver
 *
 * Description:
 *   This function is called on the thread of execution of
 *   the signal receiving task.  It processes all queued
 *   signals then returns.
 *
 ****************************************************************************/

void sig_deliver(FAR _TCB *stcb)
{
  FAR sigq_t *sigq;
  FAR sigq_t *next;
  sigset_t    savesigprocmask;
  irqstate_t  saved_state;
  int         saved_errno;

  sched_lock();

  /* Save the thread errno.  When we finished dispatching the
   * signal actions and resume the task, the errno value must
   * be unchanged by the operation of the signal handling.  In
   * particular, the EINTR indication that says that the task
   * was reawakened by a signal must be retained.
   */

  saved_errno = stcb->pterrno;
  for (sigq = (FAR sigq_t*)stcb->sigpendactionq.head; (sigq); sigq = next)
    {
      next = sigq->flink;
      sdbg("Sending signal sigq=0x%x\n", sigq);

      /* Remove the signal structure from the sigpendactionq and place it
       * in the sigpostedq.  NOTE:  Since signals are processed one at a
       * time, there should never be more than one signal in the sigpostedq
       */

      saved_state = irqsave();
      sq_rem((FAR sq_entry_t*)sigq, &(stcb->sigpendactionq));
      sq_addlast((FAR sq_entry_t*)sigq, &(stcb->sigpostedq));
      irqrestore(saved_state);

      /* Call the signal handler (unless the signal was cancelled)
       *
       * Save a copy of the old sigprocmask and install the new
       * (temporary) sigprocmask.  The new sigprocmask is the union
       * of the current sigprocmask and the sa_mask for the signal being
       * delivered plus the signal being delivered.
       */

      savesigprocmask = stcb->sigprocmask;
      stcb->sigprocmask = savesigprocmask | sigq->mask | SIGNO2SET(sigq->info.si_signo);

      /* Deliver the signal */

      (*sigq->action.sighandler)(sigq->info.si_signo, &sigq->info, NULL);

      /* Restore the original sigprocmask */

      stcb->sigprocmask = savesigprocmask;

      /* Now, handle the (rare?) case where (a) a blocked signal was
       * received while the signal handling executed but (b) restoring the
       * original sigprocmask will unblock the signal.
       */

      sig_unmaskpendingsignal();

      /* Remove the signal from the sigpostedq */

      saved_state = irqsave();
      sq_rem((FAR sq_entry_t*)sigq, &(stcb->sigpostedq));
      irqrestore(saved_state);

      /* Then deallocate it */

      sig_releasependingsigaction(sigq);
   }

  stcb->pterrno = saved_errno;
  sched_unlock();
}

