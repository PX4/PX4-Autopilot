/****************************************************************************
 * sched/sig_action.c
 *
 *   Copyright (C) 2007-2009, 2013 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <signal.h>
#include <queue.h>
#include <sched.h>
#include <errno.h>

#include "os_internal.h"
#include "sig_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define COPY_SIGACTION(t,f) \
  { (t)->sa_sigaction = (f)->sa_sigaction; \
    (t)->sa_mask      = (f)->sa_mask; \
    (t)->sa_flags     = (f)->sa_flags; }

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
 * Name: sig_allocateaction
 *
 * Description:
 *   Allocate a new element for a sigaction queue
 *
 ****************************************************************************/

static FAR sigactq_t *sig_allocateaction(void)
{
  FAR sigactq_t *sigact;

  /* Try to get the signal action structure from the free list */

  sigact = (FAR sigactq_t*)sq_remfirst(&g_sigfreeaction);

  /* Check if we got one. */

  if (!sigact)
    {
      /* Add another block of signal actions to the list */

      sig_allocateactionblock();

      /* And try again */

      sigact = (FAR sigactq_t*)sq_remfirst(&g_sigfreeaction);
      if (!sigact)
        {
          PANIC(OSERR_OUTOFMEMORY);
        }
    }

  return sigact;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sigaction
 *
 * Description:
 *   This function allows the calling process to examine and/or specify the
 *   action to be associated with a specific signal.
 *
 *   The structure sigaction, used to describe an action to be taken, is
 *   defined to include the following members:
 *
 *   - sa_u.sa_handler:  Pointer to a signal-catching function
 *   - sa_u.sa_sigaction:  Alternative form of the signal-catching function
 *   - sa_mask: An additional set of signals to be blocked during execution
 *       of a signal catching function
 *   - sa_flags.  Special flags to affect the behavior of a signal.
 *
 *   If the argument 'act' is not NULL, it points to a structure specifying
 *   the action to be associated with the specified signal.  If the argument
 *   'oact' is not NULL, the action previously associated with the signal
 *   is stored in the location pointed to by the argument 'oact.'
 *
 *   When a signal is caught by a signal-catching function installed by
 *   sigaction() function, a new signal mask is calculated and installed for
 *   the duration of the signal-catching function.  This mask is formed by
 *   taking the union of the current signal mask and the value of the
 *   sa_mask for the signal being delivered and then including the signal
 *   being delivered.  If and when the user's signal handler returns, the
 *   original signal mask is restored.
 *
 *   Once an action is installed for a specific signal, it remains installed
 *   until another action is explicitly requested by another call to sigaction().
 *
 * Parameters:
 *   sig - Signal of interest
 *   act - Location of new handler
 *   oact - Location to store only handler
 *
 * Return Value:
 *   0 (OK), or -1 (ERROR) if the signal number is invalid.
 *   (errno is not set)
 *
 * Assumptions:
 *
 * POSIX Compatibility:
 * - There are no default actions so the special value SIG_DFL is treated
 *   like SIG_IGN.
 * - All sa_flags in struct sigaction of act input are ignored (all
 *   treated like SA_SIGINFO). The one exception is if CONFIG_SCHED_CHILDSTATUS
 *   is defined; then SA_NOCLDWAIT is supported but only for SIGCHLD
 *
 ****************************************************************************/

int sigaction(int signo, FAR const struct sigaction *act, FAR struct sigaction *oact)
{
  FAR _TCB      *rtcb = (FAR _TCB*)g_readytorun.head;
  FAR sigactq_t *sigact;

  /* Since sigactions can only be installed from the running thread of
   * execution, no special precautions should be necessary.
   */

  /* Verify the signal number */

  if (!GOOD_SIGNO(signo))
    {
      set_errno(EINVAL);
      return ERROR;
    }

  /* Find the signal in the sigactionq */

  sigact = sig_findaction(rtcb, signo);

  /* Return the old sigaction value if so requested */

  if (oact)
    {
      if (sigact)
        {
          COPY_SIGACTION(oact, &sigact->act);
        }
      else
        {
          /* There isn't an old value */

          oact->sa_u._sa_handler = NULL;
          oact->sa_mask = NULL_SIGNAL_SET;
          oact->sa_flags = 0;
        }
    }

  /* If the argument act is a null pointer, signal handling is unchanged;
   * thus, the call can be used to enquire about the current handling of
   * a given signal. 
   */

  if (!act)
    {
      return OK;
    }

#if defined(CONFIG_SCHED_HAVE_PARENT) && defined(CONFIG_SCHED_CHILD_STATUS)

  /* Handle a special case.  Retention of child status can be suppressed
   * if signo == SIGCHLD and sa_flags == SA_NOCLDWAIT.
   *
   * POSIX.1 leaves it unspecified whether a SIGCHLD signal is generated
   * when a child process terminates.  In NuttX, a SIGCHLD signal is
   * generated in this case; but in some other implementations, it may not
   * be.
   */

  if (signo == SIGCHLD && (act->sa_flags & SA_NOCLDWAIT) != 0)
    {
      irqstate_t flags;

      /* We do require a critical section to muck with the TCB values that
       * can be modified by the child thread.
       */

      flags = irqsave();

      /* Mark that status should be not be retained */

      rtcb->group->tg_flags |= GROUP_FLAG_NOCLDWAIT;

      /* Free all pending exit status */

      task_removechildren(rtcb);
      irqrestore(flags);
    }
#endif

  /* Handle the case where no sigaction is supplied (SIG_IGN) */

  if (act->sa_u._sa_handler == SIG_IGN)
    {
      /* Do we still have a sigaction container from the previous setting? */

      if (sigact)
        {
          /* Yes.. Remove it from sigactionq */

          sq_rem((FAR sq_entry_t*)sigact, &rtcb->sigactionq);

          /* And deallocate it */

          sig_releaseaction(sigact);
        }
    }

  /* A sigaction has been supplied */

  else
    {
      /* Do we still have a sigaction container from the previous setting?
       * If so, then re-use for the new signal action.
       */

      if (!sigact)
        {
          /* No.. Then we need to allocate one for the new action. */

          sigact = sig_allocateaction();

          /* An error has occurred if we could not allocate the sigaction */

          if (!sigact)
           {
              set_errno(ENOMEM);
              return ERROR;
           }

          /* Put the signal number in the queue entry */

          sigact->signo = (uint8_t)signo;

          /* Add the new sigaction to sigactionq */

          sq_addlast((FAR sq_entry_t*)sigact, &rtcb->sigactionq);
        }

      /* Set the new sigaction */

      COPY_SIGACTION(&sigact->act, act);
    }

  return OK;
}

/****************************************************************************
 * Name: sig_releaseaction
 *
 * Description:
 *   Deallocate a sigaction Q entry
 *
 ****************************************************************************/

void sig_releaseaction(FAR sigactq_t *sigact)
{
  /* Just put it back on the free list */

  sq_addlast((FAR sq_entry_t*)sigact, &g_sigfreeaction);
}
