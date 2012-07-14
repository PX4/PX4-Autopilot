/****************************************************************************
 * sched/sig_action.c
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

#include <stdint.h>
#include <signal.h>
#include <queue.h>
#include <sched.h>

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
 * - Special values of sa_handler in the struct sigaction
 *   act input not handled (SIG_DFL, SIG_IGN).
 * - All sa_flags in struct sigaction of act input are
 *   ignored (all treated like SA_SIGINFO).
 *
 ****************************************************************************/

int sigaction(int signo, FAR const struct sigaction *act, FAR struct sigaction *oact)
{
  FAR _TCB      *rtcb = (FAR _TCB*)g_readytorun.head;
  FAR sigactq_t *sigact;
  int            ret = ERROR;  /* Assume failure */

  /* Since sigactions can only be installed from the running thread of
   * execution, no special precautions should be necessary.
   */

  /* Verify the signal */

  if (GOOD_SIGNO(signo))
    {
      ret = OK;  /* Assume success */

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

      /* If no sigaction was found, but one is needed, then
       * allocate one.
       */

      if (!sigact && act && act->sa_u._sa_handler)
        {
          sigact = sig_allocateaction();

          /* An error has occurred if we could not allocate the sigaction */

          if (!sigact)
            {
              ret = ERROR;
            }
          else
            {
              /* Put the signal number in the queue entry */

              sigact->signo = (uint8_t)signo;

              /* Add the new sigaction to sigactionq */

              sq_addlast((FAR sq_entry_t*)sigact, &rtcb->sigactionq);
            }
        }

      /* Set the new sigaction if so requested */

      if ((sigact) && (act))
        {
          /* Check if it is a request to install a new handler */

          if (act->sa_u._sa_handler)
            {
              COPY_SIGACTION(&sigact->act, act);
            }

          /* No.. It is a request to remove the old handler */

          else
            {
              /* Remove the old sigaction from sigactionq */

              sq_rem((FAR sq_entry_t*)sigact, &rtcb->sigactionq);

              /* And deallocate it */

              sig_releaseaction(sigact);
            }
        }
    }

  return ret;
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
