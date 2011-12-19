/************************************************************************
 * sched/sig_received.c
 *
 *   Copyright (C) 2007, 2009, 2011 Gregory Nutt. All rights reserved.
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
 ************************************************************************/

/************************************************************************
 * Included Files
 ************************************************************************/

#include <nuttx/config.h>

#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <sched.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>

#include "os_internal.h"
#include "sem_internal.h"
#include "sig_internal.h"
#include "mq_internal.h"

/************************************************************************
 * Definitions
 ************************************************************************/

/************************************************************************
 * Private Type Declarations
 ************************************************************************/

/************************************************************************
 * Global Variables
 ************************************************************************/

/************************************************************************
 * Private Variables
 ************************************************************************/

/************************************************************************
 * Private Functions
 ************************************************************************/

/************************************************************************
 * Function:  sig_queueaction
 *
 * Description:
 *   Queue a signal action for delivery to a task.
 *
 ************************************************************************/

static int sig_queueaction(FAR _TCB *stcb, siginfo_t *info)
{
  FAR sigactq_t *sigact;
  FAR sigq_t    *sigq;
  irqstate_t     saved_state;
  int            ret = OK;

  sched_lock();

  /* Find the sigaction associated with this signal */

  sigact = sig_findaction(stcb, info->si_signo);

  /* Check if a valid signal handler is available and if the signal is
   * unblocked.  NOTE:  There is no default action.
   */

  if ((sigact) && (sigact->act.sa_u._sa_sigaction))
    {
      /* Allocate a new element for the signal queue.  NOTE: sig_allocatependingsigaction
       * will force a system crash if it is unable to allocate memory for the
       * signal data */

      sigq = sig_allocatependingsigaction();
      if (!sigq) ret = ERROR;
      else
        {
          /* Populate the new signal queue element */

           sigq->action.sighandler = sigact->act.sa_u._sa_sigaction;
           sigq->mask = sigact->act.sa_mask;
           memcpy(&sigq->info, info, sizeof(siginfo_t));

           /* Put it at the end of the pending signals list */

           saved_state = irqsave();
           sq_addlast((FAR sq_entry_t*)sigq, &(stcb->sigpendactionq));
           irqrestore(saved_state);
        }
    }

  sched_unlock();
  return ret;
}

/************************************************************************
 * Function: sig_findpendingsignal
 *
 * Description:
 *   Find a specified element in the pending signal list
 *
 ************************************************************************/

static FAR sigpendq_t *sig_findpendingsignal(FAR _TCB *stcb, int signo)
{
  FAR sigpendq_t *sigpend = NULL;
  irqstate_t      saved_state;

  /* Verify the caller's sanity */

  if (stcb)
    {
      /* Pending sigals can be added from interrupt level. */

      saved_state = irqsave();

      /* Seach the list for a sigpendion on this signal */

      for(sigpend = (FAR sigpendq_t*)stcb->sigpendingq.head;
         (sigpend && sigpend->info.si_signo != signo);
         sigpend = sigpend->flink);
      irqrestore(saved_state);
   }

  return sigpend;
}

/************************************************************************
 * Function: sig_allocatependingsignal
 *
 * Description:
 *   Allocate a pending signal list entry
 *
 ************************************************************************/

static FAR sigpendq_t *sig_allocatependingsignal(void)
{
  FAR sigpendq_t *sigpend;
  irqstate_t      saved_state;

  /* Check if we were called from an interrupt handler. */

  if (up_interrupt_context())
    {
      /* Try to get the pending signal structure from the free list */

      sigpend = (FAR sigpendq_t*)sq_remfirst(&g_sigpendingsignal);
      if (!sigpend)
        {
          /* If no pending signal structure is available in the free list,
           * then try the special list of structures reserved for
           * interrupt handlers
           */

          sigpend = (FAR sigpendq_t*)sq_remfirst(&g_sigpendingirqsignal);
        }
    }

  /* If we were not called from an interrupt handler, then we are
   * free to allocate pending action structures if necessary. */

  else
    {
      /* Try to get the pending signal structure from the free list */

      saved_state = irqsave();
      sigpend = (FAR sigpendq_t*)sq_remfirst(&g_sigpendingsignal);
      irqrestore(saved_state);

      /* Check if we got one. */

      if (!sigpend)
        {
          /* No... Allocate the pending signal */

          if (!sigpend)
            {
              sigpend = (FAR sigpendq_t *)kmalloc((sizeof (sigpendq_t)));
            }

          /* Check if we got an allocated message */

          if (sigpend)
            {
              sigpend->type = SIG_ALLOC_DYN;
            }
        }
    }

  return sigpend;
}

/************************************************************************
 * Function:  sig_addpendingsignal
 *
 * Description:
 * Add the specified signal to the signal pending list.
 * NOTE:  This function will queue only one entry for each
 * pending signal.  This was done intentionally so that a
 * run-away sender cannot consume all of memory.
 ************************************************************************/

static FAR sigpendq_t *sig_addpendingsignal(FAR _TCB *stcb, siginfo_t *info)
{
  FAR sigpendq_t *sigpend;
  irqstate_t      saved_state;

  /* Check if the signal is already pending */

  sigpend = sig_findpendingsignal(stcb, info->si_signo);
  if (sigpend)
    {
      /* The signal is already pending... retain only one copy */

      memcpy(&sigpend->info, info, sizeof(siginfo_t));
    }

  /* No... There is nothing pending for this signo */

  else
    {
      /* Allocate a new pending signal entry */

      sigpend = sig_allocatependingsignal();
      if (sigpend)
        {
          /* Put the signal information into the allocated structure */

          memcpy(&sigpend->info, info, sizeof(siginfo_t));

          /* Add the structure to the pending signal list */

          saved_state = irqsave();
          sq_addlast((FAR sq_entry_t*)sigpend, &stcb->sigpendingq);
          irqrestore(saved_state);
        }
    }

  return sigpend;
}

/************************************************************************
 * Public Functions
 ************************************************************************/

/************************************************************************
 * Function: sig_received
 *
 * Description:
 *   All signals received the task (whatever the source) go
 *   through this function to be processed.  This function
 *   is responsible for:
 *
 *   - Determining if the signal is blocked.
 *   - Queuing and dispatching signal actions
 *   - Unblocking tasks that are waiting for signals
 *   - Queuing pending signals.
 *
 ************************************************************************/

int sig_received(FAR _TCB *stcb, siginfo_t *info)
{
  irqstate_t saved_state;
  int        ret = ERROR;

  sdbg("TCB=0x%08x signo=%d code=%d value=%d mask=%08x\n",
       stcb, info->si_signo, info->si_code,
       info->si_value.sival_int, stcb->sigprocmask);

  if (stcb && info)
    {
      ret = OK;

      /****************** MASKED SIGNAL HANDLING ******************/

      /* Check if the signal is masked -- if it is, it will be added to the
       * list of pending signals.
       */

      if (sigismember(&stcb->sigprocmask, info->si_signo))
        {
          /* Check if the task is waiting for this pending signal.  If so,
           * then unblock it. This must be performed in a critical section
           * because signals can be queued from the interrupt level.
           */

          saved_state = irqsave();
          if (stcb->task_state == TSTATE_WAIT_SIG &&
              sigismember(&stcb->sigwaitmask, info->si_signo))
            {
              memcpy(&stcb->sigunbinfo, info, sizeof(siginfo_t));
              stcb->sigwaitmask = NULL_SIGNAL_SET;
              up_unblock_task(stcb);
              irqrestore(saved_state);
            }

          /* Its not one we are waiting for... Add it to the list of pending
           * signals.
           */

          else
            {
              irqrestore(saved_state);
              if (!sig_addpendingsignal(stcb, info))
                {
                  PANIC(OSERR_FAILEDTOADDSIGNAL);
                }
            }
        }

      /****************** UNMASKED SIGNAL HANDLING ******************/

      else
        {
          /* Queue any sigaction's requested by this task. */

          ret = sig_queueaction(stcb, info);

          /* Then schedule execution of the signal handling action on
           * the recipients thread.
           */

          up_schedule_sigaction(stcb, sig_deliver);

          /* Check if the task is waiting for an unmasked signal.  If so,
           * then unblock it. This must be performed in a critical section
           * because signals can be queued from the interrupt level.
           */

          saved_state = irqsave();
          if (stcb->task_state == TSTATE_WAIT_SIG)
            {
              memcpy(&stcb->sigunbinfo, info, sizeof(siginfo_t));
              stcb->sigwaitmask = NULL_SIGNAL_SET;
              up_unblock_task(stcb);
            }
          irqrestore(saved_state);

          /* If the task neither was waiting for the signal nor had a signal
           * handler attached to the signal, then the default action is
           * simply to ignore the signal
           */

          /****************** OTHER SIGNAL HANDLING ******************/

         /* If the task is blocked waiting for a semaphore, then that
          * task must be unblocked when a signal is received.
          */

         if (stcb->task_state == TSTATE_WAIT_SEM)
           {
             sem_waitirq(stcb, EINTR);
           }

         /* If the task is blocked waiting on a message queue, then that
          * task must be unblocked when a signal is received.
          */

#ifndef CONFIG_DISABLE_MQUEUE
        if (stcb->task_state == TSTATE_WAIT_MQNOTEMPTY ||
            stcb->task_state == TSTATE_WAIT_MQNOTFULL)
           {
             mq_waitirq(stcb, EINTR);
           }
#endif
       }
   }

  return ret;
}
