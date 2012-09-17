/****************************************************************************
 * sched/mq_waitirq.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sched.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/mqueue.h>

#include "os_internal.h"

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
 * Name: sem_waitirq
 *
 * Description:
 *   This function is called when a signal or a timeout is received by a
 *   task that is waiting on a message queue -- either for a queue to
 *   becoming not full (on mq_send) or not empty (on mq_receive).
 *
 * Parameters:
 *   wtcb - A pointer to the TCB of the task that is waiting on a message
 *          queue, but has received a signal instead.
 *
 * Return Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

void mq_waitirq(FAR _TCB *wtcb, int errcode)
{
  FAR msgq_t *msgq;
  irqstate_t saved_state;

  /* Disable interrupts.  This is necessary because an interrupt handler may
   * attempt to send a message while we are doing this.
   */

  saved_state = irqsave();

  /* It is possible that an interrupt/context switch beat us to the punch and
   * already changed the task's state.  NOTE:  The operations within the if
   * are safe because interrupts are always disabled with the msgwaitq,
   * nwaitnotempty, and nwaitnotfull fields are modified.
   */

  if (wtcb->task_state == TSTATE_WAIT_MQNOTEMPTY ||
      wtcb->task_state == TSTATE_WAIT_MQNOTFULL)
    {
      /* Get the message queue associated with the waiter from the TCB */

      msgq = wtcb->msgwaitq;
#ifdef CONFIG_DEBUG
      if (!msgq)
        {
          /* In these states there must always be an associated message queue */

          PANIC((uint32_t)OSERR_MQNOWAITER);
        }
#endif
      wtcb->msgwaitq = NULL;

      /* Decrement the count of waiters and cancel the wait */

      if (wtcb->task_state == TSTATE_WAIT_MQNOTEMPTY)
        {
#ifdef CONFIG_DEBUG
          if (msgq->nwaitnotempty <= 0)
            {
              /* This state, there should be a positive, non-zero waiter
               * count.
               */

               PANIC((uint32_t)OSERR_MQNONEMPTYCOUNT);

            }
#endif
          msgq->nwaitnotempty--;
        }
      else
        {
#ifdef CONFIG_DEBUG
          if (msgq->nwaitnotfull <= 0)
            {
              /* This state, there should be a positive, non-zero waiter
               * count.
               */

               PANIC((uint32_t)OSERR_MQNOTFULLCOUNT);

            }
#endif
          msgq->nwaitnotfull--;
        }

      /* Mark the errno value for the thread. */

      wtcb->pterrno = errcode;

      /* Restart the task. */

      up_unblock_task(wtcb);
    }

  /* Interrupts may now be enabled. */

  irqrestore(saved_state);
}

