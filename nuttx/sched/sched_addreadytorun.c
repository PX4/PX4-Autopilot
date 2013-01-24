/****************************************************************************
 * sched/sched_addreadytorun.c
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

#include <stdbool.h>
#include <queue.h>
#include <assert.h>

#include "os_internal.h"

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
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  sched_addreadytorun
 *
 * Description:
 *   This function adds a TCB to the ready to run
 *   list.  If the currently active task has preemption disabled
 *   and the new TCB would cause this task to be preempted, the
 *   new task is added to the g_pendingtasks list instead.  The
 *   pending tasks will be made ready-to-run when preemption
 *   is unlocked.
 *
 * Inputs:
 *   btcb - Points to the blocked TCB that is ready-to-run
 *
 * Return Value:
 *   true if the currently active task (the head of the g_readytorun list)
 *   has changed.
 *
 * Assumptions:
 * - The caller has established a critical section before
 *   calling this function (calling sched_lock() first is NOT
 *   a good idea -- use irqsave()).
 * - The caller has already removed the input rtcb from
 *   whatever list it was in.
 * - The caller handles the condition that occurs if the
 *   the head of the g_readytorun list is changed.
 *
 ****************************************************************************/

bool sched_addreadytorun(FAR _TCB *btcb)
{
  FAR _TCB *rtcb = (FAR _TCB*)g_readytorun.head;
  bool ret;

  /* Check if pre-emption is disabled for the current running task and if
   * the new ready-to-run task would cause the current running task to be
   * preempted.
   */

  if (rtcb->lockcount && rtcb->sched_priority < btcb->sched_priority)
    {
      /* Yes.  Preemption would occur!  Add the new ready-to-run task to the
       * g_pendingtasks task list for now.
       */

      sched_addprioritized(btcb, (FAR dq_queue_t*)&g_pendingtasks);
      btcb->task_state = TSTATE_TASK_PENDING;
      ret = false;
    }

  /* Otherwise, add the new task to the g_readytorun task list */

  else if (sched_addprioritized(btcb, (FAR dq_queue_t*)&g_readytorun))
    {
      /* Inform the instrumentation logic that we are switching tasks */

      sched_note_switch(rtcb, btcb);

      /* The new btcb was added at the head of the g_readytorun list.  It
       * is now to new active task!
       */

      ASSERT(!rtcb->lockcount && btcb->flink != NULL);

      btcb->task_state = TSTATE_TASK_RUNNING;
      btcb->flink->task_state = TSTATE_TASK_READYTORUN;
      ret = true;
    }
  else
    {
      /* The new btcb was added in the middle of the g_readytorun list */

      btcb->task_state = TSTATE_TASK_READYTORUN;
      ret = false;
    }

  return ret;
}
