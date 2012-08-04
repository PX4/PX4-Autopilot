/****************************************************************************
 * sched/sched_setpriority.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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
#include <sched.h>
#include <errno.h>
#include <nuttx/arch.h>

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
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  sched_setpriority
 *
 * Description:
 *   This function sets the priority of a specified task.
 *
 *   NOTE: Setting a task's priority to the same value has a similar effect
 *   to sched_yield() -- The task will be moved to  after all other tasks
 *   with the same priority.
 *
 * Inputs:
 *   tcb - the TCB of task to reprioritize.
 *   sched_priority - The new task priority
 *
 * Return Value:
 *   On success, sched_setparam() returns 0 (OK). On error, -1 (ERROR) is
 *   returned, and errno is set appropriately.
 *
 *  EINVAL The parameter 'param' is invalid or does not make sense for the
 *         current scheduling policy.
 *  EPERM  The calling task does not have appropriate privileges.
 *  ESRCH  The task whose ID is pid could not be found.
 *
 * Assumptions:
 *
 ****************************************************************************/

int sched_setpriority(FAR _TCB *tcb, int sched_priority)
{
  FAR _TCB  *rtcb = (FAR _TCB*)g_readytorun.head;
  tstate_t   task_state;
  irqstate_t saved_state;

  /* Verify that the requested priority is in the valid range */

  if (sched_priority < SCHED_PRIORITY_MIN || 
      sched_priority > SCHED_PRIORITY_MAX)
    {
      errno = EINVAL;
      return ERROR;
    }

  /* We need to assure that there there is no interrupt activity while
   * performing the following.
   */

  saved_state = irqsave();

  /* There are four cases that must be considered: */

  task_state = tcb->task_state;
  switch (task_state)
    {
      /* CASE 1. The task is running or ready-to-run and a context switch
       * may be caused by the re-prioritization 
       */

      case TSTATE_TASK_RUNNING:

        /* A context switch will occur if the new priority of the running
         * task becomes less than OR EQUAL TO the next highest priority
         * ready to run task.
         */

        if (sched_priority <= tcb->flink->sched_priority)
          {
            /* A context switch will occur. */

            up_reprioritize_rtr(tcb, (uint8_t)sched_priority);
          }

        /* Otherwise, we can just change priority since it has no effect */

        else
          {
            /* Change the task priority */

            tcb->sched_priority = (uint8_t)sched_priority;
          }
        break;

      /* CASE 2. The task is running or ready-to-run and a context switch
       * may be caused by the re-prioritization
       */

      case TSTATE_TASK_READYTORUN:

        /* A context switch will occur if the new priority of the ready-to
         * run task is (strictly) greater than the current running task 
         */

        if (sched_priority > rtcb->sched_priority)
          {
            /* A context switch will occur. */

            up_reprioritize_rtr(tcb, (uint8_t)sched_priority);
          }

        /* Otherwise, we can just change priority and re-schedule (since it
         * have no other effect).
         */

        else
          {
            /* Remove the TCB from the ready-to-run task list */

            ASSERT(!sched_removereadytorun(tcb));

            /* Change the task priority */

            tcb->sched_priority = (uint8_t)sched_priority;

            /* Put it back into the ready-to-run task list */

            ASSERT(!sched_addreadytorun(tcb));
          }
        break;

      /* CASE 3. The task is not in the ready to run list.  Changing its
       * Priority cannot effect the currently executing task.
       */

      default:

        /* CASE 3a. The task resides in a prioritized list. */

        if (g_tasklisttable[task_state].prioritized)
          {
            /* Remove the TCB from the prioritized task list */

            dq_rem((FAR dq_entry_t*)tcb, (FAR dq_queue_t*)g_tasklisttable[task_state].list);

            /* Change the task priority */

            tcb->sched_priority = (uint8_t)sched_priority;

            /* Put it back into the prioritized list at the correct
             * position
             */

            sched_addprioritized(tcb, (FAR dq_queue_t*)g_tasklisttable[task_state].list);
          }

        /* CASE 3b. The task resides in a non-prioritized list. */

        else
          {
            /* Just change the task's priority */

            tcb->sched_priority = (uint8_t)sched_priority;
          }
        break;
    }

  irqrestore(saved_state);
  return OK;
}
