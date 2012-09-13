/****************************************************************************
 * arch/hc/src/common/up_blocktask.c
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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
#include <sched.h>
#include <debug.h>

#include <nuttx/arch.h>

#include "os_internal.h"
#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_block_task
 *
 * Description:
 *   The currently executing task at the head of
 *   the ready to run list must be stopped.  Save its context
 *   and move it to the inactive list specified by task_state.
 *
 * Inputs:
 *   tcb: Refers to a task in the ready-to-run list (normally
 *     the task at the head of the list).  It most be
 *     stopped, its context saved and moved into one of the
 *     waiting task lists.  It it was the task at the head
 *     of the ready-to-run list, then a context to the new
 *     ready to run task must be performed.
 *   task_state: Specifies which waiting task list should be
 *     hold the blocked task TCB.
 *
 ****************************************************************************/

void up_block_task(_TCB *tcb, tstate_t task_state)
{
  /* Verify that the context switch can be performed */

  if ((tcb->task_state < FIRST_READY_TO_RUN_STATE) ||
      (tcb->task_state > LAST_READY_TO_RUN_STATE))
    {
      PANIC(OSERR_BADBLOCKSTATE);
    }
  else
    {
      _TCB *rtcb = (_TCB*)g_readytorun.head;
      bool switch_needed;

      /* Remove the tcb task from the ready-to-run list.  If we
       * are blocking the task at the head of the task list (the
       * most likely case), then a context switch to the next
       * ready-to-run task is needed. In this case, it should
       * also be true that rtcb == tcb.
       */

      switch_needed = sched_removereadytorun(tcb);

      /* Add the task to the specified blocked task list */

      sched_addblocked(tcb, (tstate_t)task_state);

      /* If there are any pending tasks, then add them to the g_readytorun
       * task list now
       */

      if (g_pendingtasks.head)
        {
          switch_needed |= sched_mergepending();
        }

      /* Now, perform the context switch if one is needed */

      if (switch_needed)
        {
          /* Are we in an interrupt handler? */

          if (current_regs)
            {
              /* Yes, then we have to do things differently.
               * Just copy the current_regs into the OLD rtcb.
               */

               up_savestate(rtcb->xcp.regs);

              /* Restore the exception context of the rtcb at the (new) head 
               * of the g_readytorun task list.
               */

              rtcb = (_TCB*)g_readytorun.head;

              /* Then switch contexts */

              up_restorestate(rtcb->xcp.regs);
            }

          /* Copy the user C context into the TCB at the (old) head of the
           * g_readytorun Task list. if up_saveusercontext returns a non-zero
           * value, then this is really the previously running task restarting!
           */

          else if (!up_saveusercontext(rtcb->xcp.regs))
            {
              /* Restore the exception context of the rtcb at the (new) head 
               * of the g_readytorun task list.
               */

              rtcb = (_TCB*)g_readytorun.head;

              /* Then switch contexts */

              up_fullcontextrestore(rtcb->xcp.regs);
            }
        }
    }
}
