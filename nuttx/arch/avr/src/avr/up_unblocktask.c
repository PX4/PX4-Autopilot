/****************************************************************************
 *  arch/avr/src/avr/up_unblocktask.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sched.h>
#include <debug.h>
#include <nuttx/arch.h>

#include "os_internal.h"
#include "clock_internal.h"
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
 * Name: up_unblock_task
 *
 * Description:
 *   A task is currently in an inactive task list
 *   but has been prepped to execute.  Move the TCB to the
 *   ready-to-run list, restore its context, and start execution.
 *
 * Inputs:
 *   tcb: Refers to the tcb to be unblocked.  This tcb is
 *     in one of the waiting tasks lists.  It must be moved to
 *     the ready-to-run list and, if it is the highest priority
 *     ready to run taks, executed.
 *
 ****************************************************************************/

void up_unblock_task(_TCB *tcb)
{
  /* Verify that the context switch can be performed */

  if ((tcb->task_state < FIRST_BLOCKED_STATE) ||
      (tcb->task_state > LAST_BLOCKED_STATE))
    {
      PANIC(OSERR_BADUNBLOCKSTATE);
    }
  else
    {
      _TCB *rtcb = (_TCB*)g_readytorun.head;

      /* Remove the task from the blocked task list */

      sched_removeblocked(tcb);

      /* Reset its timeslice.  This is only meaningful for round
       * robin tasks but it doesn't here to do it for everything
       */

#if CONFIG_RR_INTERVAL > 0
      tcb->timeslice = CONFIG_RR_INTERVAL / MSEC_PER_TICK;
#endif

      /* Add the task in the correct location in the prioritized
       * g_readytorun task list
       */

      if (sched_addreadytorun(tcb))
        {
          /* The currently active task has changed! We need to do
           * a context switch to the new task.
           *
           * Are we in an interrupt handler? 
           */

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

          /* No, then we will need to perform the user context switch */

          else
            {
              /* Switch context to the context of the task at the head of the
               * ready to run list.
               */

               _TCB *nexttcb = (_TCB*)g_readytorun.head;
               up_switchcontext(rtcb->xcp.regs, nexttcb->xcp.regs);

              /* up_switchcontext forces a context switch to the task at the
               * head of the ready-to-run list.  It does not 'return' in the
               * normal sense.  When it does return, it is because the blocked
               * task is again ready to run and has execution priority.
               */
           }
        }
    }
}

