/************************************************************************
 * sched/sched_mergepending.c
 *
 *   Copyright (C) 2007, 2009, 2012 Gregory Nutt. All rights reserved.
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
 ************************************************************************/

/************************************************************************
 * Included Files
 ************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <sched.h>
#include <queue.h>
#include <assert.h>

#include "os_internal.h"

/************************************************************************
 * Pre-processor Definitions
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
 * Private Function Prototypes
 ************************************************************************/

/************************************************************************
 * Public Functions
 ************************************************************************/

/************************************************************************
 * Name: sched_mergepending
 *
 * Description:
 *   This function merges the prioritized g_pendingtasks list into the
 *   prioritized g_readytorun task list.
 *
 * Inputs:
 *   None
 *
 * Return Value:
 *   true if the head of the g_readytorun task list has changed.
 *
 * Assumptions:
 * - The caller has established a critical section before
 *   calling this function (calling sched_lock() first is NOT
 *   a good idea -- use irqsave()).
 * - The caller handles the condition that occurs if the
 *   the head of the sched_mergTSTATE_TASK_PENDINGs is changed.
 *
 ************************************************************************/

bool sched_mergepending(void)
{
  FAR _TCB *pndtcb;
  FAR _TCB *pndnext;
  FAR _TCB *rtrtcb;
  FAR _TCB *rtrprev;
  bool ret = false;

  /* Initialize the inner search loop */

  rtrtcb = (FAR _TCB*)g_readytorun.head;

  /* Process every TCB in the g_pendingtasks list */

  for (pndtcb = (FAR _TCB*)g_pendingtasks.head; pndtcb; pndtcb = pndnext)
    {
      pndnext = pndtcb->flink;

      /* Search the g_readytorun list to find the location to insert the 
       * new pndtcb. Each is list is maintained in ascending sched_priority
       * order.
       */

      for (;
           (rtrtcb && pndtcb->sched_priority <= rtrtcb->sched_priority);
           rtrtcb = rtrtcb->flink);

      /* Add the pndtcb to the spot found in the list.  Check if the
       * pndtcb goes at the ends of the g_readytorun list. This would be
       * error condition since the idle test must always be at the end of
       * the g_readytorun list!
       */

      if (!rtrtcb)
        {
          PANIC(OSERR_NOIDLETASK);
        }
      else
        {
          /* The pndtcb goes just before rtrtcb */

          rtrprev = rtrtcb->blink;
          if (!rtrprev)
            {
              /* Special case: Inserting pndtcb at the head of the list */
              /* Inform the instrumentation layer that we are switching tasks */

              sched_note_switch(rtrtcb, pndtcb);

              /* Then insert at the head of the list */

              pndtcb->flink      = rtrtcb;
              pndtcb->blink      = NULL;
              rtrtcb->blink      = pndtcb;
              g_readytorun.head  = (FAR dq_entry_t*)pndtcb;
              rtrtcb->task_state = TSTATE_TASK_READYTORUN;
              pndtcb->task_state = TSTATE_TASK_RUNNING;
              ret                = true;
            }
          else
            {
              /* Insert in the middle of the list */

              pndtcb->flink      = rtrtcb;
              pndtcb->blink      = rtrprev;
              rtrprev->flink     = pndtcb;
              rtrtcb->blink      = pndtcb;
              pndtcb->task_state = TSTATE_TASK_READYTORUN;
            }
        }

      /* Set up for the next time through */

      rtrtcb = pndtcb;
    }

  /* Mark the input list empty */

  g_pendingtasks.head = NULL;
  g_pendingtasks.tail = NULL;

  return ret;
}
