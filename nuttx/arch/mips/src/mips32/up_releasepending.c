/****************************************************************************
 *  arch/mips/src/mips32/up_releasepending.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
#include <syscall.h>
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
 * Name: up_release_pending
 *
 * Description:
 *   Release and ready-to-run tasks that have
 *   collected in the pending task list.  This can call a
 *   context switch if a new task is placed at the head of
 *   the ready to run list.
 *
 ****************************************************************************/

void up_release_pending(void)
{
  _TCB *rtcb = (_TCB*)g_readytorun.head;

  slldbg("From TCB=%p\n", rtcb);

  /* Merge the g_pendingtasks list into the g_readytorun task list */

  /* sched_lock(); */
  if (sched_mergepending())
    {
      /* The currently active task has changed!  We will need to
       * switch contexts.  First check if we are operating in
       * interrupt context:
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
          slldbg("New Active Task TCB=%p\n", rtcb);

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
