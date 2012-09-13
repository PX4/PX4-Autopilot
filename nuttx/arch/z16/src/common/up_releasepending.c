/****************************************************************************
 * common/up_releasepending.c
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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
#include <debug.h>

#include <nuttx/arch.h>

#include "chip/chip.h"
#include "os_internal.h"
#include "up_internal.h"

/****************************************************************************
 * Private Definitions
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
  FAR _TCB *rtcb = (FAR _TCB*)g_readytorun.head;

  slldbg("From TCB=%p\n", rtcb);

  /* Merge the g_pendingtasks list into the g_readytorun task list */

  /* sched_lock(); */
  if (sched_mergepending())
    {
      /* The currently active task has changed!  We will need to
       * switch contexts.  First check if we are operating in
       * interrupt context:
       */

      if (IN_INTERRUPT)
        {
          /* Yes, then we have to do things differently.
           * Just copy the current context into the OLD rtcb.
           */

           SAVE_IRQCONTEXT(rtcb);

          /* Restore the exception context of the rtcb at the (new) head 
           * of the g_readytorun task list.
           */

          rtcb = (FAR _TCB*)g_readytorun.head;
          slldbg("New Active Task TCB=%p\n", rtcb);

          /* Then setup so that the context will be performed on exit
           * from the interrupt.
           */

          SET_IRQCONTEXT(rtcb);
        }

      /* Copy the exception context into the TCB of the task that
       * was currently active. if SAVE_USERCONTEXT returns a non-zero
       * value, then this is really the previously running task 
       * restarting!
       */

      else if (!SAVE_USERCONTEXT(rtcb))
        {
          /* Restore the exception context of the rtcb at the (new) head 
           * of the g_readytorun task list.
           */

          rtcb = (FAR _TCB*)g_readytorun.head;
          slldbg("New Active Task TCB=%p\n", rtcb);

          /* Then switch contexts */

          RESTORE_USERCONTEXT(rtcb);
        }
    }
}
