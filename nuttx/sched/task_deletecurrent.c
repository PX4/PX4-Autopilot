/****************************************************************************
 * sched/task_deletecurrent.c
 *
 *   Copyright (C) 2008-2009, 2012 Gregory Nutt. All rights reserved.
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

#include  <nuttx/config.h>

#include  <sched.h>
#include  "os_internal.h"
#ifndef CONFIG_DISABLE_SIGNALS
# include "sig_internal.h"
#endif

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
 * Name: task_delete
 *
 * Description:
 *   This function causes the currently running task (i.e., the task at the
 *   head of the ready-to-run list) to cease to exist.  This is a part of
 *   the logic used to implement _exit().  The full implementation of _exit()
 *   is architecture-dependent.  This function should never be called from
 *   normal user code, but only from the architecture-specific implementation
 *   of exit.
 *
 * Inputs:
 *   None
 *
 * Return Value:
 *   OK on success; or ERROR on failure
 *
 * Assumeptions:
 *   Interrupts are disabled.
 *
 ****************************************************************************/

int task_deletecurrent(void)
{
  FAR _TCB  *dtcb = (FAR _TCB*)g_readytorun.head;
  FAR _TCB  *rtcb;

  /* Remove the TCB of the current task from the ready-to-run list.  A context
   * switch will definitely be necessary -- that must be done by the
   * architecture-specific logic.
   *
   * sched_removereadytorun will mark the task at the head of the ready-to-run
   * with state == TSTATE_TASK_RUNNING
   */

  (void)sched_removereadytorun(dtcb);
  rtcb = (FAR _TCB*)g_readytorun.head;

  /* We are now in a bad state -- the head of the ready to run task list
   * does not correspond to the thread that is running.  Disabling pre-
   * emption on this TCB and marking the new ready-to-run task as not
   * running (see, for example, get_errno_ptr()).
   *
   * We disable pre-emption here by directly incrementing the lockcount
   * (vs. calling sched_lock()).
   */

  rtcb->lockcount++;
  rtcb->task_state = TSTATE_TASK_READYTORUN;

  /* Move the TCB to the specified blocked task list and delete it */

  sched_addblocked(dtcb, TSTATE_TASK_INACTIVE);
  task_delete(dtcb->pid);
  rtcb->task_state = TSTATE_TASK_RUNNING;

  /* If there are any pending tasks, then add them to the ready-to-run
   * task list now
   */

  if (g_pendingtasks.head)
    {
      (void)sched_mergepending();
    }

  /* We can't use sched_unlock() to decrement the lock count because the
   * sched_mergepending() call above might have changed the task at the
   * head of the ready-to-run list.  Furthermore, we should not need to
   * perform the unlock action anyway because we know that the pending
   * task list is empty.  So all we really need to do is to decrement
   * the lockcount on rctb.
   */

  rtcb->lockcount--;
  return OK;
}

