/*****************************************************************************
 * sched/task_reparent.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
 *****************************************************************************/

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>

#include "os_internal.h"

#ifdef CONFIG_SCHED_HAVE_PARENT

/*****************************************************************************
 * Private Functions
 *****************************************************************************/

/*****************************************************************************
 * Public Functions
 *****************************************************************************/

/*****************************************************************************
 * Name: task_reparent
 *
 * Description:
 *   Change the parent of a task.
 *
 * Parameters:
 *   oldpid - PID of the old parent task (0 if this task)
 *   newpid - PID ot the new parent task (0 for the parent of this task)
 *   chpid  - PID of the child to be reparented.
 *
 * Return Value:
 *   0 (OK) on success; A negated errno value on failure.
 *
 *****************************************************************************/

int task_reparent(pid_t oldpid, pid_t newpid, pid_t chpid)
{
  _TCB *oldtcb;
  _TCB *newtcb;
  _TCB *chtcb;
  irqstate_t flags;
  int ret;

  /* If oldpid is zero, then we are parent task. */

  if (oldpid == 0)
    {
      oldpid = getpid();
    }

  /* Get the current parent task's TCB */

  oldtcb = sched_gettcb(oldpid);
  if (!oldtcb)
    {
      return -ESRCH;
    }

  /* Disable interrupts so that nothing can change from this point */

  flags = irqsave();

  /* If newpid is zero, then new is the parent of oldpid. */

  if (newpid == 0)
    {
      newpid = oldtcb->parent;
    }
  
  /* Get the new parent task's TCB */

  newtcb = sched_gettcb(newpid);
  if (!newtcb)
    {
      ret = -ESRCH;
      goto errout_with_ints;
    }

  /* Get the child tasks TCB */

  chtcb = sched_gettcb(chpid);
  if (!chtcb)
    {
      ret = -ECHILD;
      goto errout_with_ints;
    }

  /* Verify that oldpid is the parent of chpid */

  if (chtcb->parent != oldpid)
    {
      ret = -ECHILD;
      goto errout_with_ints;
    }

  /* Okay, reparent the child */

  DEBUGASSERT(oldtcb->nchildren > 0);
  chtcb->parent = newpid;
  oldtcb->nchildren--;
  newtcb->nchildren++;
  ret = OK;
  
errout_with_ints:
  irqrestore(flags);
  return ret;  
}

#endif /* CONFIG_SCHED_HAVE_PARENT */
