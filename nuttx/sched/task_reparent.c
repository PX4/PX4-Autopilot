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
#include "group_internal.h"

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
 *   ppid - PID of the new parent task (0 for grandparent, i.e. the parent
 *     of the current parent task)
 *   chpid  - PID of the child to be reparented.
 *
 * Return Value:
 *   0 (OK) on success; A negated errno value on failure.
 *
 *****************************************************************************/

int task_reparent(pid_t ppid, pid_t chpid)
{
#ifdef CONFIG_SCHED_CHILD_STATUS
  FAR struct child_status_s *child;
#endif
  _TCB *ptcb;
  _TCB *chtcb;
  _TCB *otcb;
  pid_t opid;
  irqstate_t flags;
  int ret;

  /* Disable interrupts so that nothing can change in the relatinoship of
   * the three task:  Child, current parent, and new parent.
   */

  flags = irqsave();

  /* Get the child tasks TCB (chtcb) */

  chtcb = sched_gettcb(chpid);
  if (!chtcb)
    {
      ret = -ECHILD;
      goto errout_with_ints;
    }

  /* Get the PID of the child task's parent (opid) */

  opid = chtcb->parent;

  /* Get the TCB of the child task's parent (otcb) */

  otcb = sched_gettcb(opid);
  if (!otcb)
    {
      ret = -ESRCH;
      goto errout_with_ints;
    }

  /* If new parent task's PID (ppid) is zero, then new parent is the
   * grandparent will be the new parent, i.e., the parent of the current
   * parent task.
   */

  if (ppid == 0)
    {
      ppid = otcb->parent;
    }
  
  /* Get the new parent task's TCB (ptcb) */

  ptcb = sched_gettcb(ppid);
  if (!ptcb)
    {
      ret = -ESRCH;
      goto errout_with_ints;
    }

  /* Then reparent the child */

  chtcb->parent = ppid;  /* The task specified by ppid is the new parent */

#ifdef CONFIG_SCHED_CHILD_STATUS
  /* Remove the child status entry from old parent TCB */

  child = task_removechild(otcb, chpid);
  if (child)
    {
      /* Has the new parent's task group supressed child exit status? */

      if ((ptcb->group->tg_flags && GROUP_FLAG_NOCLDWAIT) == 0)
        {
          /* No.. Add the child status entry to the new parent's task group */

          task_addchild(ptcb, child);
        }
      else
        {
          /* Yes.. Discard the child status entry */

          task_freechild(child);
        }

      /* Either case is a success */

      ret = OK;
    }
  else
    {
      /* This would not be an error if the original parent's task group has
       * suppressed child exit status.
       */

      ret = ((otcb->group->tg_flags && GROUP_FLAG_NOCLDWAIT) == 0) ? -ENOENT : OK;
    }
#else
  DEBUGASSERT(otcb->nchildren > 0);

  otcb->nchildren--;     /* The orignal parent now has one few children */
  ptcb->nchildren++;     /* The new parent has one additional child */
  ret = OK;
#endif
  
errout_with_ints:
  irqrestore(flags);
  return ret;  
}

#endif /* CONFIG_SCHED_HAVE_PARENT */
