/*****************************************************************************
 * sched/group_leave.c
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

#include <sched.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include "group_internal.h"
#include "env_internal.h"

#ifdef HAVE_TASK_GROUP

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

/*****************************************************************************
 * Private Types
 *****************************************************************************/

/*****************************************************************************
 * Private Data
 *****************************************************************************/

/*****************************************************************************
 * Private Functions
 *****************************************************************************/

/*****************************************************************************
 * Name: group_remove
 *
 * Description:
 *   Remove a group from the list of groups.
 *
 * Parameters:
 *   group - The group to be removed.
 *
 * Return Value:
 *   None.
 *
 * Assumptions:
 *   Called during task deletion in a safe context.  No special precautions
 *   are required here.
 *
 *****************************************************************************/

#ifdef HAVE_GROUP_MEMBERS
void group_remove(FAR struct task_group_s *group)
{
  FAR struct task_group_s *curr;
  FAR struct task_group_s *prev;
  irqstate_t flags;

  /* Let's be especially careful while access the global task group list.
   * This is probably un-necessary.
   */
 
  flags = irqsave();
 
  /* Find the task group structure */

  for (prev = NULL, curr = g_grouphead;
       curr && curr != group;
       prev = curr, curr = curr->flink);

  /* Did we find it?  If so, remove it from the list. */

  if (curr)
    {
      /* Do we remove it from mid-list?  Or from the head of the list? */

      if (prev)
        {
          prev->flink = curr->flink;
        }
      else
        {
          g_grouphead = curr->flink;
        }

      curr->flink = NULL;
    }

  irqrestore(flags);
}
#endif

/*****************************************************************************
 * Name: group_release
 *
 * Description:
 *   Release group resources after the last member has left the group.
 *
 * Parameters:
 *   group - The group to be removed.
 *
 * Return Value:
 *   None.
 *
 * Assumptions:
 *   Called during task deletion in a safe context.  No special precautions
 *   are required here.
 *
 *****************************************************************************/

static inline void group_release(FAR _TCB *tcb,
                                 FAR struct task_group_s *group)
{
  /* Free all un-reaped child exit status */

#if defined(CONFIG_SCHED_HAVE_PARENT) && defined(CONFIG_SCHED_CHILD_STATUS)
  group_removechildren(group);
#endif

  /* Free all file-related resources now.  We really need to close files as
   * soon as possible while we still have a functioning task.
   */

#if CONFIG_NFILE_DESCRIPTORS > 0 || CONFIG_NSOCKET_DESCRIPTORS > 0
  (void)group_releasefiles(tcb);
#endif

  /* Release all shared environment variables */

#ifndef CONFIG_DISABLE_ENVIRON
  env_release(tcb);
#endif

  /* Remove the group from the list of groups */

  group_remove(group);

  /* Release the members array */

  if (group->tg_members)
    {
      sched_free(group->tg_members);
      group->tg_members = NULL;
    }

  /* Release the group container itself */

  sched_free(group);
}

/*****************************************************************************
 * Public Functions
 *****************************************************************************/

/*****************************************************************************
 * Name: group_leave
 *
 * Description:
 *   Release a reference on a group.  This function is called when a task or
 *   thread exits.  It decrements the reference count on the group.  If the
 *   reference count decrements to zero, then it frees the group and all of
 *   resources contained in the group.
 *
 * Parameters:
 *   tcb - The TCB of the task that is exiting.
 *
 * Return Value:
 *   None.
 *
 * Assumptions:
 *   Called during task deletion in a safe context.  No special precautions
 *   are required here.
 *
 *****************************************************************************/

#ifdef HAVE_GROUP_MEMBERS
void group_leave(FAR _TCB *tcb)
{
  FAR struct task_group_s *group;

  DEBUGASSERT(tcb);

  /* Make sure that we have a group */

  group = tcb->group;
  if (group)
    {
      /* Remove the member from group */

      int ret = group_removemember(group, tcb->pid);
      DEBUGASSERT(ret >= 0);

      /* Is the group now empty? */

      if (ret == 0)
        {
          /* Release all of the resource held by the task group */

          group_release(tcb, group);
        }

      /* In any event, we can detach the group from the TCB so that we won't
       * do this again.
       */

      tcb->group = NULL;
    }
}

#else /* HAVE_GROUP_MEMBERS */

void group_leave(FAR _TCB *tcb)
{
  FAR struct task_group_s *group;

  DEBUGASSERT(tcb);

  /* Make sure that we have a group */

  group = tcb->group;
  if (group)
    {
      /* Yes, we have a group.. Is this the last member of the group? */

      if (group->tg_nmembers > 1)
        {
          /* No.. just decrement the number of members in the group */

          group->tg_nmembers--;
        }

      /* Yes.. that was the last member remaining in the group */

      else
        {
          /* Release all of the resource held by the task group */

          group_release(tcb, group);
        }

      /* In any event, we can detach the group from the TCB so we won't do
       * this again.
       */

      tcb->group = NULL;
    }
}

#endif /* HAVE_GROUP_MEMBERS */

/*****************************************************************************
 * Name: group_removemember
 *
 * Description:
 *   Remove a member from a group.
 *
 * Parameters:
 *   group - The group from which to remove the member.
 *   pid - The member to be removed.
 *
 * Return Value:
 *   On success, returns the number of members remaining in the group (>=0).
 *   Can fail only if the member is not found in the group.  On failure,
 *   returns -ENOENT
 *
 * Assumptions:
 *   Called during task deletion and also from the reparenting logic, both
 *   in a safe context.  No special precautions are required here.
 *
 *****************************************************************************/

#ifdef HAVE_GROUP_MEMBERS
int group_removemember(FAR struct task_group_s *group, pid_t pid)
{
  int i;

  DEBUGASSERT(group);

  /* Find the member in the array of members and remove it */

  for (i = 0; i < group->tg_nmembers; i++)
    {
      /* Does this member have the matching pid */

      if (group->tg_members[i] == pid)
       {
          /* Yes.. break out of the loop.  We don't do the actual
           * removal here, instead we re-test i and do the adjustments
           * outside of the loop.  We do this because we want the
           * DEBUGASSERT to work properly.
           */

          break;
       }
    }

  /* Now, test if we found the task in the array of members. */

  if (i < group->tg_nmembers)
    {
      /* Remove the member from the array of members */

      group->tg_members[i] = group->tg_members[group->tg_nmembers - 1];
      group->tg_nmembers--;
      return group->tg_nmembers;
    }

  return -ENOENT;
}
#endif /* HAVE_GROUP_MEMBERS */

#endif /* HAVE_TASK_GROUP */
