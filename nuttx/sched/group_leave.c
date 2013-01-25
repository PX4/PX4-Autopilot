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

void group_leave(FAR _TCB *tcb)
{
  FAR struct task_group_s *group;

  DEBUGASSERT(tcb);

  /* Make sure that we have a group */

  group = tcb->group;
  if (group)
    {
#ifdef HAVE_GROUP_MEMBERS
      int i;

      /* Find the member in the array of members and remove it */

      for (i = 0; i < group->tg_nmembers; i++)
        {
          /* Does this member have the matching pid */

          if (group->tg_members[i] == tcb->pid)
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

      DEBUGASSERT(i < group->tg_nmembers);
      if (i < group->tg_nmembers)
        {
          /* Yes..Is this the last member of the group? */

          if (group->tg_nmembers > 1)
            {
              /* No.. remove the member from the array of members */

              group->tg_members[i] = group->tg_members[group->tg_nmembers - 1];
              group->tg_nmembers--;
            }

          /* Yes.. that was the last member remaining in the group */

          else
            {
              /* Release all of the resource contained within the group */
              /* Free all un-reaped child exit status */

#if defined(CONFIG_SCHED_HAVE_PARENT) && defined(CONFIG_SCHED_CHILD_STATUS)
             task_removechildren(tcb);
#endif
              /* Release all shared environment variables */

#ifndef CONFIG_DISABLE_ENVIRON
              env_release(tcb);
#endif
              /* Release the group container itself */

              sched_free(group);
            }
        }
#else
      /* Yes..Is this the last member of the group? */

      if (group->tg_nmembers > 1)
        {
          /* No.. just decrement the number of members in the group */

          group->tg_nmembers--;
        }

      /* Yes.. that was the last member remaining in the group */

      else
        {
          /* Release all of the resource contained within the group */
          /* Free all un-reaped child exit status */

#if defined(CONFIG_SCHED_HAVE_PARENT) && defined(CONFIG_SCHED_CHILD_STATUS)
          task_removechildren(tcb);
#endif
          /* Release all shared environment variables */

#ifndef CONFIG_DISABLE_ENVIRON
          env_release(tcb);
#endif
          /* Release the group container itself */

          sched_free(group);
        }
#endif

      /* In any event, we can detach the group from the TCB so we won't do
       * this again.
       */

      tcb->group = NULL;
    }
}

#endif /* HAVE_TASK_GROUP */
