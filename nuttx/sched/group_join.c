/*****************************************************************************
 * sched/group_join.c
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

#include <nuttx/kmalloc.h>

#include "os_internal.h"

#ifdef HAVE_TASK_GROUP

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/
/* Is this worth making a configuration option? */

#define GROUP_REALLOC_MEMBERS 4

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
 * Name: group_bind
 *
 * Description:
 *   A thread joins the group when it is created.  This is a two step process,
 *   first, the group must bound to the new threads TCB.  group_bind() does
 *   this (at the return from group_join, things are a little unstable:  The
 *   group has been bound, but tg_nmembers hs not yet been incremented).
 *   Then, after the new thread is initialized and has a PID assigned to it,
 *   group_join() is called, incrementing the tg_nmembers count on the group.  
 *
 * Parameters:
 *   tcb - The TCB of the new "child" task that need to join the group.
 *
 * Return Value:
 *   0 (OK) on success; a negated errno value on failure.
 *
 * Assumptions:
 * - The parent task from which the group will be inherited is the task at
 *   the thead of the ready to run list.
 * - Called during thread creation in a safe context.  No special precautions
 *   are required here.
 *
 *****************************************************************************/

int group_bind(FAR _TCB *tcb)
{
  FAR _TCB *ptcb = (FAR _TCB *)g_readytorun.head;

  DEBUGASSERT(ptcb && tcb && ptcb->group && !tcb->group);

  /* Copy the group reference from the parent to the child */

  tcb->group = ptcb->group;
  return OK;
}

/*****************************************************************************
 * Name: group_join
 *
 * Description:
 *   A thread joins the group when it is created.  This is a two step process,
 *   first, the group must bound to the new threads TCB.  group_bind() does
 *   this (at the return from group_join, things are a little unstable:  The
 *   group has been bound, but tg_nmembers hs not yet been incremented).
 *   Then, after the new thread is initialized and has a PID assigned to it,
 *   group_join() is called, incrementing the tg_nmembers count on the group.  
 *
 * Parameters:
 *   tcb - The TCB of the new "child" task that need to join the group.
 *
 * Return Value:
 *   0 (OK) on success; a negated errno value on failure.
 *
 * Assumptions:
 * - The parent task from which the group will be inherited is the task at
 *   the thead of the ready to run list.
 * - Called during thread creation in a safe context.  No special precautions
 *   are required here.
 *
 *****************************************************************************/

int group_join(FAR _TCB *tcb)
{
#ifdef HAVE_GROUP_MEMBERS
  FAR struct task_group_s *group;

  DEBUGASSERT(tcb && tcb->group &&
              tcb->group->tg_nmembers < UINT8_MAX);

  /* Get the group from the TCB */

  group = tcb->group;
  
  /* Will we need to extend the size of the array of groups? */

  if (group->tg_nmembers >= group->tg_mxmembers)
    {
      FAR pid_t *newmembers;
      unsigned int newmax;

      /* Yes... reallocate the array of members */

      newmax = group->tg_mxmembers + GROUP_REALLOC_MEMBERS;
      if (newmax > UINT8_MAX)
        {
          newmax = UINT8_MAX;
        }

      newmembers = (FAR pid_t *)
        krealloc(group->tg_members, sizeof(pid_t) * newmax);

      if (!newmembers)
        {
          return -ENOMEM;
        }

      /* Save the new number of members in the reallocated members array */

      group->tg_members   = newmembers;
      group->tg_mxmembers = newmax;
    }

  /* Assign this new pid to the group. */

  group->tg_members[group->tg_nmembers] = tcb->pid;
#endif

  group->tg_nmembers++;
  return OK;
}

#endif /* HAVE_TASK_GROUP */
