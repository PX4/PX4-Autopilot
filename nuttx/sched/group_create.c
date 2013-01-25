/*****************************************************************************
 * sched/group_allocate.c
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

#ifdef HAVE_TASK_GROUP

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/
/* Is this worth making a configuration option? */

#define GROUP_INITIAL_MEMBERS 4

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
 * Name: group_allocate
 *
 * Description:
 *   Create and a new task group structure for the specified TCB. This
 *   function is called as part of the task creation sequence.  The structure
 *   allocated and zered, but otherwise uninitialized.  The full creation
 *   of the group of a two step process:  (1) First, this function allocates
 *   group structure early in the task creation sequence in order to provide a
 *   group container, then (2) group_initialize() is called to set up the
 *   group membership.
 *
 * Parameters:
 *   tcb - The tcb in need of the task group.
 *
 * Return Value:
 *   0 (OK) on success; a negated errno value on failure.
 *
 * Assumptions:
 *   Called during task creation in a safe context.  No special precautions
 *   are required here.
 *
 *****************************************************************************/

int group_allocate(FAR _TCB *tcb)
{
  DEBUGASSERT(tcb && !tcb->group);

  /* Allocate the group structure and assign it to the TCB */

  tcb->group = (FAR struct task_group_s *)kzalloc(sizeof(struct task_group_s));
  return tcb->group ? OK : -ENOMEM;
}

/*****************************************************************************
 * Name: group_initialize
 *
 * Description:
 *   Add the task as the initial member of the group.  The full creation of
 *   the group of a two step process:  (1) First, this group structure is
 *   allocated by group_allocate() early in the task creation sequence, then
 *   (2) this function  is called to set up the initial group membership.
 *
 * Parameters:
 *   tcb - The tcb in need of the task group.
 *
 * Return Value:
 *   0 (OK) on success; a negated errno value on failure.
 *
 * Assumptions:
 *   Called during task creation in a safe context.  No special precautions
 *   are required here.
 *
 *****************************************************************************/

int group_initialize(FAR _TCB *tcb)
{
#ifdef HAVE_GROUP_MEMBERS
  FAR struct task_group_s *group;

  DEBUGASSERT(tcb && tcb->group);
  group = tcb->group;

  /* Allocate space to hold GROUP_INITIAL_MEMBERS members of the group */

  group->tg_members = (FAR pid_t *)kmalloc(GROUP_INITIAL_MEMBERS*sizeof(pid_t));
  if (!group->tg_members)
    {
      free(group);
      return -ENOMEM;
    }

  /* Assign the PID of this new task as a member of the group*/

  group->tg_members[0] = tcb->pid;

  /* Initialize the non-zero elements of group structure and assign it to
   * the tcb.
   */

  group->tg_mxmembers  = GROUP_INITIAL_MEMBERS; /* Number of members in allocation */
#endif

  group->tg_nmembers   = 1;                     /* Number of members in the group */
  return OK;
}

#endif /* HAVE_TASK_GROUP */
