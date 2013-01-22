/*****************************************************************************
 * sched/task_childstatus.c
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

#if defined(CONFIG_SCHED_HAVE_PARENT) && defined(CONFIG_SCHED_CHILD_STATUS)

/*****************************************************************************
 * Private Types
 *****************************************************************************/
/* Globals are maintained in a structure to minimize name collisions.  Note
 * that there cannot be more that CONFIG_MAX_TASKS tasks in total.  So using
 * CONFIG_MAX_TASKS should be sufficient (at least one task, the IDLE thread,
 * will have no parent).
 */

struct child_pool_s
{
  struct child_status_s alloc[CONFIG_MAX_TASKS];
  FAR struct child_status_s *freelist;
};

/*****************************************************************************
 * Private Data
 *****************************************************************************/

static struct child_pool_s g_child_pool;

/*****************************************************************************
 * Private Functions
 *****************************************************************************/

/*****************************************************************************
 * Public Functions
 *****************************************************************************/

/*****************************************************************************
 * Name: task_initialize
 *
 * Description:
 *   Initialize task related status.  At present, this includes only the
 *   initialize of the child status pool.
 *
 * Parameters:
 *   None.
 *
 * Return Value:
 *   None.
 *
 * Assumptions:
 *   Called early in initializatin.  No special precautions are required.
 *
 *****************************************************************************/

void task_initialize(void)
{
  FAR struct child_status_s *curr;
  FAR struct child_status_s *prev;
  int i;

  /* Save all of the child status structures in a free list */

  prev = &g_child_pool.alloc[0];
  g_child_pool.freelist = prev;
  for (i = 0; i < CONFIG_MAX_TASKS; i++)
    {
      curr = &g_child_pool.alloc[i]
      prev->flink = curr;
      prev = curr;
    }
}

/*****************************************************************************
 * Name: task_allocchild
 *
 * Description:
 *   Allocate a child status structure by removing the next entry from a
 *   free list.
 *
 * Parameters:
 *   None.
 *
 * Return Value:
 *   On success, a non-NULL pointer to a child status structure.  NULL is
 *   returned if there are no remaining, pre-allocated child status structures.
 *
 * Assumptions:
 *   Called during task creation in a safe context.  No special precautions
 *   are required here.
 *
 *****************************************************************************/

FAR struct child_status_s *task_allocchild(void)
{
  FAR struct child_status_s *ret;

  /* Return the status block at the head of the free list */

  ret = g_child_pool.freelist;
  if (ret)
    {
      g_child_pool.freelist = ret->flink;
      ret->flink = NULL;
    }

  return ret;
}

/*****************************************************************************
 * Name: task_freechild
 *
 * Description:
 *   Release a child status structure by returning it to a free list.
 *
 * Parameters:
 *   status - The child status structure to be freed.
 *
 * Return Value:
 *   None.
 *
 * Assumptions:
 *   Called during task creation in a safe context.  No special precautions
 *   are required here.
 *
 *****************************************************************************/

void task_freechild(FAR struct child_status_s *child)
{
  /* Return the child status structure to the free list  */

  if (child)
    {
      child->flink = g_child_pool.freelist;
      g_child_pool.freelist = child;
    }
}

/*****************************************************************************
 * Name: task_addchild
 *
 * Description:
 *   Find a child status structure in the given TCB.
 *
 * Parameters:
 *   tcb    - The TCB of the parent task to containing the child status.
 *   pid    - The ID of the child to create
 *   status - Child exit status (should be zero)
 *   flags  - Child flags (see CHILD_FLAGS_* defininitions)
 *
 * Return Value:
 *   On success, a non-NULL pointer to a child status structure.  NULL is
 *   returned if (1) there are no free status structures, or (2) an entry
 *   with this PID already exists.
 *
 * Assumptions:
 *   Called during task creation processing in a safe context.  No special
 *   precautions are required here.
 *
 *****************************************************************************/

FAR struct child_status_s *task_addchild(FAR _TCB *tcb, pid_t pid, int status,
                                         uint8_t flags)
{
  FAR struct child_status_s *child;

  /* Make sure that there is not already a structure for this PID */

  child = task_findchild(tcb, pid);
  if (child)
    {
      return NULL;
    }

  /* Allocate a new status structure  */

  child = task_allocchild(void);
  if (child)
    {
      /* Initialize the structure */

      child->ch_flags  = flags;
      child->ch_pid    = pid;
      child->ch_status = status;

      /* Add the entry into the TCB list of children */

      status->flink     = tcb->children;
      tcb->childen      = status;
    }

  return child;
}

/*****************************************************************************
 * Name: task_findchild
 *
 * Description:
 *   Find a child status structure in the given TCB.  A reference to the
 *   child structure is returned, but the child remains the the TCB's list
 *   of children.
 *
 * Parameters:
 *   tcb - The TCB of the parent task to containing the child status.
 *   pid - The ID of the child to find.
 *
 * Return Value:
 *   On success, a non-NULL pointer to a child status structure.  NULL is
 *   returned if there is child status structure for that pid in the TCB.
 *
 * Assumptions:
 *   Called during SIGCHLD processing in a safe context.  No special precautions
 *   are required here.
 *
 *****************************************************************************/

FAR struct child_status_s *task_findchild(FAR _TCB *tcb, pid_t pid)
{
  FAR struct child_status_s *child;

  /* Find the status structure with the matching PID  */

  for (child = tcb->children; child; child = child->flink)
    {
      if (child->ch_pid == pid)
        {
          return child;
        }
    }

  return NULL;
}

/*****************************************************************************
 * Name: task_removechild
 *
 * Description:
 *   Remove one child structure from the TCB.  The child is removed, but is
 *   not yet freed.  task_freechild must be called in order to free the child
 *   status structure.
 *
 * Parameters:
 *   tcb - The TCB of the parent task to containing the child status.
 *   pid - The ID of the child to find.
 *
 * Return Value:
 *   On success, a non-NULL pointer to a child status structure.  NULL is
 *   returned if there is child status structure for that pid in the TCB.
 *
 * Assumptions:
 *   Called during SIGCHLD processing in a safe context.  No special precautions
 *   are required here.
 *
 *****************************************************************************/

FAR struct child_status_s *task_removechild(FAR _TCB *tcb, pid_t pid)
{
  FAR struct child_status_s *curr;
  FAR struct child_status_s *prev;

  /* Find the status structure with the matching PID  */

  for (prev = NULL, curr = tcb->children;
       curr;
       prev = curr, curr = curr->flink)
    {
      if (curr->ch_pid == pid)
        {
          break;
        }
    }

  /* Did we find it?  If so, remove it from the TCB. */

  if (curr)
    {
      /* Do we remove it from mid-list?  Or from the head of the list? */

      if (prev)
        {
          prev->flink = curr->flink;
        }
      else
        {
          tcb->children = curr->flink;
        }

      curr->flink = NULL;
    }
 
  return curr;
}

/*****************************************************************************
 * Name: task_removechildren
 *
 * Description:
 *   Remove and free all child structure from the TCB.
 *
 * Parameters:
 *   tcb - The TCB of the parent task to containing the child status.
 *
 * Return Value:
 *   None.
 *
 * Assumptions:
 *   Called during task exit processing in a safe context.  No special
 *   precautions are required here.
 *
 *****************************************************************************/

void task_removechildren(FAR _TCB *tcb)
{
  FAR struct child_status_s *curr;
  FAR struct child_status_s *next;

  /* Remove all child structures for the TCB and return them to the freelist  */

  for (curr = tcb->children; curr; curr = next)
    {
      next = curr->flink;
      task_freechild(curr);
    }
}

#endif /* CONFIG_SCHED_HAVE_PARENT && CONFIG_SCHED_CHILD_STATUS */
