/****************************************************************************
 * sched/sem_post.c
 *
 *   Copyright (C) 2007-2009, 2012 Gregory Nutt. All rights reserved.
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

#include <limits.h>
#include <semaphore.h>
#include <sched.h>
#include <nuttx/arch.h>

#include "os_internal.h"
#include "sem_internal.h"

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
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sem_post
 *
 * Description:
 *   When a task has finished with a semaphore, it will call sem_post().
 *   This function unlocks the semaphore referenced by sem by performing the
 *   semaphore unlock operation on that semaphore.
 *
 *   If the semaphore value resulting from this operation is positive, then
 *   no tasks were blocked waiting for the semaphore to become unlocked; the
 *   semaphore is simply incremented.
 *
 *   If the value of the semaphore resulting from this operation is zero,
 *   then one of the tasks blocked waiting for the semaphore shall be
 *   allowed to return successfully from its call to sem_wait().
 *
 * Parameters:
 *   sem - Semaphore descriptor
 *
 * Return Value:
 *   0 (OK) or -1 (ERROR) if unsuccessful
 *
 * Assumptions:
 *   This function cannot be called from an interrupt handler.
 *   It assumes the currently executing task is the one that
 *   is performing the unlock.
 *
 ****************************************************************************/

int sem_post(FAR sem_t *sem)
{
  FAR _TCB  *stcb = NULL;
  int        ret = ERROR;
  irqstate_t saved_state;

  /* Make sure we were supplied with a valid semaphore. */

  if (sem)
    {
      /* The following operations must be performed with interrupts
       * disabled because sem_post() may be called from an interrupt
       * handler.
       */

      saved_state = irqsave();

      /* Perform the semaphore unlock operation. */

      ASSERT(sem->semcount < SEM_VALUE_MAX);
      sem_releaseholder(sem);
      sem->semcount++;

      /* If the result of of semaphore unlock is non-positive, then
       * there must be some task waiting for the semaphore.
       */

#ifdef CONFIG_PRIORITY_INHERITANCE
      /* Don't let it run until we complete the priority restoration
       * steps.
       */

      sched_lock();
#endif
      if (sem->semcount <= 0)
        {
          /* Check if there are any tasks in the waiting for semaphore
           * task list that are waiting for this semaphore. This is a
           * prioritized list so the first one we encounter is the one
           * that we want.
           */

          for (stcb = (FAR _TCB*)g_waitingforsemaphore.head;
               (stcb && stcb->waitsem != sem);
               stcb = stcb->flink);

          if (stcb)
            {
              /* It is, let the task take the semaphore */

              stcb->waitsem = NULL;

              /* Restart the waiting task. */

              up_unblock_task(stcb);
            }
        }

      /* Check if we need to drop the priority of any threads holding
       * this semaphore.  The priority could have been boosted while they
       * held the semaphore.
       */

#ifdef CONFIG_PRIORITY_INHERITANCE
      sem_restorebaseprio(stcb, sem);
      sched_unlock();
#endif
      ret = OK;

      /* Interrupts may now be enabled. */

      irqrestore(saved_state);
    }

  return ret;
}
