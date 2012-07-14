/************************************************************************
 * sched/pthread_completejoin.c
 *
 *   Copyright (C) 2007, 2009, 2011 Gregory Nutt. All rights reserved.
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
 ************************************************************************/

/************************************************************************
 * Included Files
 ************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>
#include <pthread.h>
#include <errno.h>
#include <debug.h>

#include "os_internal.h"
#include "pthread_internal.h"

/************************************************************************
 * Pre-processor Definitions
 ************************************************************************/

/************************************************************************
 * Private Type Declarations
 ************************************************************************/

/************************************************************************
 * Global Variables
 ************************************************************************/

/************************************************************************
 * Private Variables
 ************************************************************************/

/************************************************************************
 * Private Functions
 ************************************************************************/

/************************************************************************
 * Name: pthread_notifywaiters
 *
 * Description:
 *   Notify all other threads waiting in phread join for this thread's
 *   exit data.  This must  be done by the child at child thread
 *   destruction time.
 *
 ************************************************************************/

static bool pthread_notifywaiters(FAR join_t *pjoin)
{
  int ntasks_waiting;
  int status;

  svdbg("pjoin=0x%p\n", pjoin);

  /* Are any tasks waiting for our exit value? */

  status = sem_getvalue(&pjoin->exit_sem, &ntasks_waiting);
  if (status == OK && ntasks_waiting < 0)
    {
      /* Set the data semaphore so that this thread will be
       * awakened when all waiting tasks receive the data
       */

      (void)sem_init(&pjoin->data_sem, 0, (ntasks_waiting+1));

      /* Post the semaphore to restart each thread that is waiting
       * on the semaphore
       */

      do
        {
          status = pthread_givesemaphore(&pjoin->exit_sem);
          if (status == OK)
            {
              status = sem_getvalue(&pjoin->exit_sem, &ntasks_waiting);
            }
        }
      while (ntasks_waiting < 0 && status == OK);

      /* Now wait for all these restarted tasks to obtain the return
       * value.
       */

      (void)pthread_takesemaphore(&pjoin->data_sem);
      return true;
    }

  return false;
}

/************************************************************************
 * Public Functions
 ************************************************************************/

/************************************************************************
 * Name: pthread_completejoin
 *
 * Description:
 *   A thread has been terminated -- either by returning, calling
 *   pthread_exit(), or through pthread_cancel().  In any event, we must
 *   complete any pending join events.
 *
 * Parameters:
 *   exit_value
 *
 * Returned Value:
 *   OK unless there is no join information associated with the pid.
 *   This could happen, for example, if a task started with task_create()
 *   calls pthread_exit().
 *
 * Assumptions:
 *
 ************************************************************************/

int pthread_completejoin(pid_t pid, FAR void *exit_value)
{
  FAR join_t *pjoin;

  svdbg("pid=%d exit_value=%p\n", pid, exit_value);

  /* First, find thread's structure in the private data set. */

  (void)pthread_takesemaphore(&g_join_semaphore);
  pjoin = pthread_findjoininfo(pid);
  if (!pjoin)
    {
      sdbg("Could not find join info, pid=%d\n", pid);
      (void)pthread_givesemaphore(&g_join_semaphore);
      return ERROR;
    }
  else
    {
      bool waiters;

      /* Save the return exit value in the thread structure. */

      pjoin->terminated = true;
      pjoin->exit_value = exit_value;

      /* Notify waiters of the availability of the exit value */

      waiters = pthread_notifywaiters(pjoin);

      /* If there are no waiters and if the thread is marked as detached.
       * then discard the join information now.  Otherwise, the pthread
       * join logic will call pthread_destroyjoin() when all of the threads
       * have sampled the exit value.
       */

      if (!waiters && pjoin->detached)
        {
           pthread_destroyjoin(pjoin);
        }

      /* Giving the following semaphore will allow the waiters
       * to call pthread_destroyjoin.
       */

      (void)pthread_givesemaphore(&g_join_semaphore);
    }

  return OK;
}

/************************************************************************
 * Name: pthread_destroyjoin
 *
 * Description:
 *   This is called from pthread_completejoin if the join info was
 *   detached or from pthread_join when the last waiting thread has
 *   received the thread exit info.
 *
 *   Or it may never be called if the join info was never detached or if
 *   no thread ever calls pthread_join.  In case, there is a memory leak!
 *
 * Assumptions:
 *   The caller holds g_join_semaphore
 *
 ************************************************************************/

void pthread_destroyjoin(FAR join_t *pjoin)
{
  sdbg("pjoin=0x%p\n", pjoin);

  /* Remove the join info from the set of joins */

  (void)pthread_removejoininfo((pid_t)pjoin->thread);

  /* Destroy its semaphores */

  (void)sem_destroy(&pjoin->data_sem);
  (void)sem_destroy(&pjoin->exit_sem);

  /* And deallocate the pjoin structure */

  sched_free(pjoin);
}

