/****************************************************************************
 * pthread_join.c
 *
 *   Copyright (C) 2007, 2008, 2011 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include <debug.h>

#include "os_internal.h"
#include "pthread_internal.h"

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
 * Name: pthread_join
 *
 * Description:
 *    A thread can await termination of another thread and retrieve the
 *    return value of the thread.
 *
 * Parameters:
 *   thread
 *   pexit_value
 *
 * Return Value:
 *   0 if successful.  Otherwise, one of the following error codes:
 *
 *   EINVAL  The value specified by thread does not refer to ajoinable
 *           thread.
 *   ESRCH   No thread could be found corresponding to thatspecified by the
 *           given thread ID.
 *   EDEADLK A deadlock was detected or the value of thread specifies the
 *           calling thread.
 *
 * Assumptions:
 *
 ****************************************************************************/

int pthread_join(pthread_t thread, FAR pthread_addr_t *pexit_value)
{
  FAR join_t *pjoin;
  int ret;

  sdbg("thread=%d\n", thread);

  /* First make sure that this is not an attempt to join to
   * ourself.
   */

  if ((pid_t)thread == getpid())
    {
      return EDEADLK;
    }

  /* Make sure no other task is mucking with the data structures
   * while we are performing the following operations.  NOTE:
   * we can be also sure that pthread_exit() will not execute
   * because it will also attempt to get this semaphore.
   */

  (void)pthread_takesemaphore(&g_join_semaphore);

  /* Find the join information associated with this thread.
   * This can fail for one of three reasons:  (1) There is no
   * thread associated with 'thread,' (2) the thread is a task
   * and does not have join information, or (3) the thread
   * was detached and has exited.
   */

  pjoin = pthread_findjoininfo((pid_t)thread);
  if (!pjoin)
    {
      /* Determine what kind of error to return */

      FAR _TCB *tcb = sched_gettcb((pthread_t)thread);

      sdbg("Could not find thread data\n");

      /* Case (1) or (3) -- we can't tell which.  Assume (3) */

      if (!tcb)
        {
          ret = ESRCH;
        }

      /* The thread is still active but has no join info.  In that
       * case, it must be a task and not a pthread.
       */

      else
        {
          ret = EINVAL;
        }

      (void)pthread_givesemaphore(&g_join_semaphore);
    }
  else
    {
      /* We found the join info structure.  Increment for the reference
       * to the join structure that we have.  This will keep things
       * stable for we have to do
       */

      sched_lock();
      pjoin->crefs++;

      /* Check if the thread is still running.  If not, then things are
       * simpler.  There are still race conditions to be concerned with.
       * For example, there could be multiple threads executing in the
       * 'else' block below when we enter!
       */

      if (pjoin->terminated)
        {
          sdbg("Thread has terminated\n");

          /* Get the thread exit value from the terminated thread. */

          if (pexit_value)
            {
              sdbg("exit_value=0x%p\n", pjoin->exit_value);
              *pexit_value = pjoin->exit_value;
            }
        }
      else
        {
          sdbg("Thread is still running\n");

          /* Relinquish the data set semaphore.  Since pre-emption is
           * disabled, we can be certain that no task has the
           * opportunity to run between the time we relinquish the
           * join semaphore and the time that we wait on the thread exit
           * semaphore.
           */

          (void)pthread_givesemaphore(&g_join_semaphore);

          /* Take the thread's thread exit semaphore.  We will sleep here
           * until the thread exits.  We need to exercise caution because
           * there could be multiple threads waiting here for the same
           * pthread to exit.
           */

          (void)pthread_takesemaphore(&pjoin->exit_sem);

          /* The thread has exited! Get the thread exit value */

          if (pexit_value)
            {
             *pexit_value = pjoin->exit_value;
              sdbg("exit_value=0x%p\n", pjoin->exit_value);
            }

          /* Post the thread's data semaphore so that the exitting thread
           * will know that we have received the data.
           */

          (void)pthread_givesemaphore(&pjoin->data_sem);

          /* Retake the join semaphore, we need to hold this when
           * pthread_destroyjoin is called.
           */

          (void)pthread_takesemaphore(&g_join_semaphore);
        }

      /* Pre-emption is okay now. The logic still cannot be re-entered
       * because we hold the join semaphore
       */

      sched_unlock();

      /* Release our reference to the join structure and, if the reference
       * count decrements to zero, deallocate the join structure.
       */

      if (--pjoin->crefs <= 0)
        {
          (void)pthread_destroyjoin(pjoin);
        }

      (void)pthread_givesemaphore(&g_join_semaphore);
      ret = OK;
    }

  sdbg("Returning %d\n", ret);
  return ret;
}

