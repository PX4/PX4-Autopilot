/********************************************************************************
 * sched/pthread_barrierwait.c
 *
 *   Copyright (C) 2007, 2009 Gregory Nutt. All rights reserved.
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
 ********************************************************************************/

/********************************************************************************
 * Included Files
 ********************************************************************************/

#include <nuttx/config.h>

#include <pthread.h>
#include <semaphore.h>
#include <sched.h>
#include <errno.h>
#include <debug.h>

/********************************************************************************
 * Definitions
 ********************************************************************************/

/********************************************************************************
 * Private Type Declarations
 ********************************************************************************/

/********************************************************************************
 * Global Variables
 ********************************************************************************/

/********************************************************************************
 * Private Variables
 ********************************************************************************/

/********************************************************************************
 * Private Function Prototypes
 ********************************************************************************/

/********************************************************************************
 * Public Functions
 ********************************************************************************/

/********************************************************************************
 * Name: pthread_barrier_wait
 *
 * Description:
 *   The pthread_barrier_wait() function synchronizse participating threads at
 *   the barrier referenced by 'barrier'.  The calling thread is blocked until
 *   the required number of threads have called pthread_barrier_wait() specifying
 *   the same 'barrier'.  When the required number of threads have called
 *   pthread_barrier_wait() specifying the 'barrier', the constant
 *   PTHREAD_BARRIER_SERIAL_THREAD will be returned to one unspecified thread
 *   and zero will be returned to each of the remaining threads. At this point,
 *   the barrier will be reset to the state it had as a result of the most recent
 *   pthread_barrier_init() function that referenced it.
 *
 *   The constant PTHREAD_BARRIER_SERIAL_THREAD is defined in pthread.h and its
 *   value must be distinct from any other value returned by pthread_barrier_wait().
 *
 *   The results are undefined if this function is called with an uninitialized
 *   barrier.
 *
 *   If a signal is delivered to a thread blocked on a barrier, upon return from
 *   the signal handler the thread will resume waiting at the barrier if the barrier
 *   wait has not completed; otherwise, the thread will continue as normal from
 *   the completed barrier wait. Until the thread in the signal handler returns
 *   from it, it is unspecified whether other threads may proceed past the barrier
 *   once they have all reached it.
 *
 *   A thread that has blocked on a barrier will not prevent any unblocked thread
 *   that is eligible to use the same processing resources from eventually making
 *   forward progress in its execution.  Eligibility for processing resources will
 *   be determined by the scheduling policy.
 *
 * Parameters:
 *   barrier - the barrier to wait on
 *
 * Return Value:
 *   0 (OK) on success or EINVAL if the barrier is not valid.
 *
 * Assumptions:
 *
 ********************************************************************************/

int pthread_barrier_wait(FAR pthread_barrier_t *barrier)
{
  int semcount;
  int ret = OK;

  if (!barrier)
    {
      return EINVAL;
    }

  /* Disable pre-emption throughout the following */

  sched_lock();

  /* Find out how many threads are already waiting at the barrier */

  ret = sem_getvalue(&barrier->sem, &semcount);
  if (ret != OK)
    {
      sched_unlock();
      return EINVAL;
    }

  /* If the number of waiters would be equal to the count, then we are done */

  if (1 - semcount >= barrier->count)
    {
      /* Free all of the waiting threads */

      while (semcount < 0)
        {
          (void)sem_post(&barrier->sem);
          (void)sem_getvalue(&barrier->sem, &semcount);
        }

      /* Then return PTHREAD_BARRIER_SERIAL_THREAD to the final thread */

      sched_unlock();
      return PTHREAD_BARRIER_SERIAL_THREAD;
    }
  else
    {
      /* Otherwise, this thread must wait as well */

      while (sem_wait(&barrier->sem) != OK)
        {
          /* If the thread is awakened by a signal, just continue to wait */

          int errornumber = get_errno();
          if (errornumber != EINTR)
            {
              /* If it is awakened by some other error, then there is a
               * problem
               */

              sched_unlock();
              return errornumber;
            }
        }

      /* We will only get here when we are one of the N-1 threads that were
       * waiting for the final thread at the barrier.  We just need to return
       * zero.
       */

      sched_unlock();
      return 0;
    }
}
