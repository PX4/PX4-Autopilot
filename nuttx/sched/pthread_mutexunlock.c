/****************************************************************************
 * sched/pthread_mutexunlock.c
 *
 *   Copyright (C) 2007-2009 Gregory Nutt. All rights reserved.
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

#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <errno.h>
#include <debug.h>

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
 * Name: pthread_mutex_unlock
 *
 * Description:
 *   The pthread_mutex_unlock() function releases the mutex object referenced
 *   by mutex. The manner in which a mutex is released is dependent upon the
 *   mutex's type attribute. If there are threads blocked on the mutex object
 *   referenced by mutex when pthread_mutex_unlock() is called, resulting in
 *   the mutex becoming available, the scheduling policy is used to determine
 *   which thread shall acquire the mutex. (In the case of PTHREAD_MUTEX_RECURSIVE
 *   mutexes, the mutex becomes available when the count reaches zero and the
 *   calling thread no longer has any locks on this mutex).
 *
 *   If a signal is delivered to a thread waiting for a mutex, upon return from
 *   the signal handler the thread resumes waiting for the mutex as if it was
 *   not interrupted.
 *
 * Parameters:
 *   None
 *
 * Return Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

int pthread_mutex_unlock(FAR pthread_mutex_t *mutex)
{
  int ret = OK;

  sdbg("mutex=0x%p\n", mutex);

  if (!mutex)
    {
      ret = EINVAL;
    }
  else
    {
      /* Make sure the semaphore is stable while we make the following
       * checks.  This all needs to be one atomic action.
       */

      sched_lock();

      /* Does the calling thread own the semaphore? */

      if (mutex->pid != (int)getpid())
        {
          /* No... return an error (default behavior is like PTHREAD_MUTEX_ERRORCHECK) */

          sdbg("Holder=%d returning EPERM\n", mutex->pid);
          ret = EPERM;
        }
        

      /* Yes, the caller owns the semaphore.. Is this a recursive mutex? */

#ifdef CONFIG_MUTEX_TYPES
      else if (mutex->type == PTHREAD_MUTEX_RECURSIVE && mutex->nlocks > 1)
        {
          /* This is a recursive mutex and we there are multiple locks held. Retain
           * the mutex lock, just decrement the count of locks held, and return
           * success.
           */
          mutex->nlocks--;
        }
#endif

      /* This is either a non-recursive mutex or is the outermost unlock of
       * a recursive mutex.
       */

      else
        {
          /* Nullify the pid and lock count then post the semaphore */

          mutex->pid    = 0;
#ifdef CONFIG_MUTEX_TYPES
          mutex->nlocks = 0;
#endif
          ret = pthread_givesemaphore((sem_t*)&mutex->sem);
        }
      sched_unlock();
    }

  sdbg("Returning %d\n", ret);
  return ret;
}


