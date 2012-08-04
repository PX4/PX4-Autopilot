/****************************************************************************
 * sched/pthread_mutexlock.c
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
 * Name: pthread_mutex_lock
 *
 * Description:
 *   The mutex object referenced by mutex is locked by calling
 *   pthread_mutex_lock(). If the mutex is already locked, the calling thread
 *   blocks until the mutex becomes available. This operation returns with the
 *   mutex object referenced by mutex in the locked state with the calling
 *   thread as its owner.
 *
 *   If the mutex type is PTHREAD_MUTEX_NORMAL, deadlock detection is not
 *   provided. Attempting to relock the mutex causes deadlock. If a thread
 *   attempts to unlock a mutex that it has not locked or a mutex which is
 *   unlocked, undefined behavior results.
 *
 *   If the mutex type is PTHREAD_MUTEX_ERRORCHECK, then error checking is
 *   provided. If a thread attempts to relock a mutex that it has already
 *   locked, an error will be returned. If a thread attempts to unlock a
 *   mutex that it has not locked or a mutex which is unlocked, an error will
 *   be returned.
 *
 *   If the mutex type is PTHREAD_MUTEX_RECURSIVE, then the mutex maintains
 *   the concept of a lock count. When a thread successfully acquires a mutex
 *   for the first time, the lock count is set to one. Every time a thread
 *   relocks this mutex, the lock count is incremented by one. Each time the
 *   thread unlocks the mutex, the lock count is decremented by one. When the
 *   lock count reaches zero, the mutex becomes available for other threads to
 *   acquire. If a thread attempts to unlock a mutex that it has not locked or
 *   a mutex which is unlocked, an error will be returned.
 *
 *   If a signal is delivered to a thread waiting for a mutex, upon return
 *   from the signal handler the thread resumes waiting for the mutex as if
 *   it was not interrupted.
 *
 * Parameters:
 *   mutex - A reference to the mutex to be locked.
 *
 * Return Value:
 *   0 on success or an errno value on failure.  Note that the errno EINTR
 *   is never returned by pthread_mutex_lock().
 *
 * Assumptions:
 *
 ****************************************************************************/

int pthread_mutex_lock(FAR pthread_mutex_t *mutex)
{
  int mypid = (int)getpid();
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

      /* Does this task already hold the semaphore? */

      if (mutex->pid == mypid)
        {
          /* Yes.. Is this a recursive mutex? */

#ifdef CONFIG_MUTEX_TYPES
          if (mutex->type == PTHREAD_MUTEX_RECURSIVE)
            {
              /* Yes... just increment the number of locks held and return success */
          
              mutex->nlocks++;
            }
          else
#endif
            {
              /* No, then we would deadlock... return an error (default behavior
               * is like PTHREAD_MUTEX_ERRORCHECK)
               */

              sdbg("Returning EDEADLK\n");
              ret = EDEADLK;
            }
        }
      else
        {
          /* Take the semaphore */

          ret = pthread_takesemaphore((sem_t*)&mutex->sem);

          /* If we succussfully obtained the semaphore, then indicate
           * that we own it.
           */

          if (!ret)
            {
              mutex->pid    = mypid;
#ifdef CONFIG_MUTEX_TYPES
              mutex->nlocks = 1;
#endif
            }
        }

      sched_unlock();
    }

  sdbg("Returning %d\n", ret);
  return ret;
}

