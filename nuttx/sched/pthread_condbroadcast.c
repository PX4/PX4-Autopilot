/****************************************************************************
 * sched/pthread_condbroadcast.c
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
 * Name: pthread_cond_broadcast
 *
 * Description:
 *    A thread broadcast on a condition variable.
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

int pthread_cond_broadcast(FAR pthread_cond_t *cond)
{
  int ret = OK;
  int sval;

  sdbg("cond=0x%p\n", cond);

  if (!cond)
    {
      ret = EINVAL;
    }
  else
    {
      /* Disable pre-emption until all of the waiting threads have been
       * restarted. This is necessary to assure that the sval behaves as
       * expected in the following while loop
       */

      sched_lock();

      /* Get the current value of the semaphore */

      if (sem_getvalue((sem_t*)&cond->sem, &sval) != OK)
        {
          ret = EINVAL;
        }
      else
        {
          /* Loop until all of the waiting threads have been restarted. */

          while (sval < 0)
            {
              /* If the value is less than zero (meaning that one or more
               * thread is waiting), then post the condition semaphore.
               * Only the highest priority waiting thread will get to execute
               */

              ret = pthread_givesemaphore((sem_t*)&cond->sem);

              /* Increment the semaphore count (as was done by the
               * above post).
               */

              sval++;
            }
        }

      /* Now we can let the restarted threads run */

      sched_unlock();
    }

  sdbg("Returning %d\n", ret);
  return ret;
}


