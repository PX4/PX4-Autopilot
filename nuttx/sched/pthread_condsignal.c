/****************************************************************************
 * sched/pthread_condsignal.c
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

#include <pthread.h>
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
 * Name: pthread_cond_signal
 *
 * Description:
 *    A thread can signal on a condition variable.
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

int pthread_cond_signal(FAR pthread_cond_t *cond)
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
      /* Get the current value of the semaphore */

      if (sem_getvalue((sem_t*)&cond->sem, &sval) != OK)
        {
          ret = EINVAL;
        }

      /* If the value is less than zero (meaning that one or more
       * thread is waiting), then post the condition semaphore.
       * Only the highest priority waiting thread will get to execute
       */

      else
        {
          /* One of my objectives in this design was to make pthread_cond_signal
           * usable from interrupt handlers.  However, from interrupt handlers,
           * you cannot take the associated mutex before signaling the condition.
           * As a result, I think that there could be a race condition with
           * the following logic which assumes that the if sval < 0 then the 
           * thread is waiting.  Without the mutex, there is no atomic, protected
           * operation that will guarantee this to be so.
           */

          sdbg("sval=%d\n", sval);
          if (sval < 0)
            {
              sdbg("Signalling...\n");
              ret = pthread_givesemaphore((sem_t*)&cond->sem);
            }
        }
    }

  sdbg("Returning %d\n", ret);
  return ret;
}


