/****************************************************************************
 * sched/pthread_mutexinit.c
 *
 *   Copyright (C) 2007-2009, 2011 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
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
 * Name: pthread_mutex_init
 *
 * Description:
 *   Create a mutex
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

int pthread_mutex_init(FAR pthread_mutex_t *mutex, FAR pthread_mutexattr_t *attr)
{
  int pshared = 0;
#ifdef CONFIG_MUTEX_TYPES
  uint8_t type  = PTHREAD_MUTEX_DEFAULT;
#endif
  int ret       = OK;
  int status;

  sdbg("mutex=0x%p attr=0x%p\n", mutex, attr);

  if (!mutex)
    {
      ret = EINVAL;
    }
  else
    {
      /* Were attributes specified?  If so, use them */

      if (attr)
        {
          pshared = attr->pshared;
#ifdef CONFIG_MUTEX_TYPES
          type    = attr->type;
#endif
        }

      /* Indicate that the semaphore is not held by any thread. */

      mutex->pid = 0;

      /* Initialize the mutex like a semaphore with initial count = 1 */

      status = sem_init((sem_t*)&mutex->sem, pshared, 1);
      if (status != OK)
        {
          ret = EINVAL;
        }

      /* Set up attributes unique to the mutex type */

#ifdef CONFIG_MUTEX_TYPES
      mutex->type   = type;
      mutex->nlocks = 0;
#endif
    }

  sdbg("Returning %d\n", ret);
  return ret;
}
