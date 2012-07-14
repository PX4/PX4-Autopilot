/****************************************************************************
 * sched/pthread_keycreate.c
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
 * Name: pthread_key_create
 *
 * Description:
 *   This function creates a thread-specific data key visible to all threads
 *   in the system.  Although the same key value may be used by different
 *   threads, the values bound to the key by pthread_setspecific() are
 *   maintained on a per-thread basis and persist for the life of the calling
 *   thread.
 *
 *   Upon key creation, the value NULL will be associated with the new key
 *   in all active threads.  Upon thread creation, the value NULL will be
 *   associated with all defined keys in the new thread.
 *
 * Parameters:
 *   key = A pointer to the key to create.
 *   destructor = An optional destructor() function that may be associated
 *      with each key that is invoked when a thread exits.  However, this
 *      argument is ignored in the current implementation.
 *
 * Return Value:
 *   If successful, the pthread_key_create() function will store the newly
 *   created key value at *key and return zero (OK).  Otherwise, an error
 *   number will bereturned to indicate the error:
 *
 *      EAGAIN - The system lacked sufficient resources to create another
 *         thread-specific data key, or the system-imposed limit on the total
 *         number of keys pers process {PTHREAD_KEYS_MAX} has been exceeded
 *      ENONMEM - Insufficient memory exists to create the key.
 *
 * Assumptions:
 *
 * POSIX Compatibility:
 *   - The present implementation ignores the destructor argument.
 *
 ****************************************************************************/

int pthread_key_create(FAR pthread_key_t *key, CODE void (*destructor)(void*))
{
#if CONFIG_NPTHREAD_KEYS > 0
  int ret = EAGAIN;

  /* Check if we have exceeded the system-defined number of keys. */

  if (g_pthread_num_keys < PTHREAD_KEYS_MAX)
    {
      /* Return the key value */

      *key = g_pthread_num_keys;

      /* Increment the count of global keys. */

      g_pthread_num_keys++;

      /* Return success. */

      ret = OK;
    }

  return ret;
#else
  return ENOSYS;
#endif
}
