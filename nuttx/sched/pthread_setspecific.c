/****************************************************************************
 * sched/pthread_setspecific.c
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
 * Name: pthread_setspecific
 *
 * Description:
 *   The pthread_setspecific() function associates a thread-
 *   specific value with a key obtained via a previous call
 *   to pthread_key_create().  Different threads may bind
 *   different values to the same key.  These values are
 *   typically pointers to blocks of dynamically allocated
 *   memory that have been reserved for use by the calling
 *   thread.
 *
 *   The effect of calling pthread_setspecific() with
 *   with a key value not obtained from pthread_create() or
 *   after a key has been deleted with pthread_key_delete()
 *   is undefined.
 *
 * Parameters:
 *   key = The data key to get or set
 *   value = The value to bind to the key.
 *
 * Return Value:
 *   If successful, pthread_setspecific() will return zero (OK).
 *   Otherwise, an error number will be returned:
 *
 *      ENOMEM - Insufficient memory exists to associate
 *         the value with the key.
 *      EINVAL - The key value is invalid.
 *
 * Assumptions:
 *
 * POSIX Compatibility:
 *   int pthread_setspecific(pthread_key_t key, void *value)
 *   void *pthread_getspecific(pthread_key_t key)
 *
 *   - Both calling pthread_setspecific() and pthread_getspecific()
 *     may be called from a thread-specific data destructor
 *     function.
 *
 ****************************************************************************/

int pthread_setspecific(pthread_key_t key, FAR void *value)
{
#if CONFIG_NPTHREAD_KEYS > 0
  FAR _TCB *rtcb = (FAR _TCB*)g_readytorun.head;
  int ret = EINVAL;

  /* Check if the key is valid. */

  if (key < g_pthread_num_keys)
    {
      /* Store the data in the TCB. */

      rtcb->pthread_data[key] = value;

      /* Return success. */

      ret = OK;
    }

  return ret;
#else
  return ENOSYS;
#endif
}

