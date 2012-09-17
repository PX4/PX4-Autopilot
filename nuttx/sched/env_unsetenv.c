/****************************************************************************
 * sched/env_unsetenv.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef CONFIG_DISABLE_ENVIRON

#include <sched.h>
#include <string.h>
#include <errno.h>

#include <nuttx/kmalloc.h>

#include "os_internal.h"
#include "env_internal.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: unsetenv
 *
 * Description:
 *   The unsetenv() function deletes the variable name from the environment.
 *
 * Parameters:
 *   name - The name of the variable to delete
 *
 * Return Value:
 *   Zero on success
 *
 * Assumptions:
 *   Not called from an interrupt handler
 *
 ****************************************************************************/

int unsetenv(const char *name)
{
  FAR _TCB      *rtcb;
  FAR environ_t *envp;
  FAR char      *pvar;
  int ret = OK;

  /* Verify input parameter */

  if (!name)
    {
      ret = EINVAL;
      goto errout;
    }

  /* Get a reference to the thread-private environ in the TCB.*/

  sched_lock();
  rtcb = (FAR _TCB*)g_readytorun.head;
  envp = rtcb->envp;

  /* Check if the variable exists */

  if ( envp && (pvar = env_findvar(envp, name)) != NULL)
    {
      int        alloc;
      environ_t *tmp;

      /* It does!  Remove the name=value pair from the environment. */

      (void)env_removevar(envp, pvar);

      /* Reallocate the new environment buffer */

      alloc = envp->ev_alloc;
      tmp   = (environ_t*)krealloc(envp, SIZEOF_ENVIRON_T(alloc));
      if (!tmp)
        {
          ret = ENOMEM;
          goto errout_with_lock;
        }

      /* Save the new environment pointer (it might have changed due to reallocation. */

      rtcb->envp = tmp;
    }

  sched_unlock();
  return OK;

errout_with_lock:
  sched_unlock();
errout:
  errno = ret;
  return ERROR;
}

#endif /* CONFIG_DISABLE_ENVIRON */



