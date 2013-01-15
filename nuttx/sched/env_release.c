/****************************************************************************
 * sched/env_clearenv.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef CONFIG_DISABLE_ENVIRON

#include <sched.h>
#include <errno.h>
#include "os_internal.h"
#include "env_internal.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: env_release
 *
 * Description:
 *   The env_release() function clears the environment of all name-value
 *   pairs and sets the value of the external variable environ to NULL.
 *
 * Parameters:
 *   ptcb Identifies the TCB containing the environment structure
 *
 * Return Value:
 *   zero on success
 *
 * Assumptions:
 *   Not called from an interrupt handler
 *
 ****************************************************************************/

int env_release(FAR _TCB *ptcb)
{
  int ret = OK;

  if (!ptcb)
    {
      ret = -EINVAL;
    }
  else
    {
      FAR environ_t *envp;

      /* Examine the environ data in the TCB.  Preemption is disabled because the
       * the environment could be shared among threads.
       */

      sched_lock();
      envp = ptcb->envp;
      if (ptcb->envp)
        {
          /* Check the reference count on the environment structure */

          if (envp->ev_crefs <= 1)
            {
              /* Decrementing the reference count will destroy the environment */

              sched_free(envp);
            }
          else
            {
              /* The environment will persist after decrementing the reference
               * count */

              envp->ev_crefs--;
            }

          /* In any case, the environment is no longer accessible on this thread */

          ptcb->envp = NULL;
        }

      sched_unlock();
    }

  return ret;
}

#endif /* CONFIG_DISABLE_ENVIRON */



