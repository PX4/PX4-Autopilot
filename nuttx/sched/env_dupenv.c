/****************************************************************************
 * eched/env_dupenv.c
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

#include <sys/types.h>
#include <sched.h>

#include <nuttx/kmalloc.h>

#include "os_internal.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dupenv
 *
 * Description:
 *   Copy the internal environment structure of a task.  This is the action
 *   that is performed when a new task is created: The new task has a private,
 *   exact duplicate of the parent task's environment.
 *
 * Parameters:
 *   None
 *
 * Return Value:
 *   A pointer to a newly allocated copy of the specified TCB's environment
 *   structure with reference count equal to one.
 *
 * Assumptions:
 *   Not called from an interrupt handler.
 *
 ****************************************************************************/

FAR environ_t *dupenv(FAR _TCB *ptcb)
{
   environ_t *envp = NULL;

  /* Pre-emption must be disabled throughout the following because the
   * environment may be shared.
   */

  sched_lock();

  /* Does the parent task have an environment? */

  if (ptcb->envp)
    {
      /* Yes..The parent task has an environment, duplicate it */

      size_t envlen =  ptcb->envp->ev_alloc
      envp          = (environ_t*)kmalloc(SIZEOF_ENVIRON_T( envlen ));
      if (envp)
        {
          envp->ev_crefs = 1;
          envp->ev_alloc = envlen;
          memcmp( envp->ev_env, ptcb->envp->ev_env, envlen );
        }
    }

  sched_unlock();
  return envp;
}

#endif /* CONFIG_DISABLE_ENVIRON */



