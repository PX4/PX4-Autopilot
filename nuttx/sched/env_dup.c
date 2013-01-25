/****************************************************************************
 * sched/env_dup.c
 *
 *   Copyright (C) 2007, 2009, 2011, 2013 Gregory Nutt. All rights reserved.
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
 * Name: env_dup
 *
 * Description:
 *   Copy the internal environment structure of a task.  This is the action
 *   that is performed when a new task is created: The new task has a private,
 *   exact duplicate of the parent task's environment.
 *
 * Parameters:
 *   ptcb The tcb to receive the newly allocated copy of the parent
 *        TCB's environment structure with reference count equal to one
 *
 * Return Value:
 *   zero on success
 *
 * Assumptions:
 *   Not called from an interrupt handler.
 *
 ****************************************************************************/

int env_dup(FAR _TCB *ptcb)
{
  FAR _TCB  *parent = (FAR _TCB*)g_readytorun.head;
  environ_t *envp   = NULL;
  int ret = OK;

  DEBUGASSERT(ptcb);

  /* Pre-emption must be disabled throughout the following because the
   * environment may be shared.
   */

  sched_lock();

  /* Does the parent task have an environment? */

  if (parent->envp)
    {
      /* Yes..The parent task has an environment, duplicate it */

      size_t envlen =  parent->envp->ev_alloc;
      envp          = (environ_t*)kmalloc(SIZEOF_ENVIRON_T(envlen));
      if (!envp)
        {
          ret = -ENOMEM;
        }
      else
        {
          envp->ev_crefs = 1;
          envp->ev_alloc = envlen;
          memcpy(envp->ev_env, parent->envp->ev_env, envlen);
        }
    }

  /* Save the cloned environment in the new TCB */

  ptcb->envp = envp;
  sched_unlock();
  return ret;
}

#endif /* CONFIG_DISABLE_ENVIRON */



