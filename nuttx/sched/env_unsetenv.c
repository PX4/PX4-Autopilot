/****************************************************************************
 * sched/env_unsetenv.c
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

int unsetenv(FAR const char *name)
{
  FAR _TCB *rtcb = (FAR _TCB*)g_readytorun.head;
  FAR struct task_group_s *group = rtcb->group;
  FAR char *pvar;
  FAR char *newenvp;
  int newsize;
  int ret = OK;

  DEBUGASSERT(name && group);

  /* Check if the variable exists */

  sched_lock();
  if (group && (pvar = env_findvar(group, name)) != NULL)
    {
      /* It does!  Remove the name=value pair from the environment. */

      (void)env_removevar(group, pvar);

      /* Reallocate the new environment buffer */

      newsize = group->tg_envsize;
      newenvp = (FAR char *)krealloc(group->tg_envp, newsize);
      if (!newenvp)
        {
          set_errno(ENOMEM);
          ret = ERROR;
        }
      else
        {
          /* Save the new environment pointer (it might have changed due to
           * reallocation.
           */

          group->tg_envp = newenvp;
        }
    }

  sched_unlock();
  return ret;
}

#endif /* CONFIG_DISABLE_ENVIRON */



