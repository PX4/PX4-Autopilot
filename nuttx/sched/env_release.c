/****************************************************************************
 * sched/env_release.c
 *
 *   Copyright (C) 2007, 2009, 2013 Gregory Nutt. All rights reserved.
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
 *   env_release() is called only from group_leave() when the last member of
 *   a task group exits.  The env_release() function clears the environment
 *   of all name-value pairs and sets the value of the external variable
 *   environ to NULL.
 *
 * Parameters:
 *   tcb Identifies the TCB containing the environment structure to be
 *   released.
 *
 * Return Value:
 *   None
 *
 * Assumptions:
 *   Not called from an interrupt handler
 *
 ****************************************************************************/

void env_release(FAR _TCB *tcb)
{
  FAR struct task_group_s *group;

  DEBUGASSERT(tcb && tcb->group);
  group = tcb->group;

  /* Free any allocate environment strings */

  if (group->tg_envp)
    {
      /* Free the environment */

      sched_free(group->tg_envp);
    }

  /* In any event, make sure that all environment-related varialbles in the
   * task group structure are reset to initial values.
   */

  group->tg_envsize = 0;
  group->tg_envp = NULL;
}

#endif /* CONFIG_DISABLE_ENVIRON */



