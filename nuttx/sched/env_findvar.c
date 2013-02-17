/****************************************************************************
 * sched/env_findvar.c
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

#include <stdbool.h>
#include <string.h>
#include <sched.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: env_cmpname
 ****************************************************************************/

static bool env_cmpname(const char *pszname, const char *peqname)
{
  /* Search until we find anything different in the two names */

  for (; *pszname == *peqname; pszname++, peqname++);

  /* On sucess, pszname will end with '\0' and peqname with '=' */

  if ( *pszname == '\0' && *peqname == '=' )
    {
      return true;
    }

  return false;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: env_findvar
 *
 * Description:
 *   Search the provided environment structure for the variable of the
 *   specified name.
 *
 * Parameters:
 *   group The task group containging environment array to be searched.
 *   pname The variable name to find
 *
 * Return Value:
 *   A pointer to the name=value string in the environment
 *
 * Assumptions:
 *   - Not called from an interrupt handler
 *   - Pre-emptions is disabled by caller
 *
 ****************************************************************************/

FAR char *env_findvar(FAR struct task_group_s *group, const char *pname)
{
  char *ptr;
  char *end;

  /* Verify input parameters */

  DEBUGASSERT(group && pname);

  /* Search for a name=value string with matching name */

  end = &group->tg_envp[group->tg_envsize];
  for (ptr = group->tg_envp;
       ptr < end && !env_cmpname(pname, ptr);
       ptr += (strlen(ptr) + 1));

  /* Check for success */

  return (ptr < end) ? ptr : NULL;
}

#endif /* CONFIG_DISABLE_ENVIRON */



