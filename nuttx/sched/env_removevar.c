/****************************************************************************
 * sched/env_removevar.c
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

#include <string.h>
#include <sched.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: env_removevar
 *
 * Description:
 *   Remove the referenced name=value pair from the environment
 *
 * Parameters:
 *   group The task group with the environment containing the name=value pair
 *   pvar A pointer to the name=value pair in the restroom
 *
 * Return Value:
 *   Zero on success
 *
 * Assumptions:
 *   - Not called from an interrupt handler
 *   - Caller has pre-emptions disabled
 *   - Caller will reallocate the environment structure to the correct size
 *
 ****************************************************************************/

int env_removevar(FAR struct task_group_s *group, FAR char *pvar)
{
  FAR char *end;    /* Pointer to the end+1 of the environment */
  int alloc;        /* Size of the allocated environment */
  int ret = ERROR;

  DEBUGASSERT(group && pvar);

  /* Verify that the pointer lies within the environment region */

  alloc = group->tg_envsize;          /* Size of the allocated environment */
  end   = &group->tg_envp[alloc];     /* Pointer to the end+1 of the environment */

  if (pvar >= group->tg_envp && pvar < end)
    {
      /* Set up for the removal */

      int   len  = strlen(pvar) + 1;  /* Length of name=value string to remove */
      char *src  = &pvar[len];        /* Address of name=value string after */
      char *dest = pvar;              /* Location to move the next string */
      int   count = end - src;        /* Number of bytes to move (might be zero) */

      /* Move all of the environment strings after the removed one 'down.'
       * this is inefficient, but robably not high duty.
       */

      while (count-- > 0)
        {
          *dest++ = *src++;
        }

      /* Then set to the new allocation size.  The caller is expected to
       * call realloc at some point but we don't do that here because the
       * caller may add more stuff to the environment.
       */

      group->tg_envsize -= len;
      ret = OK;
    }

  return ret;
}

#endif /* CONFIG_DISABLE_ENVIRON */



