/****************************************************************************
 * libc/string/lib_psfa_destroy.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

#include <stdlib.h>
#include <spawn.h>
#include <assert.h>

#include <nuttx/spawn.h>

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Name: posix_spawn_file_actions_destroy
 *
 * Description:
 *   The posix_spawn_file_actions_destroy() function destroys the object
 *   referenced by file_actions which was previously intialized by
 *   posix_spawn_file_actions_init(), returning any resources obtained at the
 *   time of initialization to the system for subsequent reuse.  A
 *   posix_spawn_file_actions_t may be reinitialized after having been
 *   destroyed, but must not be reused after destruction, unless it has been
 *   reinitialized.
 *
 * Input Parameters:
 *   file_actions - The posix_spawn_file_actions_t to be destroyed.
 *
 * Returned Value:
 *   On success, these functions return 0; on failure they return an error
 *   number from <errno.h>.
 *
 ****************************************************************************/

int posix_spawn_file_actions_destroy(FAR posix_spawn_file_actions_t *file_actions)
{
  FAR struct spawn_general_file_action_s *curr;
  FAR struct spawn_general_file_action_s *next;

  DEBUGASSERT(file_actions);

  /* Destroy each file action, one at a time */

  for (curr = (FAR struct spawn_general_file_action_s *)*file_actions;
       curr;
       curr = next)
    {
      /* Get the pointer to the next element before destroying the current one */

      next = curr->flink;
      free(curr);
    }

  /* Mark the list empty */

  *file_actions = NULL;
  return OK;
}
