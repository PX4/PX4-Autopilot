/****************************************************************************
 * libc/string/lib_psfa_adddup2.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
#include <errno.h>

#include "spawn/spawn.h"

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Name: posix_spawn_file_actions_adddup2
 *
 * Description:
 *   The posix_spawn_file_actions_adddup2() function adds a dup2 operation to
 *   the list of operations associated with the object referenced by
 *   file_actions, for subsequent use in a call to posix_spawn() or
 *   posix_spawnp().  The descriptor referred to by fd2 is created as
 *   if dup2() had been called on fd1 prior to the new child process
 *   starting execution.
 *
 * Input Parameters:
 *   file_actions - The posix_spawn_file_actions_t to append the action.
 *   fd1 - The first file descriptor to be argument to dup2.
 *   fd2 - The first file descriptor to be argument to dup2.
 *
 * Returned Value:
 *   On success, these functions return 0; on failure they return an error
 *   number from <errno.h>.
 *
 ****************************************************************************/

int posix_spawn_file_actions_adddup2(FAR posix_spawn_file_actions_t *file_actions,
                                     int fd1, int fd2)
{
  FAR struct spawn_dup2_file_action_s *entry;

  DEBUGASSERT(file_actions &&
              fd1 >= 0 && fd1 < CONFIG_NFILE_DESCRIPTORS &&
              fd2 >= 0 && fd2 < CONFIG_NFILE_DESCRIPTORS);

  /* Allocate the action list entry */

  entry = (FAR struct spawn_dup2_file_action_s *)
    zalloc(sizeof(struct spawn_close_file_action_s));

  if (!entry)
    {
      return ENOMEM;
    }

  /* Initialize the file action entry */

  entry->action = SPAWN_FILE_ACTION_DUP2;
  entry->fd1    = fd1;
  entry->fd2    = fd2;

  /* And add it to the file action list */

  add_file_action(file_actions, (FAR struct spawn_general_file_action_s *)entry);
  return OK;
}
