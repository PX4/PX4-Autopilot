/****************************************************************************
 * libc/string/lib_psfa_addopen.c
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
#include <string.h>
#include <spawn.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/spawn.h>

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Name: posix_spawn_file_actions_addopen
 *
 * Description:
 *   The posix_spawn_file_actions_addopen() function adds an open operation
 *   to the list of operations associated with the object referenced by
 *   file_actions, for subsequent use in a call to posix_spawn() or
 *   posix_spawnp().  The descriptor referred to by fd is opened using
 *   the path, oflag, and mode arguments as if open() had been called on it
 *   prior to the new child process starting execution.  The string path is
 *   copied by the posix_spawn_file_actions_addopen() function during this
 *   process, so storage need not be persistent in the caller.
 *
 * Input Parameters:
 *   file_actions - The posix_spawn_file_actions_t to append the action.
 *   fd - The file descriptor to be opened.
 *   path - The path to be opened.
 *   oflags - Open flags
 *   mode - File creation mode
 *
 * Returned Value:
 *   On success, these functions return 0; on failure they return an error
 *   number from <errno.h>.
 *
 ****************************************************************************/

int posix_spawn_file_actions_addopen(FAR posix_spawn_file_actions_t *file_actions,
                                     int fd, FAR const char *path, int oflags,
                                     mode_t mode)
{
  FAR struct spawn_open_file_action_s *entry;
  size_t len;
  size_t alloc;

  DEBUGASSERT(file_actions && path &&
              fd >= 0 && fd < CONFIG_NFILE_DESCRIPTORS);

  /* Get the size of the action including storage for the path plus its NUL
   * terminating character.
   */

  len   = strlen(path);
  alloc = SIZEOF_OPEN_FILE_ACTION_S(len);

  /* Allocate the action list entry of this size */

  entry = (FAR struct spawn_open_file_action_s *)zalloc(alloc);

  if (!entry)
    {
      return ENOMEM;
    }

  /* Initialize the file action entry */

  entry->action = SPAWN_FILE_ACTION_OPEN;
  entry->fd     = fd;
  entry->oflags = oflags;
  entry->mode   = mode;
  strncpy(entry->path, path, len+1);
  
  /* And add it to the file action list */

  add_file_action(file_actions, (FAR struct spawn_general_file_action_s *)entry);
  return OK;
}
