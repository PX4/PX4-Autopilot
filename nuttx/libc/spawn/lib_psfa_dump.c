/****************************************************************************
 * libc/string/lib_psfa_dump.c
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

#include <spawn.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/spawn.h>

#ifdef CONFIG_DEBUG

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_psfa_dump
 *
 * Description:
 *   Show the entryent file actions.
 *
 * Input Parameters:
 *   file_actions - The address of the file_actions to be dumped.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void posix_spawn_file_actions_dump(FAR posix_spawn_file_actions_t *file_actions)
{
  FAR struct spawn_general_file_action_s *entry;

  DEBUGASSERT(file_actions);

  dbg("File Actions[%p->%p]:\n", file_actions, *file_actions);
  if (!*file_actions)
    {
      dbg("  NONE\n");
      return;
    }
  
  /* Destroy each file action, one at a time */

  for (entry = (FAR struct spawn_general_file_action_s *)*file_actions;
       entry;
       entry = entry->flink)
    {
      switch (entry->action)
        {
        case SPAWN_FILE_ACTION_CLOSE:
          {
            FAR struct spawn_close_file_action_s *action =
              (FAR struct spawn_close_file_action_s *)entry;

            dbg("  CLOSE: fd=%d\n", action->fd);
          }
          break;

        case SPAWN_FILE_ACTION_DUP2:
          {
            FAR struct spawn_dup2_file_action_s *action =
              (FAR struct spawn_dup2_file_action_s *)entry;

            dbg("  DUP2: %d->%d\n", action->fd1, action->fd2);
          }
          break;

        case SPAWN_FILE_ACTION_OPEN:
          {
            FAR struct spawn_open_file_action_s *action =
              (FAR struct spawn_open_file_action_s *)entry;

            svdbg("  OPEN: path=%s oflags=%04x mode=%04x fd=%d\n",
                  action->path, action->oflags, action->mode, action->fd);
          }
          break;

        case SPAWN_FILE_ACTION_NONE:
        default:
          dbg("  ERROR: Unknown action: %d\n", entry->action);
          break;
        }
    }
}

#endif /* CONFIG_DEBUG */