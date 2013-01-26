/****************************************************************************
 * libc/string/lib_psfa_addaction.c
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

#include <nuttx/spawn.h>

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Name: add_file_action
 *
 * Description:
 *   Add the file action to the end for the file action list.
 *
 * Input Parameters:
 *   file_actions - The head of the file action list.
 *   entry - The file action to be added
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void add_file_action(FAR posix_spawn_file_actions_t *file_actions,
                     FAR struct spawn_general_file_action_s *entry)
{
  FAR struct spawn_general_file_action_s *prev;
  FAR struct spawn_general_file_action_s *next;

  /* Find the end of the list */

  for (prev = NULL, next = (FAR struct spawn_general_file_action_s *)*file_actions;
       next;
       prev = next, next = next->flink);

  /* Here next is NULL and prev points to the last entry in the list (or
   * is NULL if the list is empty).
   */

  if (prev)
    {
      prev->flink = entry;
    }
  else
    {
      *file_actions = (posix_spawn_file_actions_t)entry;
    }

  entry->flink = NULL;
}
