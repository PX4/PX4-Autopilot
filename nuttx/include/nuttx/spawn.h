/****************************************************************************
 * include/nuttx/spawn.h
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

#ifndef __INCLUDE_NUTTX_SPAWN_H
#define __INCLUDE_NUTTX_SPAWN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <spawn.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Type Definitions
 ****************************************************************************/
/* This enumerator identifies a file action */

enum spawn_file_actions_e
{
  SPAWN_FILE_ACTION_NONE = 0,
  SPAWN_FILE_ACTION_CLOSE,
  SPAWN_FILE_ACTION_DUP2,
  SPAWN_FILE_ACTION_OPEN
};

/* posix_spawn_file_actions_addclose(), posix_spawn_file_actions_adddup2(),
 * and posix_spawn_file_actions_addopen() will allocate memory and append
 * a new file action to an instance of posix_spawn_file_actions_t.  The
 * internal representation of these structures are defined below:
 */

struct spawn_general_file_action_s
{
  FAR struct spawn_general_file_action_s *flink;  /* Supports a singly linked list */
  enum spawn_file_actions_e action;               /* A member of enum spawn_file_actions_e */
};

struct spawn_close_file_action_s
{
  FAR struct spawn_general_file_action_s *flink;  /* Supports a singly linked list */
  enum spawn_file_actions_e action;               /* SPAWN_FILE_ACTION_CLOSE */
  int fd;                                         /* The file descriptor to close */
};

struct spawn_dup2_file_action_s
{
  FAR struct spawn_general_file_action_s *flink;  /* Supports a singly linked list */
  enum spawn_file_actions_e action;               /* SPAWN_FILE_ACTION_DUP2 */
  int fd1;                                        /* The first file descriptor for dup2() */
  int fd2;                                        /* The second file descriptor for dup2() */
};

struct spawn_open_file_action_s
{
  FAR struct spawn_general_file_action_s *flink;  /* Supports a singly linked list */
  enum spawn_file_actions_e action;               /* SPAWN_FILE_ACTION_OPEN */
  int fd;                                         /* The file descriptor after opening */
  int oflags;                                     /* Open flags */
  mode_t mode;                                    /* File creation mode */
  char path[1];                                   /* Start of the path to be
                                                   * opened */
};

#define SIZEOF_OPEN_FILE_ACTION_S(n) \
  (sizeof(struct spawn_open_file_action_s) + (n))

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

void add_file_action(FAR posix_spawn_file_actions_t *file_action,
                     FAR struct spawn_general_file_action_s *entry);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_SPAWN_H */
