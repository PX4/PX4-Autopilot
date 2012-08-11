/****************************************************************************
 * fs/fs_foreachmountpoint.c
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

#include <sys/statfs.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

#include "fs_internal.h"

#ifndef CONFIG_DISABLE_MOUNTPOUNT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure just remembers the final consumer of the mountpoint
 * information (and its argument).
 */

struct enum_mountpoint_s
{
  foreach_mountpoint_t handler;
  FAR void            *arg;
};

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int mountpoint_filter(FAR struct inode *node,
                             FAR char dirpath[PATH_MAX], FAR void *arg)
{
  FAR struct enum_mountpoint_s *info = (FAR struct enum_mountpoint_s *)arg;
  struct statfs statbuf;
  int pathlen;
  int namlen;
  int ret = OK;

  DEBUGASSERT(node && info && info->handler);

  /* Check if the inode is a mountpoint.  Mountpoints must support statfs.
   * If this one does not for some reason, then it will be ignored.
   *
   * The root node is a special case:  It has no operations (u.i_mops == NULL)
   */

  if (INODE_IS_MOUNTPT(node) && node->u.i_mops && node->u.i_mops->statfs)
    {
      /* Yes... get the full path to the inode by concatenating the inode
       * name and the path to the directory containing the inode.
       */

      pathlen = strlen(dirpath);
      namlen  = strlen(node->i_name) + 1;

      /* Make sure that this would not exceed the maximum path length */

      if (pathlen + namlen > PATH_MAX)
        {
          return -ENAMETOOLONG;
        }

      /* Append the inode name to the directory path */

      sprintf(&dirpath[pathlen], "/%s", node->i_name);

      /* Get the status of the file system */

      ret = node->u.i_mops->statfs(node, &statbuf);
      if (ret == OK)
        {
          /* And pass the full path and file system status to the handler */

          ret = info->handler(dirpath, &statbuf, info->arg);
        }

      /* Truncate the path name back to the correct length */

      dirpath[pathlen] = '\0';
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

 /****************************************************************************
 * Name: foreach_mountpoint
 *
 * Description:
 *   Visit each mountpoint in the pseudo-file system.  The traversal is
 *   terminated when the callback 'handler' returns a non-zero value, or when
 *   all of the mountpoints have been visited.
 *
 *   This is just a front end "filter" to foreach_inode() that forwards only
 *   mountpoint inodes.  It is intended to support the mount() command to
 *   when the mount command is used to enumerate mounts.
 *
 *   NOTE 1: Use with caution... The pseudo-file system is locked throughout
 *   the traversal.
 *   NOTE 2: The search algorithm is recursive and could, in principle, use
 *   an indeterminant amount of stack space.  This will not usually be a
 *   real work issue.
 *
 ****************************************************************************/

int foreach_mountpoint(foreach_mountpoint_t handler, FAR void *arg)
{
  struct enum_mountpoint_s info;

  /* Let foreach_inode do the real work */

  info.handler = handler;
  info.arg     = arg;

  return foreach_inode(mountpoint_filter, (FAR void *)&info);
}

#endif
