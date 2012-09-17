/****************************************************************************
 * fs/fs_foreachinode.c
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

#include "fs_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Is it better to allocate the struct inode_path_s from the heap? or
 * from the stack?  This decision depends on how often this is down and
 * how much stack space you can afford.
 */

#define ENUM_INODE_ALLOC 1

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure manages the full path to the inode. */

struct inode_path_s
{
  foreach_inode_t handler;
  FAR void       *arg;
  char            path[CONFIG_PATH_MAX];
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
/****************************************************************************
 * Name: foreach_inodelevel
 *
 * Description:
 *   This is the recursive 'heart' of foreach_inode.  It will visit each
 *   inode at this level in the hierarchy and recurse handle each inode
 *   at the next level down.
 *
 * Assumptions:
 *   The caller holds the inode semaphore.
 *
 ****************************************************************************/

int foreach_inodelevel(FAR struct inode *node, struct inode_path_s *info)
{
  int ret = OK;

  /* Visit each node at this level */

  for (; node; node = node->i_peer)
    {
      /* Give the next inode to the callback */

      ret = info->handler(node, info->path, info->arg);

      /* Break out of the looop early if the handler returns a non-zero
       * value
       */

      if (ret != 0)
        {
          break;
        }

      /* If there is a level 'beneath' this one, then recurse to visit all
       * of the inodes at that level.
       */

      if (node->i_child)
        {
          /* Construct the path to the next level */

          int pathlen = strlen(info->path);
          int namlen  = strlen(node->i_name) + 1;

          /* Make sure that this would not exceed the maximum path length */

          if (pathlen + namlen > PATH_MAX)
            {
              ret = -ENAMETOOLONG;
              break;
            }

          /* Append the path segment to this inode and recurse */

          sprintf(&info->path[pathlen], "/%s", node->i_name);
          ret = foreach_inodelevel(node->i_child, info);

          /* Truncate the path name back to the correct length */

          info->path[pathlen] = '\0';

          /* Return early if the handler at the lower level returned a non-
           * zero value
           */

          if (ret != 0)
            {
              break;
            }
        }
    }

  /* Return the result of the traversal. */

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: foreach_inode
 *
 * Description:
 *   Visit each inode in the pseudo-file system.  The traversal is terminated
 *   when the callback 'handler' returns a non-zero value, or when all of
 *   the inodes have been visited.
 *
 *   NOTE 1: Use with caution... The pseudo-file system is locked throughout
 *   the traversal.
 *   NOTE 2: The search algorithm is recursive and could, in principle, use
 *   an indeterminant amount of stack space.  This will not usually be a
 *   real work issue.
 *
 ****************************************************************************/

int foreach_inode(foreach_inode_t handler, FAR void *arg)
{
#ifdef ENUM_INODE_ALLOC
  FAR struct inode_path_s *info;
  int ret;

  /* Allocate the mountpoint info structure */

  info = (FAR struct inode_path_s *)malloc(sizeof(struct inode_path_s));
  if (!info)
    {
      return -ENOMEM;
    }

  /* Initialize the info structure */

  info->handler = handler;
  info->arg     = arg;
  info->path[0] = '\0';

  /* Start the recursion at the root inode */

  inode_semtake();
  ret = foreach_inodelevel(root_inode, info);
  inode_semgive();

  /* Free the info structure and return the result */

  free(info);
  return ret;

#else
  struct inode_path_s info;
  int ret;

  /* Initialize the info structure */

  info.handler = handler;
  info.arg     = arg;
  info.path[0] = '\0';

  /* Start the recursion at the root inode */

  inode_semtake();
  ret = foreach_inodelevel(root_inode, &info);
  inode_semgive();

  return ret;

#endif
}

