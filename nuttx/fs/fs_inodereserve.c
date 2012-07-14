/****************************************************************************
 * fs/fs_registerreserve.c
 *
 *   Copyright (C) 2007-2009, 2011-2012 Gregory Nutt. All rights reserved.
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

#include <assert.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>

#include "fs_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: inode_namelen
 ****************************************************************************/

static int inode_namelen(FAR const char *name)
{
  const char *tmp = name;
  while(*tmp && *tmp != '/') tmp++;
  return tmp - name;
}

/****************************************************************************
 * Name: inode_namecpy
 ****************************************************************************/

static void inode_namecpy(char *dest, const char *src)
{
  while(*src && *src != '/') *dest++ = *src++;
  *dest='\0';
}

/****************************************************************************
 * Name: inode_alloc
 ****************************************************************************/

static FAR struct inode *inode_alloc(FAR const char *name)
{
  int namelen = inode_namelen(name);
  FAR struct inode *node = (FAR struct inode*)kzalloc(FSNODE_SIZE(namelen));
  if (node)
    {
      inode_namecpy(node->i_name, name);
    }

  return node;
}

/****************************************************************************
 * Name: inode_insert
 ****************************************************************************/

static void inode_insert(FAR struct inode *node,
                         FAR struct inode *peer,
                         FAR struct inode *parent)
{
  /* If peer is non-null, then new node simply goes to the right
   * of that peer node.
   */

  if (peer)
    {
      node->i_peer = peer->i_peer;
      peer->i_peer = node;
    }

  /* If parent is non-null, then it must go at the head of its
   * list of children.
   */

  else if (parent)
    {
      node->i_peer    = parent->i_child;
      parent->i_child = node;
    }

  /* Otherwise, this must be the new root_inode */

  else
    {
      node->i_peer = root_inode;
      root_inode   = node;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: inode_reserve
 *
 * Description:
 *   Reserve an (initialized) inode the pseudo file system.
 *
 *   NOTE: Caller must hold the inode semaphore
 *
 * Input parameters:
 *   path - The path to the inode to create
 *   inode - The location to return the inode pointer
 *
 * Returned Value:
 *   Zero on success (with the inode point in 'inode'); A negated errno
 *   value is returned on failure:
 *
 *   EINVAL - 'path' is invalid for this operation
 *   EEXIST - An inode already exists at 'path'
 *   ENOMEM - Failed to allocate in-memory resources for the operation
 *
 ****************************************************************************/

int inode_reserve(FAR const char *path, FAR struct inode **inode)
{
  const char       *name = path;
  FAR struct inode *left;
  FAR struct inode *parent;

  /* Assume failure */

  DEBUGASSERT(path && inode);
  *inode = NULL;

  /* Handle paths that are interpreted as the root directory */

  if (!*path || path[0] != '/')
    {
      return -EINVAL;
    }

  /* Find the location to insert the new subtree */

  if (inode_search(&name, &left, &parent, (FAR const char **)NULL) != NULL)
    {
      /* It is an error if the node already exists in the tree */

      return -EEXIST;
    }

  /* Now we now where to insert the subtree */

  for (;;)
    {
      FAR struct inode *node;

      /* Create a new node -- we need to know if this is the
       * the leaf node or some intermediary.  We can find this
       * by looking at the next name.
       */

      FAR const char *next_name = inode_nextname(name);
      if (*next_name)
        {
          /* Insert an operationless node */

          node = inode_alloc(name);
          if (node)
            {
              inode_insert(node, left, parent);

              /* Set up for the next time through the loop */

              name   = next_name;
              left   = NULL;
              parent = node;
              continue;
            }
        }
      else
        {
          node = inode_alloc(name);
          if (node)
            {
              inode_insert(node, left, parent);
              *inode = node;
              return OK;
            }
        }

      /* We get here on failures to allocate node memory */

      return -ENOMEM;
    }
}
