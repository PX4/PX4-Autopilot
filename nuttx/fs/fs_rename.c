/****************************************************************************
 * fs/fs_rename.c
 *
 *   Copyright (C) 2007-2009 Gregory Nutt. All rights reserved.
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
#include <errno.h>
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rename
 *
 * Description:  Remove a file managed a mountpoint
 *
 ****************************************************************************/

int rename(FAR const char *oldpath, FAR const char *newpath)
{
  FAR struct inode *oldinode;
  FAR struct inode *newinode;
  const char       *oldrelpath = NULL;
  const char       *newrelpath = NULL;
  int               ret;

  /* Get an inode for the old relpath */

  oldinode = inode_find(oldpath, &oldrelpath);
  if (!oldinode)
    {
      /* There is no mountpoint that includes in this path */

      ret = ENOENT;
      goto errout;
    }

  /* Verify that the old inode is a valid mountpoint. */

  if (!INODE_IS_MOUNTPT(oldinode) || !oldinode->u.i_mops)
    {
      ret = ENXIO;
      goto errout_with_oldinode;
    }

  /* Get an inode for the new relpath -- it should like on the same mountpoint */

  newinode = inode_find(newpath, &newrelpath);
  if (!newinode)
    {
      /* There is no mountpoint that includes in this path */

      ret = ENOENT;
      goto errout_with_oldinode;
    }

  /* Verify that the two pathes lie on the same mountpt inode */

  if (oldinode != newinode)
    {
      ret = EXDEV;
      goto errout_with_newinode;
    }

  /* Perform the rename operation using the relative pathes
   * at the common mountpoint.
   */

  if (oldinode->u.i_mops->rename)
    {
      ret = oldinode->u.i_mops->rename(oldinode, oldrelpath, newrelpath);
      if (ret < 0)
        {
          ret = -ret;
          goto errout_with_newinode;
        }
    }
  else
    { 
      ret = ENOSYS;
      goto errout_with_newinode;
    }

  /* Successfully renamed */

  inode_release(oldinode);
  inode_release(newinode);
  return OK;

 errout_with_newinode:
  inode_release(newinode);
 errout_with_oldinode:
  inode_release(oldinode);
 errout:
  set_errno(ret);
  return ERROR;
}
