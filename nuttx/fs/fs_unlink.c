/****************************************************************************
 * fs_unlink.c
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

#include <unistd.h>
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
 * Name: unlink
 *
 * Description:  Remove a file managed a mountpoint
 *
 ****************************************************************************/

int unlink(FAR const char *pathname)
{
  FAR struct inode *inode;
  const char       *relpath = NULL;
  int               ret;

  /* Get an inode for this file */

  inode = inode_find(pathname, &relpath);
  if (!inode)
    {
      /* There is no mountpoint that includes in this path */

      ret = ENOENT;
      goto errout;
    }

  /* Verify that the inode is a valid mountpoint. */

  if (!INODE_IS_MOUNTPT(inode) || !inode->u.i_mops)
    {
      ret = ENXIO;
      goto errout_with_inode;
    }

  /* Perform the unlink operation using the relative path
   * at the mountpoint.
   */

  if (inode->u.i_mops->unlink)
    {
      ret = inode->u.i_mops->unlink(inode, relpath);
      if (ret < 0)
        {
          ret = -ret;
          goto errout_with_inode;
        }
    }
  else
    { 
      ret = ENOSYS;
      goto errout_with_inode;
    }

  /* Successfully unlinked */

  inode_release(inode);
  return OK;

 errout_with_inode:
  inode_release(inode);
 errout:
  set_errno(ret);
  return ERROR;
}

