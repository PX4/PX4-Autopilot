/****************************************************************************
 * fs/fs_closedir.c
 *
 *   Copyright (C) 2007-2009, 2011 Gregory Nutt. All rights reserved.
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

#include <dirent.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/dirent.h>

#include "fs_internal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: closedir
 *
 * Description:
 *    The closedir() function closes the directory stream associated with
 *    'dirp'.  The directory stream descriptor 'dirp' is not available after
 *    this call.
 *
 * Inputs:
 *   dirp -- An instance of type DIR created by a previous call to opendir();
 *
 * Return:
 *   The closedir() function returns 0 on success.  On error, -1 is
 *   returned, and errno is set appropriately.
 *
 ****************************************************************************/

int closedir(FAR DIR *dirp)
{
  struct fs_dirent_s *idir = (struct fs_dirent_s *)dirp;
  struct inode *inode;
  int ret;

  if (!idir || !idir->fd_root)
    {
      ret = EBADF;
      goto errout;
    }

  /* This is the 'root' inode of the directory.  This means different
   * things wih different filesystems.
   */

  inode = idir->fd_root;

  /* The way that we handle the close operation depends on what kind of root
   * inode we have open.
   */

#ifndef CONFIG_DISABLE_MOUNTPOINT
  if (INODE_IS_MOUNTPT(inode) && !DIRENT_ISPSUEDONODE(idir->fd_flags))
    {
      /* The node is a file system mointpoint. Verify that the mountpoint
       * supports the closedir() method (not an error if it does not)
       */

      if (inode->u.i_mops && inode->u.i_mops->closedir)
        {
          /* Perform the closedir() operation */

          ret = inode->u.i_mops->closedir(inode, idir);
          if (ret < 0)
            {
              ret = -ret;
              goto errout_with_inode;
            }
        }
    }
  else
#endif
    {
      /* The node is part of the root psuedo file system, release
       * our contained reference to the 'next' inode.
       */

      if (idir->u.psuedo.fd_next)
        {
          inode_release(idir->u.psuedo.fd_next);
        }
    }

  /* Release our references on the contained 'root' inode */

  inode_release(idir->fd_root);

  /* Then release the container */

  kfree(idir);
  return OK;

#ifndef CONFIG_DISABLE_MOUNTPOINT
errout_with_inode:
  inode_release(inode);
  kfree(idir);
#endif

errout:
  set_errno(ret);
  return ERROR;
}
