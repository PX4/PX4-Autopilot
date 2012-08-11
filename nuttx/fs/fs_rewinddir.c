/****************************************************************************
 * fs/fs_rewinddir.c
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

#include <nuttx/fs/fs.h>
#include <nuttx/fs/dirent.h>

#include "fs_internal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rewindpseudodir
 ****************************************************************************/

static inline void rewindpseudodir(struct fs_dirent_s *idir)
{
  struct inode *prev;

  inode_semtake();

  /* Reset the position to the beginning */

  prev                   = idir->u.pseudo.fd_next; /* (Save to delete later) */
  idir->u.pseudo.fd_next = idir->fd_root;          /* The next node to visit */
  idir->fd_position      = 0;                      /* Reset position */

  /* Increment the reference count on the root=next node.  We
   * should now have two references on the inode.
   */

  idir->fd_root->i_crefs++;
  inode_semgive();

  /* Then release the reference to the old next inode */

  if (prev)
    {
      inode_release(prev);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rewinddir
 *
 * Description:
 *   The  rewinddir() function resets the position of the
 *   directory stream dir to the beginning of the directory.
 *
 * Inputs:
 *   dirp -- An instance of type DIR created by a previous
 *     call to opendir();
 *
 * Return:
 *   None
 *
 ****************************************************************************/

void rewinddir(FAR DIR *dirp)
{
  struct fs_dirent_s *idir = (struct fs_dirent_s *)dirp;
#ifndef CONFIG_DISABLE_MOUNTPOINT
  struct inode *inode;
#endif

  /* Sanity checks */

  if (!idir || !idir->fd_root)
    {
      return;
    }

  /* The way we handle the readdir depends on the type of inode
   * that we are dealing with.
   */

#ifndef CONFIG_DISABLE_MOUNTPOINT
  inode = idir->fd_root;
  if (INODE_IS_MOUNTPT(inode))
    {
      /* The node is a file system mointpoint. Verify that the mountpoint
       * supports the rewinddir() method
       */

      if (inode->u.i_mops && inode->u.i_mops->rewinddir)
        {
          /* Perform the rewinddir() operation */

          inode->u.i_mops->rewinddir(inode, idir);
        }
    }
  else
#endif
    {
      /* The node is part of the root pseudo file system */

      rewindpseudodir(idir);
    }
}
