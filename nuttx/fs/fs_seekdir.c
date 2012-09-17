/****************************************************************************
 * fs/fs_seekdir.c
 *
 *   Copyright (C) 2007, 2008, 2011 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <dirent.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/dirent.h>

#include "fs_internal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: seekpseudodir
 ****************************************************************************/

static inline void seekpseudodir(struct fs_dirent_s *idir, off_t offset)
{
  struct inode *curr;
  struct inode *prev;
  off_t pos;

  /* Determine a starting point for the seek.  If the seek
   * is "forward" from the current position, then we will
   * start at the current poisition.  Otherwise, we will
   * "rewind" to the root dir.
   */

  if ( offset < idir->fd_position )
    {
      pos  = 0;
      curr = idir->fd_root;
    }
  else
    {
      pos  = idir->fd_position;
      curr = idir->u.pseudo.fd_next;
    }

  /* Traverse the peer list starting at the 'root' of the
   * the list until we find the node at 'offset".  If devices
   * are being registered and unregistered, then this can
   * be a very unpredictable operation.
   */

  inode_semtake();
  for (; curr && pos != offset; pos++, curr = curr->i_peer);

  /* Now get the inode to vist next time that readdir() is called */

  prev                   = idir->u.pseudo.fd_next;
  idir->u.pseudo.fd_next = curr; /* The next node to visit (might be null) */
  idir->fd_position      = pos;  /* Might be beyond the last dirent */

  if (curr)
    {
      /* Increment the reference count on this next node */

      curr->i_crefs++;
    }

  inode_semgive();

  if (prev)
    {
      inode_release(prev);
    }
}

/****************************************************************************
 * Name: seekmountptdir
 ****************************************************************************/

#ifndef CONFIG_DISABLE_MOUNTPOINT
static inline void seekmountptdir(struct fs_dirent_s *idir, off_t offset)
{
  struct inode *inode;
  off_t pos;

  /* Determine a starting point for the seek.  If the seek
   * is "forward" from the current position, then we will
   * start at the current poisition.  Otherwise, we will
   * "rewind" to the root dir.
   */

  inode = idir->fd_root;
  if ( offset < idir->fd_position )
    {
      if (inode->u.i_mops && inode->u.i_mops->rewinddir)
        {
          /* Perform the rewinddir() operation */

          inode->u.i_mops->rewinddir(inode, idir);
          pos = 0;
        }
      else
        {
          /* We can't do the seek and there is no way to return
           * an error indication.
           */

          return;
        }
    }
  else
    {
      pos = idir->fd_position;
    }

  /* This is a brute force approach... we will just read
   * directory entries until we are at the desired position.
   */

  while (pos < offset)
    {
      if (!inode->u.i_mops || !inode->u.i_mops->readdir ||
           inode->u.i_mops->readdir(inode, idir) < 0)
        {
          /* We can't read the next entry and there is no way to return
           * an error indication.
           */

           return;
        }

      /* Increment the position on each successful read */

      pos++;
    }

  /* If we get here the directory position has been successfully set */

  idir->fd_position = pos;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: seekdir
 *
 * Description:
 *   The seekdir() function sets the location in the directory stream from
 *   which the next readdir() call will start.  seekdir() should be used with
 *   an offset returned by telldir().
 *
 * Inputs:
 *   dirp -- An instance of type DIR created by a previous
 *     call to opendir();
 *   offset -- offset to seek to
 *
 * Return:
 *   None
 *
 ****************************************************************************/

void seekdir(FAR DIR *dirp, off_t offset)
{
  struct fs_dirent_s *idir = (struct fs_dirent_s *)dirp;

  /* Sanity checks */

  if (!idir || !idir->fd_root)
    {
      return;
    }

  /* The way we handle the readdir depends on the type of inode
   * that we are dealing with.
   */

#ifndef CONFIG_DISABLE_MOUNTPOINT
  if (INODE_IS_MOUNTPT(idir->fd_root))
    {
      /* The node is a file system mointpoint */

      seekmountptdir(idir, offset);
    }
  else
#endif
    {
      /* The node is part of the root pseudo file system */

      seekpseudodir(idir, offset);
    }
}
