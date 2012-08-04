/****************************************************************************
 * fs/fs_umount.c
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

#include <sys/mount.h>
#include <errno.h>
#include <nuttx/fs/fs.h>

#include "fs_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
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
 * Name: umount
 *
 * Description:
 *   umount() detaches the filesystem mounted at the path specified by
 *  'target.'
 *
 * Return:
 *   Zero is returned on success; -1 is returned on an error and errno is
 *   set appropriately:
 *
 *   EACCES A component of a path was not searchable or mounting a read-only
 *      filesystem was attempted without giving the MS_RDONLY flag.
 *   EBUSY The target could not be unmounted because it is busy.
 *   EFAULT The pointer argument points outside the user address space.
 *
 ****************************************************************************/

int umount(const char *target)
{
  FAR struct inode *mountpt_inode;
  FAR struct inode *blkdrvr_inode = NULL;
  int errcode = OK;
  int status;

  /* Verify required pointer arguments */

  if (!target)
    {
      errcode = EFAULT;
      goto errout;
    }

  /* Find the mountpt */

  mountpt_inode = inode_find(target, NULL);
  if (!mountpt_inode)
    {
      errcode = ENOENT;
      goto errout;
    }

  /* Verify that the inode is a mountpoint */

  if (!INODE_IS_MOUNTPT(mountpt_inode))
    { 
      errcode = EINVAL;
      goto errout_with_mountpt;
   }

  /* Unbind the block driver from the file system (destroying any fs
   * private data.
   */

  if (!mountpt_inode->u.i_mops->unbind)
    {
      /* The filesystem does not support the unbind operation ??? */

      errcode = EINVAL;
      goto errout_with_mountpt;
    }

  /* The unbind method returns the number of references to the
   * filesystem (i.e., open files), zero if the unbind was
   * performed, or a negated error code on a failure.
   */

  inode_semtake(); /* Hold the semaphore through the unbind logic */
  status = mountpt_inode->u.i_mops->unbind( mountpt_inode->i_private, &blkdrvr_inode);
  if (status < 0)
    {
      /* The inode is unhappy with the blkdrvr for some reason */

      errcode = -status;
      goto errout_with_semaphore;
    }
  else if (status > 0)
    {
      errcode = EBUSY;
      goto errout_with_semaphore;
    }

  /* Successfully unbound */

  mountpt_inode->i_private = NULL;

  /* Successfully unbound, remove the mountpoint inode from
   * the inode tree.  The inode will not be deleted yet because
   * there is still at least reference on it (from the mount)
   */

  status = inode_remove(target);
  inode_semgive();

  /* The return value of -EBUSY is normal (in fact, it should
   * not be OK)
   */

  if (status != OK && status != -EBUSY)
    {
      errcode = -status;
      goto errout_with_mountpt;
    }

  /* Release the mountpoint inode and any block driver inode
   * returned by the file system unbind above.  This should cause
   * the inode to be deleted (unless there are other references)
   */

  inode_release(mountpt_inode);

  /* Did the unbind method return a contained block driver */

  if (blkdrvr_inode)
    {
      inode_release(blkdrvr_inode);
    }

  return OK;

  /* A lot of goto's!  But they make the error handling much simpler */

errout_with_semaphore:
  inode_semgive();
errout_with_mountpt:
  inode_release(mountpt_inode);
  if (blkdrvr_inode)
    {
      inode_release(blkdrvr_inode);
    }
errout:
  set_errno(errcode);
  return ERROR;
}

