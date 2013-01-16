/****************************************************************************
 * fs/fs_mount.c
 *
 *   Copyright (C) 2007-2009, 2011-2013 Gregory Nutt. All rights reserved.
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
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/fs/fs.h>

#include "fs_internal.h"

/* At least one filesystem must be defined, or this file will not compile.
 * It may be desire-able to make filesystems dynamically registered at
 * some time in the future, but at present, this file needs to know about
 * every configured filesystem.
 */

#ifdef CONFIG_FS_READABLE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* In the canonical case, a file system is bound to a block driver.  However,
 * some less typical cases a block driver is not required.  Examples are
 * pseudo file systems (like BINFS) and MTD file systems (like NXFFS).
 *
 * These file systems all require block drivers:
 */

#if defined(CONFIG_FS_FAT) || defined(CONFIG_FS_ROMFS)
#  define BDFS_SUPPORT 1
#endif

/* These file systems do not require block drivers */

#if defined(CONFIG_FS_NXFFS) || defined(CONFIG_FS_BINFS) || defined(CONFIG_NFS) 
#  define NONBDFS_SUPPORT
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct fsmap_t
{
  FAR const char                      *fs_filesystemtype;
  FAR const struct mountpt_operations *fs_mops;
};

/****************************************************************************
 * Private Variables
 ****************************************************************************/

#ifdef BDFS_SUPPORT
#ifdef CONFIG_FS_FAT
extern const struct mountpt_operations fat_operations;
#endif
#ifdef CONFIG_FS_ROMFS
extern const struct mountpt_operations romfs_operations;
#endif

static const struct fsmap_t g_bdfsmap[] =
{
#ifdef CONFIG_FS_FAT
    { "vfat", &fat_operations },
#endif
#ifdef CONFIG_FS_ROMFS
    { "romfs", &romfs_operations },
#endif
    { NULL,   NULL },
};
#endif /* BDFS_SUPPORT*/

#ifdef NONBDFS_SUPPORT
#ifdef CONFIG_FS_NXFFS
extern const struct mountpt_operations nxffs_operations;
#endif
#ifdef CONFIG_NFS
extern const struct mountpt_operations nfs_operations;
#endif
#ifdef CONFIG_FS_BINFS
extern const struct mountpt_operations binfs_operations;
#endif

static const struct fsmap_t g_nonbdfsmap[] =
{
#ifdef CONFIG_FS_NXFFS
    { "nxffs", &nxffs_operations },
#endif
#ifdef CONFIG_NFS
    { "nfs", &nfs_operations },
#endif
#ifdef CONFIG_FS_BINFS
    { "binfs", &binfs_operations },
#endif
    { NULL,   NULL },
};
#endif /* NONBDFS_SUPPORT */

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mount_findfs
 *
 * Description:
 *    find the specified filesystem
 *
 ****************************************************************************/

#if defined(BDFS_SUPPORT) || defined(NONBDFS_SUPPORT)
static FAR const struct mountpt_operations *
mount_findfs(FAR const struct fsmap_t *fstab, FAR const char *filesystemtype)
{
  FAR const struct fsmap_t *fsmap;

  for (fsmap = fstab; fsmap->fs_filesystemtype; fsmap++)
    {
      if (strcmp(filesystemtype, fsmap->fs_filesystemtype) == 0)
        {
            return fsmap->fs_mops;
        }
    }

  return NULL;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mount
 *
 * Description:
 *   mount() attaches the filesystem specified by the 'source' block device
 *   name into the root file system at the path specified by 'target.'
 *
 * Return:
 *   Zero is returned on success; -1 is returned on an error and errno is
 *   set appropriately:
 *
 *   EACCES A component of a path was not searchable or mounting a read-only
 *      filesystem was attempted without giving the MS_RDONLY flag.
 *   EBUSY 'source' is already  mounted.
 *   EFAULT One of the pointer arguments points outside the user address
 *      space.
 *   EINVAL 'source' had an invalid superblock.
 *   ENODEV 'filesystemtype' not configured
 *   ENOENT A pathname was empty or had a nonexistent component.
 *   ENOMEM Could not allocate a memory to copy filenames or data into.
 *   ENOTBLK 'source' is not a block device
 *
 ****************************************************************************/

int mount(FAR const char *source, FAR const char *target,
          FAR const char *filesystemtype, unsigned long mountflags,
          FAR const void *data)
{
#if defined(BDFS_SUPPORT) || defined(NONBDFS_SUPPORT)
#ifdef BDFS_SUPPORT
  FAR struct inode *blkdrvr_inode = NULL;
#endif
  FAR struct inode *mountpt_inode;
  FAR const struct mountpt_operations *mops;
  void *fshandle;
  int errcode;
  int ret;

  /* Verify required pointer arguments */

  DEBUGASSERT(target && filesystemtype);

  /* Find the specified filesystem.  Try the block driver file systems first */

#ifdef BDFS_SUPPORT
  if ((mops = mount_findfs(g_bdfsmap, filesystemtype)) != NULL)
    {
      /* Make sure that a block driver argument was provided */

      DEBUGASSERT(source);

      /* Find the block driver */

      ret = find_blockdriver(source, mountflags, &blkdrvr_inode);
      if (ret < 0)
        {
          fdbg("Failed to find block driver %s\n", source);
          errcode = -ret;
          goto errout;
        }
    }
  else
#endif /* BDFS_SUPPORT */
#ifdef NONBDFS_SUPPORT
  if ((mops = mount_findfs(g_nonbdfsmap, filesystemtype)) != NULL)
    {
    }
  else
#endif /* NONBDFS_SUPPORT */
    {
      fdbg("Failed to find file system %s\n", filesystemtype);
      errcode = ENODEV;
      goto errout;
    }

  /* Insert a dummy node -- we need to hold the inode semaphore
   * to do this because we will have a momentarily bad structure.
   */

  inode_semtake();
  ret = inode_reserve(target, &mountpt_inode);
  if (ret < 0)
    {
      /* inode_reserve can fail for a couple of reasons, but the most likely
       * one is that the inode already exists. inode_reserve may return:
       *
       *  -EINVAL - 'path' is invalid for this operation
       *  -EEXIST - An inode already exists at 'path'
       *  -ENOMEM - Failed to allocate in-memory resources for the operation
       */

      fdbg("Failed to reserve inode\n");
      errcode = -ret;
      goto errout_with_semaphore;
    }

  /* Bind the block driver to an instance of the file system.  The file
   * system returns a reference to some opaque, fs-dependent structure
   * that encapsulates this binding.
   */

  if (!mops->bind)
    {
      /* The filesystem does not support the bind operation ??? */

      fdbg("Filesystem does not support bind\n");
      errcode = EINVAL;
      goto errout_with_mountpt;
    }

  /* Increment reference count for the reference we pass to the file system */

#ifdef BDFS_SUPPORT
#ifdef NONBDFS_SUPPORT
  if (blkdrvr_inode)
#endif
    {
      blkdrvr_inode->i_crefs++;
    }
#endif

  /* On failure, the bind method returns -errorcode */

#ifdef BDFS_SUPPORT
  ret = mops->bind(blkdrvr_inode, data, &fshandle);
#else
  ret = mops->bind(NULL, data, &fshandle);
#endif
  if (ret != 0)
    {
      /* The inode is unhappy with the blkdrvr for some reason.  Back out
       * the count for the reference we failed to pass and exit with an
       * error.
       */

      fdbg("Bind method failed: %d\n", ret);
#ifdef BDFS_SUPPORT
#ifdef NONBDFS_SUPPORT
      if (blkdrvr_inode)
#endif
        {
          blkdrvr_inode->i_crefs--;
        }
#endif
      errcode = -ret;
      goto errout_with_mountpt;
    }

  /* We have it, now populate it with driver specific information. */

  INODE_SET_MOUNTPT(mountpt_inode);

  mountpt_inode->u.i_mops  = mops;
#ifdef CONFIG_FILE_MODE
  mountpt_inode->i_mode    = mode;
#endif
  mountpt_inode->i_private = fshandle;
  inode_semgive();

 /* We can release our reference to the blkdrver_inode, if the filesystem
  * wants to retain the blockdriver inode (which it should), then it must
  * have called inode_addref().  There is one reference on mountpt_inode
  * that will persist until umount() is called.
  */

#ifdef BDFS_SUPPORT
#ifdef NONBDFS_SUPPORT
  if (blkdrvr_inode)
#endif
    {
      inode_release(blkdrvr_inode);
    }
#endif

  return OK;

  /* A lot of goto's!  But they make the error handling much simpler */

errout_with_mountpt:
  mountpt_inode->i_crefs = 0;
  inode_remove(target);
  inode_semgive();
#ifdef BDFS_SUPPORT
#ifdef NONBDFS_SUPPORT
  if (blkdrvr_inode)
#endif
    {
       inode_release(blkdrvr_inode);
    }
#endif

  inode_release(mountpt_inode);
  goto errout;

errout_with_semaphore:
  inode_semgive();
#ifdef BDFS_SUPPORT
#ifdef NONBDFS_SUPPORT
  if (blkdrvr_inode)
#endif
    {
      inode_release(blkdrvr_inode);
    }
#endif

errout:
  set_errno(errcode);
  return ERROR;

#else
  fdbg("No filesystems enabled\n");
  set_errno(ENOSYS);
  return error;
#endif /* BDFS_SUPPORT || NONBDFS_SUPPORT */
}

#endif /* CONFIG_FS_READABLE */

