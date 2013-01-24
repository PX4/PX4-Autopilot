/****************************************************************************
 * fs/binfs/fs_binfs.c
 *
 *   Copyright (C) 2011-2013 Gregory Nutt. All rights reserved.
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
#include <sys/statfs.h>
#include <sys/stat.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/binfs.h>
#include <nuttx/fs/dirent.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/binfmt/builtin.h>

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && defined(CONFIG_FS_BINFS)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     binfs_open(FAR struct file *filep, const char *relpath,
                          int oflags, mode_t mode);
static int     binfs_close(FAR struct file *filep);
static ssize_t binfs_read(FAR struct file *filep, char *buffer, size_t buflen);
static int     binfs_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

static int     binfs_dup(FAR const struct file *oldp, FAR struct file *newp);

static int     binfs_opendir(struct inode *mountpt, const char *relpath,
                             struct fs_dirent_s *dir);
static int     binfs_readdir(FAR struct inode *mountpt,
                             FAR struct fs_dirent_s *dir);
static int     binfs_rewinddir(FAR struct inode *mountpt,
                               FAR struct fs_dirent_s *dir);

static int     binfs_bind(FAR struct inode *blkdriver, FAR const void *data,
                          FAR void **handle);
static int     binfs_unbind(FAR void *handle, FAR struct inode **blkdriver);
static int     binfs_statfs(FAR struct inode *mountpt,
                            FAR struct statfs *buf);

static int     binfs_stat(FAR struct inode *mountpt, FAR const char *relpath,
                          FAR struct stat *buf);

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/* See fs_mount.c -- this structure is explicitly externed there.
 * We use the old-fashioned kind of initializers so that this will compile
 * with any compiler.
 */

const struct mountpt_operations binfs_operations =
{
  binfs_open,        /* open */
  binfs_close,       /* close */
  binfs_read,        /* read */
  NULL,              /* write */
  NULL,              /* seek */
  binfs_ioctl,       /* ioctl */

  NULL,              /* sync */
  binfs_dup,         /* dup */

  binfs_opendir,     /* opendir */
  NULL,              /* closedir */
  binfs_readdir,     /* readdir */
  binfs_rewinddir,   /* rewinddir */

  binfs_bind,        /* bind */
  binfs_unbind,      /* unbind */
  binfs_statfs,      /* statfs */

  NULL,              /* unlink */
  NULL,              /* mkdir */
  NULL,              /* rmdir */
  NULL,              /* rename */
  binfs_stat         /* stat */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: binfs_open
 ****************************************************************************/

static int binfs_open(FAR struct file *filep, FAR const char *relpath,
                      int oflags, mode_t mode)
{
  int index;

  fvdbg("Open '%s'\n", relpath);

  /* BINFS is read-only.  Any attempt to open with any kind of write
   * access is not permitted.
   */

  if ((oflags & O_WRONLY) != 0 || (oflags & O_RDONLY) == 0)
    {
      fdbg("ERROR: Only O_RDONLY supported\n");
      return -EACCES;
    }

  /* Check if the an entry exists with this name in the root directory.
   * so the 'relpath' must be the name of the builtin function.
   */

  index = builtin_isavail(relpath);
  if (index < 0)
    {
      fdbg("ERROR: Builting %s does not exist\n", relpath);
      return -ENOENT;
    }

  /* Save the index as the open-specific state in filep->f_priv */

  filep->f_priv = (FAR void *)index;
  return OK;
}

/****************************************************************************
 * Name: binfs_close
 ****************************************************************************/

static int binfs_close(FAR struct file *filep)
{
  fvdbg("Closing\n");
  return OK;
}

/****************************************************************************
 * Name: binfs_read
 ****************************************************************************/

static ssize_t binfs_read(FAR struct file *filep, char *buffer, size_t buflen)
{
  /* Reading is not supported.  Just return end-of-file */

  fvdbg("Read %d bytes from offset %d\n", buflen, filep->f_pos);
  return 0;
}

/****************************************************************************
 * Name: binfs_ioctl
 ****************************************************************************/

static int binfs_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  int ret;

  fvdbg("cmd: %d arg: %08lx\n", cmd, arg);

  /* Only one IOCTL command is supported */

  if (cmd == FIOC_FILENAME)
    {
      /* IN:  FAR char const ** pointer 
       * OUT: Pointer to a persistent file name (Guaranteed to persist while
       *      the file is open).
       */

      FAR const char **ptr = (FAR const char **)((uintptr_t)arg);
      if (ptr == NULL)
        {
          ret = -EINVAL;
        }
      else
        {
          *ptr = builtin_getname((int)filep->f_priv);
          ret = OK;
        }
    }
  else
    {
      ret = -ENOTTY;
    }

  return ret;
}

/****************************************************************************
 * Name: binfs_dup
 *
 * Description:
 *   Duplicate open file data in the new file structure.
 *
 ****************************************************************************/

static int binfs_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  fvdbg("Dup %p->%p\n", oldp, newp);

  /* Copy the index from the old to the new file structure */

  newp->f_priv = oldp->f_priv;
  return OK;
}

/****************************************************************************
 * Name: binfs_opendir
 *
 * Description:
 *   Open a directory for read access
 *
 ****************************************************************************/

static int binfs_opendir(struct inode *mountpt, const char *relpath,
                         struct fs_dirent_s *dir)
{
  fvdbg("relpath: \"%s\"\n", relpath ? relpath : "NULL");

  /* The requested directory must be the volume-relative "root" directory */

  if (relpath && relpath[0] != '\0')
    {
      return -ENOENT;
    }

  /* Set the index to the first entry */

  dir->u.binfs.fb_index = 0;
  return OK;
}

/****************************************************************************
 * Name: binfs_readdir
 *
 * Description: Read the next directory entry
 *
 ****************************************************************************/

static int binfs_readdir(struct inode *mountpt, struct fs_dirent_s *dir)
{
  FAR const char *name;
  unsigned int index;
  int ret;

  /* Have we reached the end of the directory */

  index = dir->u.binfs.fb_index;
  name = builtin_getname(index);
  if (name == NULL)
    {
      /* We signal the end of the directory by returning the
       * special error -ENOENT
       */

      fvdbg("Entry %d: End of directory\n", index);
      ret = -ENOENT;
    }
  else
    {
      /* Save the filename and file type */

      fvdbg("Entry %d: \"%s\"\n", index, name);
      dir->fd_dir.d_type = DTYPE_FILE;
      strncpy(dir->fd_dir.d_name, name, NAME_MAX+1);

      /* The application list is terminated by an entry with a NULL name.
       * Therefore, there is at least one more entry in the list.
       */

      index++;

      /* Set up the next directory entry offset.  NOTE that we could use the
       * standard f_pos instead of our own private fb_index.
       */

      dir->u.binfs.fb_index = index;
      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: binfs_rewindir
 *
 * Description: Reset directory read to the first entry
 *
 ****************************************************************************/

static int binfs_rewinddir(struct inode *mountpt, struct fs_dirent_s *dir)
{
  fvdbg("Entry\n");

  dir->u.binfs.fb_index = 0;
  return OK;
}

/****************************************************************************
 * Name: binfs_bind
 *
 * Description: This implements a portion of the mount operation. This
 *  function allocates and initializes the mountpoint private data and
 *  binds the blockdriver inode to the filesystem private data.  The final
 *  binding of the private data (containing the blockdriver) to the
 *  mountpoint is performed by mount().
 *
 ****************************************************************************/

static int binfs_bind(FAR struct inode *blkdriver, const void *data,
                      void **handle)
{
  fvdbg("Entry\n");
  return OK;
}

/****************************************************************************
 * Name: binfs_unbind
 *
 * Description: This implements the filesystem portion of the umount
 *   operation.
 *
 ****************************************************************************/

static int binfs_unbind(void *handle, FAR struct inode **blkdriver)
{
  fvdbg("Entry\n");
  return OK;
}

/****************************************************************************
 * Name: binfs_statfs
 *
 * Description: Return filesystem statistics
 *
 ****************************************************************************/

static int binfs_statfs(struct inode *mountpt, struct statfs *buf)
{
  fvdbg("Entry\n");

  /* Fill in the statfs info */

  memset(buf, 0, sizeof(struct statfs));
  buf->f_type    = BINFS_MAGIC;
  buf->f_bsize   = 0;
  buf->f_blocks  = 0;
  buf->f_bfree   = 0;
  buf->f_bavail  = 0;
  buf->f_namelen = NAME_MAX;
  return OK;
}

/****************************************************************************
 * Name: binfs_stat
 *
 * Description: Return information about a file or directory
 *
 ****************************************************************************/

static int binfs_stat(struct inode *mountpt, const char *relpath, struct stat *buf)
{
  fvdbg("Entry\n");

  /* The requested directory must be the volume-relative "root" directory */

  if (relpath && relpath[0] != '\0')
    {
      /* Check if there is a file with this name. */

      if (builtin_isavail(relpath) < 0)
        {
          return -ENOENT;
        }

      /* It's a execute-only file name */

      buf->st_mode = S_IFREG|S_IXOTH|S_IXGRP|S_IXUSR;
    }
  else
    {
      /* It's a read/execute-only directory name */

      buf->st_mode = S_IFDIR|S_IROTH|S_IRGRP|S_IRUSR|S_IXOTH|S_IXGRP|S_IXUSR;
    }

  /* File/directory size, access block size */

  buf->st_size    = 0;
  buf->st_blksize = 0;
  buf->st_blocks  = 0;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* !CONFIG_DISABLE_MOUNTPOINT && CONFIG_FS_BINFS */

