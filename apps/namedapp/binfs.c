/****************************************************************************
 * apps/namedapps/binfs.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs.h>
#include <nuttx/dirent.h>

#include "namedapp.h"

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && defined(CONFIG_APPS_BINDIR)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents the overall mountpoint state.  An instance of this
 * structure is retained as inode private data on each mountpoint that is
 * mounted with a fat32 filesystem.
 */

struct binfs_state_s
{
  sem_t    bm_sem;                  /* Used to assume thread-safe access */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void    binfs_semtake(struct binfs_state_s *bm);
static inline void binfs_semgive(struct binfs_state_s *bm);
static int     binfs_open(FAR struct file *filep, const char *relpath,
                        int oflags, mode_t mode);
static int     binfs_close(FAR struct file *filep);
static ssize_t binfs_read(FAR struct file *filep, char *buffer, size_t buflen);
static int     binfs_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

static int     binfs_opendir(struct inode *mountpt, const char *relpath,
                             struct fs_dirent_s *dir);
static int     binfs_readdir(struct inode *mountpt, struct fs_dirent_s *dir);
static int     binfs_rewinddir(struct inode *mountpt, struct fs_dirent_s *dir);

static int     binfs_bind(FAR struct inode *blkdriver, const void *data,
                        void **handle);
static int     binfs_unbind(void *handle, FAR struct inode **blkdriver);
static int     binfs_statfs(struct inode *mountpt, struct statfs *buf);

static int     binfs_stat(struct inode *mountpt, const char *relpath, struct stat *buf);

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
 * Name: binfs_semtake
 ****************************************************************************/

static void binfs_semtake(struct binfs_state_s *bm)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(&bm->bm_sem) != 0)
    {
      /* The only case that an error should occur here is if
       * the wait was awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

/****************************************************************************
 * Name: binfs_semgive
 ****************************************************************************/

static inline void binfs_semgive(struct binfs_state_s *bm)
{
   sem_post(&bm->bm_sem);
}

/****************************************************************************
 * Name: binfs_open
 ****************************************************************************/

static int binfs_open(FAR struct file *filep, const char *relpath,
                    int oflags, mode_t mode)
{
  struct binfs_state_s *bm;
  int                   ret = -ENOSYS;

  fvdbg("Open '%s'\n", relpath);

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv == NULL && filep->f_inode != NULL);

  /* mountpoint private data from the inode reference from the file
   * structure
   */

  bm = (struct binfs_state_s*)filep->f_inode->i_private;
  DEBUGASSERT(bm != NULL);

  /* BINFS is read-only.  Any attempt to open with any kind of write
   * access is not permitted.
   */

  if ((oflags & O_WRONLY) != 0 || (oflags & O_RDONLY) == 0)
    {
      fdbg("Only O_RDONLY supported\n");
      ret = -EACCES;
    }

  /* Save open-specific state in filep->f_priv */

  /* Opening of elements within the pseudo-file system is not yet supported */

  return ret;
}

/****************************************************************************
 * Name: binfs_close
 ****************************************************************************/

static int binfs_close(FAR struct file *filep)
{
  struct binfs_state_s *bm;
  int                   ret = -ENOSYS;

  fvdbg("Closing\n");

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover the open file state from the struct file instance */
  /* bf = filep->f_priv; */

  /* Recover the file system state from the inode */

  bm = filep->f_inode->i_private;
  DEBUGASSERT(bm != NULL);

  /* Free the open file state */
  /* free(bf); */

  filep->f_priv = NULL;

  /* Since open() is not yet supported, neither is close(). */

  return ret;
}

/****************************************************************************
 * Name: binfs_read
 ****************************************************************************/

static ssize_t binfs_read(FAR struct file *filep, char *buffer, size_t buflen)
{
  struct binfs_state_s *bm;

  fvdbg("Read %d bytes from offset %d\n", buflen, filep->f_pos);

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover the open file state data from the struct file instance */
  /* bf = filep->f_priv; */

  /* Recover the file system state from the inode */

  bm = filep->f_inode->i_private;
  DEBUGASSERT(bm != NULL);

  /* Since open is not yet supported, neither is reading */

  return -ENOSYS;
}

/****************************************************************************
 * Name: binfs_ioctl
 ****************************************************************************/

static int binfs_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  struct binfs_state_s *bm;

  fvdbg("cmd: %d arg: %08lx\n", cmd, arg);

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover the open file state from the struct file instance */
  /* bf = filep->f_priv; */

  /* Recover the file system state from the inode */

  bm = filep->f_inode->i_private;
  DEBUGASSERT(bm != NULL);

  /* No ioctl commands yet supported */

  return -ENOTTY;
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
  struct binfs_state_s   *bm;
  int                     ret;

  fvdbg("relpath: \"%s\"\n", relpath ? relpath : "NULL");

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover the file system state from the inode instance */

  bm = mountpt->i_private;
  binfs_semtake(bm);

  /* The requested directory must be the volume-relative "root" directory */

  if (relpath && relpath[0] != '\0')
    {
      ret = -ENOENT;
      goto errout_with_semaphore;
    }

  /* Set the index to the first entry */

  dir->u.binfs.fb_index = 0;
  ret = OK;

errout_with_semaphore:
  binfs_semgive(bm);
  return ret;
}

/****************************************************************************
 * Name: binfs_readdir
 *
 * Description: Read the next directory entry
 *
 ****************************************************************************/

static int binfs_readdir(struct inode *mountpt, struct fs_dirent_s *dir)
{
  struct binfs_state_s *bm;
  unsigned int index;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover the file system state from the inode instance */

  bm = mountpt->i_private;
  binfs_semtake(bm);

  /* Have we reached the end of the directory */

  index = dir->u.binfs.fb_index;
  if (namedapps[index].name == NULL)
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

      fvdbg("Entry %d: \"%s\"\n", index, namedapps[index].name);
      dir->fd_dir.d_type = DTYPE_FILE;
      strncpy(dir->fd_dir.d_name, namedapps[index].name, NAME_MAX+1);

      /* The application list is terminated by an entry with a NULL name.
       * Therefore, there is at least one more entry in the list.
       */

      index++;

      /* Set up the next directory entry offset.  NOTE that we could use the
       * standard f_pos instead of our own private fb_index.
       */

      dir->u.binfs.fb_index      = index;
      ret                        = OK;
    }

  binfs_semgive(bm);
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
  struct binfs_state_s *bm;

  fvdbg("Entry\n");

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover the file system state from the inode instance */

  bm = mountpt->i_private;
  binfs_semtake(bm);

  dir->u.binfs.fb_index      = 0;

  binfs_semgive(bm);
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
  struct binfs_state_s *bm;

  fvdbg("Entry\n");

  /* Create an instance of the mountpt state structure */

  bm = (struct binfs_state_s *)zalloc(sizeof(struct binfs_state_s));
  if (!bm)
    {
      fdbg("Failed to allocate mountpoint structure\n");
      return -ENOMEM;
    }

  /* Initialize the allocated mountpt state structure.  The filesystem is
   * responsible for one reference ont the blkdriver inode and does not
   * have to addref() here (but does have to release in ubind().
   */

  sem_init(&bm->bm_sem, 0, 1);     /* Initialize the semaphore that controls access */

  /* Mounted! */

  *handle = (void*)bm;
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
  struct binfs_state_s *bm = (struct binfs_state_s*)handle;

  fvdbg("Entry\n");

#ifdef CONFIG_DEBUG
  if (!bm)
    {
      return -EINVAL;
    }
#endif

  /* Check if there are sill any files opened on the filesystem. */

  /* Release the mountpoint private data */

  sem_destroy(&bm->bm_sem);
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
  struct binfs_state_s *bm;

  fvdbg("Entry\n");

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  bm = mountpt->i_private;
  binfs_semtake(bm);

  /* Fill in the statfs info */

  memset(buf, 0, sizeof(struct statfs));
  buf->f_type    = BINFS_MAGIC;
  buf->f_bsize   = 0;
  buf->f_blocks  = 0;
  buf->f_bfree   = 0;
  buf->f_bavail  = 0;
  buf->f_namelen = NAME_MAX;

  binfs_semgive(bm);
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
  struct binfs_state_s *bm;
  int                   ret;

  fvdbg("Entry\n");

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  bm = mountpt->i_private;
  binfs_semtake(bm);

  /* The requested directory must be the volume-relative "root" directory */

  if (relpath && relpath[0] != '\0')
    {
      /* Check if there is a file with this name. */

      if (namedapp_isavail(relpath) < 0)
        {
          ret = -ENOENT;
          goto errout_with_semaphore;
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

  buf->st_size      = 0;
  buf->st_blksize   = 0;
  buf->st_blocks    = 0;
  ret               = OK;

errout_with_semaphore:
  binfs_semgive(bm);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* !CONFIG_DISABLE_MOUNTPOINT && CONFIG_APPS_BINDIR */

