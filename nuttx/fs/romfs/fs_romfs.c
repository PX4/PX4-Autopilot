/****************************************************************************
 * rm/romfs/fs_romfs.h
 *
 *   Copyright (C) 2008-2009, 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References: Linux/Documentation/filesystems/romfs.txt
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
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <limits.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/fs/dirent.h>

#include "fs_romfs.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     romfs_open(FAR struct file *filep, FAR const char *relpath,
                          int oflags, mode_t mode);
static int     romfs_close(FAR struct file *filep);
static ssize_t romfs_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen);
static off_t   romfs_seek(FAR struct file *filep, off_t offset, int whence);
static int     romfs_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);

static  int    romfs_dup(FAR const struct file *oldp, FAR struct file *newp);

static int     romfs_opendir(FAR struct inode *mountpt,
                             FAR const char *relpath,
                             FAR struct fs_dirent_s *dir);
static int     romfs_readdir(FAR struct inode *mountpt,
                             FAR struct fs_dirent_s *dir);
static int     romfs_rewinddir(FAR struct inode *mountpt,
                               FAR struct fs_dirent_s *dir);

static int     romfs_bind(FAR struct inode *blkdriver, FAR const void *data,
                          FAR void **handle);
static int     romfs_unbind(FAR void *handle, FAR struct inode **blkdriver);
static int     romfs_statfs(FAR struct inode *mountpt,
                            FAR struct statfs *buf);

static int     romfs_stat(FAR struct inode *mountpt, FAR const char *relpath,
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

const struct mountpt_operations romfs_operations =
{
  romfs_open,      /* open */
  romfs_close,     /* close */
  romfs_read,      /* read */
  NULL,            /* write */
  romfs_seek,      /* seek */
  romfs_ioctl,     /* ioctl */

  NULL,            /* sync */
  romfs_dup,       /* dup */

  romfs_opendir,   /* opendir */
  NULL,            /* closedir */
  romfs_readdir,   /* readdir */
  romfs_rewinddir, /* rewinddir */

  romfs_bind,      /* bind */
  romfs_unbind,    /* unbind */
  romfs_statfs,    /* statfs */

  NULL,            /* unlink */
  NULL,            /* mkdir */
  NULL,            /* rmdir */
  NULL,            /* rename */
  romfs_stat       /* stat */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: romfs_open
 ****************************************************************************/

static int romfs_open(FAR struct file *filep, FAR const char *relpath,
                      int oflags, mode_t mode)
{
  struct romfs_dirinfo_s      dirinfo;
  FAR struct romfs_mountpt_s *rm;
  FAR struct romfs_file_s    *rf;
  int                         ret;

  fvdbg("Open '%s'\n", relpath);

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv == NULL && filep->f_inode != NULL);

  /* Get mountpoint private data from the inode reference from the file
   * structure
   */

  rm = (FAR struct romfs_mountpt_s*)filep->f_inode->i_private;

  DEBUGASSERT(rm != NULL);

  /* Check if the mount is still healthy */

  romfs_semtake(rm);
  ret = romfs_checkmount(rm);
  if (ret != OK)
    {
      fdbg("romfs_checkmount failed: %d\n", ret);
      goto errout_with_semaphore;
    }

  /* ROMFS is read-only.  Any attempt to open with any kind of write
   * access is not permitted.
   */

  if ((oflags & O_WRONLY) != 0 || (oflags & O_RDONLY) == 0)
    {
      fdbg("Only O_RDONLY supported\n");
      ret = -EACCES;
      goto errout_with_semaphore;
    }

  /* Initialize the directory info structure */

  memset(&dirinfo, 0, sizeof(struct romfs_dirinfo_s));

  /* Locate the directory entry for this path */

  ret = romfs_finddirentry(rm, &dirinfo, relpath);
  if (ret < 0)
    {
      fdbg("Failed to find directory directory entry for '%s': %d\n",
           relpath, ret);
      goto errout_with_semaphore;
    }

  /* The full path exists -- but is the final component a file
   * or a directory?
   */

  if (IS_DIRECTORY(dirinfo.rd_next))
    {
      /* It is a directory */

      ret = -EISDIR;
      fdbg("'%s' is a directory\n", relpath);
      goto errout_with_semaphore;
    }

#ifdef CONFIG_FILE_MODE
# warning "Missing check for privileges based on inode->i_mode"
#endif

  /* Create an instance of the file private data to describe the opened
   * file.
   */

  rf = (FAR struct romfs_file_s *)kzalloc(sizeof(struct romfs_file_s));
  if (!rf)
    {
      fdbg("Failed to allocate private data\n", ret);
      ret = -ENOMEM;
      goto errout_with_semaphore;
    }

  /* Initialize the file private data (only need to initialize
   * non-zero elements)
   */

  rf->rf_size = dirinfo.rd_size;

  /* Get the start of the file data */

  ret = romfs_datastart(rm, dirinfo.rd_dir.fr_curroffset,
                        &rf->rf_startoffset);
  if (ret < 0)
    {
      fdbg("Failed to locate start of file data: %d\n", ret);
      goto errout_with_semaphore;
    }

  /* Configure buffering to support access to this file */

  ret = romfs_fileconfigure(rm, rf);
  if (ret < 0)
    {
      fdbg("Failed configure buffering: %d\n", ret);
      goto errout_with_semaphore;
    }

  /* Attach the private date to the struct file instance */

  filep->f_priv = rf;

  /* Then insert the new instance into the mountpoint structure.
   * It needs to be there (1) to handle error conditions that effect
   * all files, and (2) to inform the umount logic that we are busy
   * (but a simple reference count could have done that).
   */

  rf->rf_next = rm->rm_head;
  rm->rm_head = rf->rf_next;

  romfs_semgive(rm);
  return OK;

  /* Error exits */

errout_with_semaphore:
  romfs_semgive(rm);
  return ret;
}

/****************************************************************************
 * Name: romfs_close
 ****************************************************************************/

static int romfs_close(FAR struct file *filep)
{
  FAR struct romfs_mountpt_s *rm;
  FAR struct romfs_file_s    *rf;
  int                         ret = OK;

  fvdbg("Closing\n");

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  rf = filep->f_priv;
  rm = filep->f_inode->i_private;

  DEBUGASSERT(rm != NULL);

  /* Do not check if the mount is healthy.  We must support closing of
   * the file even when there is healthy mount.
   */

  /* Deallocate the memory structures created when the open method
   * was called.
   *
   * Free the sector buffer that was used to manage partial sector
   * accesses.
   */

  if (!rm->rm_xipbase && rf->rf_buffer)
    {
      kfree(rf->rf_buffer);
    }

  /* Then free the file structure itself. */

  kfree(rf);
  filep->f_priv = NULL;
  return ret;
}

/****************************************************************************
 * Name: romfs_read
 ****************************************************************************/

static ssize_t romfs_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen)
{
  FAR struct romfs_mountpt_s *rm;
  FAR struct romfs_file_s    *rf;
  unsigned int                bytesread;
  unsigned int                readsize;
  unsigned int                nsectors;
  uint32_t                    offset;
  size_t                      bytesleft;
  off_t                       sector;
  FAR uint8_t                *userbuffer = (FAR uint8_t*)buffer;
  int                         sectorndx;
  int                         ret;

  fvdbg("Read %d bytes from offset %d\n", buflen, filep->f_pos);

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  rf = filep->f_priv;
  rm = filep->f_inode->i_private;

  DEBUGASSERT(rm != NULL);

  /* Make sure that the mount is still healthy */

  romfs_semtake(rm);
  ret = romfs_checkmount(rm);
  if (ret != OK)
    {
      fdbg("romfs_checkmount failed: %d\n", ret);
      goto errout_with_semaphore;
    }

  /* Get the number of bytes left in the file */

  bytesleft = rf->rf_size - filep->f_pos;

  /* Truncate read count so that it does not exceed the number
   * of bytes left in the file.
   */

  if (buflen > bytesleft)
    {
      buflen = bytesleft;
    }

  /* Loop until either (1) all data has been transferred, or (2) an
   * error occurs.
   */

  readsize = 0;
  while (buflen > 0)
    {
      /* Get the first sector and index to read from. */

      offset     = rf->rf_startoffset + filep->f_pos;
      sector     = SEC_NSECTORS(rm, offset);
      sectorndx  = offset & SEC_NDXMASK(rm);
      bytesread  = 0;

      /* Check if the user has provided a buffer large enough to
       * hold one or more complete sectors -AND- the read is
       * aligned to a sector boundary.
       */

      nsectors = SEC_NSECTORS(rm, buflen);
      if (nsectors > 0 && sectorndx == 0)
        {
          /* Read maximum contiguous sectors directly to the user's
           * buffer without using our tiny read buffer.
           */

          /* Read all of the sectors directly into user memory */

          fvdbg("Read %d sectors starting with %d\n", nsectors, sector);
          ret = romfs_hwread(rm, userbuffer, sector, nsectors);
          if (ret < 0)
            {
              fdbg("romfs_hwread failed: %d\n", ret);
              goto errout_with_semaphore;
            }

          sector    += nsectors;
          bytesread  = nsectors * rm->rm_hwsectorsize;
        }
      else
        {
          /* We are reading a partial sector.  First, read the whole sector
           * into the file data buffer.  This is a caching buffer so if
           * it is already there then all is well.
           */

          fvdbg("Read sector %d\n", sector);
          ret = romfs_filecacheread(rm, rf, sector);
          if (ret < 0)
            {
              fdbg("romfs_filecacheread failed: %d\n", ret);
              goto errout_with_semaphore;
            }

          /* Copy the partial sector into the user buffer */

          bytesread = rm->rm_hwsectorsize - sectorndx;
          if (bytesread > buflen)
            {
              /* We will not read to the end of the buffer */

              bytesread = buflen;
            }
          else
            {
              /* We will read to the end of the buffer (or beyond) */

             sector++;
            }

          fvdbg("Return %d bytes from sector offset %d\n", bytesread, sectorndx);
          memcpy(userbuffer, &rf->rf_buffer[sectorndx], bytesread);
        }

      /* Set up for the next sector read */

      userbuffer   += bytesread;
      filep->f_pos += bytesread;
      readsize     += bytesread;
      buflen       -= bytesread;
    }

  romfs_semgive(rm);
  return readsize;

errout_with_semaphore:
  romfs_semgive(rm);
  return ret;
}

/****************************************************************************
 * Name: romfs_seek
 ****************************************************************************/

static off_t romfs_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct romfs_mountpt_s *rm;
  FAR struct romfs_file_s    *rf;
  off_t                       position;
  int                         ret;

  fvdbg("Seek to offset: %d whence: %d\n", offset, whence);

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  rf    = filep->f_priv;
  rm    = filep->f_inode->i_private;

  DEBUGASSERT(rm != NULL);

  /* Map the offset according to the whence option */

  switch (whence)
    {
    case SEEK_SET: /* The offset is set to offset bytes. */
        position = offset;
        break;

    case SEEK_CUR: /* The offset is set to its current location plus
                    * offset bytes. */

        position = offset + filep->f_pos;
        break;

    case SEEK_END: /* The offset is set to the size of the file plus
                    * offset bytes. */

        position = offset + rf->rf_size;
        break;

    default:
        fdbg("Whence is invalid: %d\n", whence);
        return -EINVAL;
    }

  /* Make sure that the mount is still healthy */

  romfs_semtake(rm);
  ret = romfs_checkmount(rm);
  if (ret != OK)
    {
       fdbg("romfs_checkmount failed: %d\n", ret);
       goto errout_with_semaphore;
    }

  /* Limit positions to the end of the file. */

  if (position > rf->rf_size)
    {
      /* Otherwise, the position is limited to the file size */

      position = rf->rf_size;
    }

  /* Set file position and return success */

  filep->f_pos = position;
  fvdbg("New file position: %d\n", filep->f_pos);

  romfs_semgive(rm);
  return OK;

errout_with_semaphore:
  romfs_semgive(rm);
  return ret;
}

/****************************************************************************
 * Name: romfs_ioctl
 ****************************************************************************/

static int romfs_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct romfs_mountpt_s *rm;
  FAR struct romfs_file_s    *rf;
  FAR void                  **ppv = (FAR void**)arg;

  fvdbg("cmd: %d arg: %08lx\n", cmd, arg);

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  rf = filep->f_priv;
  rm = filep->f_inode->i_private;

  DEBUGASSERT(rm != NULL);

  /* Only one ioctl command is supported */

  if (cmd == FIOC_MMAP && rm->rm_xipbase && ppv)
    {
      /* Return the address on the media corresponding to the start of
       * the file.
       */

      *ppv = (void*)(rm->rm_xipbase + rf->rf_startoffset);
      return OK;
    }

  fdbg("Invalid cmd: %d \n", cmd);
  return -ENOTTY;
}

/****************************************************************************
 * Name: romfs_dup
 ****************************************************************************/

static int romfs_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  FAR struct romfs_mountpt_s *rm;
  FAR struct romfs_file_s *oldrf;
  FAR struct romfs_file_s *newrf;
  int ret;

  fvdbg("Dup %p->%p\n", oldp, newp);

  /* Sanity checks */

  DEBUGASSERT(oldp->f_priv != NULL &&
              newp->f_priv == NULL &&
              newp->f_inode != NULL);

  /* Get mountpoint private data from the inode reference from the file
   * structure
   */

  rm = (FAR struct romfs_mountpt_s*)newp->f_inode->i_private;
  DEBUGASSERT(rm != NULL);

  /* Check if the mount is still healthy */

  romfs_semtake(rm);
  ret = romfs_checkmount(rm);
  if (ret != OK)
    {
      fdbg("romfs_checkmount failed: %d\n", ret);
      goto errout_with_semaphore;
    }

  /* Recover the old private data from the old struct file instance */

  oldrf = oldp->f_priv;

  /* Create an new instance of the file private data to describe the new
   * dup'ed file.
   */

  newrf = (FAR struct romfs_file_s *)kmalloc(sizeof(struct romfs_file_s));
  if (!newrf)
    {
      fdbg("Failed to allocate private data\n", ret);
      ret = -ENOMEM;
      goto errout_with_semaphore;
    }

  /* Copy all file private data (except for the buffer) */

  newrf->rf_startoffset = oldrf->rf_startoffset;
  newrf->rf_size        = oldrf->rf_size;

  /* Configure buffering to support access to this file */

  ret = romfs_fileconfigure(rm, newrf);
  if (ret < 0)
    {
      fdbg("Failed configure buffering: %d\n", ret);
      goto errout_with_semaphore;
    }

  /* Attach the new private date to the new struct file instance */

  newp->f_priv = newrf;

  /* Then insert the new instance into the mountpoint structure.
   * It needs to be there (1) to handle error conditions that effect
   * all files, and (2) to inform the umount logic that we are busy
   * (but a simple reference count could have done that).
   */

  newrf->rf_next = rm->rm_head;
  rm->rm_head = newrf->rf_next;

  romfs_semgive(rm);
  return OK;

  /* Error exits */

errout_with_semaphore:
  romfs_semgive(rm);
  return ret;
}

/****************************************************************************
 * Name: romfs_opendir
 *
 * Description:
 *   Open a directory for read access
 *
 ****************************************************************************/

static int romfs_opendir(FAR struct inode *mountpt, FAR const char *relpath,
                         FAR struct fs_dirent_s *dir)
{
  FAR struct romfs_mountpt_s *rm;
  FAR struct romfs_dirinfo_s  dirinfo;
  int                         ret;

  fvdbg("relpath: '%s'\n", relpath);

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover our private data from the inode instance */

  rm = mountpt->i_private;

  /* Make sure that the mount is still healthy */

  romfs_semtake(rm);
  ret = romfs_checkmount(rm);
  if (ret != OK)
    {
      fdbg("romfs_checkmount failed: %d\n", ret);
      goto errout_with_semaphore;
    }

  /* Find the requested directory */

  ret = romfs_finddirentry(rm, &dirinfo, relpath);
  if (ret < 0)
    {
      fdbg("Failed to find directory '%s': %d\n", relpath, ret);
      goto errout_with_semaphore;
    }

  /* Verify that it is some kind of directory */

  if (!IS_DIRECTORY(dirinfo.rd_next))
    {
      /* The entry is not a directory */

      fdbg("'%s' is not a directory: %d\n", relpath);
      ret = -ENOTDIR;
      goto errout_with_semaphore;
    }

  /* The entry is a directory */

  memcpy(&dir->u.romfs, &dirinfo.rd_dir, sizeof(struct fs_romfsdir_s));
  romfs_semgive(rm);
  return OK;

errout_with_semaphore:
  romfs_semgive(rm);
  return ERROR;
}

/****************************************************************************
 * Name: romfs_readdir
 *
 * Description: Read the next directory entry
 *
 ****************************************************************************/

static int romfs_readdir(FAR struct inode *mountpt,
                         FAR struct fs_dirent_s *dir)
{
  FAR struct romfs_mountpt_s *rm;
  uint32_t                    linkoffset;
  uint32_t                    next;
  uint32_t                    info;
  uint32_t                    size;
  int                         ret;

  fvdbg("Entry\n");

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover our private data from the inode instance */

  rm = mountpt->i_private;

  /* Make sure that the mount is still healthy */

  romfs_semtake(rm);
  ret = romfs_checkmount(rm);
  if (ret != OK)
    {
      fdbg("romfs_checkmount failed: %d\n", ret);
      goto errout_with_semaphore;
    }

  /* Loop, skipping over unsupported items in the file system */

  for (;;)
    {
      /* Have we reached the end of the directory */

      if (!dir->u.romfs.fr_curroffset)
        {
          /* We signal the end of the directory by returning the
           * special error -ENOENT
           */

          fdbg("End of directory\n");
          ret = -ENOENT;
          goto errout_with_semaphore;
        }

      /* Parse the directory entry */

      ret = romfs_parsedirentry(rm, dir->u.romfs.fr_curroffset, &linkoffset,
                                &next, &info, &size);
      if (ret < 0)
        {
          fdbg("romfs_parsedirentry failed: %d\n", ret);
          goto errout_with_semaphore;
        }

      /* Save the filename */

      ret = romfs_parsefilename(rm, dir->u.romfs.fr_curroffset, dir->fd_dir.d_name);
      if (ret < 0)
        {
          fdbg("romfs_parsefilename failed: %d\n", ret);
          goto errout_with_semaphore;
        }

      /* Set up the next directory entry offset */

      dir->u.romfs.fr_curroffset = next & RFNEXT_OFFSETMASK;

      /* Check the file type */

      if (IS_DIRECTORY(next))
        {
          dir->fd_dir.d_type = DTYPE_DIRECTORY;
          break;
        }
      else if (IS_FILE(next))
        {
          dir->fd_dir.d_type = DTYPE_FILE;
          break;
        }
    }

errout_with_semaphore:
  romfs_semgive(rm);
  return ret;
}

/****************************************************************************
 * Name: romfs_rewindir
 *
 * Description: Reset directory read to the first entry
 *
 ****************************************************************************/

static int romfs_rewinddir(FAR struct inode *mountpt,
                           FAR struct fs_dirent_s *dir)
{
  FAR struct romfs_mountpt_s *rm;
  int ret;

  fvdbg("Entry\n");

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover our private data from the inode instance */

  rm = mountpt->i_private;

  /* Make sure that the mount is still healthy */

  romfs_semtake(rm);
  ret = romfs_checkmount(rm);
  if (ret == OK)
    {
      dir->u.romfs.fr_curroffset = dir->u.romfs.fr_firstoffset;
    }

  romfs_semgive(rm);
  return ret;
}

/****************************************************************************
 * Name: romfs_bind
 *
 * Description: This implements a portion of the mount operation. This
 *  function allocates and initializes the mountpoint private data and
 *  binds the blockdriver inode to the filesystem private data.  The final
 *  binding of the private data (containing the blockdriver) to the
 *  mountpoint is performed by mount().
 *
 ****************************************************************************/

static int romfs_bind(FAR struct inode *blkdriver, FAR const void *data,
                      FAR void **handle)
{
  struct romfs_mountpt_s *rm;
  int ret;

  fvdbg("Entry\n");

  /* Open the block driver */

  if (!blkdriver || !blkdriver->u.i_bops)
    {
      fdbg("No block driver/ops\n");
      return -ENODEV;
    }

  if (blkdriver->u.i_bops->open &&
      blkdriver->u.i_bops->open(blkdriver) != OK)
    {
      fdbg("No open method\n");
      return -ENODEV;
    }

  /* Create an instance of the mountpt state structure */

  rm = (FAR struct romfs_mountpt_s *)kzalloc(sizeof(struct romfs_mountpt_s));
  if (!rm)
    {
      fdbg("Failed to allocate mountpoint structure\n");
      return -ENOMEM;
    }

  /* Initialize the allocated mountpt state structure.  The filesystem is
   * responsible for one reference ont the blkdriver inode and does not
   * have to addref() here (but does have to release in ubind().
   */

  sem_init(&rm->rm_sem, 0, 0);     /* Initialize the semaphore that controls access */
  rm->rm_blkdriver   = blkdriver;  /* Save the block driver reference */

  /* Get the hardware configuration and setup buffering appropriately */

  ret = romfs_hwconfigure(rm);
  if (ret < 0)
    {
      fdbg("romfs_hwconfigure failed: %d\n", ret);
      goto errout_with_sem;
    }

  /* Then complete the mount by getting the ROMFS configuratrion from
   * the ROMF header
   */

  ret = romfs_fsconfigure(rm);
  if (ret < 0)
    {
      fdbg("romfs_fsconfigure failed: %d\n", ret);
      goto errout_with_buffer;
    }

  /* Mounted! */

  *handle = (void*)rm;
  romfs_semgive(rm);
  return OK;

errout_with_buffer:
  if (!rm->rm_xipbase)
    {
      kfree(rm->rm_buffer);
    }

errout_with_sem:
  sem_destroy(&rm->rm_sem);
  kfree(rm);
  return ret;
}

/****************************************************************************
 * Name: romfs_unbind
 *
 * Description: This implements the filesystem portion of the umount
 *   operation.
 *
 ****************************************************************************/

static int romfs_unbind(FAR void *handle, FAR struct inode **blkdriver)
{
  FAR struct romfs_mountpt_s *rm = (FAR struct romfs_mountpt_s*)handle;
  int ret;

  fvdbg("Entry\n");

#ifdef CONFIG_DEBUG
  if (!rm)
    {
      return -EINVAL;
    }
#endif

  /* Check if there are sill any files opened on the filesystem. */

  romfs_semtake(rm);
  if (rm->rm_head)
    {
      /* We cannot unmount now.. there are open files */

      fdbg("There are open files\n");
      ret = -EBUSY;
    }
  else
    {
       /* Unmount ... close the block driver */

      if (rm->rm_blkdriver)
        {
          struct inode *inode = rm->rm_blkdriver;
          if (inode)
            {
              if (inode->u.i_bops && inode->u.i_bops->close)
                {
                  (void)inode->u.i_bops->close(inode);
                }

              /* We hold a reference to the block driver but should
               * not but mucking with inodes in this context.  So, we will just return
               * our contained reference to the block driver inode and let the umount
               * logic dispose of it.
               */

              if (blkdriver)
                {
                  *blkdriver = inode;
                }
            }
        }

      /* Release the mountpoint private data */

      if (!rm->rm_xipbase && rm->rm_buffer)
        {
          kfree(rm->rm_buffer);
        }

      sem_destroy(&rm->rm_sem);
      kfree(rm);
      return OK;
    }

  romfs_semgive(rm);
  return ret;
}

/****************************************************************************
 * Name: romfs_statfs
 *
 * Description: Return filesystem statistics
 *
 ****************************************************************************/

static int romfs_statfs(FAR struct inode *mountpt, FAR struct statfs *buf)
{
  FAR struct romfs_mountpt_s *rm;
  int ret;

  fvdbg("Entry\n");

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  rm = mountpt->i_private;

  /* Check if the mount is still healthy */

  romfs_semtake(rm);
  ret = romfs_checkmount(rm);
  if (ret < 0)
    {
      fdbg("romfs_checkmount failed: %d\n", ret);
      goto errout_with_semaphore;
    }

  /* Fill in the statfs info */

  memset(buf, 0, sizeof(struct statfs));
  buf->f_type    = ROMFS_MAGIC;

  /* We will claim that the optimal transfer size is the size of one sector */

  buf->f_bsize   = rm->rm_hwsectorsize;

  /* Everything else follows in units of sectors */

  buf->f_blocks  = SEC_NSECTORS(rm, rm->rm_volsize + SEC_NDXMASK(rm));
  buf->f_bfree   = 0;
  buf->f_bavail  = 0;
  buf->f_namelen = NAME_MAX;

  romfs_semgive(rm);
  return OK;

errout_with_semaphore:
  romfs_semgive(rm);
  return ret;
}

/****************************************************************************
 * Name: romfs_stat
 *
 * Description: Return information about a file or directory
 *
 ****************************************************************************/

static int romfs_stat(FAR struct inode *mountpt, FAR const char *relpath,
                      FAR struct stat *buf)
{
  FAR struct romfs_mountpt_s *rm;
  FAR struct romfs_dirinfo_s dirinfo;
  int ret;

  fvdbg("Entry\n");

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  rm = mountpt->i_private;

  /* Check if the mount is still healthy */

  romfs_semtake(rm);
  ret = romfs_checkmount(rm);
  if (ret != OK)
    {
      fdbg("romfs_checkmount failed: %d\n", ret);
      goto errout_with_semaphore;
    }

  /* Find the directory entry corresponding to relpath. */

  ret = romfs_finddirentry(rm, &dirinfo, relpath);

  /* If nothing was found, then we fail with EEXIST */

  if (ret < 0)
    {
      fvdbg("Failed to find directory: %d\n", ret);
      goto errout_with_semaphore;
    }

  memset(buf, 0, sizeof(struct stat));
  if (IS_DIRECTORY(dirinfo.rd_next))
    {
      /* It's a read-only directory name */

      buf->st_mode = S_IFDIR|S_IROTH|S_IRGRP|S_IRUSR;
      if (IS_EXECUTABLE(dirinfo.rd_next))
        {
          buf->st_mode |= S_IXOTH|S_IXGRP|S_IXUSR;
        }
    }
  else if (IS_FILE(dirinfo.rd_next))
    {
      /* It's a read-only file name */

      buf->st_mode = S_IFREG|S_IROTH|S_IRGRP|S_IRUSR;
      if (IS_EXECUTABLE(dirinfo.rd_next))
        {
          buf->st_mode |= S_IXOTH|S_IXGRP|S_IXUSR;
        }
    }
  else
    {
      /* Otherwise, pretend like the unsupported node does not exist */

      fvdbg("Unsupported inode: %d\n", dirinfo.rd_next);
      ret = -ENOENT;
      goto errout_with_semaphore;
    }

  /* File/directory size, access block size */

  buf->st_size      = dirinfo.rd_size;
  buf->st_blksize   = rm->rm_hwsectorsize;
  buf->st_blocks    = (buf->st_size + buf->st_blksize - 1) / buf->st_blksize;

  ret = OK;

errout_with_semaphore:
  romfs_semgive(rm);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
