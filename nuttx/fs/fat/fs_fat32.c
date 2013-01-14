/****************************************************************************
 * fs/fat/fs_fat32.c
 *
 *   Copyright (C) 2007-2009, 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   Microsoft FAT documentation
 *   Some good ideas were leveraged from the FAT implementation:
 *     'Copyright (C) 2007, ChaN, all right reserved.'
 *     which has an unrestricted license.
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
#include <sys/stat.h>
#include <sys/statfs.h>

#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <semaphore.h>
#include <assert.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/fat.h>
#include <nuttx/fs/dirent.h>

#include "fs_internal.h"
#include "fs_fat32.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     fat_open(FAR struct file *filep, const char *relpath,
                        int oflags, mode_t mode);
static int     fat_close(FAR struct file *filep);
static ssize_t fat_read(FAR struct file *filep, char *buffer, size_t buflen);
static ssize_t fat_write(FAR struct file *filep, const char *buffer,
                         size_t buflen);
static off_t   fat_seek(FAR struct file *filep, off_t offset, int whence);
static int     fat_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int     fat_sync(FAR struct file *filep);

static int     fat_opendir(struct inode *mountpt, const char *relpath,
                           struct fs_dirent_s *dir);
static int     fat_readdir(struct inode *mountpt, struct fs_dirent_s *dir);
static int     fat_rewinddir(struct inode *mountpt, struct fs_dirent_s *dir);

static int     fat_bind(FAR struct inode *blkdriver, const void *data,
                        void **handle);
static int     fat_unbind(void *handle, FAR struct inode **blkdriver);
static int     fat_statfs(struct inode *mountpt, struct statfs *buf);

static int     fat_unlink(struct inode *mountpt, const char *relpath);
static int     fat_mkdir(struct inode *mountpt, const char *relpath,
                         mode_t mode);
static int     fat_rmdir(struct inode *mountpt, const char *relpath);
static int     fat_rename(struct inode *mountpt, const char *oldrelpath,
                          const char *newrelpath);
static int     fat_stat(struct inode *mountpt, const char *relpath, struct stat *buf);

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

const struct mountpt_operations fat_operations =
{
  fat_open,          /* open */
  fat_close,         /* close */
  fat_read,          /* read */
  fat_write,         /* write */
  fat_seek,          /* seek */
  fat_ioctl,         /* ioctl */

  fat_sync,          /* sync */
  NULL,              /* dup */

  fat_opendir,       /* opendir */
  NULL,              /* closedir */
  fat_readdir,       /* readdir */
  fat_rewinddir,     /* rewinddir */

  fat_bind,          /* bind */
  fat_unbind,        /* unbind */
  fat_statfs,        /* statfs */

  fat_unlink,        /* unlinke */
  fat_mkdir,         /* mkdir */
  fat_rmdir,         /* rmdir */
  fat_rename,        /* rename */
  fat_stat           /* stat */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fat_open
 ****************************************************************************/

static int fat_open(FAR struct file *filep, const char *relpath,
                    int oflags, mode_t mode)
{
  struct fat_dirinfo_s  dirinfo;
  struct inode         *inode;
  struct fat_mountpt_s *fs;
  struct fat_file_s    *ff;
  uint8_t              *direntry;
  int                   ret;

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv == NULL && filep->f_inode != NULL);

  /* Get the mountpoint inode reference from the file structure and the
   * mountpoint private data from the inode structure
   */

  inode = filep->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL);

  /* Check if the mount is still healthy */

  fat_semtake(fs);
  ret = fat_checkmount(fs);
  if (ret != OK)
    {
      goto errout_with_semaphore;
    }

  /* Initialize the directory info structure */

  memset(&dirinfo, 0, sizeof(struct fat_dirinfo_s));

  /* Locate the directory entry for this path */

  ret = fat_finddirentry(fs, &dirinfo, relpath);

  /* Three possibililities: (1) a node exists for the relpath and
   * dirinfo describes the directory entry of the entity, (2) the
   * node does not exist, or (3) some error occurred.
   */

  if (ret == OK)
    {
      bool readonly;

      /* The name exists -- but is it a file or a directory? */

      direntry = &fs->fs_buffer[dirinfo.fd_seq.ds_offset];
      if (dirinfo.fd_root ||
         (DIR_GETATTRIBUTES(direntry) & FATATTR_DIRECTORY))
        {
          /* It is a directory */

          ret = -EISDIR;
          goto errout_with_semaphore;
        }

      /* It would be an error if we are asked to create it exclusively */

      if ((oflags & (O_CREAT|O_EXCL)) == (O_CREAT|O_EXCL))
        {
          /* Already exists -- can't create it exclusively */

          ret = -EEXIST;
          goto errout_with_semaphore;
        }

#ifdef CONFIG_FILE_MODE
#  warning "Missing check for privileges based on inode->i_mode"
#endif

      /* Check if the caller has sufficient privileges to open the file */

      readonly = ((DIR_GETATTRIBUTES(direntry) & FATATTR_READONLY) != 0);
      if (((oflags & O_WRONLY) != 0) && readonly)
        {
          ret = -EACCES;
          goto errout_with_semaphore;
        }

      /* If O_TRUNC is specified and the file is opened for writing,
       * then truncate the file.  This operation requires that the file is
       * writable, but we have already checked that. O_TRUNC without write
       * access is ignored.
       */

      if ((oflags & (O_TRUNC|O_WRONLY)) == (O_TRUNC|O_WRONLY))
        {
          /* Truncate the file to zero length */

          ret = fat_dirtruncate(fs, &dirinfo);
          if (ret < 0)
            {
              goto errout_with_semaphore;
            }
        }

      /* fall through to finish the file open operations */
    }
  else if (ret == -ENOENT)
    {
      /* The file does not exist.  Were we asked to create it? */

      if ((oflags & O_CREAT) == 0)
        {
          /* No.. then we fail with -ENOENT */

          ret = -ENOENT;
          goto errout_with_semaphore;
        }

      /* Yes.. create the file */

      ret = fat_dircreate(fs, &dirinfo);
      if (ret < 0)
        {
          goto errout_with_semaphore;
        }

      /* Fall through to finish the file open operation */
      
      direntry = &fs->fs_buffer[dirinfo.fd_seq.ds_offset];
    }
  else
    {
      /* An error occurred while checking for file existence --
       * such as if an invalid path were provided.
       */

      goto errout_with_semaphore;
    }

  /* Create an instance of the file private date to describe the opened
   * file.
   */

  ff = (struct fat_file_s *)kzalloc(sizeof(struct fat_file_s));
  if (!ff)
    {
      ret = -ENOMEM;
      goto errout_with_semaphore;
    }

  /* Create a file buffer to support partial sector accesses */

  ff->ff_buffer = (uint8_t*)fat_io_alloc(fs->fs_hwsectorsize);
  if (!ff->ff_buffer)
    {
      ret = -ENOMEM;
      goto errout_with_struct;
    }

  /* Initialize the file private data (only need to initialize non-zero elements) */

  ff->ff_open             = true;
  ff->ff_oflags           = oflags;

  /* Save information that can be used later to recover the directory entry */

  ff->ff_dirsector        = fs->fs_currentsector;
  ff->ff_dirindex         = dirinfo.dir.fd_index;

  /* File cluster/size info */

  ff->ff_startcluster     =
    ((uint32_t)DIR_GETFSTCLUSTHI(direntry) << 16) |
      DIR_GETFSTCLUSTLO(direntry);

  ff->ff_currentcluster   = ff->ff_startcluster;
  ff->ff_sectorsincluster = fs->fs_fatsecperclus;
  ff->ff_size             = DIR_GETFILESIZE(direntry);

  /* Attach the private date to the struct file instance */

  filep->f_priv = ff;

  /* Then insert the new instance into the mountpoint structure.
   * It needs to be there (1) to handle error conditions that effect
   * all files, and (2) to inform the umount logic that we are busy
   * (but a simple reference count could have done that).
   */

  ff->ff_next = fs->fs_head;
  fs->fs_head = ff->ff_next;

  fat_semgive(fs);
 
  /* In write/append mode, we need to set the file pointer to the end of the file */

  if ((oflags & (O_APPEND|O_WRONLY)) == (O_APPEND|O_WRONLY))
    {
      off_t offset = fat_seek(filep, ff->ff_size, SEEK_SET);
      if (offset < 0)
        {
          kfree(ff);
          return (int)offset;
        }
    }

  return OK;

  /* Error exits -- goto's are nasty things, but they sure can make error
   * handling a lot simpler.
   */

errout_with_struct:
  kfree(ff);

errout_with_semaphore:
  fat_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: fat_close
 ****************************************************************************/

static int fat_close(FAR struct file *filep)
{
  struct inode         *inode;
  struct fat_file_s    *ff;
  int                   ret = OK;

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  ff    = filep->f_priv;
  inode = filep->f_inode;

  /* Do not check if the mount is healthy.  We must support closing of
   * the file even when there is healthy mount.
   */

  /* Synchronize the file buffers and disk content; update times */

  ret = fat_sync(filep);

  /* Then deallocate the memory structures created when the open method
   * was called.
   *
   * Free the sector buffer that was used to manage partial sector accesses.
   */

  if (ff->ff_buffer)
    {
      fat_io_free(ff->ff_buffer, fs->fs_hwsectorsize);
    }

  /* Then free the file structure itself. */

  kfree(ff);
  filep->f_priv = NULL;
  return ret;
}

/****************************************************************************
 * Name: fat_read
 ****************************************************************************/

static ssize_t fat_read(FAR struct file *filep, char *buffer, size_t buflen)
{
  struct inode         *inode;
  struct fat_mountpt_s *fs;
  struct fat_file_s    *ff;
  unsigned int          bytesread;
  unsigned int          readsize;
  unsigned int          nsectors;
  size_t                bytesleft;
  int32_t               cluster;
  uint8_t               *userbuffer = (uint8_t*)buffer;
  int                   sectorindex;
  int                   ret;

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  ff    = filep->f_priv;
  inode = filep->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL);

  /* Make sure that the mount is still healthy */

  fat_semtake(fs);
  ret = fat_checkmount(fs);
  if (ret != OK)
    {
      goto errout_with_semaphore;
    }

  /* Check if the file was opened with read access */

  if ((ff->ff_oflags & O_RDOK) == 0)
    {
      ret = -EACCES;
      goto errout_with_semaphore;
    }

  /* Get the number of bytes left in the file */

  bytesleft = ff->ff_size - filep->f_pos;

  /* Truncate read count so that it does not exceed the number
   * of bytes left in the file.
   */

  if (buflen > bytesleft)
    {
      buflen = bytesleft;
    }

  /* Get the first sector to read from. */

  if (!ff->ff_currentsector)
    {
      /* The current sector can be determined from the current cluster
       * and the file offset.
       */

      ret = fat_currentsector(fs, ff, filep->f_pos);
      if (ret < 0)
        {
           return ret;
        }
    }

  /* Loop until either (1) all data has been transferred, or (2) an
   * error occurs.  We assume we start with the current sector
  * (ff_currentsector) which may be uninitialized.
   */

  readsize    = 0;
  sectorindex = filep->f_pos & SEC_NDXMASK(fs);

  while (buflen > 0)
    {
      bytesread  = 0;

      /* Check if the user has provided a buffer large enough to
       * hold one or more complete sectors -AND- the read is
       * aligned to a sector boundary.
       */

      nsectors = buflen / fs->fs_hwsectorsize;
      if (nsectors > 0 && sectorindex == 0)
        {
          /* Read maximum contiguous sectors directly to the user's
           * buffer without using our tiny read buffer.
           *
           * Limit the number of sectors that we read on this time
           * through the loop to the remaining contiguous sectors
           * in this cluster
           */

          if (nsectors > ff->ff_sectorsincluster)
            {
              nsectors = ff->ff_sectorsincluster;
            }

          /* We are not sure of the state of the file buffer so
           * the safest thing to do is just invalidate it
           */

          (void)fat_ffcacheinvalidate(fs, ff);

          /* Read all of the sectors directly into user memory */

          ret = fat_hwread(fs, userbuffer, ff->ff_currentsector, nsectors);
          if (ret < 0)
            {
              goto errout_with_semaphore;
            }

          ff->ff_sectorsincluster -= nsectors;
          ff->ff_currentsector    += nsectors;
          bytesread                = nsectors * fs->fs_hwsectorsize;
        }
      else
        {
          /* We are reading a partial sector.  First, read the whole sector
           * into the file data buffer.  This is a caching buffer so if
           * it is already there then all is well.
           */

          ret = fat_ffcacheread(fs, ff, ff->ff_currentsector);
          if (ret < 0)
            {
              goto errout_with_semaphore;
            }

          /* Copy the partial sector into the user buffer */

          bytesread = fs->fs_hwsectorsize - sectorindex;
          if (bytesread > buflen)
            {
              /* We will not read to the end of the buffer */

              bytesread = buflen;
            }
          else
            {
              /* We will read to the end of the buffer (or beyond) */

              ff->ff_sectorsincluster--;
              ff->ff_currentsector++;
            }

          memcpy(userbuffer, &ff->ff_buffer[sectorindex], bytesread);
        }

      /* Set up for the next sector read */

      userbuffer   += bytesread;
      filep->f_pos += bytesread;
      readsize     += bytesread;
      buflen       -= bytesread;
      sectorindex   = filep->f_pos & SEC_NDXMASK(fs);

      /* Check if the current read stream has incremented to the next
       * cluster boundary
       */

      if (ff->ff_sectorsincluster < 1)
        {
          /* Find the next cluster in the FAT. */

          cluster = fat_getcluster(fs, ff->ff_currentcluster);
          if (cluster < 2 || cluster >= fs->fs_nclusters)
            {
              ret = -EINVAL; /* Not the right error */
              goto errout_with_semaphore;
            }

          /* Setup to read the first sector from the new cluster */

          ff->ff_currentcluster   = cluster;
          ff->ff_currentsector    = fat_cluster2sector(fs, cluster);
          ff->ff_sectorsincluster = fs->fs_fatsecperclus;
        }
    }

  fat_semgive(fs);
  return readsize;

errout_with_semaphore:
  fat_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: fat_write
 ****************************************************************************/

static ssize_t fat_write(FAR struct file *filep, const char *buffer,
                         size_t buflen)
{
  struct inode         *inode;
  struct fat_mountpt_s *fs;
  struct fat_file_s    *ff;
  int32_t               cluster;
  unsigned int          byteswritten;
  unsigned int          writesize;
  unsigned int          nsectors;
  uint8_t              *userbuffer = (uint8_t*)buffer;
  int                   sectorindex;
  int                   ret;

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  ff    = filep->f_priv;
  inode = filep->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL);

  /* Make sure that the mount is still healthy */

  fat_semtake(fs);
  ret = fat_checkmount(fs);
  if (ret != OK)
    {
      goto errout_with_semaphore;
    }

  /* Check if the file was opened for write access */

  if ((ff->ff_oflags & O_WROK) == 0)
    {
      ret = -EACCES;
      goto errout_with_semaphore;
    }

  /* Check if the file size would exceed the range of off_t */

  if (ff->ff_size + buflen < ff->ff_size)
    {
      ret = -EFBIG;
      goto errout_with_semaphore;
    }

  /* Get the first sector to write to. */

  if (!ff->ff_currentsector)
    {
      /* Has the starting cluster been defined? */

      if (ff->ff_startcluster == 0)
        {
          /* No.. we have to create a new cluster chain */

          ff->ff_startcluster     = fat_createchain(fs);
          ff->ff_currentcluster   = ff->ff_startcluster;
          ff->ff_sectorsincluster = fs->fs_fatsecperclus;
        }

      /* The current sector can then be determined from the currentcluster
       * and the file offset.
       */

      ret = fat_currentsector(fs, ff, filep->f_pos);
      if (ret < 0)
        {
           return ret;
        }
    }

  /* Loop until either (1) all data has been transferred, or (2) an
   * error occurs.  We assume we start with the current sector in
   * cache (ff_currentsector)
   */

  byteswritten = 0;
  sectorindex = filep->f_pos & SEC_NDXMASK(fs);

  while (buflen > 0)
    {
      /* Check if the user has provided a buffer large enough to
       * hold one or more complete sectors.
       */

      nsectors = buflen / fs->fs_hwsectorsize;
      if (nsectors > 0 && sectorindex == 0)
        {
          /* Write maximum contiguous sectors directly from the user's
           * buffer without using our tiny read buffer.
           *
           * Limit the number of sectors that we write on this time
           * through the loop to the remaining contiguous sectors
           * in this cluster
           */

          if (nsectors > ff->ff_sectorsincluster)
            {
              nsectors = ff->ff_sectorsincluster;
            }

          /* We are not sure of the state of the sector cache so the
           * safest thing to do is write back any dirty, cached sector
           * and invalidate the current cache content.
           */

          (void)fat_ffcacheinvalidate(fs, ff);

          /* Write all of the sectors directly from user memory */

          ret = fat_hwwrite(fs, userbuffer, ff->ff_currentsector, nsectors);
          if (ret < 0)
            {
              goto errout_with_semaphore;
            }

          ff->ff_sectorsincluster -= nsectors;
          ff->ff_currentsector    += nsectors;
          writesize                = nsectors * fs->fs_hwsectorsize;
          ff->ff_bflags           |= FFBUFF_MODIFIED;
        }
      else
        {
          /* We are writing a partial sector -OR- the current sector
           * has not yet been filled.
           *
           * We will first have to read the full sector in memory as
           * part of a read-modify-write operation.  NOTE we don't
           * have to read the data on a rare case: When we are extending
           * the file (filep->f_pos == ff->ff_size) -AND- the new data
           * happens to be aligned at the beginning of the sector
           * (sectorindex == 0).
           */

          if (filep->f_pos < ff->ff_size || sectorindex != 0)
            {
              /* Read the current sector into memory (perhaps first flushing
               * the old, dirty sector to disk).
               */

              ret = fat_ffcacheread(fs, ff, ff->ff_currentsector);
              if (ret < 0)
                {
                  goto errout_with_semaphore;
                }
            }
          else
            {
               /* Flush unwritten data in the sector cache. */

               ret = fat_ffcacheflush(fs, ff);
               if (ret < 0)
                 {
                   goto errout_with_semaphore;
                 }

              /* Now mark the clean cache buffer as the current sector. */

              ff->ff_cachesector = ff->ff_currentsector;
            }

          /* Copy the partial sector from the user buffer */

          writesize = fs->fs_hwsectorsize - sectorindex;
          if (writesize > buflen)
            {
             /* We will not write to the end of the buffer.  Set
              * write size to the size of the user buffer.
              */

              writesize = buflen;
            }
          else
            {
              /* We will write to the end of the buffer (or beyond).  Bump
               * up the current sector number (actually the next sector number).
               */

              ff->ff_sectorsincluster--;
              ff->ff_currentsector++;
            }

          /* Copy the data into the cached sector and make sure that the
           * cached sector is marked "dirty"
           */

          memcpy(&ff->ff_buffer[sectorindex], userbuffer, writesize);
          ff->ff_bflags |= (FFBUFF_DIRTY|FFBUFF_VALID|FFBUFF_MODIFIED);
        }

      /* Set up for the next write */

      userbuffer   += writesize;
      filep->f_pos += writesize;
      byteswritten += writesize;
      buflen       -= writesize;
      sectorindex   = filep->f_pos & SEC_NDXMASK(fs);

      /* Check if the current read stream has incremented to the next
       * cluster boundary
       */

      if (ff->ff_sectorsincluster < 1)
        {
          /* Extend the current cluster by one (unless lseek was used to
           * move the file position back from the end of the file)
           */

          cluster = fat_extendchain(fs, ff->ff_currentcluster);

          /* Verify the cluster number */

          if (cluster < 0)
            {
              ret = cluster;
              goto errout_with_semaphore;
            }
          else if (cluster < 2 || cluster >= fs->fs_nclusters)
            {
              ret = -ENOSPC;
              goto errout_with_semaphore;
            }

          /* Setup to write the first sector from the new cluster */

          ff->ff_currentcluster   = cluster;
          ff->ff_sectorsincluster = fs->fs_fatsecperclus;
          ff->ff_currentsector    = fat_cluster2sector(fs, cluster);
        }
   }

  /* The transfer has completed without error.  Update the file size */

  if (filep->f_pos > ff->ff_size)
    {
      ff->ff_size = filep->f_pos;
    }

  fat_semgive(fs);
  return byteswritten;

errout_with_semaphore:
  fat_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: fat_seek
 ****************************************************************************/

static off_t fat_seek(FAR struct file *filep, off_t offset, int whence)
{
  struct inode         *inode;
  struct fat_mountpt_s *fs;
  struct fat_file_s    *ff;
  int32_t               cluster;
  off_t                 position;
  unsigned int          clustersize;
  int                   ret;

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  ff    = filep->f_priv;
  inode = filep->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL);

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

          position = offset + ff->ff_size;
          break;

      default:
          return -EINVAL;
    }

  /* Make sure that the mount is still healthy */

  fat_semtake(fs);
  ret = fat_checkmount(fs);
  if (ret != OK)
    {
      goto errout_with_semaphore;
    }

  /* Check if there is unwritten data in the file buffer */

  ret = fat_ffcacheflush(fs, ff);
  if (ret < 0)
    {
      goto errout_with_semaphore;
    }

  /* Attempts to set the position beyound the end of file will
   * work if the file is open for write access.
   */

  if (position > ff->ff_size && (ff->ff_oflags & O_WROK) == 0)
    {
        /* Otherwise, the position is limited to the file size */

        position = ff->ff_size;
    }

  /* Set file position to the beginning of the file (first cluster,
   * first sector in cluster)
   */

  filep->f_pos            = 0;
  ff->ff_sectorsincluster = fs->fs_fatsecperclus;

  /* Get the start cluster of the file */

  cluster = ff->ff_startcluster;

  /* Create a new cluster chain if the file does not have one (and
   * if we are seeking beyond zero
   */

  if (!cluster && position > 0)
    {
      cluster = fat_createchain(fs);
      if (cluster < 0)
        {
          ret = cluster;
          goto errout_with_semaphore;
        }
      ff->ff_startcluster = cluster;
    }

  /* Move file position if necessary */

  if (cluster)
    {
      /* If the file has a cluster chain, follow it to the
       * requested position.
       */

      clustersize = fs->fs_fatsecperclus * fs->fs_hwsectorsize;
      for (;;)
        {
          /* Skip over clusters prior to the one containing
           * the requested position.
           */

          ff->ff_currentcluster = cluster;
          if (position < clustersize)
            {
              break;
            }

          /* Extend the cluster chain if write in enabled.  NOTE:
           * this is not consistent with the lseek description:
           * "The  lseek() function allows the file offset to be
           * set beyond the end of the file (but this does not
           * change the size of the file).  If data is later written
           * at  this  point, subsequent reads of the data in the
           * gap (a "hole") return null bytes ('\0') until data
           * is actually written into the gap."
           */

          if ((ff->ff_oflags & O_WROK) != 0)
            {
              /* Extend the cluster chain (fat_extendchain
               * will follow the existing chain or add new
               * clusters as needed.
               */

              cluster = fat_extendchain(fs, cluster);
            }
          else
            {
              /* Otherwise we can only follong the existing chain */

              cluster = fat_getcluster(fs, cluster);
            }

          if (cluster < 0)
            {
              /* An error occurred getting the cluster */

              ret = cluster;
              goto errout_with_semaphore;
            }

          /* Zero means that there is no further clusters available
           * in the chain.
           */

          if (cluster == 0)
            {
              /* At the position to the current locaiton and
               * break out.
               */

              position = clustersize;
              break;
            }

          if (cluster >= fs->fs_nclusters)
            {
              ret = -ENOSPC;
              goto errout_with_semaphore;
            }

          /* Otherwise, update the position and continue looking */

          filep->f_pos += clustersize;
          position     -= clustersize;
        }

      /* We get here after we have found the sector containing
       * the requested position.
       *
       * Save the new file position
       */

      filep->f_pos += position;

      /* Then get the current sector from the cluster and the offset
       * into the cluster from the position
       */

      (void)fat_currentsector(fs, ff, filep->f_pos);

      /* Load the sector corresponding to the position */

      if ((position & SEC_NDXMASK(fs)) != 0)
        {
          ret = fat_ffcacheread(fs, ff, ff->ff_currentsector);
          if (ret < 0)
            {
              goto errout_with_semaphore;
            }
        }
    }

  /* If we extended the size of the file, then mark the file as modified. */

  if ((ff->ff_oflags & O_WROK) != 0 &&  filep->f_pos > ff->ff_size)
    {
        ff->ff_size    = filep->f_pos;
        ff->ff_bflags |= FFBUFF_MODIFIED;
    }

  fat_semgive(fs);
  return OK;

errout_with_semaphore:
  fat_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: fat_ioctl
 ****************************************************************************/

static int fat_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  struct inode         *inode;
  struct fat_mountpt_s *fs;
  int                   ret;

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  inode = filep->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL);

  /* Make sure that the mount is still healthy */

  fat_semtake(fs);
  ret = fat_checkmount(fs);
  if (ret != OK)
    {
      fat_semgive(fs);
      return ret;
    }

  /* ioctl calls are just passed through to the contained block driver */

  fat_semgive(fs);
  return -ENOSYS;
}

/****************************************************************************
 * Name: fat_sync
 *
 * Description: Synchronize the file state on disk to match internal, in-
 *   memory state.
 *
 ****************************************************************************/

static int fat_sync(FAR struct file *filep)
{
  struct inode         *inode;
  struct fat_mountpt_s *fs;
  struct fat_file_s    *ff;
  uint32_t              wrttime;
  uint8_t              *direntry;
  int                   ret;

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  ff    = filep->f_priv;
  inode = filep->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL);

  /* Make sure that the mount is still healthy */

  fat_semtake(fs);
  ret = fat_checkmount(fs);
  if (ret != OK)
    {
      goto errout_with_semaphore;
    }

  /* Check if the has been modified in any way */

  if ((ff->ff_bflags & FFBUFF_MODIFIED) != 0)
    {
      /* Flush any unwritten data in the file buffer */

      ret = fat_ffcacheflush(fs, ff);
      if (ret < 0)
        {
          goto errout_with_semaphore;
        }

      /* Update the directory entry.  First read the directory
       * entry into the fs_buffer (preserving the ff_buffer)
       */

      ret = fat_fscacheread(fs, ff->ff_dirsector);
      if (ret < 0)
        {
          goto errout_with_semaphore;
        }

      /* Recover a pointer to the specific directory entry
       * in the sector using the saved directory index.
       */

      direntry = &fs->fs_buffer[(ff->ff_dirindex & DIRSEC_NDXMASK(fs)) * DIR_SIZE];

      /* Set the archive bit, set the write time, and update
       * anything that may have* changed in the directory
       * entry: the file size, and the start cluster
       */

      direntry[DIR_ATTRIBUTES] |= FATATTR_ARCHIVE;

      DIR_PUTFILESIZE(direntry, ff->ff_size);
      DIR_PUTFSTCLUSTLO(direntry, ff->ff_startcluster);
      DIR_PUTFSTCLUSTHI(direntry, ff->ff_startcluster >> 16);

      wrttime = fat_systime2fattime();
      DIR_PUTWRTTIME(direntry, wrttime & 0xffff);
      DIR_PUTWRTDATE(direntry, wrttime >> 16);

      /* Clear the modified bit in the flags */

      ff->ff_bflags &= ~FFBUFF_MODIFIED;

      /* Flush these change to disk and update FSINFO (if
       * appropriate.
       */

      fs->fs_dirty = true;
      ret          = fat_updatefsinfo(fs);
    }

errout_with_semaphore:
  fat_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: fat_opendir
 *
 * Description: Open a directory for read access
 *
 ****************************************************************************/

static int fat_opendir(struct inode *mountpt, const char *relpath, struct fs_dirent_s *dir)
{
  struct fat_mountpt_s *fs;
  struct fat_dirinfo_s  dirinfo;
  uint8_t              *direntry;
  int                   ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover our private data from the inode instance */

  fs = mountpt->i_private;

  /* Make sure that the mount is still healthy */

  fat_semtake(fs);
  ret = fat_checkmount(fs);
  if (ret != OK)
    {
      goto errout_with_semaphore;
    }

  /* Find the requested directory */

  ret = fat_finddirentry(fs, &dirinfo, relpath);
  if (ret < 0)
    {
      goto errout_with_semaphore;
    }
  direntry = &fs->fs_buffer[dirinfo.fd_seq.ds_offset];

  /* Check if this is the root directory */

  if (dirinfo.fd_root)
    {
      /* Handle the FAT12/16/32 root directory using the values setup by
       * fat_finddirentry() above.
       */

      dir->u.fat.fd_startcluster = dirinfo.dir.fd_startcluster;
      dir->u.fat.fd_currcluster  = dirinfo.dir.fd_currcluster;
      dir->u.fat.fd_currsector   = dirinfo.dir.fd_currsector;
      dir->u.fat.fd_index        = dirinfo.dir.fd_index;
    }

  /* This is not the root directory.  Verify that it is some kind of directory */

  else if ((DIR_GETATTRIBUTES(direntry) & FATATTR_DIRECTORY) == 0)
    {
       /* The entry is not a directory */

       ret = -ENOTDIR;
       goto errout_with_semaphore;
    }
  else
    {
       /* The entry is a directory (but not the root directory) */

      dir->u.fat.fd_startcluster = 
          ((uint32_t)DIR_GETFSTCLUSTHI(direntry) << 16) |
                   DIR_GETFSTCLUSTLO(direntry);
      dir->u.fat.fd_currcluster  = dir->u.fat.fd_startcluster;
      dir->u.fat.fd_currsector   = fat_cluster2sector(fs, dir->u.fat.fd_currcluster);
      dir->u.fat.fd_index        = 2;
    }

  fat_semgive(fs);
  return OK;

errout_with_semaphore:
  fat_semgive(fs);
  return ERROR;
}

/****************************************************************************
 * Name: fat_readdir
 *
 * Description: Read the next directory entry
 *
 ****************************************************************************/

static int fat_readdir(struct inode *mountpt, struct fs_dirent_s *dir)
{
  struct fat_mountpt_s *fs;
  unsigned int          dirindex;
  uint8_t              *direntry;
  uint8_t               ch;
  uint8_t               attribute;
  bool                  found;
  int                   ret = OK;

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover our private data from the inode instance */

  fs = mountpt->i_private;

  /* Make sure that the mount is still healthy */

  fat_semtake(fs);
  ret = fat_checkmount(fs);
  if (ret != OK)
    {
      goto errout_with_semaphore;
    }

  /* Read the next directory entry */

  dir->fd_dir.d_name[0] = '\0';
  found = false;
 
  while (dir->u.fat.fd_currsector && !found)
    {
      ret = fat_fscacheread(fs, dir->u.fat.fd_currsector);
      if (ret < 0)
        {
          goto errout_with_semaphore;
        }

      /* Get a reference to the current directory entry */

      dirindex = (dir->u.fat.fd_index & DIRSEC_NDXMASK(fs)) * DIR_SIZE;
      direntry = &fs->fs_buffer[dirindex];

      /* Has it reached to end of the directory */

      ch = *direntry;
      if (ch == DIR0_ALLEMPTY)
        {
          /* We signal the end of the directory by returning the
           * special error -ENOENT
           */

          ret = -ENOENT;
          goto errout_with_semaphore;
        }

      /* No, is the current entry a valid entry? */

      attribute = DIR_GETATTRIBUTES(direntry);

#ifdef CONFIG_FAT_LFN
      if (ch != DIR0_EMPTY &&
          ((attribute & FATATTR_VOLUMEID) == 0 ||
           ((ch & LDIR0_LAST) != 0 && attribute == LDDIR_LFNATTR)))
#else
      if (ch != DIR0_EMPTY && (attribute & FATATTR_VOLUMEID) == 0)
#endif
        {
          /* Yes.. get the name from the directory entry.  NOTE: For the case
           * of the long file name entry, this will advance the several
           * several directory entries.
           */

          ret = fat_dirname2path(fs, dir);
          if (ret == OK)
            {
              /* The name was successfully extracted.  Re-read the
               * attributes:  If this is long directory entry, then the
               * attributes that we need will be the the final, short file
               * name entry and not in the directory entry where we started
               * looking for the file name.  We can be assured that, on
               * success,  fat_dirname2path() will leave the short file name
               * entry in the cache regardless of the kind of directory
               * entry.  We simply have to re-read it to cover the the long
               * file name case.
               */

#ifdef CONFIG_FAT_LFN

              /* Get a reference to the current, short file name directory
               * entry.
               */

              dirindex = (dir->u.fat.fd_index & DIRSEC_NDXMASK(fs)) * DIR_SIZE;
              direntry = &fs->fs_buffer[dirindex];

              /* Then re-read the attributes from the short file name entry */

              attribute = DIR_GETATTRIBUTES(direntry);
#endif
              /* Now get the file type from the directory attributes. */

              if ((attribute & FATATTR_DIRECTORY) == 0)
                {
                  dir->fd_dir.d_type = DTYPE_FILE;
                }
              else
                {
                  dir->fd_dir.d_type = DTYPE_DIRECTORY;
                }

              /* Mark the entry found.  We will set up the next directory index,
               * and then exit with success.
               */

              found = true;
            }
        }

      /* Set up the next directory index */

      if (fat_nextdirentry(fs, &dir->u.fat) != OK)
        {
          ret = -ENOENT;
          goto errout_with_semaphore;
        }
    }

  fat_semgive(fs);
  return OK;

errout_with_semaphore:
  fat_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: fat_rewindir
 *
 * Description: Reset directory read to the first entry
 *
 ****************************************************************************/

static int fat_rewinddir(struct inode *mountpt, struct fs_dirent_s *dir)
{
  struct fat_mountpt_s *fs;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover our private data from the inode instance */

  fs = mountpt->i_private;

  /* Make sure that the mount is still healthy */

  fat_semtake(fs);
  ret = fat_checkmount(fs);
  if (ret != OK)
    {
      goto errout_with_semaphore;
    }

  /* Check if this is the root directory.  If it is the root directory, we
   * reset the fd_index to 0, starting with the initial, entry.
   */

  if (fs->fs_type != FSTYPE_FAT32 &&
      dir->u.fat.fd_startcluster == 0)
    {
      /* Handle the FAT12/16 root directory */

      dir->u.fat.fd_currcluster  = 0;
      dir->u.fat.fd_currsector   = fs->fs_rootbase;
      dir->u.fat.fd_index        = 0;
    }
  else if (fs->fs_type == FSTYPE_FAT32 &&
           dir->u.fat.fd_startcluster == fs->fs_rootbase)
    {
      /* Handle the FAT32 root directory */

      dir->u.fat.fd_currcluster = dir->u.fat.fd_startcluster;
      dir->u.fat.fd_currsector  = fat_cluster2sector(fs, fs->fs_rootbase);
      dir->u.fat.fd_index       = 0;
    }

  /* This is not the root directory.  Here the fd_index is set to 2, skipping over
   * both the "." and ".." entries.
   */

  else
    {
      dir->u.fat.fd_currcluster  = dir->u.fat.fd_startcluster;
      dir->u.fat.fd_currsector   = fat_cluster2sector(fs, dir->u.fat.fd_currcluster);
      dir->u.fat.fd_index        = 2;
    }

  fat_semgive(fs);
  return OK;

errout_with_semaphore:
  fat_semgive(fs);
  return ERROR;
}

/****************************************************************************
 * Name: fat_bind
 *
 * Description: This implements a portion of the mount operation. This
 *  function allocates and initializes the mountpoint private data and
 *  binds the blockdriver inode to the filesystem private data.  The final
 *  binding of the private data (containing the blockdriver) to the
 *  mountpoint is performed by mount().
 *
 ****************************************************************************/

static int fat_bind(FAR struct inode *blkdriver, const void *data,
                    void **handle)
{
  struct fat_mountpt_s *fs;
  int ret;

  /* Open the block driver */

  if (!blkdriver || !blkdriver->u.i_bops)
    {
      return -ENODEV;
    }

  if (blkdriver->u.i_bops->open &&
      blkdriver->u.i_bops->open(blkdriver) != OK)
    {
      return -ENODEV;
    }

  /* Create an instance of the mountpt state structure */

  fs = (struct fat_mountpt_s *)kzalloc(sizeof(struct fat_mountpt_s));
  if (!fs)
    {
      return -ENOMEM;
    }

  /* Initialize the allocated mountpt state structure.  The filesystem is
   * responsible for one reference ont the blkdriver inode and does not
   * have to addref() here (but does have to release in ubind().
   */

  fs->fs_blkdriver = blkdriver;  /* Save the block driver reference */
  sem_init(&fs->fs_sem, 0, 0);   /* Initialize the semaphore that controls access */

  /* Then get information about the FAT32 filesystem on the devices managed
   * by this block driver.
   */

  ret = fat_mount(fs, true);
  if (ret != 0)
    {
      sem_destroy(&fs->fs_sem);
      kfree(fs);
      return ret;
    }

  *handle = (void*)fs;
  fat_semgive(fs);
  return OK;
}

/****************************************************************************
 * Name: fat_unbind
 *
 * Description: This implements the filesystem portion of the umount
 *   operation.
 *
 ****************************************************************************/

static int fat_unbind(void *handle, FAR struct inode **blkdriver)
{
  struct fat_mountpt_s *fs = (struct fat_mountpt_s*)handle;
  int ret;

  if (!fs)
    {
      return -EINVAL;
    }

  /* Check if there are sill any files opened on the filesystem. */

  ret = OK; /* Assume success */
  fat_semtake(fs);
  if (fs->fs_head)
    {
      /* We cannot unmount now.. there are open files */

      ret = -EBUSY;
    }
  else
    {
       /* Unmount ... close the block driver */

      if (fs->fs_blkdriver)
        {
          struct inode *inode = fs->fs_blkdriver;
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

      if (fs->fs_buffer)
        {
          fat_io_free(fs->fs_buffer, fs->fs_hwsectorsize);
        }

      kfree(fs);
    }

  fat_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: fat_statfs
 *
 * Description: Return filesystem statistics
 *
 ****************************************************************************/

static int fat_statfs(struct inode *mountpt, struct statfs *buf)
{
  struct fat_mountpt_s *fs;
  int                   ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  /* Check if the mount is still healthy */

  fat_semtake(fs);
  ret = fat_checkmount(fs);
  if (ret < 0)
    {
       goto errout_with_semaphore;
    }

  /* Fill in the statfs info */

  memset(buf, 0, sizeof(struct statfs));
  buf->f_type    = MSDOS_SUPER_MAGIC;

  /* We will claim that the optimal transfer size is the size of a cluster in bytes */

  buf->f_bsize   = fs->fs_fatsecperclus * fs->fs_hwsectorsize;

  /* Everything else follows in units of clusters */

  ret = fat_nfreeclusters(fs, &buf->f_bfree); /* Free blocks in the file system */
  if (ret >= 0)
    {
      buf->f_blocks  = fs->fs_nclusters;      /* Total data blocks in the file system */
      buf->f_bavail  = buf->f_bfree;          /* Free blocks avail to non-superuser */
#ifdef CONFIG_FAT_LFN
      buf->f_namelen = LDIR_MAXFNAME;         /* Maximum length of filenames */
#else
      buf->f_namelen = (8+1+3);               /* Maximum length of filenames */
#endif
    }

errout_with_semaphore:
  fat_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: fat_unlink
 *
 * Description: Remove a file
 *
 ****************************************************************************/

static int fat_unlink(struct inode *mountpt, const char *relpath)
{
  struct fat_mountpt_s *fs;
  int                   ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  /* Check if the mount is still healthy */

  fat_semtake(fs);
  ret = fat_checkmount(fs);
  if (ret == OK)
    {
      /* If the file is open, the correct behavior is to remove the file
       * name, but to keep the file cluster chain in place until the last
       * open reference to the file is closed.
       */

#ifdef CONFIG_CPP_HAVE_WARNING
#  warning "Need to defer deleting cluster chain if the file is open"
#endif

      /* Remove the file */

      ret = fat_remove(fs, relpath, false);
    }

  fat_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: fat_mkdir
 *
 * Description: Create a directory
 *
 ****************************************************************************/

static int fat_mkdir(struct inode *mountpt, const char *relpath, mode_t mode)
{
  struct fat_mountpt_s *fs;
  struct fat_dirinfo_s  dirinfo;
  uint8_t     *direntry;
  uint8_t     *direntry2;
  off_t        parentsector;
  off_t        dirsector;
  int32_t      dircluster;
  uint32_t     parentcluster;
  uint32_t     crtime;
  unsigned int i;
  int          ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  /* Check if the mount is still healthy */

  fat_semtake(fs);
  ret = fat_checkmount(fs);
  if (ret != OK)
    {
      goto errout_with_semaphore;
    }

  /* Find the directory where the new directory should be created. */

  ret = fat_finddirentry(fs, &dirinfo, relpath);

  /* If anything exists at this location, then we fail with EEXIST */

  if (ret == OK)
    {
      ret = -EEXIST;
      goto errout_with_semaphore;
    }

  /* What we want to see is for fat_finddirentry to fail with -ENOENT.
   * This error means that no failure occurred but that nothing exists
   * with this name.  NOTE:  The name has already been set in dirinfo
   * structure.
   */

  if (ret != -ENOENT)
    {
      goto errout_with_semaphore;
    }

  /* NOTE: There is no check that dirinfo.fd_name contains the final
   * directory name.  We could be creating an intermediate directory
   * in the full relpath.
   */

  /* Allocate a directory entry for the new directory in this directory */

  ret = fat_allocatedirentry(fs, &dirinfo);
  if (ret != OK)
    {
      goto errout_with_semaphore;
    }
  parentsector = fs->fs_currentsector;

  /* Allocate a cluster for new directory */

  dircluster = fat_createchain(fs);
  if (dircluster < 0)
    {
      ret = dircluster;
      goto errout_with_semaphore;
    }
  else if (dircluster < 2)
    {
      ret = -ENOSPC;
      goto errout_with_semaphore;
    }

  dirsector = fat_cluster2sector(fs, dircluster);
  if (dirsector < 0)
    {
      ret = dirsector;
      goto errout_with_semaphore;
    }

  /* Flush any existing, dirty data in fs_buffer (because we need 
   * it to create the directory entries.
   */

  ret = fat_fscacheflush(fs);
  if (ret < 0)
    {
      goto errout_with_semaphore;
    }

  /* Get a pointer to the first directory entry in the sector */

  direntry = fs->fs_buffer;

  /* Now erase the contents of fs_buffer */

  fs->fs_currentsector = dirsector;
  memset(direntry, 0, fs->fs_hwsectorsize);

  /* Now clear all sectors in the new directory cluster (except for the first) */

  for (i = 1; i < fs->fs_fatsecperclus; i++)
    {
      ret = fat_hwwrite(fs, direntry, ++dirsector, 1);
      if (ret < 0)
        {
          goto errout_with_semaphore;
        }
    }

  /* Now create the "." directory entry in the first directory slot.  These
   * are special directory entries and are not handled by the normal directory
   * management routines.
   */

  memset(&direntry[DIR_NAME], ' ', DIR_MAXFNAME);
  direntry[DIR_NAME] = '.';
  DIR_PUTATTRIBUTES(direntry, FATATTR_DIRECTORY);

  crtime = fat_systime2fattime();
  DIR_PUTCRTIME(direntry, crtime & 0xffff);
  DIR_PUTWRTTIME(direntry, crtime & 0xffff);
  DIR_PUTCRDATE(direntry, crtime >> 16);
  DIR_PUTWRTDATE(direntry, crtime >> 16);

  /* Create ".." directory entry in the second directory slot */

  direntry2 = direntry + DIR_SIZE;

  /* So far, the two entries are nearly the same */

  memcpy(direntry2, direntry, DIR_SIZE);
  direntry2[DIR_NAME+1] = '.';

  /* Now add the cluster information to both directory entries */

  DIR_PUTFSTCLUSTHI(direntry, dircluster >> 16);
  DIR_PUTFSTCLUSTLO(direntry, dircluster);

  parentcluster = dirinfo.dir.fd_startcluster;
  if (fs->fs_type != FSTYPE_FAT32 && parentcluster == fs->fs_rootbase)
    {
      parentcluster = 0;
    }

  DIR_PUTFSTCLUSTHI(direntry2, parentcluster >> 16);
  DIR_PUTFSTCLUSTLO(direntry2, parentcluster);

  /* Save the first sector of the directory cluster and re-read
   * the parentsector
   */

  fs->fs_dirty = true;
  ret = fat_fscacheread(fs, parentsector);
  if (ret < 0)
    {
      goto errout_with_semaphore;
    }

  /* Write the new entry directory entry in the parent directory */

  ret = fat_dirwrite(fs, &dirinfo, FATATTR_DIRECTORY, crtime);
  if (ret < 0)
    {
      goto errout_with_semaphore;
    }

  /* Set subdirectory start cluster. We assume that fat_dirwrite() did not
   * change the sector in the cache.
   */

  direntry = &fs->fs_buffer[dirinfo.fd_seq.ds_offset];
  DIR_PUTFSTCLUSTLO(direntry, dircluster);
  DIR_PUTFSTCLUSTHI(direntry, dircluster >> 16);
  fs->fs_dirty = true;

  /* Now update the FAT32 FSINFO sector */

  ret = fat_updatefsinfo(fs);
  if (ret < 0)
    {
      goto errout_with_semaphore;
    }

  fat_semgive(fs);
  return OK;

errout_with_semaphore:
  fat_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: fat_rmdir
 *
 * Description: Remove a directory
 *
 ****************************************************************************/

int fat_rmdir(struct inode *mountpt, const char *relpath)
{
  struct fat_mountpt_s *fs;
  int                   ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  /* Check if the mount is still healthy */

  fat_semtake(fs);
  ret = fat_checkmount(fs);
  if (ret == OK)
    {
      /* If the directory is open, the correct behavior is to remove the directory
       * name, but to keep the directory cluster chain in place until the last
       * open reference to the directory is closed.
       */

#ifdef CONFIG_CPP_HAVE_WARNING
#  warning "Need to defer deleting cluster chain if the directory is open"
#endif

      /* Remove the directory */

      ret = fat_remove(fs, relpath, true);
    }

  fat_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: fat_rename
 *
 * Description: Rename a file or directory
 *
 ****************************************************************************/

int fat_rename(struct inode *mountpt, const char *oldrelpath,
               const char *newrelpath)
{
  struct fat_mountpt_s *fs;
  struct fat_dirinfo_s  dirinfo;
  struct fat_dirseq_s   dirseq;
  uint8_t              *direntry;
  uint8_t               dirstate[DIR_SIZE-DIR_ATTRIBUTES];
  int                   ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  /* Check if the mount is still healthy */

  fat_semtake(fs);
  ret = fat_checkmount(fs);
  if (ret != OK)
    {
      goto errout_with_semaphore;
    }

  /* Find the directory entry for the oldrelpath (there may be multiple
   * directory entries if long file name support is enabled).
   */

  ret = fat_finddirentry(fs, &dirinfo, oldrelpath);
  if (ret != OK)
    {
      /* Some error occurred -- probably -ENOENT */

      goto errout_with_semaphore;
    }

  /* One more check:  Make sure that the oldrelpath does not refer to the
   * root directory.  We can't rename the root directory.
   */

  if (dirinfo.fd_root)
    {
      ret = -EXDEV;
      goto errout_with_semaphore;
    }

  /* Save the information that will need to recover the directory sector and
   * directory entry offset to the old directory.
   *
   * Save the positional information of the old directory entry.
   */

  memcpy(&dirseq, &dirinfo.fd_seq, sizeof(struct fat_dirseq_s));

  /* Save the non-name-related portion of the directory entry intact */

  direntry = &fs->fs_buffer[dirinfo.fd_seq.ds_offset];
  memcpy(dirstate, &direntry[DIR_ATTRIBUTES], DIR_SIZE-DIR_ATTRIBUTES);

  /* Now find the directory where we should create the newpath object */

  ret = fat_finddirentry(fs, &dirinfo, newrelpath);
  if (ret == OK)
    {
      /* It is an error if the object at newrelpath already exists */

      ret = -EEXIST;
      goto errout_with_semaphore;
    }

  /* What we expect is -ENOENT mean that the full directory path was
   * followed but that the object does not exists in the terminal directory.
   */

  if (ret != -ENOENT)
    {
      goto errout_with_semaphore;
    }

  /* Reserve a directory entry. If long file name support is enabled, then
   * this might, in fact, allocate a sequence of directory entries.  A side
   * effect of fat_allocatedirentry() in either case is that it leaves the
   * short file name entry in the sector cache.
   */

  ret = fat_allocatedirentry(fs, &dirinfo);
  if (ret != OK)
    {
      goto errout_with_semaphore;
    }

  /* Then write the new file name into the directory entry.  This, of course,
   * may involve writing multiple directory entries if long file name
   * support is enabled.  A side effect of fat_allocatedirentry() in either
   * case is that it leaves the short file name entry in the sector cache.
   */

  ret = fat_dirnamewrite(fs, &dirinfo);
  if (ret < 0)
    {
      goto errout_with_semaphore;
    }

  /* Copy the unchanged information into the new short file name entry. */

  direntry = &fs->fs_buffer[dirinfo.fd_seq.ds_offset];
  memcpy(&direntry[DIR_ATTRIBUTES], dirstate, DIR_SIZE-DIR_ATTRIBUTES);
  fs->fs_dirty = true;

  /* Remove the old entry, flushing the new directory entry to disk.  If
   * the old file name was a long file name, then multiple directory
   * entries may be freed.
   */

  ret = fat_freedirentry(fs, &dirseq);
  if (ret < 0)
    {
      goto errout_with_semaphore;
    }
  
  /* Write the old entry to disk and update FSINFO if necessary */

  ret = fat_updatefsinfo(fs);
  if (ret < 0)
    {
      goto errout_with_semaphore;
    }

  fat_semgive(fs);
  return OK;

errout_with_semaphore:
  fat_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: fat_stat
 *
 * Description: Return information about a file or directory
 *
 ****************************************************************************/

static int fat_stat(struct inode *mountpt, const char *relpath, struct stat *buf)
{
  struct fat_mountpt_s *fs;
  struct fat_dirinfo_s  dirinfo;
  uint16_t              fatdate;
  uint16_t              date2;
  uint16_t              fattime;
  uint8_t              *direntry;
  uint8_t               attribute;
  int                   ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  /* Check if the mount is still healthy */

  fat_semtake(fs);
  ret = fat_checkmount(fs);
  if (ret != OK)
    {
      goto errout_with_semaphore;
    }

  /* Find the directory entry corresponding to relpath. */

  ret = fat_finddirentry(fs, &dirinfo, relpath);

  /* If nothing was found, then we fail with the reported error */

  if (ret < 0)
    {
      goto errout_with_semaphore;
    }

  memset(buf, 0, sizeof(struct stat));
  if (dirinfo.fd_root)
    {
      /* It's directory name of the mount point */

      buf->st_mode = S_IFDIR|S_IROTH|S_IRGRP|S_IRUSR|S_IWOTH|S_IWGRP|S_IWUSR;
      ret = OK;
      goto errout_with_semaphore;
    }

  /* Get the FAT attribute and map it so some meaningful mode_t values */

  direntry  = &fs->fs_buffer[dirinfo.fd_seq.ds_offset];
  attribute = DIR_GETATTRIBUTES(direntry);
  if ((attribute & FATATTR_VOLUMEID) != 0)
    {
      ret = -ENOENT;
      goto errout_with_semaphore;
    }

  /* Set the access permissions.  The file/directory is always readable
   * by everyone but may be writeable by no-one.
   */

  buf->st_mode = S_IROTH|S_IRGRP|S_IRUSR;
  if ((attribute & FATATTR_READONLY) == 0)
    {
      buf->st_mode |= S_IWOTH|S_IWGRP|S_IWUSR;
    }

  /* We will report only types file or directory */

  if ((attribute & FATATTR_DIRECTORY) != 0)
    {
      buf->st_mode |= S_IFDIR;
    }
  else
    {
      buf->st_mode |= S_IFREG;
    }

  /* File/directory size, access block size */

  buf->st_size      = DIR_GETFILESIZE(direntry);
  buf->st_blksize   = fs->fs_fatsecperclus * fs->fs_hwsectorsize;
  buf->st_blocks    = (buf->st_size + buf->st_blksize - 1) / buf->st_blksize;

  /* Times */

  fatdate           = DIR_GETWRTDATE(direntry);
  fattime           = DIR_GETWRTTIME(direntry);
  buf->st_mtime     = fat_fattime2systime(fattime, fatdate);

  date2             = DIR_GETLASTACCDATE(direntry);
  if (fatdate == date2)
    {
      buf->st_atime = buf->st_mtime;
    }
  else
    {
      buf->st_atime = fat_fattime2systime(0, date2);
    }

  fatdate           = DIR_GETCRDATE(direntry);
  fattime           = DIR_GETCRTIME(direntry);
  buf->st_ctime     = fat_fattime2systime(fattime, fatdate);

  ret = OK;

errout_with_semaphore:
  fat_semgive(fs);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

