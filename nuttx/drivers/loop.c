/****************************************************************************
 * drivers/loop.c
 *
 *   Copyright (C) 2008-2009, 2011 Gregory Nutt. All rights reserved.
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
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/mount.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <semaphore.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define loop_semgive(d) sem_post(&(d)->sem)  /* To match loop_semtake */
#define MAX_OPENCNT     (255)                /* Limit of uint8_t */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct loop_struct_s
{
  sem_t        sem;          /* For safe read-modify-write operations */
  uint32_t     nsectors;     /* Number of sectors on device */
  off_t        offset;       /* Offset (in bytes) to the first sector */
  uint16_t     sectsize;     /* The size of one sector */
  uint8_t      opencnt;      /* Count of open references to the loop device */
#ifdef CONFIG_FS_WRITABLE
  bool         writeenabled; /* true: can write to device */
#endif
  int          fd;           /* Descriptor of char device/file */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void    loop_semtake(FAR struct loop_struct_s *dev);
static int     loop_open(FAR struct inode *inode);
static int     loop_close(FAR struct inode *inode);
static ssize_t loop_read(FAR struct inode *inode, unsigned char *buffer,
                       size_t start_sector, unsigned int nsectors);
#ifdef CONFIG_FS_WRITABLE
static ssize_t loop_write(FAR struct inode *inode, const unsigned char *buffer,
                        size_t start_sector, unsigned int nsectors);
#endif
static int     loop_geometry(FAR struct inode *inode, struct geometry *geometry);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct block_operations g_bops =
{
  loop_open,     /* open     */
  loop_close,    /* close    */
  loop_read,     /* read     */
#ifdef CONFIG_FS_WRITABLE
  loop_write,    /* write    */
#else
  NULL,          /* write    */
#endif
  loop_geometry, /* geometry */
  NULL           /* ioctl    */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: loop_semtake
 ****************************************************************************/

static void loop_semtake(FAR struct loop_struct_s *dev)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(&dev->sem) != 0)
    {
      /* The only case that an error should occur here is if
       * the wait was awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

/****************************************************************************
 * Name: loop_open
 *
 * Description: Open the block device
 *
 ****************************************************************************/

static int loop_open(FAR struct inode *inode)
{
  FAR struct loop_struct_s *dev;
  int ret = OK;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct loop_struct_s *)inode->i_private;

  /* Make sure we have exclusive access to the state structure */

  loop_semtake(dev);
  if (dev->opencnt == MAX_OPENCNT)
    {
      return -EMFILE;
    }
  else
    {
      /* Increment the open count */

      dev->opencnt++;
    }

  loop_semgive(dev);
  return ret;
}

/****************************************************************************
 * Name: loop_close
 *
 * Description: close the block device
 *
 ****************************************************************************/

static int loop_close(FAR struct inode *inode)
{
  FAR struct loop_struct_s *dev;
  int ret = OK;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct loop_struct_s *)inode->i_private;

  /* Make sure we have exclusive access to the state structure */

  loop_semtake(dev);
  if (dev->opencnt == 0)
    {
      return -EIO;
    }
  else
    {
      /* Decrement the open count */

      dev->opencnt--;
    }

  loop_semgive(dev);
  return ret;
}

/****************************************************************************
 * Name: loop_read
 *
 * Description:  Read the specified numer of sectors
 *
 ****************************************************************************/

static ssize_t loop_read(FAR struct inode *inode, unsigned char *buffer,
                       size_t start_sector, unsigned int nsectors)
{
  FAR struct loop_struct_s *dev;
  ssize_t nbytesread;
  off_t offset;
  int ret;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct loop_struct_s *)inode->i_private;

  if (start_sector + nsectors > dev->nsectors)
    {
      dbg("Read past end of file\n");
      return -EIO;
    }

  /* Calculate the offset to read the sectors and seek to the position */

  offset = start_sector * dev->sectsize + dev->offset;
  ret = lseek(dev->fd, offset, SEEK_SET);
  if (ret == (off_t)-1)
    {
      dbg("Seek failed for offset=%d: %d\n", (int)offset, errno);
      return -EIO;
    }

  /* Then read the requested number of sectors from that position */

  do
    {
      nbytesread = read(dev->fd, buffer, nsectors * dev->sectsize);
      if (nbytesread < 0 && errno != EINTR)
        {
          dbg("Read failed: %d\n", errno);
          return -errno;
        }
    }
  while (nbytesread < 0);

  /* Return the number of sectors read */

  return nbytesread / dev->sectsize;
}

/****************************************************************************
 * Name: loop_write
 *
 * Description: Write the specified number of sectors
 *
 ****************************************************************************/

#ifdef CONFIG_FS_WRITABLE
static ssize_t loop_write(FAR struct inode *inode, const unsigned char *buffer,
                        size_t start_sector, unsigned int nsectors)
{
  FAR struct loop_struct_s *dev;
  ssize_t nbyteswritten;
  off_t offset;
  int ret;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct loop_struct_s *)inode->i_private;

  /* Calculate the offset to write the sectors and seek to the position */

  offset = start_sector * dev->sectsize + dev->offset;
  ret = lseek(dev->fd, offset, SEEK_SET);
  if (ret == (off_t)-1)
    {
      dbg("Seek failed for offset=%d: %d\n", (int)offset, errno);
    }

  /* Then write the requested number of sectors to that position */

  do
    {
      nbyteswritten = write(dev->fd, buffer, nsectors * dev->sectsize);
      if (nbyteswritten < 0 && errno != EINTR)
        {
          dbg("Write failed: %d\n", errno);
          return -errno;
        }
    }
  while (nbyteswritten < 0);

  /* Return the number of sectors written */

  return nbyteswritten / dev->sectsize;
}
#endif

/****************************************************************************
 * Name: loop_geometry
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int loop_geometry(FAR struct inode *inode, struct geometry *geometry)
{
  FAR struct loop_struct_s *dev;

  DEBUGASSERT(inode);
  if (geometry)
    {
      dev = (FAR struct loop_struct_s *)inode->i_private;
      geometry->geo_available     = true;
      geometry->geo_mediachanged  = false;
#ifdef CONFIG_FS_WRITABLE
      geometry->geo_writeenabled  = dev->writeenabled;
#else
      geometry->geo_writeenabled  = false;
#endif
      geometry->geo_nsectors      = dev->nsectors;
      geometry->geo_sectorsize    = dev->sectsize;
      return OK;
    }

  return -EINVAL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: losetup
 *
 * Description:
 *   Setup the loop device so that it exports the file referenced by 'filename'
 *   as a block device.
 *
 ****************************************************************************/

int losetup(const char *devname, const char *filename, uint16_t sectsize,
            off_t offset, bool readonly)
{
  FAR struct loop_struct_s *dev;
  struct stat sb;
  int ret;

  /* Sanity check */

#ifdef CONFIG_DEBUG
  if (!devname || !filename || !sectsize)
    {
      return -EINVAL;
    }
#endif

  /* Get the size of the file */

  ret = stat(filename, &sb);
  if (ret < 0)
    {
      dbg("Failed to stat %s: %d\n", filename, errno);
      return -errno;
    }

  /* Check if the file system is big enough for one block */

  if (sb.st_size - offset < sectsize)
    {
      dbg("File is too small for blocksize\n");
      return -ERANGE;
    }

  /* Allocate a loop device structure */

  dev = (FAR struct loop_struct_s *)kzalloc(sizeof(struct loop_struct_s));
  if (!dev)
    {
      return -ENOMEM;
    }

  /* Initialize the loop device structure. */

  sem_init(&dev->sem, 0, 1);
  dev->nsectors  = (sb.st_size - offset) / sectsize;
  dev->sectsize  = sectsize;
  dev->offset    = offset;

  /* Open the file. */

#ifdef CONFIG_FS_WRITABLE
  dev->writeenabled = false; /* Assume failure */
  dev->fd           = -1;

  /* First try to open the device R/W access (unless we are asked
   * to open it readonly).
   */

  if (!readonly)
    {
      dev->fd = open(filename, O_RDWR);
    }

  if (dev->fd >= 0)
    {
      dev->writeenabled = true; /* Success */
    }
  else
#endif
    {
      /* If that fails, then try to open the device read-only */

      dev->fd = open(filename, O_RDWR);
      if (dev->fd < 0)
        {
          dbg("Failed to open %s: %d\n", filename, errno);
          ret = -errno;
          goto errout_with_dev;
        }
    }

  /* Inode private data will be reference to the loop device structure */

  ret = register_blockdriver(devname, &g_bops, 0, dev);
  if (ret < 0)
    {
      fdbg("register_blockdriver failed: %d\n", -ret);
      goto errout_with_fd;
    }

  return OK;

errout_with_fd:
  close(dev->fd);
errout_with_dev:
  kfree(dev);
  return ret;
}

/****************************************************************************
 * Name: loteardown
 *
 * Description:
 *   Undo the setup performed by losetup
 *
 ****************************************************************************/

int loteardown(const char *devname)
{
  FAR struct loop_struct_s *dev;
  FAR struct inode *inode;
  int ret;

  /* Sanity check */

#ifdef CONFIG_DEBUG
  if (!devname)
    {
      return -EINVAL;
    }
#endif

  /* Open the block driver associated with devname so that we can get the inode
   * reference.
   */

  ret = open_blockdriver(devname, MS_RDONLY, &inode);
  if (ret < 0)
    {
      dbg("Failed to open %s: %d\n", devname, -ret);
      return ret;
    }

  /* Inode private data is a reference to the loop device stgructure */

  dev = (FAR struct loop_struct_s *)inode->i_private;
  close_blockdriver(inode);

  DEBUGASSERT(dev);

  /* Are there still open references to the device */

  if (dev->opencnt > 0)
    {
      return -EBUSY;
    }

  /* Otherwise, unregister the block device */

  ret = unregister_blockdriver(devname);

  /* Release the device structure */

  if (dev->fd >= 0)
    {
      (void)close(dev->fd);
    }

  kfree(dev);
  return ret;
}
