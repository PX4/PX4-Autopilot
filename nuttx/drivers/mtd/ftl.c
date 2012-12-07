/****************************************************************************
 * drivers/mtd/ftl.c
 *
 *   Copyright (C) 2009, 2011-2012 Gregory Nutt. All rights reserved.
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
#include <sys/ioctl.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd.h>
#include <nuttx/rwbuffer.h>

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

#if defined(CONFIG_FS_READAHEAD) || (defined(CONFIG_FS_WRITABLE) && defined(CONFIG_FS_WRITEBUFFER))
#  defined CONFIG_FTL_RWBUFFER 1
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ftl_struct_s
{
  FAR struct mtd_dev_s *mtd;     /* Contained MTD interface */
  struct mtd_geometry_s geo;     /* Device geometry */
#ifdef CONFIG_FTL_RWBUFFER
  struct rwbuffer_s     rwb;     /* Read-ahead/write buffer support */
#endif
  uint16_t              blkper;  /* R/W blocks per erase block */
#ifdef CONFIG_FS_WRITABLE
  FAR uint8_t          *eblock;  /* One, in-memory erase block */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     ftl_open(FAR struct inode *inode);
static int     ftl_close(FAR struct inode *inode);
static ssize_t ftl_reload(FAR void *priv, FAR uint8_t *buffer,
                 off_t startblock, size_t nblocks);
static ssize_t ftl_read(FAR struct inode *inode, unsigned char *buffer,
                 size_t start_sector, unsigned int nsectors);
#ifdef CONFIG_FS_WRITABLE
static ssize_t ftl_flush(FAR void *priv, FAR const uint8_t *buffer,
                 off_t startblock, size_t nblocks);
static ssize_t ftl_write(FAR struct inode *inode, const unsigned char *buffer,
                 size_t start_sector, unsigned int nsectors);
#endif
static int     ftl_geometry(FAR struct inode *inode, struct geometry *geometry);
static int     ftl_ioctl(FAR struct inode *inode, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct block_operations g_bops =
{
  ftl_open,     /* open     */
  ftl_close,    /* close    */
  ftl_read,     /* read     */
#ifdef CONFIG_FS_WRITABLE
  ftl_write,    /* write    */
#else
  NULL,        /* write    */
#endif
  ftl_geometry, /* geometry */
  ftl_ioctl     /* ioctl    */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ftl_open
 *
 * Description: Open the block device
 *
 ****************************************************************************/

static int ftl_open(FAR struct inode *inode)
{
  fvdbg("Entry\n");
  return OK;
}

/****************************************************************************
 * Name: ftl_close
 *
 * Description: close the block device
 *
 ****************************************************************************/

static int ftl_close(FAR struct inode *inode)
{
  fvdbg("Entry\n");
  return OK;
}

/****************************************************************************
 * Name: ftl_reload
 *
 * Description:  Read the specified numer of sectors
 *
 ****************************************************************************/

static ssize_t ftl_reload(FAR void *priv, FAR uint8_t *buffer,
                          off_t startblock, size_t nblocks)
{
  struct ftl_struct_s *dev = (struct ftl_struct_s *)priv;
  ssize_t nread;

  /* Read the full erase block into the buffer */

  nread   = MTD_BREAD(dev->mtd, startblock, nblocks, buffer);
  if (nread != nblocks)
    {
      fdbg("Read %d blocks starting at block %d failed: %d\n",
            nblocks, startblock, nread);
    }
  return nread;
}

/****************************************************************************
 * Name: ftl_read
 *
 * Description:  Read the specified numer of sectors
 *
 ****************************************************************************/

static ssize_t ftl_read(FAR struct inode *inode, unsigned char *buffer,
                        size_t start_sector, unsigned int nsectors)
{
  struct ftl_struct_s *dev;

  fvdbg("sector: %d nsectors: %d\n", start_sector, nsectors);

  DEBUGASSERT(inode && inode->i_private);
  dev = (struct ftl_struct_s *)inode->i_private;
#ifdef CONFIG_FS_READAHEAD
  return rwb_read(&dev->rwb, start_sector, nsectors, buffer);
#else
  return ftl_reload(dev, buffer, start_sector, nsectors);
#endif
}

/****************************************************************************
 * Name: ftl_flush
 *
 * Description: Write the specified number of sectors
 *
 ****************************************************************************/

#ifdef CONFIG_FS_WRITABLE
static ssize_t ftl_flush(FAR void *priv, FAR const uint8_t *buffer,
                         off_t startblock, size_t nblocks)
{
  struct ftl_struct_s *dev = (struct ftl_struct_s *)priv;
  off_t  alignedblock;
  off_t  mask;
  off_t  rwblock;
  off_t  eraseblock;
  off_t  offset;
  size_t remaining;
  size_t nxfrd;
  int    nbytes;
  int    ret;
 
  /* Get the aligned block.  Here is is assumed: (1) The number of R/W blocks
   * per erase block is a power of 2, and (2) the erase begins with that same
   * alignment.
   */

   mask         = dev->blkper - 1;
   alignedblock = (startblock + mask) & ~mask;

  /* Handle partial erase blocks before the first unaligned block */

  remaining = nblocks;
  if (alignedblock > startblock)
    {
      /* Check if the write is shorter than to the end of the erase block */

      bool short_write = (remaining < (alignedblock - startblock));
      
      /* Read the full erase block into the buffer */

      rwblock = startblock & ~mask;
      nxfrd   = MTD_BREAD(dev->mtd, rwblock, dev->blkper, dev->eblock);
      if (nxfrd != dev->blkper)
        {
          fdbg("Read erase block %d failed: %d\n", rwblock, nxfrd);
          return -EIO;
        }

      /* Then erase the erase block */

      eraseblock = rwblock / dev->blkper;
      ret        = MTD_ERASE(dev->mtd, eraseblock, 1);
      if (ret < 0)
        {
          fdbg("Erase block=%d failed: %d\n", eraseblock, ret);
          return ret;
        }

      /* Copy the user data at the end of the buffered erase block */

      offset = (startblock & mask) * dev->geo.blocksize;
      
      if (short_write)
        {
          nbytes = remaining * dev->geo.blocksize;
        }
      else
        {
          nbytes = dev->geo.erasesize - offset;
        }
      
      fvdbg("Copy %d bytes into erase block=%d at offset=%d\n",
             nbytes, eraseblock, offset);

      memcpy(dev->eblock + offset, buffer, nbytes);

      /* And write the erase back to flash */

      nxfrd = MTD_BWRITE(dev->mtd, rwblock, dev->blkper, dev->eblock);
      if (nxfrd != dev->blkper)
        {
          fdbg("Write erase block %d failed: %d\n", rwblock, nxfrd);
          return -EIO;
        }

      /* Then update for amount written */

      if (short_write)
        {
          remaining = 0;
        }
      else
        {
          remaining -= dev->blkper - (startblock & mask);
        }
      
      buffer += nbytes;
    }

  /* How handle full erase pages in the middle */

  while (remaining >= dev->blkper)
    {
      /* Erase the erase block */

      eraseblock = alignedblock / dev->blkper;
      ret        = MTD_ERASE(dev->mtd, eraseblock, 1);
      if (ret < 0)
        {
          fdbg("Erase block=%d failed: %d\n", eraseblock, ret);
          return ret;
        }

      /* Write a full erase back to flash */

      fvdbg("Write %d bytes into erase block=%d at offset=0\n",
             dev->geo.erasesize, alignedblock);

      nxfrd = MTD_BWRITE(dev->mtd, alignedblock, dev->blkper, buffer);
      if (nxfrd != dev->blkper)
        {
          fdbg("Write erase block %d failed: %d\n", alignedblock, nxfrd);
          return -EIO;
        }

      /* Then update for amount written */

      alignedblock += dev->blkper;
      remaining    -= dev->blkper;
      buffer       += dev->geo.erasesize;
    }

  /* Finally, handler any partial blocks after the last full erase block */

  if (remaining > 0)
    {
      /* Read the full erase block into the buffer */

     nxfrd   = MTD_BREAD(dev->mtd, alignedblock, dev->blkper, dev->eblock);
      if (nxfrd != dev->blkper)
        {
          fdbg("Read erase block %d failed: %d\n", alignedblock, nxfrd);
          return -EIO;
        }

      /* Then erase the erase block */

      eraseblock = alignedblock / dev->blkper;
      ret        = MTD_ERASE(dev->mtd, eraseblock, 1);
      if (ret < 0)
        {
          fdbg("Erase block=%d failed: %d\n", eraseblock, ret);
          return ret;
        }

      /* Copy the user data at the beginning the buffered erase block */

      nbytes = remaining * dev->geo.blocksize;
      fvdbg("Copy %d bytes into erase block=%d at offset=0\n",
             nbytes, alignedblock);
      memcpy(dev->eblock, buffer, nbytes);

      /* And write the erase back to flash */

      nxfrd = MTD_BWRITE(dev->mtd, alignedblock, dev->blkper, dev->eblock);
      if (nxfrd != dev->blkper)
        {
          fdbg("Write erase block %d failed: %d\n", alignedblock, nxfrd);
          return -EIO;
        }
    }

  return nblocks;
}
#endif

/****************************************************************************
 * Name: ftl_write
 *
 * Description: Write (or buffer) the specified number of sectors
 *
 ****************************************************************************/

#ifdef CONFIG_FS_WRITABLE
static ssize_t ftl_write(FAR struct inode *inode, const unsigned char *buffer,
                        size_t start_sector, unsigned int nsectors)
{
  struct ftl_struct_s *dev;

  fvdbg("sector: %d nsectors: %d\n", start_sector, nsectors);

  DEBUGASSERT(inode && inode->i_private);
  dev = (struct ftl_struct_s *)inode->i_private;
#ifdef CONFIG_FS_WRITEBUFFER
  return rwb_write(&dev->rwb, start_sector, nsectors, buffer);
#else
  return ftl_flush(dev, buffer, start_sector, nsectors);
#endif
}
#endif

/****************************************************************************
 * Name: ftl_geometry
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int ftl_geometry(FAR struct inode *inode, struct geometry *geometry)
{
  struct ftl_struct_s *dev;

  fvdbg("Entry\n");

  DEBUGASSERT(inode);
  if (geometry)
    {
      dev = (struct ftl_struct_s *)inode->i_private;
      geometry->geo_available     = true;
      geometry->geo_mediachanged  = false;
#ifdef CONFIG_FS_WRITABLE
      geometry->geo_writeenabled  = true;
#else
      geometry->geo_writeenabled  = false;
#endif
      geometry->geo_nsectors      = dev->geo.neraseblocks * dev->blkper;
      geometry->geo_sectorsize    = dev->geo.blocksize;

      fvdbg("available: true mediachanged: false writeenabled: %s\n",
            geometry->geo_writeenabled ? "true" : "false");
      fvdbg("nsectors: %d sectorsize: %d\n",
            geometry->geo_nsectors, geometry->geo_sectorsize);
 
      return OK;
    }
  return -EINVAL;
}

/****************************************************************************
 * Name: ftl_ioctl
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int ftl_ioctl(FAR struct inode *inode, int cmd, unsigned long arg)
{
  struct ftl_struct_s *dev ;
  int ret;

  fvdbg("Entry\n");
  DEBUGASSERT(inode && inode->i_private);

  /* Only one block driver ioctl command is supported by this driver (and
   * that command is just passed on to the MTD driver in a slightly
   * different form).
   */

  if (cmd == BIOC_XIPBASE)
    {
      /* The argument accompanying the BIOC_XIPBASE should be non-NULL.  If
       * DEBUG is enabled, we will catch it here instead of in the MTD
       * driver.
       */

#ifdef CONFIG_DEBUG
      if (arg == 0)
        {
          fdbg("ERROR: BIOC_XIPBASE argument is NULL\n");
          return -EINVAL;
        }
#endif

      /* Just change the BIOC_XIPBASE command to the MTDIOC_XIPBASE command. */

      cmd = MTDIOC_XIPBASE;
    }

  /* No other block driver ioctl commmands are not recognized by this
   * driver.  Other possible MTD driver ioctl commands are passed through
   * to the MTD driver (unchanged).
   */

  dev = (struct ftl_struct_s *)inode->i_private;
  ret = MTD_IOCTL(dev->mtd, cmd, arg);
  if (ret < 0)
    {
      fdbg("ERROR: MTD ioctl(%04x) failed: %d\n", cmd, ret);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ftl_initialize
 *
 * Description:
 *   Initialize to provide a block driver wrapper around an MTD interface
 *
 * Input Parameters:
 *   minor - The minor device number.  The MTD block device will be
 *      registered as as /dev/mtdblockN where N is the minor number.
 *   mtd - The MTD device that supports the FLASH interface.
 *
 ****************************************************************************/

int ftl_initialize(int minor, FAR struct mtd_dev_s *mtd)
{
  struct ftl_struct_s *dev;
  char devname[16];
  int ret = -ENOMEM;

  /* Sanity check */

#ifdef CONFIG_DEBUG
  if (minor < 0 || minor > 255 || !mtd)
    {
      return -EINVAL;
    }
#endif

  /* Allocate a FTL device structure */

  dev = (struct ftl_struct_s *)kmalloc(sizeof(struct ftl_struct_s));
  if (dev)
    {
      /* Initialize the FTL device structure */

      dev->mtd = mtd;

      /* Get the device geometry. (casting to uintptr_t first eliminates
       * complaints on some architectures where the sizeof long is different
       * from the size of a pointer).
       */

      ret = MTD_IOCTL(mtd, MTDIOC_GEOMETRY, (unsigned long)((uintptr_t)&dev->geo));
      if (ret < 0)
        {
          fdbg("MTD ioctl(MTDIOC_GEOMETRY) failed: %d\n", ret);
          kfree(dev);
          return ret;
        }

      /* Allocate one, in-memory erase block buffer */

#ifdef CONFIG_FS_WRITABLE
      dev->eblock  = (FAR uint8_t *)kmalloc(dev->geo.erasesize);
      if (!dev->eblock)
        {
          fdbg("Failed to allocate an erase block buffer\n");
          kfree(dev);
          return -ENOMEM;
        }
#endif

      /* Get the number of R/W blocks per erase block */

      dev->blkper = dev->geo.erasesize / dev->geo.blocksize;
      DEBUGASSERT(dev->blkper * dev->geo.blocksize == dev->geo.erasesize);

      /* Configure read-ahead/write buffering */

#ifdef CONFIG_FTL_RWBUFFER
      dev->rwb.blocksize   = dev->geo.blocksize;
      dev->rwb.nblocks     = dev->geo.neraseblocks * dev->blkper;
      dev->rwb.dev         = (FAR void *)dev;

#if defined(CONFIG_FS_WRITABLE) && defined(CONFIG_FS_WRITEBUFFER)
      dev->rwb.wrmaxblocks = dev->blkper;
      dev->rwb.wrflush     = ftl_flush;
#endif

#ifdef CONFIG_FS_READAHEAD
      dev->rwb.rhmaxblocks = dev->blkper;
      dev->rwb.rhreload    = ftl_reload;
#endif
      ret = rwb_initialize(&dev->rwb);
      if (ret < 0)
        {
          fdbg("rwb_initialize failed: %d\n", ret);
          kfree(dev);
          return ret;
        }
#endif

      /* Create a MTD block device name */

      snprintf(devname, 16, "/dev/mtdblock%d", minor);

      /* Inode private data is a reference to the FTL device structure */

      ret = register_blockdriver(devname, &g_bops, 0, dev);
      if (ret < 0)
        {
          fdbg("register_blockdriver failed: %d\n", -ret);
          kfree(dev);
        }
    }
  return ret;
}
