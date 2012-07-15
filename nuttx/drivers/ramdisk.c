/****************************************************************************
 * drivers/ramdisk.c
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
#include <nuttx/ramdisk.h>

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rd_struct_s
{
  uint32_t       rd_nsectors;     /* Number of sectors on device */
  uint16_t       rd_sectsize;     /* The size of one sector */
#ifdef CONFIG_FS_WRITABLE
  bool           rd_writeenabled; /* true: can write to ram disk */
  uint8_t       *rd_buffer;       /* RAM disk backup memory */
#else
  const uint8_t *rd_buffer;       /* ROM disk backup memory */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     rd_open(FAR struct inode *inode);
static int     rd_close(FAR struct inode *inode);
static ssize_t rd_read(FAR struct inode *inode, unsigned char *buffer,
                       size_t start_sector, unsigned int nsectors);
#ifdef CONFIG_FS_WRITABLE
static ssize_t rd_write(FAR struct inode *inode, const unsigned char *buffer,
                        size_t start_sector, unsigned int nsectors);
#endif
static int     rd_geometry(FAR struct inode *inode, struct geometry *geometry);
static int     rd_ioctl(FAR struct inode *inode, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct block_operations g_bops =
{
  rd_open,     /* open     */
  rd_close,    /* close    */
  rd_read,     /* read     */
#ifdef CONFIG_FS_WRITABLE
  rd_write,    /* write    */
#else
  NULL,        /* write    */
#endif
  rd_geometry, /* geometry */
  rd_ioctl     /* ioctl    */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rd_open
 *
 * Description: Open the block device
 *
 ****************************************************************************/

static int rd_open(FAR struct inode *inode)
{
  fvdbg("Entry\n");
  return OK;
}

/****************************************************************************
 * Name: rd_closel
 *
 * Description: close the block device
 *
 ****************************************************************************/

static int rd_close(FAR struct inode *inode)
{
  fvdbg("Entry\n");
  return OK;
}

/****************************************************************************
 * Name: rd_read
 *
 * Description:  Read the specified numer of sectors
 *
 ****************************************************************************/

static ssize_t rd_read(FAR struct inode *inode, unsigned char *buffer,
                       size_t start_sector, unsigned int nsectors)
{
  struct rd_struct_s *dev;

  DEBUGASSERT(inode && inode->i_private);
  dev = (struct rd_struct_s *)inode->i_private;

  fvdbg("sector: %d nsectors: %d sectorsize: %d\n",
        start_sector, dev->rd_sectsize, nsectors);

  if (start_sector < dev->rd_nsectors &&
      start_sector + nsectors <= dev->rd_nsectors)
    {
       fvdbg("Transfer %d bytes from %p\n",
             nsectors * dev->rd_sectsize,
             &dev->rd_buffer[start_sector * dev->rd_sectsize]);

       memcpy(buffer,
             &dev->rd_buffer[start_sector * dev->rd_sectsize],
             nsectors * dev->rd_sectsize);
      return nsectors;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: rd_write
 *
 * Description: Write the specified number of sectors
 *
 ****************************************************************************/

#ifdef CONFIG_FS_WRITABLE
static ssize_t rd_write(FAR struct inode *inode, const unsigned char *buffer,
                        size_t start_sector, unsigned int nsectors)
{
  struct rd_struct_s *dev;

  DEBUGASSERT(inode && inode->i_private);
  dev = (struct rd_struct_s *)inode->i_private;

  fvdbg("sector: %d nsectors: %d sectorsize: %d\n",
        start_sector, dev->rd_sectsize, nsectors);

  if (!dev->rd_writeenabled)
    {
      return -EACCES;
    }
  else if (start_sector < dev->rd_nsectors &&
           start_sector + nsectors <= dev->rd_nsectors)
    {
      fvdbg("Transfer %d bytes from %p\n",
             nsectors * dev->rd_sectsize,
             &dev->rd_buffer[start_sector * dev->rd_sectsize]);

      memcpy(&dev->rd_buffer[start_sector * dev->rd_sectsize],
             buffer,
             nsectors * dev->rd_sectsize);
      return nsectors;
    }

  return -EFBIG;
}
#endif

/****************************************************************************
 * Name: rd_geometry
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int rd_geometry(FAR struct inode *inode, struct geometry *geometry)
{
  struct rd_struct_s *dev;

  fvdbg("Entry\n");

  DEBUGASSERT(inode);
  if (geometry)
    {
      dev = (struct rd_struct_s *)inode->i_private;
      geometry->geo_available     = true;
      geometry->geo_mediachanged  = false;
#ifdef CONFIG_FS_WRITABLE
      geometry->geo_writeenabled  = dev->rd_writeenabled;
#else
      geometry->geo_writeenabled  = false;
#endif
      geometry->geo_nsectors      = dev->rd_nsectors;
      geometry->geo_sectorsize    = dev->rd_sectsize;

      fvdbg("available: true mediachanged: false writeenabled: %s\n",
            geometry->geo_writeenabled ? "true" : "false");
      fvdbg("nsectors: %d sectorsize: %d\n",
            geometry->geo_nsectors, geometry->geo_sectorsize);
 
      return OK;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: rd_ioctl
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int rd_ioctl(FAR struct inode *inode, int cmd, unsigned long arg)
{
  struct rd_struct_s *dev ;
  void **ppv = (void**)((uintptr_t)arg);

  fvdbg("Entry\n");

  /* Only one ioctl command is supported */

  DEBUGASSERT(inode && inode->i_private);
  if (cmd == BIOC_XIPBASE && ppv)
    {
      dev  = (struct rd_struct_s *)inode->i_private;
      *ppv = (void*)dev->rd_buffer;

      fvdbg("ppv: %p\n", *ppv);
      return OK;
    }

  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ramdisk_register
 *
 * Description: Register the a ramdisk

 ****************************************************************************/

#ifdef CONFIG_FS_WRITABLE
int ramdisk_register(int minor, uint8_t *buffer, uint32_t nsectors,
                     uint16_t sectsize, bool writeenabled)
#else
int romdisk_register(int minor, uint8_t *buffer, uint32_t nsectors,
                     uint16_t sectsize)
#endif
{
  struct rd_struct_s *dev;
  char devname[16];
  int ret = -ENOMEM;

  fvdbg("buffer: %p nsectors: %d sectsize: %d\n", buffer, nsectors, sectsize);

  /* Sanity check */

#ifdef CONFIG_DEBUG
  if (minor < 0 || minor > 255 || !buffer || !nsectors || !sectsize)
    {
      return -EINVAL;
    }
#endif

  /* Allocate a ramdisk device structure */

  dev = (struct rd_struct_s *)kmalloc(sizeof(struct rd_struct_s));
  if (dev)
    {
      /* Initialize the ramdisk device structure */

      dev->rd_nsectors     = nsectors;     /* Number of sectors on device */
      dev->rd_sectsize     = sectsize;     /* The size of one sector */
#ifdef CONFIG_FS_WRITABLE
      dev->rd_writeenabled = writeenabled; /* true: can write to ram disk */
#endif
      dev->rd_buffer       = buffer;       /* RAM disk backup memory */

      /* Create a ramdisk device name */

      snprintf(devname, 16, "/dev/ram%d", minor);

      /* Inode private data is a reference to the ramdisk device stgructure */

      ret = register_blockdriver(devname, &g_bops, 0, dev);
      if (ret < 0)
        {
          fdbg("register_blockdriver failed: %d\n", -ret);
          kfree(dev);
        }
    }
  return ret;
}
