/****************************************************************************
 * include/nuttx/mtd.h
 * Memory Technology Device (MTD) interface
 *
 *   Copyright (C) 2009-2012 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_MTD_H
#define __INCLUDE_NUTTX_MTD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Macros to hide implementation */

#define MTD_ERASE(d,s,n)   ((d)->erase  ? (d)->erase(d,s,n)    : (-ENOSYS))
#define MTD_BREAD(d,s,n,b) ((d)->bread  ? (d)->bread(d,s,n,b)  : (-ENOSYS))
#define MTD_READ(d,s,n,b)  ((d)->read   ? (d)->read(d,s,n,b)   : (-ENOSYS))
#define MTD_BWRITE(d,s,n,b)((d)->bwrite ? (d)->bwrite(d,s,n,b) : (-ENOSYS))
#define MTD_IOCTL(d,c,a)   ((d)->ioctl  ? (d)->ioctl(d,c,a)    : (-ENOSYS))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The following defines the geometry for the device.  It treats the device
 * as though it where just an array of fixed size blocks.  That is most likely
 * not true, but the client will expect the device logic to do whatever is
 * necessary to make it appear so.
 */

struct mtd_geometry_s
{
  uint16_t blocksize;   /* Size of one read/write block */
  uint16_t erasesize;   /* Size of one erase blocks -- must be a multiple
                         * of blocksize. */
  size_t neraseblocks;  /* Number of erase blocks */
};

/* This structure defines the interface to a simple memory technology device.
 * It will likely need to be extended in the future to support more complex
 * devices.
 */

struct mtd_dev_s
{
  /* The following methods operate on the MTD: */

  /* Erase the specified erase blocks (units are erase blocks) */

  int (*erase)(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks);

  /* Read/write from the specified read/write blocks */

  ssize_t (*bread)(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks,
                   FAR uint8_t *buffer);
  ssize_t (*bwrite)(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks,
                    FAR const uint8_t *buffer);

  /* Some devices may support byte oriented reads (optional).  Most MTD devices
   * are inherently block oriented so byte-oriented writing is not supported. It
   * is recommended that low-level drivers not support read() if it requires
   * buffering.
   */

  ssize_t (*read)(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
                  FAR uint8_t *buffer);

  /* Support other, less frequently used commands:
   *  - MTDIOC_GEOMETRY:  Get MTD geometry
   *  - MTDIOC_XIPBASE:   Convert block to physical address for eXecute-In-Place
   *  - MTDIOC_BULKERASE: Erase the entire device
   * (see include/nuttx/fs/ioctl.h) 
   */

  int (*ioctl)(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg);
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
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

EXTERN int ftl_initialize(int minor, FAR struct mtd_dev_s *mtd);

/****************************************************************************
 * Name: flash_eraseall
 *
 * Description:
 *   Call a block driver with the MDIOC_BULKERASE ioctl command.  This will
 *   cause the MTD driver to erase all of the flash.
 *
 ****************************************************************************/

EXTERN int flash_eraseall(FAR const char *driver);

/****************************************************************************
 * Name: rammtd_initialize
 *
 * Description:
 *   Create and initialize a RAM MTD device instance.
 *
 * Input Parameters:
 *   start - Address of the beginning of the allocated RAM regions.
 *   size  - The size in bytes of the allocated RAM region.
 *
 ****************************************************************************/

EXTERN FAR struct mtd_dev_s *rammtd_initialize(FAR uint8_t *start, size_t size);

/****************************************************************************
 * Name: m25p_initialize
 *
 * Description:
 *   Create an initialized MTD device instance.  MTD devices are not registered
 *   in the file system, but are created as instances that can be bound to
 *   other functions (such as a block or character driver front end).
 *
 ****************************************************************************/

EXTERN FAR struct mtd_dev_s *m25p_initialize(FAR struct spi_dev_s *dev);

/****************************************************************************
 * Name: at45db_initialize
 *
 * Description:
 *   Create an initialized MTD device instance.  MTD devices are not registered
 *   in the file system, but are created as instances that can be bound to
 *   other functions (such as a block or character driver front end).
 *
 ****************************************************************************/

EXTERN FAR struct mtd_dev_s *at45db_initialize(FAR struct spi_dev_s *dev);

/****************************************************************************
 * Name: at24c_initialize
 *
 * Description:
 *   Create an initialized MTD device instance.  MTD devices are not registered
 *   in the file system, but are created as instances that can be bound to
 *   other functions (such as a block or character driver front end).
 *
 ****************************************************************************/

EXTERN FAR struct mtd_dev_s *at24c_initialize(FAR struct i2c_dev_s *dev);

/****************************************************************************
 * Name: sst25_initialize
 *
 * Description:
 *   Create an initialized MTD device instance.  MTD devices are not registered
 *   in the file system, but are created as instances that can be bound to
 *   other functions (such as a block or character driver front end).
 *
 ****************************************************************************/

EXTERN FAR struct mtd_dev_s *sst25_initialize(FAR struct spi_dev_s *dev);

/****************************************************************************
 * Name: w25_initialize
 *
 * Description:
 *   Create an initialized MTD device instance.  MTD devices are not registered
 *   in the file system, but are created as instances that can be bound to
 *   other functions (such as a block or character driver front end).
 *
 ****************************************************************************/

EXTERN FAR struct mtd_dev_s *w25_initialize(FAR struct spi_dev_s *dev);

EXTERN FAR struct mtd_dev_s *at25_initialize(FAR struct spi_dev_s *dev);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_MTD_H */
