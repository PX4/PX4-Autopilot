/****************************************************************************
 * drivers/mtd/skeleton.c
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
#include <stdint.h>
#include <errno.h>

#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This type represents the state of the MTD device.  The struct mtd_dev_s
 * must appear at the beginning of the definition so that you can freely
 * cast between pointers to struct mtd_dev_s and struct skel_dev_s.
 */

struct skel_dev_s
{
  struct mtd_dev_s mtd;

  /* Other implementation specific data may follow here */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* MTD driver methods */

static int skel_erase(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks);
static ssize_t skel_bread(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks,
                          FAR uint8_t *buf);
static ssize_t skel_bwrite(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks,
                           FAR const uint8_t *buf);
static ssize_t skel_read(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
                         FAR uint8_t *buffer);
static int skel_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct skel_dev_s g_skeldev =
{
  { skel_erase, skel_rbead, skel_bwrite, skel_read, skel_ioctl },
  /* Initialization of any other implemenation specific data goes here */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: skel_erase
 ****************************************************************************/

static int skel_erase(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks)
{
  FAR struct skel_dev_s *priv = (FAR struct skel_dev_s *)dev;

  /* The interface definition assumes that all erase blocks ar the same size.
   * If that is not true for this particular device, then transform the
   * start block and nblocks as necessary.
   */

  /* Erase the specified blocks and return status (OK or a negated errno) */

  return OK;
}

/****************************************************************************
 * Name: skel_bread
 ****************************************************************************/

static ssize_t skel_bread(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks,
                          FAR uint8_t *buf)
{
  FAR struct skel_dev_s *priv = (FAR struct skel_dev_s *)dev;

  /* The interface definition assumes that all read/write blocks ar the same size.
   * If that is not true for this particular device, then transform the
   * start block and nblocks as necessary.
   */

  /* Read the specified blocks into the provided user buffer and return status
   * (The positive, number of blocks actually read or a negated errno). 
   */

  return 0;
}

/****************************************************************************
 * Name: skel_bwrite
 ****************************************************************************/

static ssize_t skel_bwrite(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks,
                           FAR const uint8_t *buf)
{
  FAR struct skel_dev_s *priv = (FAR struct skel_dev_s *)dev;

  /* The interface definition assumes that all read/write blocks ar the same size.
   * If that is not true for this particular device, then transform the
   * start block and nblocks as necessary.
   */

  /* Write the specified blocks from the provided user buffer and return status
   * (The positive, number of blocks actually written or a negated errno)
   */

  return 0;
}

/****************************************************************************
 * Name: skel_read
 ****************************************************************************/

static ssize_t skel_read(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
                         FAR uint8_t *buffer)
{
  FAR struct skel_dev_s *priv = (FAR struct skel_dev_s *)dev;

  /* Some devices may support byte oriented read (optional).  Byte-oriented
   * writing is inherently block oriented on most MTD devices and is not supported.
   * It is recommended that low-level drivers not support read() if it requires
   * buffering -- let the higher level logic handle that.  If the read method is
   * not implemented, just set the method pointer to NULL in the struct mtd_dev_s
   * instance.
   */

  /* The interface definition assumes that all read/write blocks ar the same size.
   * If that is not true for this particular device, then transform the
   * start block and nblocks as necessary.
   */

  /* Read the specified blocks into the provided user buffer and return status
   * (The positive, number of blocks actually read or a negated errno)
   */

  return 0;
}

/****************************************************************************
 * Name: skel_ioctl
 ****************************************************************************/

static int skel_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct skel_dev_s *priv = (FAR struct skel_dev_s *)dev;
  int ret = -EINVAL; /* Assume good command with bad parameters */

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo = (FARstruct mtd_geometry_s *)arg;
          if (geo)
            {
              /* Populate the geometry structure with information need to know
               * the capacity and how to access the device.
               *
               * NOTE: that the device is treated as though it where just an array
               * of fixed size blocks.  That is most likely not true, but the client
               * will expect the device logic to do whatever is necessary to make it
               * appear so.
               */

              geo->blocksize    = 512;  /* Size of one read/write block */
              geo->erasesize    = 4096; /* Size of one erase block */
              geo->neraseblocks = 1024; /* Number of erase blocks */
              ret               = OK;
          }
        }
        break;

      case MTDIOC_XIPBASE:
        {
          FAR void **ppv = (FAR void**)arg;

          if (ppv)
            {
              /* If media is directly acccesible, return (void*) base address
               * of device memory.  NULL otherwise.  It is acceptable to omit
               * this case altogether and simply return -ENOTTY.
               */

              *ppv = NULL;
              ret  = OK;
            }
        }
        break;

      case MTDIOC_BULKERASE:
        {
	        /* Erase the entire device */

	        ret = OK;
        }
        break;
 
      default:
        ret = -ENOTTY; /* Bad command */
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: skel_initialize
 *
 * Description:
 *   Create and initialize an MTD device instance.  MTD devices are not
 *   registered in the file system, but are created as instances that can
 *   be bound to other functions (such as a block or character driver front
 *   end).
 *
 ****************************************************************************/

void skel_initialize(void)
{
  /* Perform initialization as necessary */

  /* Return the implementation-specific state structure as the MTD device */

  return (FAR struct mtd_dev_s *)&g_skeldev;
}
