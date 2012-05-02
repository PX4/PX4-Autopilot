/****************************************************************************
 * drivers/mtd/rammtd.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_RAMMTD_BLOCKSIZE
#  define CONFIG_RAMMTD_BLOCKSIZE 512
#endif

#ifndef CONFIG_RAMMTD_ERASESIZE
#  define CONFIG_RAMMTD_ERASESIZE 4096
#endif

#ifndef CONFIG_RAMMTD_ERASESTATE
#  define CONFIG_RAMMTD_ERASESTATE 0xff
#endif

#if CONFIG_RAMMTD_ERASESTATE != 0xff && CONFIG_RAMMTD_ERASESTATE != 0x00
#  error "Unsupported value for CONFIG_RAMMTD_ERASESTATE"
#endif

#if CONFIG_RAMMTD_BLOCKSIZE > CONFIG_RAMMTD_ERASESIZE
#  error "Must have CONFIG_RAMMTD_BLOCKSIZE <= CONFIG_RAMMTD_ERASESIZE"
#endif

#undef  CONFIG_RAMMTD_BLKPER
#define CONFIG_RAMMTD_BLKPER (CONFIG_RAMMTD_ERASESIZE/CONFIG_RAMMTD_BLOCKSIZE)

#if CONFIG_RAMMTD_BLKPER*CONFIG_RAMMTD_BLOCKSIZE != CONFIG_RAMMTD_ERASESIZE
#  error "CONFIG_RAMMTD_ERASESIZE must be an even multiple of CONFIG_RAMMTD_BLOCKSIZE"
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This type represents the state of the MTD device.  The struct mtd_dev_s
 * must appear at the beginning of the definition so that you can freely
 * cast between pointers to struct mtd_dev_s and struct ram_dev_s.
 */

struct ram_dev_s
{
  struct mtd_dev_s mtd;      /* MTD device */
  FAR uint8_t     *start;    /* Start of RAM */
  size_t           nblocks;  /* Number of erase blocks */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* The RAM MTD driver may be useful just as it is, but another good use of
 * the RAM MTD driver is as a FLASH simulation -- to support testing of FLASH
 * based logic without having FLASH.  CONFIG_RAMMTD_FLASHSIM will add some
 * extra logic to improve the level of FLASH simulation.
 */

#define ram_read(dest, src, len) memcpy(dest, src, len)
#ifdef CONFIG_RAMMTD_FLASHSIM
static void *ram_write(FAR void *dest, FAR const void *src, size_t len);
#else
#  define ram_write(dest, src, len) memcpy(dest, src, len)
#endif

/* MTD driver methods */

static int ram_erase(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks);
static ssize_t ram_bread(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks,
                          FAR uint8_t *buf);
static ssize_t ram_bwrite(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks,
                           FAR const uint8_t *buf);
static int ram_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ram_write
 ****************************************************************************/

#ifdef CONFIG_RAMMTD_FLASHSIM
static void *ram_write(FAR void *dest, FAR const void *src, size_t len)
{
  FAR uint8_t       *pout = (FAR uint8_t *)dest;
  FAR const uint8_t *pin  = (FAR const uint8_t *)src;

  while (len-- > 0)
    {
      /* Get the source and destination values */

      uint8_t oldvalue = *pout;
      uint8_t srcvalue = *pin++;
      uint8_t newvalue;

      /* Get the new destination value, accounting for bits that cannot be
       * changes because they are not in the erased state.
       */

#if CONFIG_RAMMTD_ERASESTATE == 0xff
      newvalue = oldvalue & srcvalue; /* We can only clear bits */
#else /* CONFIG_RAMMTD_ERASESTATE == 0x00 */
      newvalue = oldvalue | srcvalue; /* We can only set bits */
#endif

      /* Report any attempt to change the value of bits that are not in the
       *  erased state.
       */

#ifdef CONFIG_DEBUG
      if (newvalue != srcvalue)
        {
          dbg("ERROR: Bad write: source=%02x dest=%02x result=%02x\n",
              srcvalue, oldvalue, newvalue);
        }
#endif

      /* Write the modified value to simulated FLASH */

      *pout++ = newvalue;
    }

  return dest;
}
#endif

/****************************************************************************
 * Name: ram_erase
 ****************************************************************************/

static int ram_erase(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks)
{
  FAR struct ram_dev_s *priv = (FAR struct ram_dev_s *)dev;
  off_t offset;
  size_t nbytes;

  DEBUGASSERT(dev);

  /* Don't let the erase exceed the size of the ram buffer */

  if (startblock >= priv->nblocks)
    {
      return 0;
    }

  if (startblock + nblocks > priv->nblocks)
    {
      nblocks = priv->nblocks - startblock;
    }

  /* Convert the erase block to a logical block and the number of blocks
   * in logical block numbers
   */

  startblock *= CONFIG_RAMMTD_BLKPER;
  nblocks    *= CONFIG_RAMMTD_BLKPER;

  /* Get the offset corresponding to the first block and the size
   * corresponding to the number of blocks.
   */

  offset = startblock * CONFIG_RAMMTD_BLOCKSIZE;
  nbytes = nblocks * CONFIG_RAMMTD_BLOCKSIZE;

  /* Then erase the data in RAM */

  memset(&priv->start[offset], CONFIG_RAMMTD_ERASESTATE, nbytes);
  return OK;
}

/****************************************************************************
 * Name: ram_bread
 ****************************************************************************/

static ssize_t ram_bread(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks,
                          FAR uint8_t *buf)
{
  FAR struct ram_dev_s *priv = (FAR struct ram_dev_s *)dev;
  off_t offset;
  off_t maxblock;
  size_t nbytes;

  DEBUGASSERT(dev && buf);

  /* Don't let the read exceed the size of the ram buffer */

  maxblock = priv->nblocks * CONFIG_RAMMTD_BLKPER;
  if (startblock >= maxblock)
    {
      return 0;
    }

  if (startblock + nblocks > maxblock)
    {
      nblocks = maxblock - startblock;
    }

  /* Get the offset corresponding to the first block and the size
   * corresponding to the number of blocks.
   */

  offset = startblock * CONFIG_RAMMTD_BLOCKSIZE;
  nbytes = nblocks * CONFIG_RAMMTD_BLOCKSIZE;

  /* Then read the data frp, RAM */

  ram_read(buf, &priv->start[offset], nbytes);
  return nblocks;
}

/****************************************************************************
 * Name: ram_bwrite
 ****************************************************************************/

static ssize_t ram_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                          size_t nblocks, FAR const uint8_t *buf)
{
  FAR struct ram_dev_s *priv = (FAR struct ram_dev_s *)dev;
  off_t offset;
  off_t maxblock;
  size_t nbytes;

  DEBUGASSERT(dev && buf);

  /* Don't let the write exceed the size of the ram buffer */

  maxblock = priv->nblocks * CONFIG_RAMMTD_BLKPER;
  if (startblock >= maxblock)
    {
      return 0;
    }

  if (startblock + nblocks > maxblock)
    {
      nblocks = maxblock - startblock;
    }

  /* Get the offset corresponding to the first block and the size
   * corresponding to the number of blocks.
   */

  offset = startblock * CONFIG_RAMMTD_BLOCKSIZE;
  nbytes = nblocks * CONFIG_RAMMTD_BLOCKSIZE;

  /* Then write the data to RAM */

  ram_write(&priv->start[offset], buf, nbytes);
  return nblocks;
}

/****************************************************************************
 * Name: ram_ioctl
 ****************************************************************************/

static int ram_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct ram_dev_s *priv = (FAR struct ram_dev_s *)dev;
  int ret = -EINVAL; /* Assume good command with bad parameters */

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo = (FAR struct mtd_geometry_s *)((uintptr_t)arg);
          if (geo)
            {
              /* Populate the geometry structure with information need to know
               * the capacity and how to access the device.
               */

              geo->blocksize    = CONFIG_RAMMTD_BLOCKSIZE;
              geo->erasesize    = CONFIG_RAMMTD_ERASESIZE;
              geo->neraseblocks = priv->nblocks;
              ret               = OK;
          }
        }
        break;

      case MTDIOC_XIPBASE:
        {
          FAR void **ppv = (FAR void**)((uintptr_t)arg);
          if (ppv)
            {
              /* Return (void*) base address of device memory */

              *ppv = (FAR void*)priv->start;
              ret  = OK;
            }
        }
        break;

      case MTDIOC_BULKERASE:
        {
            size_t size = priv->nblocks * CONFIG_RAMMTD_ERASESIZE;

	        /* Erase the entire device */

            memset(priv->start, CONFIG_RAMMTD_ERASESTATE, size);
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

FAR struct mtd_dev_s *rammtd_initialize(FAR uint8_t *start, size_t size)
{
  FAR struct ram_dev_s *priv;
  size_t nblocks;

  /* Create an instance of the RAM MTD device state structure */

  priv = (FAR struct ram_dev_s *)kmalloc(sizeof(struct ram_dev_s));
  if (!priv)
    {
      fdbg("Failed to allocate the RAM MTD state structure\n");
      return NULL;
    }

  /* Force the size to be an even number of the erase block size */

  nblocks = size / CONFIG_RAMMTD_ERASESIZE;
  if (nblocks <= 0)
    {
      fdbg("Need to provide at least one full erase block\n");
      return NULL;
    }
  
  /* Perform initialization as necessary */

  priv->mtd.erase  = ram_erase;
  priv->mtd.bread  = ram_bread;
  priv->mtd.bwrite = ram_bwrite;
  priv->mtd.ioctl  = ram_ioctl;
  priv->mtd.erase  = ram_erase;

  priv->start      = start;
  priv->nblocks    = nblocks;
  return &priv->mtd;
}
