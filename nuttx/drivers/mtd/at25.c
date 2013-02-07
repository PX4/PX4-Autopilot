/************************************************************************************
 * drivers/mtd/at25.c
 * Driver for SPI-based AT25DF321 (32Mbit) flash.
 *
 *   Copyright (C) 2009-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Petteri Aimonen <jpa@nx.mail.kapsi.fi>
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi.h>
#include <nuttx/mtd.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#ifndef CONFIG_AT25_SPIMODE
#  define CONFIG_AT25_SPIMODE SPIDEV_MODE0
#endif

/* AT25 Registers *******************************************************************/
/* Indentification register values */

#define AT25_MANUFACTURER         0x1F
#define AT25_AT25DF321_TYPE       0x47 /* 32 M-bit */

/*  AT25DF321 capacity is 4,194,304 bytes:
 *  (64 sectors) * (65,536 bytes per sector)
 *  (16384 pages) * (256 bytes per page)
 */

#define AT25_AT25DF321_SECTOR_SHIFT  12    /* Sector size 1 << 12 = 4096 */
#define AT25_AT25DF321_NSECTORS      1024
#define AT25_AT25DF321_PAGE_SHIFT    9     /* Page size 1 << 9 = 512 */
#define AT25_AT25DF321_NPAGES        8192

/* Instructions */
/*      Command        Value      N Description             Addr Dummy Data */
#define AT25_WREN      0x06    /* 1 Write Enable              0   0     0 */
#define AT25_WRDI      0x04    /* 1 Write Disable             0   0     0 */
#define AT25_RDID      0x9f    /* 1 Read Identification       0   0     1-3 */
#define AT25_RDSR      0x05    /* 1 Read Status Register      0   0     >=1 */
#define AT25_WRSR      0x01    /* 1 Write Status Register     0   0     1 */
#define AT25_READ      0x03    /* 1 Read Data Bytes           3   0     >=1 */
#define AT25_FAST_READ 0x0b    /* 1 Higher speed read         3   1     >=1 */
#define AT25_PP        0x02    /* 1 Page Program              3   0     1-256 */
#define AT25_SE        0x20    /* 1 Sector Erase              3   0     0 */
#define AT25_BE        0xc7    /* 1 Bulk Erase                0   0     0 */
#define AT25_DP        0xb9    /* 2 Deep power down           0   0     0 */
#define AT25_RES       0xab    /* 2 Read Electronic Signature 0   3     >=1 */

/* Status register bit definitions */

#define AT25_SR_WIP            (1 << 0)    /* Bit 0: Write in progress bit */
#define AT25_SR_WEL            (1 << 1)    /* Bit 1: Write enable latch bit */
#define AT25_SR_EPE            (1 << 5)    /* Bit 5: Erase/program error */
#define AT25_SR_UNPROT         0x00        /* Global unprotect command */

#define AT25_DUMMY     0xa5

/************************************************************************************
 * Private Types
 ************************************************************************************/

/* This type represents the state of the MTD device.  The struct mtd_dev_s
 * must appear at the beginning of the definition so that you can freely
 * cast between pointers to struct mtd_dev_s and struct at25_dev_s.
 */

struct at25_dev_s
{
  struct mtd_dev_s mtd;      /* MTD interface */
  FAR struct spi_dev_s *dev; /* Saved SPI interface instance */
  uint8_t  sectorshift;      /* 16 or 18 */
  uint8_t  pageshift;        /* 8 */
  uint16_t nsectors;         /* 128 or 64 */
  uint32_t npages;           /* 32,768 or 65,536 */
};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

/* Helpers */

static void at25_lock(FAR struct spi_dev_s *dev);
static inline void at25_unlock(FAR struct spi_dev_s *dev);
static inline int at25_readid(struct at25_dev_s *priv);
static void at25_waitwritecomplete(struct at25_dev_s *priv);
static void at25_writeenable(struct at25_dev_s *priv);
static inline void at25_sectorerase(struct at25_dev_s *priv, off_t offset);
static inline int  at25_bulkerase(struct at25_dev_s *priv);
static inline void at25_pagewrite(struct at25_dev_s *priv, FAR const uint8_t *buffer,
                                  off_t offset);

/* MTD driver methods */

static int at25_erase(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks);
static ssize_t at25_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                          size_t nblocks, FAR uint8_t *buf);
static ssize_t at25_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks, FAR const uint8_t *buf);
static ssize_t at25_read(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
                         FAR uint8_t *buffer);
static int at25_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg);

/************************************************************************************
 * Private Data
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: at25_lock
 ************************************************************************************/

static void at25_lock(FAR struct spi_dev_s *dev)
{
  /* On SPI busses where there are multiple devices, it will be necessary to
   * lock SPI to have exclusive access to the busses for a sequence of
   * transfers.  The bus should be locked before the chip is selected.
   *
   * This is a blocking call and will not return until we have exclusiv access to
   * the SPI buss.  We will retain that exclusive access until the bus is unlocked.
   */

  (void)SPI_LOCK(dev, true);

  /* After locking the SPI bus, the we also need call the setfrequency, setbits, and
   * setmode methods to make sure that the SPI is properly configured for the device.
   * If the SPI buss is being shared, then it may have been left in an incompatible
   * state.
   */

  SPI_SETMODE(dev, CONFIG_AT25_SPIMODE);
  SPI_SETBITS(dev, 8);
  (void)SPI_SETFREQUENCY(dev, 20000000);
}

/************************************************************************************
 * Name: at25_unlock
 ************************************************************************************/

static inline void at25_unlock(FAR struct spi_dev_s *dev)
{
  (void)SPI_LOCK(dev, false);
}

/************************************************************************************
 * Name: at25_readid
 ************************************************************************************/

static inline int at25_readid(struct at25_dev_s *priv)
{
  uint16_t manufacturer;
  uint16_t memory;
  uint16_t version;

  fvdbg("priv: %p\n", priv);

  /* Lock the SPI bus, configure the bus, and select this FLASH part. */

  at25_lock(priv->dev);
  SPI_SELECT(priv->dev, SPIDEV_FLASH, true);

  /* Send the "Read ID (RDID)" command and read the first three ID bytes */

  (void)SPI_SEND(priv->dev, AT25_RDID);
  manufacturer = SPI_SEND(priv->dev, AT25_DUMMY);
  memory       = SPI_SEND(priv->dev, AT25_DUMMY);
  
  /* Deselect the FLASH and unlock the bus */

  SPI_SELECT(priv->dev, SPIDEV_FLASH, false);
  at25_unlock(priv->dev);

  fvdbg("manufacturer: %02x memory: %02x\n",
        manufacturer, memory);

  /* Check for a valid manufacturer and memory type */

  if (manufacturer == AT25_MANUFACTURER && memory == AT25_AT25DF321_TYPE)
    {
        priv->sectorshift = AT25_AT25DF321_SECTOR_SHIFT;
        priv->nsectors    = AT25_AT25DF321_NSECTORS;
        priv->pageshift   = AT25_AT25DF321_PAGE_SHIFT;
        priv->npages      = AT25_AT25DF321_NPAGES;
        return OK;
    }

  return -ENODEV;
}

/************************************************************************************
 * Name: at25_waitwritecomplete
 ************************************************************************************/

static void at25_waitwritecomplete(struct at25_dev_s *priv)
{
  uint8_t status;

  /* Are we the only device on the bus? */

#ifdef CONFIG_SPI_OWNBUS

  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH, true);

  /* Send "Read Status Register (RDSR)" command */

  (void)SPI_SEND(priv->dev, AT25_RDSR);
  
  /* Loop as long as the memory is busy with a write cycle */

  do
    {
      /* Send a dummy byte to generate the clock needed to shift out the status */

      status = SPI_SEND(priv->dev, AT25_DUMMY);
    }
  while ((status & AT25_SR_WIP) != 0);

  /* Deselect the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH, false);

#else

  /* Loop as long as the memory is busy with a write cycle */

  do
    {
      /* Select this FLASH part */

      SPI_SELECT(priv->dev, SPIDEV_FLASH, true);

      /* Send "Read Status Register (RDSR)" command */

      (void)SPI_SEND(priv->dev, AT25_RDSR);

      /* Send a dummy byte to generate the clock needed to shift out the status */

      status = SPI_SEND(priv->dev, AT25_DUMMY);

      /* Deselect the FLASH */

      SPI_SELECT(priv->dev, SPIDEV_FLASH, false);

      /* Given that writing could take up to few tens of milliseconds, and erasing
       * could take more.  The following short delay in the "busy" case will allow
       * other peripherals to access the SPI bus.
       */

      if ((status & AT25_SR_WIP) != 0)
        {
          at25_unlock(priv->dev);
          usleep(10000);
          at25_lock(priv->dev);
        }
    }
  while ((status & AT25_SR_WIP) != 0);
#endif

  if (status & AT25_SR_EPE)
    {
      fdbg("Write error, status: 0x%02x\n", status);
    }
  
  fvdbg("Complete, status: 0x%02x\n", status);
}

/************************************************************************************
 * Name:  at25_writeenable
 ************************************************************************************/

static void at25_writeenable(struct at25_dev_s *priv)
{
  SPI_SELECT(priv->dev, SPIDEV_FLASH, true);
  (void)SPI_SEND(priv->dev, AT25_WREN);
  SPI_SELECT(priv->dev, SPIDEV_FLASH, false);
  fvdbg("Enabled\n");
}

/************************************************************************************
 * Name:  at25_sectorerase
 ************************************************************************************/

static inline void at25_sectorerase(struct at25_dev_s *priv, off_t sector)
{
  off_t offset = sector << priv->sectorshift;

  fvdbg("sector: %08lx\n", (long)sector);

  /* Wait for any preceding write to complete.  We could simplify things by
   * perform this wait at the end of each write operation (rather than at
   * the beginning of ALL operations), but have the wait first will slightly
   * improve performance.
   */

  at25_waitwritecomplete(priv);

  /* Send write enable instruction */

  at25_writeenable(priv);

  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH, true);

  /* Send the "Sector Erase (SE)" instruction */

  (void)SPI_SEND(priv->dev, AT25_SE);

  /* Send the sector offset high byte first.  For all of the supported
   * parts, the sector number is completely contained in the first byte
   * and the values used in the following two bytes don't really matter.
   */

  (void)SPI_SEND(priv->dev, (offset >> 16) & 0xff);
  (void)SPI_SEND(priv->dev, (offset >> 8) & 0xff);
  (void)SPI_SEND(priv->dev, offset & 0xff);

  /* Deselect the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH, false);
  fvdbg("Erased\n");
}

/************************************************************************************
 * Name:  at25_bulkerase
 ************************************************************************************/

static inline int at25_bulkerase(struct at25_dev_s *priv)
{
  fvdbg("priv: %p\n", priv);

  /* Wait for any preceding write to complete.  We could simplify things by
   * perform this wait at the end of each write operation (rather than at
   * the beginning of ALL operations), but have the wait first will slightly
   * improve performance.
   */

  at25_waitwritecomplete(priv);

  /* Send write enable instruction */

  at25_writeenable(priv);

  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH, true);

  /* Send the "Bulk Erase (BE)" instruction */

  (void)SPI_SEND(priv->dev, AT25_BE);

  /* Deselect the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH, false);
  fvdbg("Return: OK\n");
  return OK;
}

/************************************************************************************
 * Name:  at25_pagewrite
 ************************************************************************************/

static inline void at25_pagewrite(struct at25_dev_s *priv, FAR const uint8_t *buffer,
                                  off_t page)
{
  off_t offset = page << 8;

  fvdbg("page: %08lx offset: %08lx\n", (long)page, (long)offset);

  /* Wait for any preceding write to complete.  We could simplify things by
   * perform this wait at the end of each write operation (rather than at
   * the beginning of ALL operations), but have the wait first will slightly
   * improve performance.
   */

  at25_waitwritecomplete(priv);

  /* Enable the write access to the FLASH */

  at25_writeenable(priv);
  
  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH, true);

  /* Send "Page Program (PP)" command */

  (void)SPI_SEND(priv->dev, AT25_PP);

  /* Send the page offset high byte first. */

  (void)SPI_SEND(priv->dev, (offset >> 16) & 0xff);
  (void)SPI_SEND(priv->dev, (offset >> 8) & 0xff);
  (void)SPI_SEND(priv->dev, offset & 0xff);

  /* Then write the specified number of bytes */

  SPI_SNDBLOCK(priv->dev, buffer, 256);
  
  /* Deselect the FLASH: Chip Select high */

  SPI_SELECT(priv->dev, SPIDEV_FLASH, false);
  fvdbg("Written\n");
}

/************************************************************************************
 * Name: at25_erase
 ************************************************************************************/

static int at25_erase(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks)
{
  FAR struct at25_dev_s *priv = (FAR struct at25_dev_s *)dev;
  size_t blocksleft = nblocks;

  fvdbg("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* Lock access to the SPI bus until we complete the erase */

  at25_lock(priv->dev);
  while (blocksleft-- > 0)
    {
      /* Erase each sector */

      at25_sectorerase(priv, startblock);
      startblock++;
    }

  at25_unlock(priv->dev);
  return (int)nblocks;
}

/************************************************************************************
 * Name: at25_bread
 ************************************************************************************/

static ssize_t at25_bread(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks,
                          FAR uint8_t *buffer)
{
  FAR struct at25_dev_s *priv = (FAR struct at25_dev_s *)dev;
  ssize_t nbytes;

  fvdbg("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* On this device, we can handle the block read just like the byte-oriented read */

  nbytes = at25_read(dev, startblock << priv->pageshift, nblocks << priv->pageshift, buffer);
  if (nbytes > 0)
    {
        return nbytes >> priv->pageshift;
    }

  return (int)nbytes;
}

/************************************************************************************
 * Name: at25_bwrite
 ************************************************************************************/

static ssize_t at25_bwrite(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks,
                           FAR const uint8_t *buffer)
{
  FAR struct at25_dev_s *priv = (FAR struct at25_dev_s *)dev;
  size_t blocksleft = nblocks;

  fvdbg("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* Lock the SPI bus and write each page to FLASH */

  at25_lock(priv->dev);
  while (blocksleft-- > 0)
    {
      at25_pagewrite(priv, buffer, startblock * 2);
      at25_pagewrite(priv, buffer + 256, startblock * 2 + 1);
      buffer += 1 << priv->pageshift;
      startblock++;
   }

  at25_unlock(priv->dev);
  return nblocks;
}

/************************************************************************************
 * Name: at25_read
 ************************************************************************************/

static ssize_t at25_read(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
                         FAR uint8_t *buffer)
{
  FAR struct at25_dev_s *priv = (FAR struct at25_dev_s *)dev;

  fvdbg("offset: %08lx nbytes: %d\n", (long)offset, (int)nbytes);

  /* Wait for any preceding write to complete.  We could simplify things by
   * perform this wait at the end of each write operation (rather than at
   * the beginning of ALL operations), but have the wait first will slightly
   * improve performance.
   */

  at25_waitwritecomplete(priv);

  /* Lock the SPI bus and select this FLASH part */

  at25_lock(priv->dev);
  SPI_SELECT(priv->dev, SPIDEV_FLASH, true);

  /* Send "Read from Memory " instruction */

  (void)SPI_SEND(priv->dev, AT25_READ);

  /* Send the page offset high byte first. */

  (void)SPI_SEND(priv->dev, (offset >> 16) & 0xff);
  (void)SPI_SEND(priv->dev, (offset >> 8) & 0xff);
  (void)SPI_SEND(priv->dev, offset & 0xff);

  /* Then read all of the requested bytes */

  SPI_RECVBLOCK(priv->dev, buffer, nbytes);

  /* Deselect the FLASH and unlock the SPI bus */

  SPI_SELECT(priv->dev, SPIDEV_FLASH, false);
  at25_unlock(priv->dev);

  fvdbg("return nbytes: %d\n", (int)nbytes);
  return nbytes;
}

/************************************************************************************
 * Name: at25_ioctl
 ************************************************************************************/

static int at25_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct at25_dev_s *priv = (FAR struct at25_dev_s *)dev;
  int ret = -EINVAL; /* Assume good command with bad parameters */

  fvdbg("cmd: %d \n", cmd);

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo = (FAR struct mtd_geometry_s *)((uintptr_t)arg);
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

              geo->blocksize    = (1 << priv->pageshift);
              geo->erasesize    = (1 << priv->sectorshift);
              geo->neraseblocks = priv->nsectors;
              ret               = OK;

              fvdbg("blocksize: %d erasesize: %d neraseblocks: %d\n",
                    geo->blocksize, geo->erasesize, geo->neraseblocks);
            }
        }
        break;

      case MTDIOC_BULKERASE:
        {
            /* Erase the entire device */

            at25_lock(priv->dev);
            ret = at25_bulkerase(priv);
            at25_unlock(priv->dev);
        }
        break;
 
      case MTDIOC_XIPBASE:
      default:
        ret = -ENOTTY; /* Bad command */
        break;
    }

  fvdbg("return %d\n", ret);
  return ret;
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: at25_initialize
 *
 * Description:
 *   Create an initialize MTD device instance.  MTD devices are not registered
 *   in the file system, but are created as instances that can be bound to
 *   other functions (such as a block or character driver front end).
 *
 ************************************************************************************/

FAR struct mtd_dev_s *at25_initialize(FAR struct spi_dev_s *dev)
{
  FAR struct at25_dev_s *priv;
  int ret;

  fvdbg("dev: %p\n", dev);

  /* Allocate a state structure (we allocate the structure instead of using
   * a fixed, static allocation so that we can handle multiple FLASH devices.
   * The current implementation would handle only one FLASH part per SPI
   * device (only because of the SPIDEV_FLASH definition) and so would have
   * to be extended to handle multiple FLASH parts on the same SPI bus.
   */

  priv = (FAR struct at25_dev_s *)kmalloc(sizeof(struct at25_dev_s));
  if (priv)
    {
      /* Initialize the allocated structure */

      priv->mtd.erase  = at25_erase;
      priv->mtd.bread  = at25_bread;
      priv->mtd.bwrite = at25_bwrite;
      priv->mtd.read   = at25_read;
      priv->mtd.ioctl  = at25_ioctl;
      priv->dev        = dev;

      /* Deselect the FLASH */

      SPI_SELECT(dev, SPIDEV_FLASH, false);

      /* Identify the FLASH chip and get its capacity */

      ret = at25_readid(priv);
      if (ret != OK)
        {
          /* Unrecognized! Discard all of that work we just did and return NULL */

          fdbg("Unrecognized\n");
          kfree(priv);
          priv = NULL;
        }
      else
        {
          /* Unprotect all sectors */

          at25_writeenable(priv);
          SPI_SELECT(priv->dev, SPIDEV_FLASH, true);
          (void)SPI_SEND(priv->dev, AT25_WRSR);
          (void)SPI_SEND(priv->dev, AT25_SR_UNPROT);
          SPI_SELECT(priv->dev, SPIDEV_FLASH, false);
        }
    }

  /* Return the implementation-specific state structure as the MTD device */

  fvdbg("Return %p\n", priv);
  return (FAR struct mtd_dev_s *)priv;
}
