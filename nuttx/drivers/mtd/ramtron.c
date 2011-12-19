/************************************************************************************
 * drivers/mtd/ramtron.c
 * Driver for SPI-based RAMTRON NVRAM Devices FM25V10 and others (not tested)
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Copyright (C) 2009-2010 Gregory Nutt. All rights reserved.
 *   Author: Uros Platise <uros.platise@isotel.eu>
 *           Gregory Nutt <spudmonkey@racsa.co.cr>
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

/* OPTIONS:
 *  - additional non-jedec standard device: FM25H20 
 *    must be enabled with the CONFIG_RAMTRON_FRAM_NON_JEDEC=y
 * 
 * NOTE:
 *  - frequency is fixed to desired max by RAMTRON_INIT_CLK_MAX
 *    if new devices with different speed arrive, then SETFREQUENCY() 
 *    needs to handle freq changes and INIT_CLK_MAX must be reduced
 *    to fit all devices. Note that STM32_SPI driver is prone to
 *    too high freq. parameters and limit it within physical constraints.
 * 
 * TODO:
 *  - add support for sleep
 *  - add support for faster read FSTRD command
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <errno.h>
#include <debug.h>
#include <assert.h>

#include <nuttx/kmalloc.h>
#include <nuttx/ioctl.h>
#include <nuttx/spi.h>
#include <nuttx/mtd.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/*  RAMTRON devices are flat!
 *  For purpose of the VFAT file system we emulate the following configuration:
 */

#define RAMTRON_EMULATE_SECTOR_SHIFT  9
#define RAMTRON_EMULATE_PAGE_SHIFT    9

/* RAMTRON Indentification register values */

#define RAMTRON_MANUFACTURER         0x7F
#define RAMTRON_MEMORY_TYPE          0xC2

/* Instructions:
 *      Command          Value       N Description             Addr Dummy Data */
#define RAMTRON_WREN      0x06    /* 1 Write Enable              0   0     0 */
#define RAMTRON_WRDI      0x04    /* 1 Write Disable             0   0     0 */
#define RAMTRON_RDSR      0x05    /* 1 Read Status Register      0   0     >=1 */
#define RAMTRON_WRSR      0x01    /* 1 Write Status Register     0   0     1 */
#define RAMTRON_READ      0x03    /* 1 Read Data Bytes           A   0     >=1 */
#define RAMTRON_FSTRD     0x0b    /* 1 Higher speed read         A   1     >=1 */
#define RAMTRON_WRITE     0x02    /* 1 Write                     A   0     1-256 */
#define RAMTRON_SLEEP     0xb9    // TODO:
#define RAMTRON_RDID      0x9f    /* 1 Read Identification       0   0     1-3 */
#define RAMTRON_SN        0xc3	  // TODO:


/* Status register bit definitions */

#define RAMTRON_SR_WIP            (1 << 0)                /* Bit 0: Write in progress bit */
#define RAMTRON_SR_WEL            (1 << 1)                /* Bit 1: Write enable latch bit */
#define RAMTRON_SR_BP_SHIFT       (2)                     /* Bits 2-4: Block protect bits */
#define RAMTRON_SR_BP_MASK        (7 << RAMTRON_SR_BP_SHIFT)
#  define RAMTRON_SR_BP_NONE      (0 << RAMTRON_SR_BP_SHIFT) /* Unprotected */
#  define RAMTRON_SR_BP_UPPER64th (1 << RAMTRON_SR_BP_SHIFT) /* Upper 64th */
#  define RAMTRON_SR_BP_UPPER32nd (2 << RAMTRON_SR_BP_SHIFT) /* Upper 32nd */
#  define RAMTRON_SR_BP_UPPER16th (3 << RAMTRON_SR_BP_SHIFT) /* Upper 16th */
#  define RAMTRON_SR_BP_UPPER8th  (4 << RAMTRON_SR_BP_SHIFT) /* Upper 8th */
#  define RAMTRON_SR_BP_UPPERQTR  (5 << RAMTRON_SR_BP_SHIFT) /* Upper quarter */
#  define RAMTRON_SR_BP_UPPERHALF (6 << RAMTRON_SR_BP_SHIFT) /* Upper half */
#  define RAMTRON_SR_BP_ALL       (7 << RAMTRON_SR_BP_SHIFT) /* All sectors */
#define RAMTRON_SR_SRWD           (1 << 7)                /* Bit 7: Status register write protect */

#define RAMTRON_DUMMY     0xa5

/************************************************************************************
 * Private Types
 ************************************************************************************/

struct ramtron_parts_s
{
	const char *name;
	uint8_t     id1;
	uint8_t     id2;
	uint32_t    size;
	uint8_t     addr_len;
	uint32_t	speed;
};

/* This type represents the state of the MTD device.  The struct mtd_dev_s
 * must appear at the beginning of the definition so that you can freely
 * cast between pointers to struct mtd_dev_s and struct ramtron_dev_s.
 */

struct ramtron_dev_s
{
  struct mtd_dev_s mtd;      /* MTD interface */
  FAR struct spi_dev_s *dev; /* Saved SPI interface instance */
  uint8_t  sectorshift;
  uint8_t  pageshift;
  uint16_t nsectors;
  uint32_t npages;
  const struct ramtron_parts_s *part;	/* part instance */
};

/************************************************************************************
 * Supported Part Lists
 ************************************************************************************/

// Defines the initial speed compatible with all devices. In case of RAMTRON
// the defined devices within the part list have all the same speed.
#define RAMTRON_INIT_CLK_MAX		40000000UL

static struct ramtron_parts_s ramtron_parts[] =
{
	{
		"FM25V02",                    /* name */
		0x22,                         /* id1 */
		0x00,                         /* id2 */
		32L*1024L,                    /* size */
		2,                            /* addr_len */
		40000000                      /* speed */
	},
	{
		"FM25VN02",                    /* name */
		0x22,                          /* id1 */
		0x01,                          /* id2 */
		32L*1024L,                     /* size */
		2,                             /* addr_len */
		40000000                       /* speed */
	},
	{
		"FM25V05",                    /* name */
		0x23,                         /* id1 */
		0x00,                         /* id2 */
		64L*1024L,                    /* size */
		2,                            /* addr_len */
		40000000                      /* speed */
	},
	{
		"FM25VN05",                    /* name */
		0x23,                          /* id1 */
		0x01,                          /* id2 */
		64L*1024L,                     /* size */
		2,                             /* addr_len */
		40000000                       /* speed */
	},
	{
		"FM25V10",                    /* name */
		0x24,                         /* id1 */
		0x00,                         /* id2 */
		128L*1024L,                   /* size */
		3,                            /* addr_len */
		40000000                      /* speed */
	},
	{
		"FM25VN10",                   /* name */
		0x24,                         /* id1 */
		0x01,                         /* id2 */
		128L*1024L,                   /* size */
		3,                            /* addr_len */
		40000000                      /* speed */
	},
#ifdef CONFIG_RAMTRON_FRAM_NON_JEDEC
	{
		"FM25H20",                    /* name */
		0xff,                         /* id1 */
		0xff,                         /* id2 */
		256L*1024L,                   /* size */
		3,                            /* addr_len */
		40000000                      /* speed */
	},
	{
		NULL,                         /* name */
        0,                            /* id1 */
        0,                            /* id2 */
		0,                            /* size */
        0,                            /* addr_len */
        0                             /* speed */
	}
#endif
};


/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

/* Helpers */

static void ramtron_lock(FAR struct spi_dev_s *dev);
static inline void ramtron_unlock(FAR struct spi_dev_s *dev);
static inline int ramtron_readid(struct ramtron_dev_s *priv);
static void ramtron_waitwritecomplete(struct ramtron_dev_s *priv);
static void ramtron_writeenable(struct ramtron_dev_s *priv);
static inline void ramtron_pagewrite(struct ramtron_dev_s *priv, FAR const uint8_t *buffer,
                                  off_t offset);

/* MTD driver methods */

static int ramtron_erase(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks);
static ssize_t ramtron_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                          size_t nblocks, FAR uint8_t *buf);
static ssize_t ramtron_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks, FAR const uint8_t *buf);
static ssize_t ramtron_read(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
                         FAR uint8_t *buffer);
static int ramtron_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg);

/************************************************************************************
 * Private Data
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: ramtron_lock
 ************************************************************************************/

static void ramtron_lock(FAR struct spi_dev_s *dev)
{
  /* On SPI busses where there are multiple devices, it will be necessary to
   * lock SPI to have exclusive access to the busses for a sequence of
   * transfers.  The bus should be locked before the chip is selected.
   *
   * This is a blocking call and will not return until we have exclusiv access to
   * the SPI buss.  We will retain that exclusive access until the bus is unlocked.
   */

  SPI_LOCK(dev, true);

  /* After locking the SPI bus, the we also need call the setfrequency, setbits, and
   * setmode methods to make sure that the SPI is properly configured for the device.
   * If the SPI buss is being shared, then it may have been left in an incompatible
   * state.
   */

  SPI_SETMODE(dev, SPIDEV_MODE3);
  SPI_SETBITS(dev, 8);
  
  (void)SPI_SETFREQUENCY(dev, RAMTRON_INIT_CLK_MAX);
}

/************************************************************************************
 * Name: ramtron_unlock
 ************************************************************************************/

static inline void ramtron_unlock(FAR struct spi_dev_s *dev)
{
  SPI_LOCK(dev, false);
}

/************************************************************************************
 * Name: ramtron_readid
 ************************************************************************************/

static inline int ramtron_readid(struct ramtron_dev_s *priv)
{
  uint16_t manufacturer, memory, capacity, part;
  int i;

  fvdbg("priv: %p\n", priv);

  /* Lock the SPI bus, configure the bus, and select this FLASH part. */

  ramtron_lock(priv->dev);
  SPI_SELECT(priv->dev, SPIDEV_FLASH, true);

  /* Send the "Read ID (RDID)" command and read the first three ID bytes */

  (void)SPI_SEND(priv->dev, RAMTRON_RDID);
  for (i=0; i<6; i++) manufacturer = SPI_SEND(priv->dev, RAMTRON_DUMMY);
  memory       = SPI_SEND(priv->dev, RAMTRON_DUMMY);
  capacity     = SPI_SEND(priv->dev, RAMTRON_DUMMY);	// fram.id1
  part         = SPI_SEND(priv->dev, RAMTRON_DUMMY);	// fram.id2

  /* Deselect the FLASH and unlock the bus */

  SPI_SELECT(priv->dev, SPIDEV_FLASH, false);
  ramtron_unlock(priv->dev);
  
  // Select part from the part list
  for (priv->part = ramtron_parts;
	   priv->part->name != NULL && !(priv->part->id1 == capacity && priv->part->id2 == part);
	   priv->part++);
	   
  if (priv->part->name) {
    fvdbg("RAMTRON %s of size %d bytes (mf:%02x mem:%02x cap:%02x part:%02x)\n",
           priv->part->name, priv->part->size, manufacturer, memory, capacity, part);

    priv->sectorshift = RAMTRON_EMULATE_SECTOR_SHIFT;
    priv->nsectors    = priv->part->size / (1 << RAMTRON_EMULATE_SECTOR_SHIFT);
    priv->pageshift   = RAMTRON_EMULATE_PAGE_SHIFT;
    priv->npages      = priv->part->size / (1 << RAMTRON_EMULATE_PAGE_SHIFT);
    return OK;
  }
 
  fvdbg("RAMTRON device not found\n");
  return -ENODEV;
}

/************************************************************************************
 * Name: ramtron_waitwritecomplete
 ************************************************************************************/

static void ramtron_waitwritecomplete(struct ramtron_dev_s *priv)
{
  uint8_t status;

  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH, true);

  /* Send "Read Status Register (RDSR)" command */

  (void)SPI_SEND(priv->dev, RAMTRON_RDSR);
  
  /* Loop as long as the memory is busy with a write cycle */

  do
    {
      /* Send a dummy byte to generate the clock needed to shift out the status */

      status = SPI_SEND(priv->dev, RAMTRON_DUMMY);
    }
  while ((status & RAMTRON_SR_WIP) != 0);

  /* Deselect the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH, false);
  fvdbg("Complete\n");
}

/************************************************************************************
 * Name:  ramtron_writeenable
 ************************************************************************************/

static void ramtron_writeenable(struct ramtron_dev_s *priv)
{
  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH, true);

  /* Send "Write Enable (WREN)" command */

  (void)SPI_SEND(priv->dev, RAMTRON_WREN);
  
  /* Deselect the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH, false);
  fvdbg("Enabled\n");
}

/************************************************************************************
 * Name:  ramtron_sendaddr
 ************************************************************************************/

static inline void ramtron_sendaddr(const struct ramtron_dev_s *priv, uint32_t addr)
{
  DEBUGASSERT(priv->part->addr_len == 3 || priv->part->addr_len == 2);
  
  if (priv->part->addr_len == 3)
	(void)SPI_SEND(priv->dev, (addr >> 16) & 0xff);
	
  (void)SPI_SEND(priv->dev, (addr >> 8) & 0xff);
  (void)SPI_SEND(priv->dev, addr & 0xff);
}

/************************************************************************************
 * Name:  ramtron_pagewrite
 ************************************************************************************/

static inline void ramtron_pagewrite(struct ramtron_dev_s *priv, FAR const uint8_t *buffer,
                                  off_t page)
{
  off_t offset = page << priv->pageshift;

  fvdbg("page: %08lx offset: %08lx\n", (long)page, (long)offset);

  /* Wait for any preceding write to complete.  We could simplify things by
   * perform this wait at the end of each write operation (rather than at
   * the beginning of ALL operations), but have the wait first will slightly
   * improve performance.
   */

  ramtron_waitwritecomplete(priv);

  /* Enable the write access to the FLASH */

  ramtron_writeenable(priv);
  
  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH, true);

  /* Send "Page Program (PP)" command */

  (void)SPI_SEND(priv->dev, RAMTRON_WRITE);

  /* Send the page offset high byte first. */

  ramtron_sendaddr(priv, offset);

  /* Then write the specified number of bytes */

  SPI_SNDBLOCK(priv->dev, buffer, 1 << priv->pageshift);
  
  /* Deselect the FLASH: Chip Select high */

  SPI_SELECT(priv->dev, SPIDEV_FLASH, false);
  fvdbg("Written\n");
}

/************************************************************************************
 * Name: ramtron_erase
 ************************************************************************************/

static int ramtron_erase(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks)
{
  fvdbg("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);
  fvdbg("On RAMTRON devices erasing makes no sense, returning as OK\n");
  return (int)nblocks;
}

/************************************************************************************
 * Name: ramtron_bread
 ************************************************************************************/

static ssize_t ramtron_bread(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks,
                          FAR uint8_t *buffer)
{
  FAR struct ramtron_dev_s *priv = (FAR struct ramtron_dev_s *)dev;
  ssize_t nbytes;

  fvdbg("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* On this device, we can handle the block read just like the byte-oriented read */

  nbytes = ramtron_read(dev, startblock << priv->pageshift, nblocks << priv->pageshift, buffer);
  if (nbytes > 0)
    {
        return nbytes >> priv->pageshift;
    }
  return (int)nbytes;
}

/************************************************************************************
 * Name: ramtron_bwrite
 ************************************************************************************/

static ssize_t ramtron_bwrite(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks,
                           FAR const uint8_t *buffer)
{
  FAR struct ramtron_dev_s *priv = (FAR struct ramtron_dev_s *)dev;
  size_t blocksleft = nblocks;

  fvdbg("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* Lock the SPI bus and write each page to FLASH */

  ramtron_lock(priv->dev);
  while (blocksleft-- > 0)
    {
      ramtron_pagewrite(priv, buffer, startblock);
      startblock++;
   }
  ramtron_unlock(priv->dev);

  return nblocks;
}

/************************************************************************************
 * Name: ramtron_read
 ************************************************************************************/

static ssize_t ramtron_read(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
                         FAR uint8_t *buffer)
{
  FAR struct ramtron_dev_s *priv = (FAR struct ramtron_dev_s *)dev;

  fvdbg("offset: %08lx nbytes: %d\n", (long)offset, (int)nbytes);

  /* Wait for any preceding write to complete.  We could simplify things by
   * perform this wait at the end of each write operation (rather than at
   * the beginning of ALL operations), but have the wait first will slightly
   * improve performance.
   */

  ramtron_waitwritecomplete(priv);

  /* Lock the SPI bus and select this FLASH part */

  ramtron_lock(priv->dev);
  SPI_SELECT(priv->dev, SPIDEV_FLASH, true);

  /* Send "Read from Memory " instruction */

  (void)SPI_SEND(priv->dev, RAMTRON_READ);

  /* Send the page offset high byte first. */

  ramtron_sendaddr(priv, offset);

  /* Then read all of the requested bytes */

  SPI_RECVBLOCK(priv->dev, buffer, nbytes);

  /* Deselect the FLASH and unlock the SPI bus */

  SPI_SELECT(priv->dev, SPIDEV_FLASH, false);
  ramtron_unlock(priv->dev);
  fvdbg("return nbytes: %d\n", (int)nbytes);
  return nbytes;
}

/************************************************************************************
 * Name: ramtron_ioctl
 ************************************************************************************/

static int ramtron_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct ramtron_dev_s *priv = (FAR struct ramtron_dev_s *)dev;
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
        fvdbg("BULDERASE: Makes no sense in ramtron. Let's confirm operation as OK\n");
        ret = OK;
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
 * Name: ramtron_initialize
 *
 * Description:
 *   Create an initialize MTD device instance.  MTD devices are not registered
 *   in the file system, but are created as instances that can be bound to
 *   other functions (such as a block or character driver front end).
 *
 ************************************************************************************/

FAR struct mtd_dev_s *ramtron_initialize(FAR struct spi_dev_s *dev)
{
  FAR struct ramtron_dev_s *priv;

  fvdbg("dev: %p\n", dev);

  /* Allocate a state structure (we allocate the structure instead of using
   * a fixed, static allocation so that we can handle multiple FLASH devices.
   * The current implementation would handle only one FLASH part per SPI
   * device (only because of the SPIDEV_FLASH definition) and so would have
   * to be extended to handle multiple FLASH parts on the same SPI bus.
   */

  priv = (FAR struct ramtron_dev_s *)kmalloc(sizeof(struct ramtron_dev_s));
  if (priv)
    {
      /* Initialize the allocated structure */

      priv->mtd.erase  = ramtron_erase;
      priv->mtd.bread  = ramtron_bread;
      priv->mtd.bwrite = ramtron_bwrite;
      priv->mtd.read   = ramtron_read;
      priv->mtd.ioctl  = ramtron_ioctl;
      priv->dev        = dev;

      /* Deselect the FLASH */

      SPI_SELECT(dev, SPIDEV_FLASH, false);

      /* Identify the FLASH chip and get its capacity */

      if (ramtron_readid(priv) != OK)
        {
          /* Unrecognized! Discard all of that work we just did and return NULL */
          kfree(priv);
          priv = NULL;
        }
    }

  /* Return the implementation-specific state structure as the MTD device */

  fvdbg("Return %p\n", priv);
  return (FAR struct mtd_dev_s *)priv;
}
