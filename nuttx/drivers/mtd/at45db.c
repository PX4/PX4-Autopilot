/************************************************************************************
 * drivers/mtd/at45db.c
 * Driver for SPI-based AT45DB161D (16Mbit)
 *
 *   Copyright (C) 2010-2011 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

/* Ordering Code Detail:
 *
 * AT 45DB 16 1 D – SS U
 * |  |    |  | |   |  `- Device grade
 * |  |    |  | |   `- Package Option
 * |  |    |  | `- Device revision
 * |  |    |  `- Interface: 1=serial
 * |  |    `- Capacity: 16=16Mbit
 * |  `- Product family
 * `- Atmel designator
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

#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi.h>
#include <nuttx/mtd.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Configuration ********************************************************************/
/* CONFIG_AT45DB_PREWAIT enables higher performance write logic:  We leave the chip
 * busy after write and erase operations.  This improves write and erase performance
 * because we do not have to wait as long between transactions (other processing can
 * occur while the chip is busy) but means that the chip must stay powered:
 */

#if defined(CONFIG_AT45DB_PWRSAVE) && defined(CONFIG_AT45DB_PREWAIT)
#  error "Both CONFIG_AT45DB_PWRSAVE and CONFIG_AT45DB_PREWAIT are defined"
#endif

/* If the user has provided no frequency, use 1MHz */

#ifndef CONFIG_AT45DB_FREQUENCY
#  define CONFIG_AT45DB_FREQUENCY 1000000
#endif

/* SPI Commands *********************************************************************/

/* Read commands */

#define AT45DB_RDMN          0xd2 /* Main Memory Page Read */
#define AT45DB_RDARRY        0xe8 /* Continuous Array Read (Legacy Command) */
#define AT45DB_RDARRAYLF     0x03 /* Continuous Array Read (Low Frequency) */
#define AT45DB_RDARRAYHF     0x0b /* Continuous Array Read (High Frequency) */
#define AT45DB_RDBF1LF       0xd1 /* Buffer 1 Read (Low Frequency) */
#define AT45DB_RDBF2LF       0xd3 /* Buffer 2 Read (Low Frequency) */
#define AT45DB_RDBF1         0xd4 /* Buffer 1 Read */
#define AT45DB_RDBF2         0xd6 /* Buffer 2 Read */

/* Program and Erase Commands */

#define AT45DB_WRBF1         0x84 /* Buffer 1 Write */
#define AT45DB_WRBF2         0x87 /* Buffer 2 Write */
#define AT45DB_BF1TOMNE      0x83 /* Buffer 1 to Main Memory Page Program with Built-in Erase */
#define AT45DB_BF2TOMNE      0x86 /* Buffer 2 to Main Memory Page Program with Built-in Erase */
#define AT45DB_BF1TOMN       0x88 /* Buffer 1 to Main Memory Page Program without Built-in Erase */
#define AT45DB_BF2TOMN       0x89 /* Buffer 2 to Main Memory Page Program without Built-in Erase  */
#define AT45DB_PGERASE       0x81 /* Page Erase */
#define AT45DB_BLKERASE      0x50 /* Block Erase */
#define AT45DB_SECTERASE     0x7c /* Sector Erase */
#define AT45DB_CHIPERASE1    0xc7 /* Chip Erase - byte 1 */
#  define AT45DB_CHIPERASE2  0x94 /* Chip Erase - byte 2 */
#  define AT45DB_CHIPERASE3  0x80 /* Chip Erase - byte 3 */
#  define AT45DB_CHIPERASE4  0x9a /* Chip Erase - byte 4 */
#define AT45DB_MNTHRUBF1     0x82 /* Main Memory Page Program Through Buffer 1 */
#define AT45DB_MNTHRUBF2     0x85 /* Main Memory Page Program Through Buffer 2 */

/* Protection and Security Commands */

#define AT45DB_ENABPROT1     0x3d /* Enable Sector Protection - byte 1 */
#  define AT45DB_ENABPROT2   0x2a /* Enable Sector Protection - byte 2 */
#  define AT45DB_ENABPROT3   0x7f /* Enable Sector Protection - byte 3 */
#  define AT45DB_ENABPROT4   0xa9 /* Enable Sector Protection - byte 4 */
#define AT45DB_DISABPROT1    0x3d /* Disable Sector Protection - byte 1 */
#  define AT45DB_DISABPROT2  0x2a /* Disable Sector Protection - byte 2 */
#  define AT45DB_DISABPROT3  0x7f /* Disable Sector Protection - byte 3 */
#  define AT45DB_DISABPROT4  0x9a /* Disable Sector Protection - byte 4 */
#define AT45DB_ERASEPROT1    0x3d /* Erase Sector Protection Register - byte 1 */
#  define AT45DB_ERASEPROT2  0x2a /* Erase Sector Protection Register - byte 2 */
#  define AT45DB_ERASEPROT3  0x7f /* Erase Sector Protection Register - byte 3 */
#  define AT45DB_ERASEPROT4  0xcf /* Erase Sector Protection Register - byte 4 */
#define AT45DB_PROGPROT1     0x3d /* Program Sector Protection Register - byte 1 */
#  define AT45DB_PROGPROT2   0x2a /* Program Sector Protection Register - byte 2 */
#  define AT45DB_PROGPROT3   0x7f /* Program Sector Protection Register - byte 3 */
#  define AT45DB_PROGPROT4   0xfc /* Program Sector Protection Register - byte 4 */
#define AT45DB_RDPROT        0x32 /* Read Sector Protection Register */
#define AT45DB_LOCKDOWN1     0x3d /* Sector Lockdown - byte 1 */
#  define AT45DB_LOCKDOWN2   0x2a /* Sector Lockdown - byte 2 */
#  define AT45DB_LOCKDOWN3   0x7f /* Sector Lockdown - byte 3 */
#  define AT45DB_LOCKDOWN4   0x30 /* Sector Lockdown - byte 4 */
#define AT45DB_RDLOCKDOWN    0x35 /* Read Sector Lockdown Register  */
#define AT45DB_PROGSEC1      0x9b /* Program Security Register - byte 1 */
#  define AT45DB_PROGSEC2    0x00 /* Program Security Register - byte 2 */
#  define AT45DB_PROGSEC3    0x00 /* Program Security Register - byte 3 */
#  define AT45DB_PROGSEC4    0x00 /* Program Security Register - byte 4 */
#define AT45DB_RDSEC         0x77 /* Read Security Register */

/* Additional commands */

#define AT45DB_MNTOBF1XFR    0x53 /* Main Memory Page to Buffer 1 Transfer */
#define AT45DB_MNTOBF2XFR    0x55 /* Main Memory Page to Buffer 2 Transfer */
#define AT45DB_MNBF1CMP      0x60 /* Main Memory Page to Buffer 1 Compare  */
#define AT45DB_MNBF2CMP      0x61 /* Main Memory Page to Buffer 2 Compare */
#define AT45DB_AUTOWRBF1     0x58 /* Auto Page Rewrite through Buffer 1 */
#define AT45DB_AUTOWRBF2     0x59 /* Auto Page Rewrite through Buffer 2 */
#define AT45DB_PWRDOWN       0xb9 /* Deep Power-down */
#define AT45DB_RESUME        0xab /* Resume from Deep Power-down */
#define AT45DB_RDSR          0xd7 /* Status Register Read */
#define AT45DB_RDDEVID       0x9f /* Manufacturer and Device ID Read */

#define AT45DB_MANUFACTURER  0x1f /* Manufacturer ID: Atmel */
#define AT45DB_DEVID1_CAPMSK 0x1f /* Bits 0-4: Capacity */
#define AT45DB_DEVID1_1MBIT  0x02 /* xxx0 0010 = 1Mbit AT45DB011 */
#define AT45DB_DEVID1_2MBIT  0x03 /* xxx0 0012 = 2Mbit AT45DB021 */
#define AT45DB_DEVID1_4MBIT  0x04 /* xxx0 0100 = 4Mbit AT45DB041 */
#define AT45DB_DEVID1_8MBIT  0x05 /* xxx0 0101 = 8Mbit AT45DB081 */
#define AT45DB_DEVID1_16MBIT 0x06 /* xxx0 0110 = 16Mbit AT45DB161 */
#define AT45DB_DEVID1_32MBIT 0x07 /* xxx0 0111 = 32Mbit AT45DB321 */
#define AT45DB_DEVID1_64MBIT 0x08 /* xxx0 1000 = 32Mbit AT45DB641 */
#define AT45DB_DEVID1_FAMMSK 0xe0 /* Bits 5-7: Family */
#define AT45DB_DEVID1_DFLASH 0x20 /* 001x xxxx = Dataflash */
#define AT45DB_DEVID1_AT26DF 0x40 /* 010x xxxx = AT26DFxxx series (Not supported) */
#define AT45DB_DEVID2_VERMSK 0x1f /* Bits 0-4: MLC mask */
#define AT45DB_DEVID2_MLCMSK 0xe0 /* Bits 5-7: MLC mask */

/* Status register bit definitions */

#define AT45DB_SR_RDY       (1 << 7) /* Bit 7: RDY/ Not BUSY */
#define AT45DB_SR_COMP      (1 << 6) /* Bit 6: COMP */
#define AT45DB_SR_PROTECT   (1 << 1) /* Bit 1: PROTECT */
#define AT45DB_SR_PGSIZE    (1 << 0) /* Bit 0: PAGE_SIZE */

/* 1 Block = 16 pages; 1 sector = 256 pages */

#define PG_PER_BLOCK        (16)
#define PG_PER_SECTOR       (256)

/************************************************************************************
 * Private Types
 ************************************************************************************/

/* This type represents the state of the MTD device.  The struct mtd_dev_s
 * must appear at the beginning of the definition so that you can freely
 * cast between pointers to struct mtd_dev_s and struct at45db_dev_s.
 */

struct at45db_dev_s
{
  struct mtd_dev_s mtd;      /* MTD interface */
  FAR struct spi_dev_s *spi; /* Saved SPI interface instance */
  uint8_t  pageshift;        /* log2 of the page size (eg. 1 << 9 = 512) */
  uint32_t npages;           /* Number of pages in the device */
};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

/* Lock and per-transaction configuration */

static void at45db_lock(struct at45db_dev_s *priv);
static inline void at45db_unlock(struct at45db_dev_s *priv);

/* Power management */

#ifdef CONFIG_AT45DB_PWRSAVE
static void at45db_pwrdown(struct at45db_dev_s *priv);
static void at45db_resume(struct at45db_dev_s *priv);
#else
#  define  at45db_pwrdown(priv)
#  define  at45db_resume(priv)
#endif

/* Low-level AT45DB Helpers */

static inline int at45db_rdid(struct at45db_dev_s *priv);
static inline uint8_t at45db_rdsr(struct at45db_dev_s *priv);
static uint8_t at45db_waitbusy(struct at45db_dev_s *priv);
static inline void at45db_pgerase(struct at45db_dev_s *priv, off_t offset);
static inline int  at32db_chiperase(struct at45db_dev_s *priv);
static inline void at45db_pgwrite(struct at45db_dev_s *priv, FAR const uint8_t *buffer,
                                  off_t offset);

/* MTD driver methods */

static int at45db_erase(FAR struct mtd_dev_s *mtd, off_t startblock, size_t nblocks);
static ssize_t at45db_bread(FAR struct mtd_dev_s *mtd, off_t startblock,
                          size_t nblocks, FAR uint8_t *buf);
static ssize_t at45db_bwrite(FAR struct mtd_dev_s *mtd, off_t startblock,
                           size_t nblocks, FAR const uint8_t *buf);
static ssize_t at45db_read(FAR struct mtd_dev_s *mtd, off_t offset, size_t nbytes,
                         FAR uint8_t *buffer);
static int at45db_ioctl(FAR struct mtd_dev_s *mtd, int cmd, unsigned long arg);

/************************************************************************************
 * Private Data
 ************************************************************************************/

/* Chip erase sequence */

#define CHIP_ERASE_SIZE 4
static const uint8_t g_chiperase[CHIP_ERASE_SIZE] = {0xc7, 0x94, 0x80, 0x9a};

/* Sequence to program the device to binary page sizes{256, 512, 1024} */

#define BINPGSIZE_SIZE 4
static const uint8_t g_binpgsize[BINPGSIZE_SIZE] = {0x3d, 0x2a, 0x80, 0xa6};

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: at45db_lock
 ************************************************************************************/

static void at45db_lock(struct at45db_dev_s *priv)
{
  /* On SPI busses where there are multiple devices, it will be necessary to
   * lock SPI to have exclusive access to the busses for a sequence of
   * transfers.  The bus should be locked before the chip is selected.
   *
   * This is a blocking call and will not return until we have exclusiv access to
   * the SPI buss.  We will retain that exclusive access until the bus is unlocked.
   */

  SPI_LOCK(priv->spi, true);

  /* After locking the SPI bus, the we also need call the setfrequency, setbits, and
   * setmode methods to make sure that the SPI is properly configured for the device.
   * If the SPI buss is being shared, then it may have been left in an incompatible
   * state.
   */

  SPI_SETMODE(priv->spi, SPIDEV_MODE0);
  SPI_SETBITS(priv->spi, 8);
  (void)SPI_SETFREQUENCY(priv->spi, CONFIG_AT45DB_FREQUENCY);
}

/************************************************************************************
 * Name: at45db_unlock
 ************************************************************************************/

static inline void at45db_unlock(struct at45db_dev_s *priv)
{
  SPI_LOCK(priv->spi, false);
}

/************************************************************************************
 * Name: at45db_pwrdown
 ************************************************************************************/

#ifdef CONFIG_AT45DB_PWRSAVE
static void at45db_pwrdown(struct at45db_dev_s *priv)
{
  SPI_SELECT(priv->spi, SPIDEV_FLASH, true);
  SPI_SEND(priv->spi, AT45DB_PWRDOWN);
  SPI_SELECT(priv->spi, SPIDEV_FLASH, false);
}
#endif

/************************************************************************************
 * Name: at45db_resume
 ************************************************************************************/

#ifdef CONFIG_AT45DB_PWRSAVE
static void at45db_resume(struct at45db_dev_s *priv)
{
  SPI_SELECT(priv->spi, SPIDEV_FLASH, true);
  SPI_SEND(priv->spi, AT45DB_RESUME);
  SPI_SELECT(priv->spi, SPIDEV_FLASH, false);
  up_udelay(50);
}
#endif

/************************************************************************************
 * Name: at45db_rdid
 ************************************************************************************/

static inline int at45db_rdid(struct at45db_dev_s *priv)
{
  uint8_t capacity;
  uint8_t devid[3];

  fvdbg("priv: %p\n", priv);

  /* Configure the bus, and select this FLASH part. (The caller should alread have
   * loced the bus for exclusive access)
   */

  SPI_SELECT(priv->spi, SPIDEV_FLASH, true);

  /* Send the " Manufacturer and Device ID Read" command and read the next three
   * ID bytes from the FLASH.
   */

  (void)SPI_SEND(priv->spi, AT45DB_RDDEVID);
  SPI_RECVBLOCK(priv->spi, devid, 3);

  /* Deselect the FLASH */

  SPI_SELECT(priv->spi, SPIDEV_FLASH, false);

  fvdbg("manufacturer: %02x devid1: %02x devid2: %02x\n",
        devid[0], devid[1], devid[2]);
 
  /* Check for a valid manufacturer and memory family */

  if (devid[0] == AT45DB_MANUFACTURER &&
     (devid[1] & AT45DB_DEVID1_FAMMSK) == AT45DB_DEVID1_DFLASH)
    {
      /* Okay.. is it a FLASH capacity that we understand? */

      capacity = devid[1] & AT45DB_DEVID1_CAPMSK;
      switch (capacity)
        {
        case AT45DB_DEVID1_1MBIT:
          /* Save the FLASH geometry for the 16Mbit AT45DB011 */

          priv->pageshift   = 8;    /* Page size = 256 bytes */
          priv->npages      = 512;  /* 512 pages */
          return OK;

        case AT45DB_DEVID1_2MBIT:
          /* Save the FLASH geometry for the 16Mbit AT45DB021 */

          priv->pageshift   = 8;    /* Page size = 256/264 bytes */
          priv->npages      = 1024; /* 1024 pages */
          return OK;

        case AT45DB_DEVID1_4MBIT:
          /* Save the FLASH geometry for the 16Mbit AT45DB041 */

          priv->pageshift   = 8;    /* Page size = 256/264 bytes */
          priv->npages      = 2048; /* 2048 pages */
          return OK;

        case AT45DB_DEVID1_8MBIT:
          /* Save the FLASH geometry for the 16Mbit AT45DB081 */

          priv->pageshift   = 8;    /* Page size = 256/264 bytes */
          priv->npages      = 4096; /* 4096 pages */
          return OK;

        case AT45DB_DEVID1_16MBIT:
          /* Save the FLASH geometry for the 16Mbit AT45DB161 */

          priv->pageshift   = 9;    /* Page size = 512/528 bytes */
          priv->npages      = 4096; /* 4096 pages */
          return OK;

        case AT45DB_DEVID1_32MBIT:
          /* Save the FLASH geometry for the 16Mbit AT45DB321 */

          priv->pageshift   = 9;    /* Page size = 512/528 bytes */
          priv->npages      = 8192; /* 8192 pages */
          return OK;

        case AT45DB_DEVID1_64MBIT:
          /* Save the FLASH geometry for the 16Mbit AT45DB321 */

          priv->pageshift   = 10;   /* Page size = 1024/1056 bytes */
          priv->npages      = 8192; /* 8192 pages */
          return OK;

        default:
          return -ENODEV;
        }
    }

  return -ENODEV;
}

/************************************************************************************
 * Name: at45db_rdsr
 ************************************************************************************/

static inline uint8_t at45db_rdsr(struct at45db_dev_s *priv)
{
  uint8_t retval;

  SPI_SELECT(priv->spi, SPIDEV_FLASH, true);
  SPI_SEND(priv->spi, AT45DB_RDSR);
  retval = SPI_SEND(priv->spi, 0xff);
  SPI_SELECT(priv->spi, SPIDEV_FLASH, false);
  return retval;
}

/************************************************************************************
 * Name: at45db_waitbusy
 ************************************************************************************/

static uint8_t at45db_waitbusy(struct at45db_dev_s *priv)
{
  uint8_t sr;

  /* Poll the device, waiting for it to report that it is ready */

  do
  {
    up_udelay(10);
    sr = (uint8_t)at45db_rdsr(priv);
  }
  while ((sr & AT45DB_SR_RDY) == 0);
  return sr;
}

/************************************************************************************
 * Name:  at45db_pgerase
 ************************************************************************************/

static inline void at45db_pgerase(struct at45db_dev_s *priv, off_t sector)
{
  uint8_t erasecmd[4];
  off_t offset = sector << priv->pageshift;

  fvdbg("sector: %08lx\n", (long)sector);

  /* Higher performance write logic:  We leave the chip busy after write and erase
   * operations.  This improves write and erase performance because we do not have
   * to wait as long between transactions (other processing can occur while the chip
   * is busy) but means that the chip must stay powered and that we must check if
   * the chip is still busy on each entry point.
   */

#ifdef CONFIG_AT45DB_PREWAIT
  at45db_waitbusy(priv);
#endif

  /* "The Page Erase command can be used to individually erase any page in the main
   *  memory array allowing the Buffer to Main Memory Page Program to be utilized at a
   *  later time. ... To perform a page erase in the binary page size ..., the
   *  opcode 81H must be loaded into the device, followed by three address bytes
   *  ... When a low-to-high transition occurs on the CS pin, the part will erase the
   *  selected page (the erased state is a logical 1). ... the status register and the
   *  RDY/BUSY pin will indicate that the part is busy."
   */

  erasecmd[0] = AT45DB_PGERASE;   /* Page erase command */
  erasecmd[1] = (offset >> 16) & 0xff; /* 24-bit offset MS bytes */
  erasecmd[2] = (offset >>  8) & 0xff; /* 24-bit offset middle bytes */
  erasecmd[3] =  offset        & 0xff; /* 24-bit offset LS bytes */

  /* Erase the page */

  SPI_SELECT(priv->spi, SPIDEV_FLASH, true);
  SPI_SNDBLOCK(priv->spi, erasecmd, 4);
  SPI_SELECT(priv->spi, SPIDEV_FLASH, false);

  /* Wait for any erase to complete if we are not trying to improve write
   * performance. (see comments above).
   */

#ifndef CONFIG_AT45DB_PREWAIT
  at45db_waitbusy(priv);
#endif
  fvdbg("Erased\n");
}

/************************************************************************************
 * Name:  at32db_chiperase
 ************************************************************************************/

static inline int at32db_chiperase(struct at45db_dev_s *priv)
{
  fvdbg("priv: %p\n", priv);

  /* Higher performance write logic:  We leave the chip busy after write and erase
   * operations.  This improves write and erase performance because we do not have
   * to wait as long between transactions (other processing can occur while the chip
   * is busy) but means that the chip must stay powered and that we must check if
   * the chip is still busy on each entry point.
   */

#ifdef CONFIG_AT45DB_PREWAIT
  at45db_waitbusy(priv);
#endif

  /* "The entire main memory can be erased at one time by using the Chip Erase
   * command. To execute the Chip Erase command, a 4-byte command sequence C7H, 94H,
   * 80H and 9AH must be clocked into the device. ... After the last bit of the opcode
   * sequence has been clocked in, the CS pin can be deasserted to start the erase
   * process. ... the Status Register will indicate that the device is busy. The Chip
   * Erase command will not affect sectors that are protected or locked down...
   */

  SPI_SELECT(priv->spi, SPIDEV_FLASH, true);
  SPI_SNDBLOCK(priv->spi, g_chiperase, CHIP_ERASE_SIZE);
  SPI_SELECT(priv->spi, SPIDEV_FLASH, false);

  /* Wait for any erase to complete if we are not trying to improve write
   * performance. (see comments above).
   */

#ifndef CONFIG_AT45DB_PREWAIT
  at45db_waitbusy(priv);
#endif
  return OK;
}

/************************************************************************************
 * Name:  at45db_pgwrite
 ************************************************************************************/

static inline void at45db_pgwrite(struct at45db_dev_s *priv, FAR const uint8_t *buffer,
                                  off_t page)
{
  uint8_t wrcmd [4];
  off_t offset = page << priv->pageshift;

  fvdbg("page: %08lx offset: %08lx\n", (long)page, (long)offset);

  /* We assume that sectors are not write protected */

  wrcmd[0] = AT45DB_MNTHRUBF1;      /* To main memory through buffer 1 */
  wrcmd[1] = (offset >> 16) & 0xff; /* 24-bit address MS byte */
  wrcmd[2] = (offset >>  8) & 0xff; /* 24-bit address middle byte */
  wrcmd[3] =  offset        & 0xff; /* 24-bit address LS byte */

  /* Higher performance write logic:  We leave the chip busy after write and erase
   * operations.  This improves write and erase performance because we do not have
   * to wait as long between transactions (other processing can occur while the chip
   * is busy) but means that the chip must stay powered and that we must check if
   * the chip is still busy on each entry point.
   */

#ifdef CONFIG_AT45DB_PREWAIT
  at45db_waitbusy(priv);
#endif

  SPI_SELECT(priv->spi, SPIDEV_FLASH, true);
  SPI_SNDBLOCK(priv->spi, wrcmd, 4);
  SPI_SNDBLOCK(priv->spi, buffer, 1 << priv->pageshift);
  SPI_SELECT(priv->spi, SPIDEV_FLASH, false);

  /* Wait for any erase to complete if we are not trying to improve write
   * performance. (see comments above).
   */

#ifndef CONFIG_AT45DB_PREWAIT
  at45db_waitbusy(priv);
#endif
  fvdbg("Written\n");
}

/************************************************************************************
 * Name: at45db_erase
 ************************************************************************************/

static int at45db_erase(FAR struct mtd_dev_s *mtd, off_t startblock, size_t nblocks)
{
  FAR struct at45db_dev_s *priv = (FAR struct at45db_dev_s *)mtd;
  size_t pgsleft = nblocks;

  fvdbg("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* Take the lock so that we have exclusive access to the bus, then power up the
   * FLASH device.
   */
 
  at45db_lock(priv);
  at45db_resume(priv);

  /* Then erase each page */

  while (pgsleft-- > 0)
    {
      /* Erase each sector */

      at45db_pgerase(priv, startblock);
      startblock++;
    }

  at45db_pwrdown(priv);
  at45db_unlock(priv);
  return (int)nblocks;
}

/************************************************************************************
 * Name: at45db_bread
 ************************************************************************************/

static ssize_t at45db_bread(FAR struct mtd_dev_s *mtd, off_t startblock, size_t nblocks,
                            FAR uint8_t *buffer)
{
  FAR struct at45db_dev_s *priv = (FAR struct at45db_dev_s *)mtd;
  ssize_t nbytes;

 /* On this device, we can handle the block read just like the byte-oriented read */

  nbytes = at45db_read(mtd, startblock << priv->pageshift, nblocks << priv->pageshift, buffer);
  if (nbytes > 0)
    {
      return nbytes >> priv->pageshift;
    }
  return nbytes;
}

/************************************************************************************
 * Name: at45db_bwrite
 ************************************************************************************/

static ssize_t at45db_bwrite(FAR struct mtd_dev_s *mtd, off_t startblock, size_t nblocks,
                           FAR const uint8_t *buffer)
{
  FAR struct at45db_dev_s *priv = (FAR struct at45db_dev_s *)mtd;
  size_t pgsleft = nblocks;

  fvdbg("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* Take the lock so that we have exclusive access to the bus, then power up the
   * FLASH device.
   */
 
  at45db_lock(priv);
  at45db_resume(priv);

  /* Write each page to FLASH */

  while (pgsleft-- > 0)
    {
      at45db_pgwrite(priv, buffer, startblock);
      startblock++;
   }

  at45db_pwrdown(priv);
  at45db_unlock(priv);

  return nblocks;
}

/************************************************************************************
 * Name: at45db_read
 ************************************************************************************/

static ssize_t at45db_read(FAR struct mtd_dev_s *mtd, off_t offset, size_t nbytes,
                         FAR uint8_t *buffer)
{
  FAR struct at45db_dev_s *priv = (FAR struct at45db_dev_s *)mtd;
  uint8_t rdcmd [5];

  fvdbg("offset: %08lx nbytes: %d\n", (long)offset, (int)nbytes);

  /* Set up for the read */

  rdcmd[0] = AT45DB_RDARRAYHF;       /* FAST_READ is safe at all supported SPI speeds. */
  rdcmd[1] = (offset >> 16) & 0xff;  /* 24-bit address upper byte */
  rdcmd[2] = (offset >>  8) & 0xff;  /* 24-bit address middle byte */
  rdcmd[3] =  offset        & 0xff;  /* 24-bit address least significant byte */
  rdcmd[4] = 0;                      /* Dummy byte */

  /* Take the lock so that we have exclusive access to the bus, then power up the
   * FLASH device.
   */
 
  at45db_lock(priv);
  at45db_resume(priv);
 
  /* Higher performance write logic:  We leave the chip busy after write and erase
   * operations.  This improves write and erase performance because we do not have
   * to wait as long between transactions (other processing can occur while the chip
   * is busy) but means that the chip must stay powered and that we must check if
   * the chip is still busy on each entry point.
   */

#ifdef CONFIG_AT45DB_PREWAIT
  at45db_waitbusy(priv);
#endif

  /* Perform the read */

  SPI_SELECT(priv->spi, SPIDEV_FLASH, true);
  SPI_SNDBLOCK(priv->spi, rdcmd, 5);
  SPI_RECVBLOCK(priv->spi, buffer, nbytes);
  SPI_SELECT(priv->spi, SPIDEV_FLASH, false);

  at45db_pwrdown(priv);
  at45db_unlock(priv);

  fvdbg("return nbytes: %d\n", (int)nbytes);
  return nbytes;
}

/************************************************************************************
 * Name: at45db_ioctl
 ************************************************************************************/

static int at45db_ioctl(FAR struct mtd_dev_s *mtd, int cmd, unsigned long arg)
{
  FAR struct at45db_dev_s *priv = (FAR struct at45db_dev_s *)mtd;
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
              geo->erasesize    = geo->blocksize;
              geo->neraseblocks = priv->npages;
              ret               = OK;

              fvdbg("blocksize: %d erasesize: %d neraseblocks: %d\n",
                    geo->blocksize, geo->erasesize, geo->neraseblocks);
            }
        }
        break;

      case MTDIOC_BULKERASE:
        {
             /* Take the lock so that we have exclusive access to the bus, then
              * power up the FLASH device.
              */
 
             at45db_lock(priv);
             at45db_resume(priv);

            /* Erase the entire device */

            ret = at32db_chiperase(priv);
            at45db_pwrdown(priv);
            at45db_unlock(priv);
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
 * Name: at45db_initialize
 *
 * Description:
 *   Create an initialize MTD device instance.  MTD devices are not registered
 *   in the file system, but are created as instances that can be bound to
 *   other functions (such as a block or character driver front end).
 *
 ************************************************************************************/

FAR struct mtd_dev_s *at45db_initialize(FAR struct spi_dev_s *spi)
{
  FAR struct at45db_dev_s *priv;
  uint8_t sr;
  int ret;

  fvdbg("spi: %p\n", spi);

  /* Allocate a state structure (we allocate the structure instead of using
   * a fixed, static allocation so that we can handle multiple FLASH devices.
   * The current implementation would handle only one FLASH part per SPI
   * device (only because of the SPIDEV_FLASH definition) and so would have
   * to be extended to handle multiple FLASH parts on the same SPI bus.
   */

  priv = (FAR struct at45db_dev_s *)kmalloc(sizeof(struct at45db_dev_s));
  if (priv)
    {
      /* Initialize the allocated structure */

      priv->mtd.erase  = at45db_erase;
      priv->mtd.bread  = at45db_bread;
      priv->mtd.bwrite = at45db_bwrite;
      priv->mtd.read   = at45db_read;
      priv->mtd.ioctl  = at45db_ioctl;
      priv->spi        = spi;

      /* Deselect the FLASH */

      SPI_SELECT(spi, SPIDEV_FLASH, false);

      /* Lock and configure the SPI bus. */

      at45db_lock(priv);
      at45db_resume(priv);

      /* Identify the FLASH chip and get its capacity */

      ret = at45db_rdid(priv);
      if (ret != OK)
        {
          /* Unrecognized! Discard all of that work we just did and return NULL */

          fdbg("Unrecognized\n");
          goto errout;
        }

      /* Get the value of the status register (as soon as the device is ready) */

      sr = at45db_waitbusy(priv);

      /* Check if the device is configured as 256, 512 or 1024 bytes-per-page device */

      if ((sr & AT45DB_SR_PGSIZE) == 0)
        {
          /* No, re-program it for the binary page size.  NOTE:  A power cycle
           * is required after the device has be re-programmed.
           */

          fdbg("Reprogramming page size\n");
          SPI_SELECT(priv->spi, SPIDEV_FLASH, true);
          SPI_SNDBLOCK(priv->spi, g_binpgsize, BINPGSIZE_SIZE);
          SPI_SELECT(priv->spi, SPIDEV_FLASH, false);
          goto errout;
        }

      /* Release the lock and power down the device */

      at45db_pwrdown(priv);
      at45db_unlock(priv);
    }  

  fvdbg("Return %p\n", priv);
  return (FAR struct mtd_dev_s *)priv;

/* On any failure, we need free memory allocations and release the lock that
 * we hold on the SPI bus.  On failures, assume that we cannot talk to the 
 * device to do any more.
 */

errout:
  at45db_unlock(priv);
  kfree(priv);
  return NULL;
}
