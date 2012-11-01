/****************************************************************************
 * drivers/mmcsd/mmcsd_spi.c
 *
 *   Copyright (C) 2008-2010, 2011-2012 Gregory Nutt. All rights reserved.
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
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/spi.h>
#include <nuttx/fs/fs.h>
#include <nuttx/mmcsd.h>

#include "mmcsd_spi.h"
#include "mmcsd_csd.h"
#include "mmcsd_internal.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_MMCSD_NSLOTS
#  ifdef CONFIG_CPP_HAVE_WARNING
#    warning "CONFIG_MMCSD_NSLOTS not defined"
#  endif
#  define CONFIG_MMCSD_NSLOTS 1
#endif

#define MMCSD_IDMODE_CLOCK           (400000)

#if defined(CONFIG_FS_WRITABLE) && !defined(CONFIG_MMCSD_READONLY)
#  define MMCSD_MODE 0666
#else
#  define MMCSD_MODE 0444
#endif

#ifndef CONFIG_MMCSD_SPICLOCK
#  define CONFIG_MMCSD_SPICLOCK 20000000
#endif

#ifndef CONFIG_MMCSD_SECTOR512
#  define CONFIG_MMCSD_SECTOR512          /* Force 512 byte sectors on all cards */
#endif

/* Slot struct info *********************************************************/
/* Slot status definitions */

#define MMCSD_SLOTSTATUS_NOTREADY    0x01 /* Card not initialized */
#define MMCSD_SLOTSTATUS_NODISK      0x02 /* No card in the slot */
#define MMCSD_SLOTSTATUS_WRPROTECT   0x04 /* Card is write protected */
#define MMCSD_SLOTSTATUS_MEDIACHGD   0x08 /* Media changed in slot */

/* Values in the MMC/SD command table ***************************************/
/* These define the value returned by the MMC/SD command */

#define MMCSD_CMDRESP_R1             0
#define MMCSD_CMDRESP_R1B            1
#define MMCSD_CMDRESP_R2             2
#define MMCSD_CMDRESP_R3             3
#define MMCSD_CMDRESP_R7             4

#ifdef CONFIG_MMCSD_SECTOR512
#  define SECTORSIZE(s)              (512)
#else
#  define SECTORSIZE(s)              ((s)->sectorsize)
#endif

/* Time delays in units of the system clock. CLK_TCK is the number of clock
 * ticks per second.
 */

#define MMCSD_DELAY_10MS             (CLK_TCK/100  + 1)
#define MMCSD_DELAY_50MS             (CLK_TCK/20   + 1)
#define MMCSD_DELAY_100MS            (CLK_TCK/10   + 1)
#define MMCSD_DELAY_250MS            (CLK_TCK/4    + 1)
#define MMCSD_DELAY_500MS            (CLK_TCK/2    + 1)
#define MMCSD_DELAY_1SEC             (CLK_TCK      + 1)
#define MMCSD_DELAY_10SEC            (10 * CLK_TCK + 1)

#define ELAPSED_TIME(t)              (clock_systimer()-(t))
#define START_TIME                   (clock_systimer())

/* SD read timeout: ~100msec, Write Time out ~250ms.  Units of clock ticks */

#define SD_READACCESS                MMCSD_DELAY_100MS
#define SD_WRITEACCESS               MMCSD_DELAY_250MS

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents the state of one card slot */

struct mmcsd_slot_s
{
  FAR struct spi_dev_s *spi; /* SPI port bound to this slot */
  sem_t  sem;            /* Assures mutually exclusive accesss to card and SPI */
  uint8_t  state;        /* State of the slot (see MMCSD_SLOTSTATUS_* definitions) */
  uint8_t  type;         /* Disk type */
  uint8_t  csd[16];      /* Copy of card CSD */
#ifndef CONFIG_MMCSD_SECTOR512
  uint16_t sectorsize;   /* Media block size (in bytes) */
#endif
  uint32_t nsectors;     /* Number of blocks on the media */
  uint32_t taccess;      /* Card access time */
  uint32_t twrite;       /* Card write time */
  uint32_t ocr;          /* Last 4 bytes of OCR (R3) */
  uint32_t r7;           /* Last 4 bytes of R7 */
};

struct mmcsd_cmdinfo_s
{
  uint8_t  cmd;
  uint8_t  resp;
  uint8_t  chksum;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Misc *********************************************************************/

static void   mmcsd_semtake(sem_t *sem);

/* Card SPI interface *******************************************************/

static int      mmcsd_waitready(FAR struct mmcsd_slot_s *slot);
static uint32_t mmcsd_sendcmd(FAR struct mmcsd_slot_s *slot,
                  const struct mmcsd_cmdinfo_s *cmd, uint32_t arg);
static void     mmcsd_setblklen(FAR struct mmcsd_slot_s *slot,
                  uint32_t length);
static uint32_t mmcsd_nsac(FAR struct mmcsd_slot_s *slot, uint8_t *csd,
                  uint32_t frequency);
static uint32_t mmcsd_taac(FAR struct mmcsd_slot_s *slot, uint8_t *csd);
static void     mmcsd_decodecsd(FAR struct mmcsd_slot_s *slot, uint8_t *csd);
static void     mmcsd_checkwrprotect(FAR struct mmcsd_slot_s *slot,
                  uint8_t *csd);
static int      mmcsd_getcardinfo(FAR struct mmcsd_slot_s *slot,
                  uint8_t *buffer, const struct mmcsd_cmdinfo_s *cmd);

#define mmcsd_getcsd(slot, csd) mmcsd_getcardinfo(slot, csd, &g_cmd9);
#define mmcsd_getcid(slot, cid) mmcsd_getcardinfo(slot, cid, &g_cmd10);

static int      mmcsd_recvblock(FAR struct mmcsd_slot_s *slot,
                 uint8_t *buffer, int nbytes);
#if defined(CONFIG_FS_WRITABLE) && !defined(CONFIG_MMCSD_READONLY)
static int      mmcsd_xmitblock(FAR struct mmcsd_slot_s *slot,
                 const uint8_t *buffer, int nbytes, uint8_t token);
#endif

/* Block driver interfaces **************************************************/

static int       mmcsd_open(FAR struct inode *inode);
static int       mmcsd_close(FAR struct inode *inode);
static ssize_t   mmcsd_read(FAR struct inode *inode, unsigned char *buffer,
                   size_t start_sector, unsigned int nsectors);
#if defined(CONFIG_FS_WRITABLE) && !defined(CONFIG_MMCSD_READONLY)
static ssize_t   mmcsd_write(FAR struct inode *inode,
                   const unsigned char *buffer, size_t start_sector,
                   unsigned int nsectors);
#endif
static int       mmcsd_geometry(FAR struct inode *inode,
                    struct geometry *geometry);

/* Initialization ***********************************************************/

static int      mmcsd_mediainitialize(FAR struct mmcsd_slot_s *slot);
static void     mmcsd_mediachanged(void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Driver state *************************************************************/

/* These are the lock driver methods supported by this file */

static const struct block_operations g_bops =
{
  mmcsd_open,     /* open     */
  mmcsd_close,    /* close    */
  mmcsd_read,     /* read     */
#if defined(CONFIG_FS_WRITABLE) && !defined(CONFIG_MMCSD_READONLY)
  mmcsd_write,    /* write    */
#else
  NULL,           /* write    */
#endif
  mmcsd_geometry, /* geometry */
  NULL            /* ioctl    */
};

/* A slot structure allocated for each configured slot */

static struct mmcsd_slot_s g_mmcsdslot[CONFIG_MMCSD_NSLOTS];

/* Timing *******************************************************************/

/* We will use the TRAN_SPEED from the CSD to determine the maximum SPI
 * clocking (TRAN_SPEED defines the maximum transfer rate per bit per data
 * line).
 *
 * The CSD TRAN_SPEED is provided as a 3 bit rate unit (RU) and a 4 bit time
 * value (TU). We need the transfer frequency which is:  RU*TU bits/sec
 *
 * g_transpeedru holds RU/10 and g_transpeedtu holds TU*10 so that the
 * correct value is returned in the product
 */

static const uint32_t g_transpeedru[8] =
{
     10000,   /*  0:  10 Kbit/sec / 10 */
    100000,   /*  1:  1 Mbit/sec / 10 */
   1000000,   /*  2:  10 Mbit/sec / 10 */
  10000000,   /*  3:  100 Mbit/sec / 10*/

  0, 0, 0, 0  /* 4-7: Reserved values */
};

static const uint32_t g_transpeedtu[16] =
{
   0, 10, 12, 13, /*  0-3:  Reserved, 1.0, 1.1, 1.2, 1.3 */
  15, 20, 25, 30, /*  4-7:  1.5, 2.0, 2.5, 3.0 */
  35, 40, 45, 50, /*  8-11: 3.5, 4.0, 4.5, 5.0 */
  55, 60, 70, 80, /* 12-15: 5.5, 6.0, 7.0, 8.0 */
};

/* The TAAC defines the asynchronous part of the data access time.  The
 * read access time the sum of the TAAC and the NSAC.  These define the
 * time from the end bit of the read command to start bit of the data block.
 *
 * The TAAC consists of a 3-bit time unit (TU) and a 4-bit time value (TV).
 * TAAC is in units of time; NSAC is in units of SPI clocks.
 * The access time we need is then given by:
 *
 *   taccess = TU*TV + NSAC/spifrequency
 *
 * g_taactu holds TU in units of nanoseconds and microseconds (you have to use
 * the index to distiguish).  g_taactv holds TV with 8-bits of fraction.
 */

#define MAX_USTUNDX 2
static const uint16_t g_taactu[8] =
{
  /* Units of nanoseconds */

      1, /* 0:   1 ns */
     10, /* 1:  10 ns */
    100, /* 2: 100 ns */

  /* Units of microseconds */

      1, /* 3:   1 us 1,000 ns */
     10, /* 4:  10 us 10,000 ns */
    100, /* 5: 100 us 100,000 ns */
   1000, /* 6:   1 ms 1,000,000 ns*/
  10000, /* 7:  10 ms 10,000,000 ns */
};

static const uint16_t g_taactv[] =
{
  0x000,  0x100, 0x133, 0x14d, /*  0-3:  Reserved, 1.0, 1.2, 1.3 */
  0x180,  0x200, 0x280, 0x300, /*  4-7:   1.5, 2.0, 2.5, 3.0 */
  0x380,  0x400, 0x480, 0x500, /*  8-11:  3.5, 4.0, 4.5, 5.0 */
  0x580,  0x600, 0x700, 0x800  /* 12-15:  5.5, 6.0, 7.0, 8.0 */
};

/* Commands *****************************************************************/

static const struct mmcsd_cmdinfo_s g_cmd0   = {CMD0,   MMCSD_CMDRESP_R1, 0x95};
static const struct mmcsd_cmdinfo_s g_cmd1   = {CMD1,   MMCSD_CMDRESP_R1, 0xff};
static const struct mmcsd_cmdinfo_s g_cmd8   = {CMD8,   MMCSD_CMDRESP_R7, 0x87};
static const struct mmcsd_cmdinfo_s g_cmd9   = {CMD9,   MMCSD_CMDRESP_R1, 0xff};
static const struct mmcsd_cmdinfo_s g_cmd10  = {CMD10,  MMCSD_CMDRESP_R1, 0xff};
static const struct mmcsd_cmdinfo_s g_cmd12  = {CMD12,  MMCSD_CMDRESP_R1, 0xff};
static const struct mmcsd_cmdinfo_s g_cmd16  = {CMD16,  MMCSD_CMDRESP_R1, 0xff};
static const struct mmcsd_cmdinfo_s g_cmd17  = {CMD17,  MMCSD_CMDRESP_R1, 0xff};
static const struct mmcsd_cmdinfo_s g_cmd18  = {CMD18,  MMCSD_CMDRESP_R1, 0xff};
static const struct mmcsd_cmdinfo_s g_cmd24  = {CMD24,  MMCSD_CMDRESP_R1, 0xff};
static const struct mmcsd_cmdinfo_s g_cmd25  = {CMD25,  MMCSD_CMDRESP_R1, 0xff};
static const struct mmcsd_cmdinfo_s g_cmd55  = {CMD55,  MMCSD_CMDRESP_R1, 0xff};
static const struct mmcsd_cmdinfo_s g_cmd58  = {CMD58,  MMCSD_CMDRESP_R3, 0xff};
static const struct mmcsd_cmdinfo_s g_acmd23 = {ACMD23, MMCSD_CMDRESP_R1, 0xff};
static const struct mmcsd_cmdinfo_s g_acmd41 = {ACMD41, MMCSD_CMDRESP_R1, 0xff};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mmcsd_semtake
 ****************************************************************************/

static void mmcsd_semtake(sem_t *sem)
{
  while (sem_wait(sem) != 0)
    {
      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

#define mmcsd_semgive(sem) sem_post(sem)

/****************************************************************************
 * Name: mmcsd_waitready
 *
 * Description:
 *   Wait until the card is no longer busy
 *
 * Assumptions:
 *   MMC/SD card already selected
 *
 ****************************************************************************/

static int mmcsd_waitready(FAR struct mmcsd_slot_s *slot)
{
  FAR struct spi_dev_s *spi = slot->spi;
  uint8_t response;
  uint32_t start;
  uint32_t elapsed;

  /* Wait until the card is no longer busy (up to 500MS) */

  start = START_TIME;
  do
    {
      response = SPI_SEND(spi, 0xff);
      if (response == 0xff)
        {
          return OK;
        }
      elapsed = ELAPSED_TIME(start);
    }
  while (elapsed < MMCSD_DELAY_500MS);

  fdbg("Card still busy, last response: %02x\n", response);
  return -EBUSY;
}

/****************************************************************************
 * Name: mmcsd_sendcmd
 *
 * Description:
 *   Send a command to MMC
 *
 * Assumptions:
 *   MMC/SD card already selected
 *
 ****************************************************************************/

static uint32_t mmcsd_sendcmd(FAR struct mmcsd_slot_s *slot,
                              const struct mmcsd_cmdinfo_s *cmd, uint32_t arg)
{
  FAR struct spi_dev_s *spi = slot->spi;
  uint32_t result;
  uint8_t response = 0xff;
  int ret;
  int i;

  /* Wait until the card is not busy */

  ret = mmcsd_waitready(slot);
  if (ret != OK)
    {
      return ret;
    }

  /* Send command code */

  SPI_SEND(spi, cmd->cmd);

  /* Send command's arguments (should be zero if there are no arguements) */

  SPI_SEND(spi, (arg >> 24) & 0xff);
  SPI_SEND(spi, (arg >> 16) & 0xff);
  SPI_SEND(spi, (arg >> 8) & 0xff);
  SPI_SEND(spi, arg & 0xff);

  /* Send CRC if needed.  The SPI interface is initialized in non-protected
   * mode.  However, the reset command (CMD0) and CMD8 are received by the
   * card while it is still in SD mode and, therefore, must have a valid
   * CRC field.
   */

  SPI_SEND(spi, cmd->chksum);

  /* Skip stuff byte on CMD12 */

  if (cmd->cmd == CMD12)
    {
      SPI_SEND(spi, 0xff);
    }

  /* Get the response to the command.  A valid response will have bit7=0.
   * Usually, the non-response is 0xff, but I have seen 0xc0 too.
   */

  for (i = 0; i < 9 && (response & 0x80) != 0; i++)
    {
      response = SPI_SEND(spi, 0xff);
    }

  if ((response & 0x80) != 0)
    {
      fdbg("Failed: i=%d response=%02x\n", i, response);
      return (uint32_t)-1;
    }

  /* Interpret the response according to the command */

  result = response;
  switch (cmd->resp)
    {
    /* The R1B response is two bytes long */

    case MMCSD_CMDRESP_R1B:
      {
        uint32_t busy = 0;
        uint32_t start;
        uint32_t elapsed;

        start = START_TIME;
        do
          {
            busy = SPI_SEND(spi, 0xff);
            elapsed = ELAPSED_TIME(start);
          }
        while (elapsed < slot->twrite && busy != 0xff);

        if (busy != 0xff)
          {
            fdbg("Failed: card still busy (%02x)\n", busy);
            return (uint32_t)-1;
          }

        fvdbg("CMD%d[%08x] R1B=%02x\n",
              cmd->cmd & 0x3f, arg, response);
      }
      break;

    /* The R1 response is a single byte */

    case MMCSD_CMDRESP_R1:
      {
        fvdbg("CMD%d[%08x] R1=%02x\n",
              cmd->cmd & 0x3f, arg, response);
      }
      break;

    /* The R2 response is two bytes long */

    case MMCSD_CMDRESP_R2:
      {
        result  = ((uint32_t)(response & 0xff) << 8);
        result |= SPI_SEND(spi, 0xff) & 0xff;

        fvdbg("CMD%d[%08x] R2=%04x\n",
              cmd->cmd & 0x3f, arg, result);
      }
      break;

    /* The R3 response is 5 bytes long. The first byte is identical to R1. */

    case MMCSD_CMDRESP_R3:
      {
        slot->ocr  = ((uint32_t)(SPI_SEND(spi, 0xff) & 0xff) << 24);
        slot->ocr |= ((uint32_t)(SPI_SEND(spi, 0xff) & 0xff) << 16);
        slot->ocr |= ((uint32_t)(SPI_SEND(spi, 0xff) & 0xff) << 8);
        slot->ocr |= SPI_SEND(spi, 0xff) & 0xff;

        fvdbg("CMD%d[%08x] R1=%02x OCR=%08x\n",
              cmd->cmd & 0x3f, arg, response, slot->ocr);
      }
      break;

    /* The R7 response is 5 bytes long. The first byte is identical to R1. */

    case MMCSD_CMDRESP_R7:
    default:
      {
        slot->r7  = ((uint32_t)(SPI_SEND(spi, 0xff) & 0xff) << 24);
        slot->r7 |= ((uint32_t)(SPI_SEND(spi, 0xff) & 0xff) << 16);
        slot->r7 |= ((uint32_t)(SPI_SEND(spi, 0xff) & 0xff) << 8);
        slot->r7 |= SPI_SEND(spi, 0xff) & 0xff;

        fvdbg("CMD%d[%08x] R1=%02x R7=%08x\n",
              cmd->cmd & 0x3f, arg, response, slot->r7);
      }
      break;
    }

  return result;
}

/****************************************************************************
 * Name: mmcsd_setblklen
 *
 * Description:
 *   Set block length
 *
 * Assumptions:
 *   MMC/SD card already selected
 *
 ****************************************************************************/

static void mmcsd_setblklen(FAR struct mmcsd_slot_s *slot, uint32_t length)
{
  uint32_t response;

  fvdbg("Set block length to %d\n", length);
  response = mmcsd_sendcmd(slot, &g_cmd16, length);
  if (response != MMCSD_SPIR1_OK)
    {
      fdbg("Failed to set block length: %02x\n", response);
    }
}

/****************************************************************************
 * Name: mmcsd_nsac
 *
 * Description: Convert the value of the NSAC to microseconds
 *
 ****************************************************************************/

static uint32_t mmcsd_nsac(FAR struct mmcsd_slot_s *slot, uint8_t *csd,
                           uint32_t frequency)
{
  /* NSAC is 8-bits wide and is in units of 100 clock cycles.  Therefore, the
   * maximum value is 25.5K clock cycles.
   */

  uint32_t nsac = MMCSD_CSD_NSAC(csd) * ((uint32_t)100*1000);
  uint32_t fhkz = (frequency + 500) / 1000;
  return (nsac + (fhkz >> 1)) / fhkz;
}

/****************************************************************************
 * Name: mmcsd_taac
 *
 * Description: Convert the value of the TAAC to microseconds
 *
 ****************************************************************************/

static uint32_t mmcsd_taac(FAR struct mmcsd_slot_s *slot, uint8_t *csd)
{
  int tundx;

  /*The TAAC consists of a 3-bit time unit (TU) and a 4-bit time value (TV).
   * TAAC is in units of time; NSAC is in units of SPI clocks.
   * The access time we need is then given by:
   *
   *   taccess = TU*TV + NSAC/spifrequency
   *
   * g_taactu holds TU in units of nanoseconds and microseconds (you have to use
   * the index to distiguish.  g_taactv holds TV with 8-bits of fraction.
   */

  tundx  = MMCSD_CSD_TAAC_TIMEUNIT(csd);
  if (tundx <= MAX_USTUNDX)
    {
      /* The maximum value of the nanosecond TAAC is 800 ns. The rounded
       * answer in microseconds will be at most 1.
       */

      return 1;
    }
  else
    {
      /* Return the answer in microseconds */

      return (g_taactu[tundx]*g_taactv[MMCSD_CSD_TAAC_TIMEVALUE(csd)] + 0x80) >> 8;
    }
}

/****************************************************************************
 * Name: mmcsd_decodecsd
 *
 * Description:
 *
 ****************************************************************************/

static void mmcsd_decodecsd(FAR struct mmcsd_slot_s *slot, uint8_t *csd)
{
  FAR struct spi_dev_s *spi = slot->spi;
  uint32_t maxfrequency;
  uint32_t frequency;
  uint32_t readbllen;
  uint32_t csizemult;
  uint32_t csize;

  /* Calculate SPI max clock */

  maxfrequency =
    g_transpeedtu[MMCSD_CSD_TRANSPEED_TIMEVALUE(csd)] *
    g_transpeedru[MMCSD_CSD_TRANSPEED_TRANSFERRATEUNIT(csd)];

  /* Clip the max frequency to account for board limitations */

  frequency = maxfrequency;
  if (frequency > CONFIG_MMCSD_SPICLOCK)
    {
      frequency = CONFIG_MMCSD_SPICLOCK;
    }

  /* Set the actual SPI frequency as close as possible to that value */

  frequency = SPI_SETFREQUENCY(spi, frequency);

  /* Now determine the delay to access data */

  if (slot->type == MMCSD_CARDTYPE_MMC)
    {
      /* The TAAC consists of a 3-bit time unit (TU) and a 4-bit time value (TV).
       * TAAC is in units of time; NSAC is in units of SPI clocks.
       * The access time we need is then given by:
       *
       *   taccess = TU*TV + NSAC/spifrequency
       *
       * Example: TAAC = 1.5 ms,  NSAC = 0, r2wfactor = 4, CLK_TCK=100
       *          taccessus = 1,500uS
       *          taccess   = (1,500 * 100) / 100,000) + 1 = 2 (ideal, 1.5)
       *          twrite    = (1,500 * 4 * 100) / 100,000) + 1 = 7 (ideal 6.0)
       *
       * First get the access time in microseconds
       */

      uint32_t taccessus = mmcsd_taac(slot, csd) + mmcsd_nsac(slot, csd, frequency);

      /* Then convert to system clock ticks.  The maximum read access is 10 times
       * the tacc value: taccess = 10 * (taccessus / 1,000,000) * CLK_TCK, or
       */

      slot->taccess = (taccessus * CLK_TCK) / 100000 + 1;

      /* NOTE that we add one to taccess to assure that we wait at least this
       * time.  The write access time is larger by the R2WFACTOR: */

      slot->taccess = (taccessus * MMCSD_CSD_R2WFACTOR(csd) * CLK_TCK) / 100000 + 1;
    }
  else
    {
      /* For SD, the average is still given by the TAAC+NSAC, but the
       * maximum are the constants 100 and 250MS
       */

      slot->taccess  = SD_READACCESS;
      slot->twrite   = SD_WRITEACCESS;
    }

  fvdbg("SPI Frequency\n");
  fvdbg("  Maximum:         %d Hz\n", maxfrequency);
  fvdbg("  Actual:          %d Hz\n", frequency);
  fvdbg("Read access time:  %d ticks\n", slot->taccess);
  fvdbg("Write access time: %d ticks\n", slot->twrite);

  /* Get the physical geometry of the card: sector size and number of
   * sectors. The card's total capacity is computed from
   *
   *   capacity  = BLOCKNR * BLOCK_LEN
   *   BLOCKNR   = (C_SIZE+1)*MULT
   *   MULT      = 2**(C_SIZE_MULT+2)    (C_SIZE_MULT < 8)
   *   BLOCK_LEN = 2**READ_BL_LEN        (READ_BL_LEN < 12)
   *
   * Or
   *
   *   capacity = ((C_SIZE+1) << (READD_BL_LEN + C_SIZE_MULT + 2))
   *
   * In units of the sector size (1 << READ_BL_LEN), then simplifies to
   *
   *   nsectors = ((C_SIZE+1) << (C_SIZE_MULT + 2))
   */

  if (MMCSD_CSD_CSDSTRUCT(csd) != 0)
    {
      /* SDC structure ver 2.xx */
      /* Note: On SD card WRITE_BL_LEN is always the same as READ_BL_LEN */

      readbllen = SD20_CSD_READBLLEN(csd);
      csizemult = SD20_CSD_CSIZEMULT(csd) + 2;
      csize     = SD20_CSD_CSIZE(csd) + 1;
    }
  else
    {
      /* MMC or SD structure ver 1.xx */
      /* Note: On SD card WRITE_BL_LEN is always the same as READ_BL_LEN */

      readbllen = MMCSD_CSD_READBLLEN(csd);
      csizemult = MMCSD_CSD_CSIZEMULT(csd) + 2;
      csize     = MMCSD_CSD_CSIZE(csd) + 1;
    }

  /* SDHC ver2.x cards have fixed block transfer size of 512 bytes.  SDC
   * ver1.x cards with capacity less than 1Gb, will have sector size
   * 512 byes. SDC ver1.x cards with capacity of 2Gb will report readbllen
   * of 1024 but should use 512 bytes for block transfers.  SDC ver1.x 4Gb
   * cards will report readbllen of 2048 bytes -- are they also 512 bytes?
   */

#ifdef CONFIG_MMCSD_SECTOR512
  if (readbllen > 9)
    {
      csizemult += (readbllen - 9);
    }
  else
    {
      DEBUGASSERT(readbllen == 9);
    }
#else
  if (IS_SDV2(slot->type))
    {
      if (readbllen > 9)
        {
          fdbg("Forcing 512 byte sector size\n");
          csizemult += (readbllen - 9);
          readbllen  = 9;
        }
    }

  slot->sectorsize = 1 << readbllen;
#endif
  slot->nsectors   = csize << csizemult;
  fvdbg("Sector size:       %d\n", SECTORSIZE(slot));
  fvdbg("Number of sectors: %d\n", slot->nsectors);
}

/****************************************************************************
 * Name: mmcsd_checkwrprotect
 *
 * Description:
 *
 ****************************************************************************/

static void mmcsd_checkwrprotect(FAR struct mmcsd_slot_s *slot, uint8_t *csd)
{
  FAR struct spi_dev_s *spi = slot->spi;

  /* Check if (1) the slot is reporting that reporting that write protection
   * is set, (2) the card reports permanent write protect, or (2) the card
   * reports temporary write protect.
   */

  if ((SPI_STATUS(spi, SPIDEV_MMCSD) & SPI_STATUS_WRPROTECTED) != 0 ||
      MMCSD_CSD_PERMWRITEPROTECT(csd) ||
      MMCSD_CSD_TMPWRITEPROTECT(csd))
    {
      slot->state |= MMCSD_SLOTSTATUS_WRPROTECT;
    }
  else
    {
      slot->state &= ~MMCSD_SLOTSTATUS_WRPROTECT;
    }
}

/****************************************************************************
 * Name: mmcsd_getcardinfo
 *
 * Description:
 *   Read CSD or CID  registers
 *
 * Assumptions:
 *   MMC/SD card already selected
 *
 ****************************************************************************/

static int mmcsd_getcardinfo(FAR struct mmcsd_slot_s *slot, uint8_t *buffer,
                             const struct mmcsd_cmdinfo_s *cmd)
{
  FAR struct spi_dev_s *spi = slot->spi;
  uint32_t result;
  uint8_t response;
  int i;

  SPI_SEND(spi, 0xff);

  /* Send the CMD9 or CMD10 */

  result = mmcsd_sendcmd(slot, cmd, 0);
  if (result != MMCSD_SPIR1_OK)
    {
      fdbg("CMD9/10 failed: R1=%02x\n", result);
      return -EIO;
    }

  /* Try up to 8 times to find the start of block (or until an error occurs) */

  for (i = 0; i < 8; i++)
    {
      response = SPI_SEND(spi, 0xff);
      fvdbg("%d. SPI send returned %02x\n", i, response);

      /* If a read operation fails and the card cannot provide the requested
       * data, it will send a data error token instead.  The 4 least
       * significant bits are the same as those in the R2 response.
       */

      if (response != 0 && (response & MMCSD_SPIDET_UPPER) == 0)
        {
          fdbg("%d. Data transfer error: %02x\n", i, response);
          return -EIO;
        }
      else if (response == MMCSD_SPIDT_STARTBLKSNGL)
        {
          for (i = 0; i < 16; ++i)
            {
              *buffer++ = SPI_SEND(spi, 0xff);
            }

          /* CRC receive */

          SPI_SEND(spi, 0xff);
          SPI_SEND(spi, 0xff);
          return OK;
        }
    }

  fdbg("%d. Did not find start of block\n");
  return -EIO;
}

/****************************************************************************
 * Name: mmcsd_recvblock
 *
 * Description:  Receive a data block from the card
 *
 ****************************************************************************/

static int mmcsd_recvblock(FAR struct mmcsd_slot_s *slot, uint8_t *buffer, int nbytes)
{
  FAR struct spi_dev_s *spi = slot->spi;
  uint32_t start;
  uint32_t elapsed;
  uint8_t  token;

  /* Wait up to the maximum to receive a valid data token.  taccess is the
   * time from when the command is sent until the first byte of data is
   * received */

  start = START_TIME;
  do
    {
      token = SPI_SEND(spi, 0xff);
      elapsed = ELAPSED_TIME(start);
    }
  while (token == 0xff && elapsed < slot->taccess);

  if (token == MMCSD_SPIDT_STARTBLKSNGL)
    {
      /* Receive the block */

      SPI_RECVBLOCK(spi, buffer, nbytes);

      /* Discard the CRC */

      SPI_SEND(spi, 0xff);
      SPI_SEND(spi, 0xff);
      return OK;
    }

  fdbg("Did not receive data token (%02x)\n", token);
  return ERROR;
}

/****************************************************************************
 * Name: mmcsd_xmitblock
 *
 * Description:  Transmit a data block to the card
 *
 ****************************************************************************/

#if defined(CONFIG_FS_WRITABLE) && !defined(CONFIG_MMCSD_READONLY)
static int mmcsd_xmitblock(FAR struct mmcsd_slot_s *slot, const uint8_t *buffer,
                           int nbytes, uint8_t token)
{
  FAR struct spi_dev_s *spi = slot->spi;
  uint8_t response;

  /* Start the block transfer:
   * 1. 0xff (sync)
   * 2. 0xfe or 0xfc (start of block token)
   * 3. Followed by the block of data and 2 byte CRC
   */

  SPI_SEND(spi, 0xff);                     /* sync */
  SPI_SEND(spi, token);                    /* data token */

  /* Transmit the block to the MMC/SD card */

  (void)SPI_SNDBLOCK(spi, buffer, nbytes);

  /* Add the bogus CRC.  By default, the SPI interface is initialized in
   * non-protected mode.  However, we still have to send bogus CRC values
   */

  SPI_SEND(spi, 0xff);
  SPI_SEND(spi, 0xff);

  /* Now get the data response */

  response = SPI_SEND(spi, 0xff);
  if ((response & MMCSD_SPIDR_MASK) != MMCSD_SPIDR_ACCEPTED)
    {
      fdbg("Bad data response: %02x\n", response);
      return -EIO;
    }
  return OK;
}
#endif /* CONFIG_FS_WRITABLE && !CONFIG_MMCSD_READONLY */

/****************************************************************************
 * Block Driver Operations
 ****************************************************************************/

/****************************************************************************
 * Name: mmcsd_open
 *
 * Description: Open the block device
 *
 ****************************************************************************/

static int mmcsd_open(FAR struct inode *inode)
{
  FAR struct mmcsd_slot_s *slot;
  FAR struct spi_dev_s *spi;
  int ret;

  fvdbg("Entry\n");

#ifdef CONFIG_DEBUG
  if (!inode || !inode->i_private)
    {
      fdbg("Internal confusion\n");
      return -EIO;
    }
#endif

  /* Extract our private data from the inode structure */

  slot = (FAR struct mmcsd_slot_s *)inode->i_private;
  spi  = slot->spi;

#ifdef CONFIG_DEBUG
  if (!spi)
    {
      fdbg("Internal confusion\n");
      return -EIO;
    }
#endif

  /* Verify that an MMC/SD card has been inserted */

  ret = -ENODEV;
  mmcsd_semtake(&slot->sem);
  if ((SPI_STATUS(spi, SPIDEV_MMCSD) & SPI_STATUS_PRESENT) != 0)
    {
      /* Yes.. a card is present.  Has it been initialized? */

      if (slot->type == MMCSD_CARDTYPE_UNKNOWN)
        {
          /* Ininitialize for the media in the slot */

          ret = mmcsd_mediainitialize(slot);
          if (ret < 0)
            {
              fvdbg("Failed to initialize card\n");
              goto errout_with_sem;
            }
        }

      /* Make sure that the card is ready */

      SPI_SELECT(spi, SPIDEV_MMCSD, true);
      ret = mmcsd_waitready(slot);
      SPI_SELECT(spi, SPIDEV_MMCSD, false);
    }

errout_with_sem:
  mmcsd_semgive(&slot->sem);
  return ret;
}

/****************************************************************************
 * Name: mmcsd_close
 *
 * Description: close the block device
 *
 ****************************************************************************/

static int mmcsd_close(FAR struct inode *inode)
{
  fvdbg("Entry\n");
  return OK;
}

/****************************************************************************
 * Name: mmcsd_read
 *
 * Description:  Read the specified numer of sectors
 *
 ****************************************************************************/

static ssize_t mmcsd_read(FAR struct inode *inode, unsigned char *buffer,
                          size_t start_sector, unsigned int nsectors)
{
  FAR struct mmcsd_slot_s *slot;
  FAR struct spi_dev_s *spi;
  size_t nbytes;
  off_t  offset;
  uint8_t  response;
  int    i;

  fvdbg("start_sector=%d nsectors=%d\n", start_sector, nsectors);

#ifdef CONFIG_DEBUG
  if (!buffer)
    {
      fdbg("Invalid parameters\n");
      return -EINVAL;
    }

  if (!inode || !inode->i_private)
    {
      fdbg("Internal confusion\n");
      return -EIO;
    }
#endif

  /* Extract our private data from the inode structure */

  slot = (FAR struct mmcsd_slot_s *)inode->i_private;
  spi  = slot->spi;

#ifdef CONFIG_DEBUG
  if (!spi)
    {
      fdbg("Internal confusion\n");
      return -EIO;
    }
#endif

  /* Verify that card is available */

  if (slot->state & MMCSD_SLOTSTATUS_NOTREADY)
    {
      fdbg("Slot not ready\n");
      return -ENODEV;
    }

  /* Do nothing on zero-length transfer */

  if (nsectors < 1)
    {
      return 0;
    }

  /* Convert sector and nsectors to nbytes and byte offset */

  nbytes = nsectors * SECTORSIZE(slot);
  if (IS_BLOCK(slot->type))
    {
      offset = start_sector;
      fvdbg("nbytes=%d sector offset=%d\n", nbytes, offset);
    }
  else
    {
      offset = start_sector * SECTORSIZE(slot);
      fvdbg("nbytes=%d byte offset=%d\n", nbytes, offset);
    }

  /* Select the slave */

  mmcsd_semtake(&slot->sem);
  SPI_SELECT(spi, SPIDEV_MMCSD, true);

  /* Single or multiple block read? */

  if (nsectors == 1)
    {
      /* Send CMD17: Reads a block of the size selected by the SET_BLOCKLEN
       * command and verify that good R1 status is returned
       */

      response = mmcsd_sendcmd(slot, &g_cmd17, offset);
      if (response != MMCSD_SPIR1_OK)
        {
          fdbg("CMD17 failed: R1=%02x\n", response);
          goto errout_with_eio;
        }

      /* Receive the block */

      if (mmcsd_recvblock(slot, buffer, SECTORSIZE(slot)) != 0)
        {
          fdbg("Failed: to receive the block\n");
          goto errout_with_eio;
        }
    }
  else
    {
      /* Send CMD18: Reads a block of the size selected by the SET_BLOCKLEN
       * command and verify that good R1 status is returned
       */

      response = mmcsd_sendcmd(slot, &g_cmd18, offset);
      if (response != MMCSD_SPIR1_OK)
        {
          fdbg("CMD118 failed: R1=%02x\n", response);
          goto errout_with_eio;
        }

      /* Receive each block */

      for (i = 0; i < nsectors; i++)
        {
          if (mmcsd_recvblock(slot, buffer, SECTORSIZE(slot)) != 0)
            {
              fdbg("Failed: to receive the block\n");
              goto errout_with_eio;
            }
         buffer += SECTORSIZE(slot);
       }

      /* Send CMD12: Stops transmission */

      response = mmcsd_sendcmd(slot, &g_cmd12, 0);
    }

  /* On success, return the number of sectors transfer */

  SPI_SELECT(spi, SPIDEV_MMCSD, false);
  SPI_SEND(spi, 0xff);
  mmcsd_semgive(&slot->sem);

  fvdbg("Read %d bytes:\n", nbytes);
  mmcsd_dumpbuffer("Read buffer", buffer, nbytes);
  return nsectors;

errout_with_eio:
  SPI_SELECT(spi, SPIDEV_MMCSD, false);
  mmcsd_semgive(&slot->sem);
  return -EIO;
}

/****************************************************************************
 * Name: mmcsd_write
 *
 * Description:
 *   Write the specified number of sectors
 *
 ****************************************************************************/

#if defined(CONFIG_FS_WRITABLE) && !defined(CONFIG_MMCSD_READONLY)
static ssize_t mmcsd_write(FAR struct inode *inode, const unsigned char *buffer,
                        size_t start_sector, unsigned int nsectors)
{
  FAR struct mmcsd_slot_s *slot;
  FAR struct spi_dev_s *spi;
  size_t nbytes;
  off_t  offset;
  uint8_t response;
  int ret;
  int i;

  fvdbg("start_sector=%d nsectors=%d\n", start_sector, nsectors);

#ifdef CONFIG_DEBUG
  if (!buffer)
    {
      fdbg("Invalid parameters\n");
      return -EINVAL;
    }

  if (!inode || !inode->i_private)
    {
      fdbg("Internal confusion\n");
      return -EIO;
    }
#endif

  /* Extract our private data from the inode structure */

  slot = (FAR struct mmcsd_slot_s *)inode->i_private;
  spi  = slot->spi;

#ifdef CONFIG_DEBUG
  if (!spi)
    {
      fdbg("Internal confusion\n");
      return -EIO;
    }
#endif

  /* Verify that card is available */

  if (slot->state & MMCSD_SLOTSTATUS_NOTREADY)
    {
      fdbg("Slot not ready\n");
      return -ENODEV;
    }

  /* Verify that the card is write enabled */

  if (slot->state & MMCSD_SLOTSTATUS_WRPROTECT)
    {
      fdbg("Not write enabled\n");
      return -EACCES;
    }

  /* Do nothing on zero-length transfer */

  if (nsectors < 1)
    {
      return 0;
    }

  /* Convert sector and nsectors to nbytes and byte offset */

  nbytes = nsectors * SECTORSIZE(slot);
  if (IS_BLOCK(slot->type))
    {
      offset = start_sector;
      fvdbg("nbytes=%d sector offset=%d\n", nbytes, offset);
    }
  else
    {
      offset = start_sector * SECTORSIZE(slot);
      fvdbg("nbytes=%d byte offset=%d\n", nbytes, offset);
    }
  mmcsd_dumpbuffer("Write buffer", buffer, nbytes);

  /* Select the slave */

  mmcsd_semtake(&slot->sem);
  SPI_SELECT(spi, SPIDEV_MMCSD, true);

  /* Single or multiple block transfer? */

  if (nsectors == 1)
    {
      /* Send CMD24 (WRITE_BLOCK) and verify that good R1 status is returned */

      response = mmcsd_sendcmd(slot, &g_cmd24, offset);
      if (response != MMCSD_SPIR1_OK)
        {
          fdbg("CMD24 failed: R1=%02x\n", response);
          goto errout_with_sem;
        }

      /* Then transfer the sector */

      if (mmcsd_xmitblock(slot, buffer, SECTORSIZE(slot), 0xfe) != 0)
        {
          fdbg("Block transfer failed\n");
          goto errout_with_sem;
        }
    }
  else
    {
      /* Set the number of blocks to be pre-erased (SD only) */

      if (IS_SD(slot->type))
        {
          response = mmcsd_sendcmd(slot, &g_acmd23, nsectors);
          if (response != MMCSD_SPIR1_OK)
            {
              fdbg("ACMD23 failed: R1=%02x\n", response);
              goto errout_with_sem;
            }
       }

      /* Send CMD25:  Continuously write blocks of data until the
       * tranmission is stopped.
       */

      response = mmcsd_sendcmd(slot, &g_cmd25, offset);
      if (response != MMCSD_SPIR1_OK)
        {
          fdbg("CMD25 failed: R1=%02x\n", response);
          goto errout_with_sem;
        }

      /* Transmit each block */

      for (i = 0; i < nsectors; i++)
        {
          if (mmcsd_xmitblock(slot, buffer, SECTORSIZE(slot), 0xfc) != 0)
            {
              fdbg("Failed: to receive the block\n");
              goto errout_with_sem;
            }
          buffer += SECTORSIZE(slot);
        }

      /* Send the stop transmission token */

      SPI_SEND(spi, MMCSD_SPIDT_STOPTRANS);
    }

  /* Wait until the card is no longer busy */

  ret = mmcsd_waitready(slot);
  SPI_SELECT(spi, SPIDEV_MMCSD, false);
  SPI_SEND(spi, 0xff);
  mmcsd_semgive(&slot->sem);

  /* The success return value is the number of sectors written */

  return nsectors;

errout_with_sem:
  SPI_SELECT(spi, SPIDEV_MMCSD, false);
  mmcsd_semgive(&slot->sem);
  return -EIO;
}
#endif

/****************************************************************************
 * Name: mmcsd_geometry
 *
 * Description:
 *   Return device geometry
 *
 ****************************************************************************/

static int mmcsd_geometry(FAR struct inode *inode, struct geometry *geometry)
{
  FAR struct mmcsd_slot_s *slot;
  FAR struct spi_dev_s *spi;
  uint8_t csd[16];
  int ret;

#ifdef CONFIG_DEBUG
  if (!geometry)
    {
      fdbg("Invalid parameters\n");
      return -EINVAL;
    }

  if (!inode || !inode->i_private)
    {
      fdbg("Internal confusion\n");
      return -EIO;
    }
#endif

  /* Extract our private data from the inode structure */

  slot = (FAR struct mmcsd_slot_s *)inode->i_private;
  spi  = slot->spi;

#ifdef CONFIG_DEBUG
  if (!spi)
    {
      fdbg("Internal confusion\n");
      return -EIO;
    }
#endif

  /* Re-sample the CSD */

  mmcsd_semtake(&slot->sem);
  SPI_SELECT(spi, SPIDEV_MMCSD, true);
  ret = mmcsd_getcsd(slot, csd);
  SPI_SELECT(spi, SPIDEV_MMCSD, false);

  if (ret < 0)
    {
      mmcsd_semgive(&slot->sem);
      fdbg("mmcsd_getcsd returned %d\n", ret);
      return ret;
    }

  /* Check for changes related to write protection */

  mmcsd_checkwrprotect(slot, csd);

  /* Then return the card geometry */

  geometry->geo_available =
    ((slot->state & (MMCSD_SLOTSTATUS_NOTREADY|MMCSD_SLOTSTATUS_NODISK)) == 0);
  geometry->geo_mediachanged =
    ((slot->state & MMCSD_SLOTSTATUS_MEDIACHGD) != 0);
#if defined(CONFIG_FS_WRITABLE) && !defined(CONFIG_MMCSD_READONLY)
  geometry->geo_writeenabled =
    ((slot->state & MMCSD_SLOTSTATUS_WRPROTECT) == 0);
#else
  geometry->geo_writeenabled = false;
#endif
  geometry->geo_nsectors   = slot->nsectors;
  geometry->geo_sectorsize = SECTORSIZE(slot);

  /* After reporting mediachanged, clear the indication so that it is not
   * reported again.
   */

  slot->state &= ~MMCSD_SLOTSTATUS_MEDIACHGD;
  mmcsd_semgive(&slot->sem);

  fvdbg("geo_available:     %d\n", geometry->geo_available);
  fvdbg("geo_mediachanged:  %d\n", geometry->geo_mediachanged);
  fvdbg("geo_writeenabled:  %d\n", geometry->geo_writeenabled);
  fvdbg("geo_nsectors:      %d\n", geometry->geo_nsectors);
  fvdbg("geo_sectorsize:    %d\n", geometry->geo_sectorsize);

  return OK;
}

/****************************************************************************
 * Initialization
 ****************************************************************************/

/****************************************************************************
 * Name: mmcsd_mediainitialize
 *
 * Description:
 *   Detect media and initialize
 *
 ****************************************************************************/

static int mmcsd_mediainitialize(FAR struct mmcsd_slot_s *slot)
{
  FAR struct spi_dev_s *spi = slot->spi;
  uint8_t csd[16];
  uint32_t result = MMCSD_SPIR1_IDLESTATE;
  uint32_t start;
  uint32_t elapsed;
  int i, j;

  /* Assume that the card is not ready (we'll clear this on successful card
   * initialization.
   */

  slot->state |= MMCSD_SLOTSTATUS_NOTREADY;

  /* Check if there is a card present in the slot.  This is normally a matter is
   * of GPIO sensing and does not really involve SPI, but by putting this
   * functionality in the SPI interface, we encapuslate the SPI MMC/SD
   * interface
   */

  if ((SPI_STATUS(spi, SPIDEV_MMCSD) & SPI_STATUS_PRESENT) == 0)
    {
      fdbg("No card present\n");
      slot->state |= MMCSD_SLOTSTATUS_NODISK;
      return -ENODEV;
    }

  /* Clock Freq. Identification Mode < 400kHz */

  SPI_SETFREQUENCY(spi, MMCSD_IDMODE_CLOCK);

  /* Set the maximum access time out */

  slot->taccess = SD_READACCESS;

  /* The SD card wakes up in SD mode. It will enter SPI mode if the chip select signal is
   * asserted (negative) during the reception of the reset command (CMD0) and the card is in
   * IDLE state.
   */

  for (i = 0; i < 2; i++)
    {
      /* After power up at least 74 clock cycles are required prior to
       * starting bus communication
       */

      for (j = 10; j; j--)
        {
          SPI_SEND(spi, 0xff);
        }

      /* Send CMD0 (GO_TO_IDLE) with CS asserted to put MMC/SD in
       * IDLE/SPI mode. Return from CMD0 is R1 which should now
       * show IDLE STATE
       */

      fvdbg("Send CMD0\n");
      SPI_SELECT(spi, SPIDEV_MMCSD, true);
      result = mmcsd_sendcmd(slot, &g_cmd0, 0);
      if (result == MMCSD_SPIR1_IDLESTATE)
        {
          /* Break out of the loop with card selected */

          fvdbg("Card is in IDLE state\n");
          break;
        }

      /* De-select card and try again */

      SPI_SELECT(spi, SPIDEV_MMCSD, false);
    }

  /* Verify that we exit the above loop with the card reporting IDLE state */

  if (result != MMCSD_SPIR1_IDLESTATE)
    {
      fdbg("Send CMD0 failed: R1=%02x\n", result);
      SPI_SELECT(spi, SPIDEV_MMCSD, false);
      return -EIO;
    }

  slot->type = MMCSD_CARDTYPE_UNKNOWN;

  /* Check for SDHC Version 2.x.  CMD 8 is reserved on SD version 1.0 and MMC. */

  fvdbg("Send CMD8\n");
  result = mmcsd_sendcmd(slot, &g_cmd8, 0x1aa);
  if (result == MMCSD_SPIR1_IDLESTATE)
    {
      /* Verify the operating voltage and that the 0xaa was correctly echoed */

      if (((slot->r7 & MMCSD_SPIR7_VOLTAGE_MASK) == MMCSD_SPIR7_VOLTAGE_27) &&
          ((slot->r7 & MMCSD_SPIR7_ECHO_MASK) == 0xaa))
        {
          /* Try CMD55/ACMD41 for up to 1 second or until the card exits
           * the IDLE state
           */

          start   = START_TIME;
          elapsed = 0;
          do
            {
              fvdbg("%d. Send CMD55/ACMD41\n", elapsed);
              result = mmcsd_sendcmd(slot, &g_cmd55, 0);
              if (result == MMCSD_SPIR1_IDLESTATE || result == MMCSD_SPIR1_OK)
                {
                  result = mmcsd_sendcmd(slot, &g_acmd41, (uint32_t)1 << 30);
                  if (result == MMCSD_SPIR1_OK)
                    {
                      break;
                    }
                }
              elapsed = ELAPSED_TIME(start);
            }
          while (elapsed < MMCSD_DELAY_1SEC);

          /* Check if ACMD41 was sent successfully */

          if (elapsed < MMCSD_DELAY_1SEC)
           {
             fvdbg("Send CMD58\n");
             SPI_SEND(spi, 0xff);
             result = mmcsd_sendcmd(slot, &g_cmd58, 0);
             if (result == MMCSD_SPIR1_OK)
               {
                  fvdbg("OCR: %08x\n", slot->ocr);
                  if ((slot->ocr & MMCSD_OCR_CCS) != 0)
                    {
                      fdbg("Identified SD ver2 card/with block access\n");
                      slot->type = MMCSD_CARDTYPE_SDV2|MMCSD_CARDTYPE_BLOCK;
                    }
                  else
                    {
                      fdbg("Identified SD ver2 card\n");
                      slot->type = MMCSD_CARDTYPE_SDV2;
                    }
               }
           }
        }
    }

  /* Check for SDC version 1.x or MMC */

  else
    {
      /* Both the MMC card and the SD card support CMD55 */

      fvdbg("Send CMD55/ACMD41\n");
      result = mmcsd_sendcmd(slot, &g_cmd55, 0);
      if (result == MMCSD_SPIR1_IDLESTATE || result == MMCSD_SPIR1_OK)
        {
          /* But ACMD41 is supported only on SD */

          result = mmcsd_sendcmd(slot, &g_acmd41, 0);
          if (result == MMCSD_SPIR1_IDLESTATE || result == MMCSD_SPIR1_OK)
            {
              fdbg("Identified SD ver1 card\n");
              slot->type = MMCSD_CARDTYPE_SDV1;
            }
        }

      /* Make sure that we are out of the Idle state */

      start   = START_TIME;
      elapsed = 0;
      do
        {
          if (IS_SD(slot->type))
            {
              fvdbg("%d. Send CMD55/ACMD41\n", elapsed);
              result = mmcsd_sendcmd(slot, &g_cmd55, 0);
              if (result == MMCSD_SPIR1_IDLESTATE || result == MMCSD_SPIR1_OK)
                {
                  result = mmcsd_sendcmd(slot, &g_acmd41, 0);
                  if (result == MMCSD_SPIR1_OK)
                    {
                       break;
                    }
                }
            }
          else
            {
              fvdbg("%d. Send CMD1\n", i);
              result = mmcsd_sendcmd(slot, &g_cmd1, 0);
              if (result == MMCSD_SPIR1_OK)
                {
                   fdbg("%d. Identified MMC card\n", i);
                   slot->type = MMCSD_CARDTYPE_MMC;
                   break;
                }
             }
          elapsed = ELAPSED_TIME(start);
        }
      while (elapsed < MMCSD_DELAY_1SEC);

      if (elapsed >= MMCSD_DELAY_1SEC)
        {
          fdbg("Failed to exit IDLE state\n");
          SPI_SELECT(spi, SPIDEV_MMCSD, false);
          return -EIO;
        }
    }

  if (slot->type == MMCSD_CARDTYPE_UNKNOWN)
    {
      fdbg("Failed to identify card\n");
      SPI_SELECT(spi, SPIDEV_MMCSD, false);
      return -EIO;
    }

  /* Read CSD. CSD must always be valid */

  fvdbg("Get CSD\n");
  result = mmcsd_getcsd(slot, csd);
  if (result != OK)
    {
      fdbg("mmcsd_getcsd(CMD9) failed: %d\n", result);
      SPI_SELECT(spi, SPIDEV_MMCSD, false);
      return -EIO;
    }
  mmcsd_dmpcsd(csd, slot->type);

  /* CSD data and set block size */

  mmcsd_decodecsd(slot, csd);
  mmcsd_checkwrprotect(slot, csd);

  /* SDHC ver2.x cards have fixed block transfer size of 512 bytes.  SDC
   * ver1.x cards with capacity less than 1Gb, will have sector size
   * 512 byes. SDC ver1.x cards with capacity of 2Gb will report readbllen
   * of 1024 but should use 512 bytes for block transfers.  SDC ver1.x 4Gb
   * cards will report readbllen of 2048 bytes -- are they also 512 bytes?
   * I think that none of these high capacity cards support setting the
   * block length??
   */

#ifdef CONFIG_MMCSD_SECTOR512
  /* Using 512 byte sectors, the maximum ver1.x capacity is 4096 x 512 blocks.
   * The saved slot->nsectors is converted to 512 byte blocks, so if slot->nsectors
   * exceeds 4096 x 512, then we must be dealing with a card with read_bl_len
   * of 1024 or 2048.
   */

  if (!IS_SDV2(slot->type) && slot->nsectors <= ((uint32_t)4096*12))
    {
      /* Don't set the block len on high capacity cards (ver1.x or ver2.x) */

      mmcsd_setblklen(slot, SECTORSIZE(slot));
    }
#else
  if (!IS_SDV2(slot->type))
    {
      /* Don't set the block len on ver2.x cards */

      mmcsd_setblklen(slot, SECTORSIZE(slot));
    }
#endif

  slot->state &= ~MMCSD_SLOTSTATUS_NOTREADY;
  SPI_SELECT(spi, SPIDEV_MMCSD, false);
  return OK;
}

/****************************************************************************
 * Name: mmcsd_mediachanged
 *
 * Description:
 *   Handle initialization/media change events
 *
 ****************************************************************************/

static void mmcsd_mediachanged(void *arg)
{
  struct mmcsd_slot_s *slot = (struct mmcsd_slot_s*)arg;
  FAR struct spi_dev_s *spi;
  uint8_t oldstate;
  int ret;

#ifdef CONFIG_DEBUG
  if (!slot || !slot->spi)
    {
      fdbg("Internal confusion\n");
      return;
    }
#endif
  spi  = slot->spi;

  /* Save the current slot state and reassess the new state */

  mmcsd_semtake(&slot->sem);
  oldstate = slot->state;

  /* Check if media was removed or inserted */

  slot->state &= ~(MMCSD_SLOTSTATUS_NODISK|MMCSD_SLOTSTATUS_NOTREADY|MMCSD_SLOTSTATUS_MEDIACHGD);
  if ((SPI_STATUS(spi, SPIDEV_MMCSD) & SPI_STATUS_PRESENT) == 0)
    {
      /* Media is not present */

      fdbg("No card present\n");
      slot->state |= (MMCSD_SLOTSTATUS_NODISK|MMCSD_SLOTSTATUS_NOTREADY);

      /* Was media removed? */

      if ((oldstate & MMCSD_SLOTSTATUS_NODISK) == 0)
        {
          slot->state |= MMCSD_SLOTSTATUS_MEDIACHGD;
        }
    }

  /* Media is present, was it just inserted? Or, if it was previously not ready,
   * then try re-initializing it
   */

  else if ((oldstate & (MMCSD_SLOTSTATUS_NODISK|MMCSD_SLOTSTATUS_NOTREADY)) != 0)
    {
      /* (Re-)ininitialize for the media in the slot */

      ret = mmcsd_mediainitialize(slot);
      if (ret == 0)
        {
          fvdbg("mmcsd_mediainitialize returned OK\n");
          slot->state |= MMCSD_SLOTSTATUS_MEDIACHGD;
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mmcsd_spislotinitialize
 *
 * Description:
 *   Initialize one slot for operation using the SPI MMC/SD interface
 *
 * Input Parameters:
 *   minor - The MMC/SD minor device number.  The MMC/SD device will be
 *     registered as /dev/mmcsdN where N is the minor number
 *   slotno - The slot number to use.  This is only meaningful for architectures
 *     that support multiple MMC/SD slots.  This value must be in the range
 *     {0, ..., CONFIG_MMCSD_NSLOTS}.
 *   spi - And instance of an SPI interface obtained by called
 *     up_spiinitialize() with the appropriate port number (see spi.h)
 *
 ****************************************************************************/

int mmcsd_spislotinitialize(int minor, int slotno, FAR struct spi_dev_s *spi)
{
  struct mmcsd_slot_s *slot;
  char devname[16];
  int ret;

#ifdef CONFIG_DEBUG
  if ((unsigned)slotno >= CONFIG_MMCSD_NSLOTS || (unsigned)minor > 255 || !spi)
    {
      fdbg("Invalid arguments\n");
      return -EINVAL;
    }
#endif

  /* Select the slot structure */

  slot = &g_mmcsdslot[slotno];
  memset(slot, 0, sizeof(struct mmcsd_slot_s));
  sem_init(&slot->sem, 0, 1);

#ifdef CONFIG_DEBUG
  if (slot->spi)
    {
      fdbg("Already registered\n");
      return -EBUSY;
    }
#endif

  /* Bind the SPI port to the slot */

  slot->spi = spi;

  /* Ininitialize for the media in the slot (if any) */

  ret = mmcsd_mediainitialize(slot);
  if (ret == 0)
    {
      fvdbg("mmcsd_mediainitialize returned OK\n");
      slot->state |= MMCSD_SLOTSTATUS_MEDIACHGD;
    }

  /* Create a MMC/SD device name */

  snprintf(devname, 16, "/dev/mmcsd%d", minor);

  /* Register the driver, even on a failure condition.  A
   * card may be inserted later, for example.
   */

  ret = register_blockdriver(devname, &g_bops, MMCSD_MODE, slot);
  if (ret < 0)
    {
      fdbg("register_blockdriver failed: %d\n", -ret);
      slot->spi = NULL;
      return ret;
    }

  /* Register a media change callback to handler insertion and
   * removal of cards.
   */

  (void)SPI_REGISTERCALLBACK(spi, mmcsd_mediachanged, (void*)slot);
  return OK;
}
