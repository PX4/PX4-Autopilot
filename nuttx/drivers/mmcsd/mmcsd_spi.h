/****************************************************************************
 * drivers/mmcsd/mmcsd_spi.h
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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

#ifndef __DRIVERS_MMCSD_MMCSD_SPI_H
#define __DRIVERS_MMCSD_MMCSD_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* SPI *******************************************************************/

/* SPI Command Set */

#define CMD0   0x40  /* GO_IDLE_STATE: Resets all cards to idle state */
#define CMD1   0x41  /* SEND_OP_COND: Sends capacity support information */
#define CMD6   0x46  /* SWITCH_FUNC: Checks switchable function */
#define CMD8   0x48  /* SEND_IF_COND: Sends SD Memory Card interface condition */
#define CMD9   0x49  /* SEND_CSD: Asks  card to send its card specific data (CSD) */
#define CMD10  0x4a  /* SEND_CID: Asks card to send its card identification (CID) */
#define CMD12  0x4c  /* STOP_TRANSMISSION: Forces the card to stop transmission */
#define CMD13  0x4d  /* SEND_STATUS: Asks card to send its status register */
#define CMD16  0x50  /* SET_BLOCKLEN: Sets a block length (in bytes) */
#define CMD17  0x51  /* READ_SINGLE_BLOCK: Reads a block of the selected size */
#define CMD18  0x52  /* READ_MULTIPLE_BLOCK: Continuously transfers blocks from card to host */
#define CMD20  0x54  /* CMD_WRITEBLOCK: Write block to memory (MMC) */
#define CMD24  0x58  /* WRITE_BLOCK: Writes a block of the selected size */
#define CMD25  0x59  /* WRITE_MULTIPLE_BLOCK: Continuously writes blocks of data */
#define CMD27  0x5b  /* PROGRAM_CSD: Set programmable bits of the CSD */
#define CMD28  0x5c  /* SET_WRITE_PROT: Sets the write protection bit of group */
#define CMD29  0x5d  /* CLR_WRITE_PROT: Clears the write protection bit of group */
#define CMD30  0x5e  /* SEND_WRITE_PROT: Asks card to send state of write protection bits */
#define CMD32  0x60  /* ERASE_WR_BLK_START_ADDR: Sets address of first block to erase */
#define CMD33  0x61  /* ERASE_WR_BLK_END_ADDR: Sets address of last block to erase */
#define CMD34  0x62  /* UNTAG_SECTOR: (MMC) */
#define CMD35  0x63  /* TAG_ERASE_GROUP_START: (MMC) */
#define CMD36  0x64  /* TAG_ERASE_GOUPR_END: (MMC) */
#define CMD37  0x65  /* UNTAG_ERASE_GROUP: (MMC) */
#define CMD38  0x66  /* ERASE: Erases all previously selected write blocks */
#define CMD40  0x68  /* CRC_ON_OFF: (MMC) */
#define CMD42  0x6a  /* LOCK_UNLOCK: Used to Set/Reset the Password or lock/unlock card */
#define CMD55  0x77  /* APP_CMD: Tells card that the next command is an application specific command */
#define CMD56  0x78  /* GEN_CMD: Used transfer a block to or get block from card */
#define CMD58  0x7a  /* READ_OCR :Reads the OCR register of a card */
#define CMD59  0x7b  /* CRC_ON_OFF: Turns the CRC option on or off */
#define ACMD13 0x4d  /* SD_STATUS: Send the SD Status */
#define ACMD22 0x56  /* SEND_NUM_WR_BLOCKS: Send number of the errorfree blocks */
#define ACMD23 0x57  /* SET_WR_BLK_ERASE_COUNT: Set number blocks to erase before writing */
#define ACMD41 0x69  /* SD_SEND_OP_COND: Sends host capacity support information */
#define ACMD42 0x6a  /* SET_CLR_CARD_DETECT: Connect/disconnect pull-up resistor on CS */
#define ACMD51 0x73  /* SEND_SCR: Reads the SD Configuration Register (SCR) */

/* SPI 8-bit R1 response */

#define MMCSD_SPIR1_OK            0x00 /* No error bits set */
#define MMCSD_SPIR1_IDLESTATE     0x01 /* Idle state */
#define MMCSD_SPIR1_ERASERESET    0x02 /* Erase reset */
#define MMCSD_SPIR1_ILLEGALCMD    0x04 /* Illegal command */
#define MMCSD_SPIR1_CRCERROR      0x08 /* Com CRC error */
#define MMCSD_SPIR1_ERASEERROR    0x10 /* Erase sequence error */
#define MMCSD_SPIR1_ADDRERROR     0x20 /* Address error */
#define MMCSD_SPIR1_PARAMERROR    0x40 /* Parameter error */

/* SPI 8-bit R2 response */

#define MMCSD_SPIR2_CARDLOCKED    0x0001 /* Card is locked */
#define MMCSD_SPIR2_WPERASESKIP   0x0002 /* WP erase skip */
#define MMCSD_SPIR2_LOCKFAIL      0x0002 /* Lock/unlock cmd failed */
#define MMCSD_SPIR2_ERROR         0x0004 /* Error */
#define MMCSD_SPIR2_CCERROR       0x0008 /* CC error */
#define MMCSD_SPIR2_CARDECCFAIL   0x0010 /* Card ECC failed */
#define MMCSD_SPIR2_WPVIOLATION   0x0020 /* WP violoation */
#define MMCSD_SPIR2_ERASEPARAM    0x0040 /* Erase parameter */
#define MMCSD_SPIR2_OUTOFRANGE    0x0080 /* Out of range */
#define MMCSD_SPIR2_CSDOVERWRITE  0x0080 /* CSD overwrite */
#define MMCSD_SPIR2_IDLESTATE     0x0100 /* In idle state */
#define MMCSD_SPIR2_ERASERESET    0x0200 /* Erase reset */
#define MMCSD_SPIR2_ILLEGALCMD    0x0400 /* Illegal command */
#define MMCSD_SPIR2_CRCERROR      0x0800 /* Com CRC error */
#define MMCSD_SPIR2_ERASEERROR    0x1000 /* Erase sequence error */
#define MMCSD_SPIR2_ADDRERROR     0x2000 /* Address error */
#define MMCSD_SPIR2_PARAMERROR    0x4000 /* Parameter error */

/* Last 4 bytes of the 5 byte R7 response */

#define MMCSD_SPIR7_VERSION_SHIFT (28)   /* Bits 28-31: Command version number */
#define MMCSD_SPIR7_VERSION_MASK  ((uint32_t)0x0f << MMCSD_SPIR7_VERSION_SHIFT)
#define MMCSD_SPIR7_VOLTAGE_SHIFT (8)    /* Bits 8-11: Voltage accepted */
#define MMCSD_SPIR7_VOLTAGE_MASK  ((uint32_t)0x0f << MMCSD_SPIR7_VOLTAGE_SHIFT)
#define   MMCSD_SPIR7_VOLTAGE_27  ((uint32_t)0x01 << MMCSD_SPIR7_VOLTAGE_SHIFT) /* 2.7-3.6V */
#define MMCSD_SPIR7_ECHO_SHIFT    (0)    /* Bits 0-7: Echoed check pattern */
#define MMCSD_SPIR7_ECHO_MASK     ((uint32_t)0xff << MMCSD_SPIR7_ECHO_SHIFT)

/* Data Response */

#define MMCSD_SPIDR_MASK          0x1f   /* Mask for valid data response bits */
#define MMCSD_SPIDR_ACCEPTED      0x05   /* Data accepted */
#define MMCSD_SPIDR_CRCERROR      0x0b   /* Data rejected due to CRC error */
#define MMCSD_SPIDR_WRERROR       0x0d   /* Data rejected due to write error */

/* Data Tokens */

#define MMCSD_SPIDT_STARTBLKSNGL  0xfe   /* First byte of block, single block */
#define MMCSD_SPIDT_STARTBLKMULTI 0xfc   /* First byte of block, multi-block */
#define MMCSD_SPIDT_STOPTRANS     0xfd   /* Stop transmission */

/* Data error token */

#define MMCSD_SPIDET_UPPER        0xf0   /* The upper four bits are zero */
#define MMCSD_SPIDET_ERROR        0x01   /* Error */
#define MMCSD_SPIDET_CCERROR      0x02   /* CC error */
#define MMCSD_SPIDET_CARDECCFAIL  0x04   /* Card ECC failed */
#define MMCSD_SPIDET_OUTOFRANGE   0x08   /* Out of range */

/* Operating Conditions register */

#define MMCSD_OCR_V27             ((uint32_t)1 << 15) /* Bit 15: 2.7-2.8V */
#define MMCSD_OCR_V28             ((uint32_t)1 << 16) /* Bit 16: 2.8-2.9V */
#define MMCSD_OCR_V29             ((uint32_t)1 << 17) /* Bit 17: 2.9-3.0V */
#define MMCSD_OCR_V30             ((uint32_t)1 << 18) /* Bit 18: 3.0-3.1V */
#define MMCSD_OCR_V31             ((uint32_t)1 << 19) /* Bit 19: 3.1-3.2V */
#define MMCSD_OCR_V32             ((uint32_t)1 << 20) /* Bit 20: 3.2-3.3V */
#define MMCSD_OCR_V33             ((uint32_t)1 << 21) /* Bit 21: 3.3-3.4V */
#define MMCSD_OCR_V34             ((uint32_t)1 << 22) /* Bit 22: 3.4-3.5V */
#define MMCSD_OCR_V35             ((uint32_t)1 << 23) /* Bit 23: 3.5-3.6V */
#define MMCSD_OCR_CCS             ((uint32_t)1 << 30) /* Bit 30: Card capacity status */
#define MMCSD_OCR_BUSY            ((uint32_t)1 << 31) /* Bit 31: Card powered up status bit */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __DRIVERS_MMCSD_MMCSD_SPI_H */
