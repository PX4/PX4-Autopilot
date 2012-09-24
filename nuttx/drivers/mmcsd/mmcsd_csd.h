/****************************************************************************
 * drivers/mmcsd/mmcsd_csd.h
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

#ifndef __DRIVERS_MMCSD_MMCSD_CSD_H
#define __DRIVERS_MMCSD_MMCSD_CSD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* CSD **********************************************************************/

/* The CSD is a 16 byte, / 128-bit packed structure.  The following macros
 * can be used to extract each packed field from the CSD.  Two types of
 * macros are supported, selected by CONFIG_MMCSD_BE16:  (1) byte data (with
 * MS byte first), or (2) 16-bit big-endian data (with MS hword first).
 */

#ifdef CONFIG_MMCSD_BE16

/* CSD_STRUCTURE 126-127 */

#define MMCSD_CSD_CSDSTRUCT(csd) (csd[0] >> 14)

/* SPEC_VERS 122-125 = (word 0, bits 13:10) (MMC) Spec version */

#define MMC_CSD_SPECVERS(csd) ((csd[0] >> 10) & 0x0f)

/* Reserved 120-125 */

/* TAAC 112-119 = Data read access-time-1
 *   TIME_VALUE 3-6 = Time mantissa
 *   TIME_UNIT  0-2 =  Time exponent
 */

#define MMCSD_CSD_TAAC_TIMEVALUE(csd) ((csd[0] >> 3) & 0x0f)
#define MMCSD_CSD_TAAC_TIMEUNIT(csd) (csd[0] & 7)

/* NSAC 111:104 = Data read access-time-2 in CLK cycle(NSAC*100) */

#define MMCSD_CSD_NSAC(csd) (csd[1] >> 8)

/* TRAN_SPEED 96-103 = Max. data transfer rate
 *   TIME_VALUE 3-6 = Rate exponent
 *   TRANSFER_RATE_UNIT 0-2 = Rate mantissa
 */

#define MMCSD_CSD_TRANSPEED_TIMEVALUE(csd) ((csd[1] >> 3) & 0x0f)
#define MMCSD_CSD_TRANSPEED_TRANSFERRATEUNIT(csd) (csd[1] & 7)

/* CCC 84-95 = Card command classes */

#define MMCSD_CSD_CCC(csd) ((csd[2] >> 4) & 0x0fff)

 /* READ_BL_LEN 80-83 = Max. read data block length */

#define MMCSD_CSD_READBLLEN(csd) (csd[2] & 0x0f)

/* READ_BL_PARTIAL 79-79 = Partial blocks for read allowed */

#define MMCSD_CSD_READBLPARTIAL(csd) (csd[3] >> 15)

/* WRITE_BLK_MISALIGN 78-78 = Write block misalignment */

#define MMCSD_CSD_WRITEBLKMISALIN(csd) ((csd[3] >> 14) & 1)

/* READ_BLK_MISALIGN 77:77 = Read block misalignment */

#define MMCSD_CSD_READBLKMISALIN(csd) ((csd[3] >> 13) & 1)

/* DSR_IMP 76-76 = DSR implemented */

#define MMCSD_CSD_DSRIMP(csd) ((csd[3] >> 12) & 1)

/* C_SIZE 62-73 Device size */

#define MMCSD_CSD_CSIZE(csd) (((csd[3] & 0x03ff) << 2) | ((csd[4] >> 14) & 3))

/* VDD_R_CURR_MIN 59-61 = Max. read current at Vcc min */

#define MMCSD_CSD_VDDRCURRMIN(csd) ((csd[4] >> 11) & 7)

/* VDD_R_CURR_MAX 56-58 =  Max. read current at Vcc max */

#define MMCSD_CSD_VDDRCURRMAX(csd) ((csd[4] >> 8) & 7)

/* VDD_W_CURR_MIN 53-55 = Max. write current at Vcc min */

#define MMCSD_CSD_VDDWCURRMIN(csd) ((csd[4] >> 5) & 7)

/* VDD_W_CURR_MAX 50-52 = Max. write current at Vcc max */

#define MMCSD_CSD_VDDWCURRMAX(csd) ((csd[4] >> 2) & 7)

/* C_SIZE_MULT 47-49 Device size multiplier */

#define MMCSD_CSD_CSIZEMULT(csd) (((csd[4] & 3) << 1) | (csd[5] >> 15))

/* ER_BLK_EN 46-46 =Erase single block enable (SD) */

#define SD_CSD_SDERBLKEN(csd) ((csd[5] >> 14) & 1)

/* SECTOR_SIZE 39-45 = Erase sector size (SD) */

#define SD_CSD_SECTORSIZE(csd) ((csd[5] >> 7) & 0x7f)

/* SECTOR_SIZE 42-46 = Erase sector size (MMC) */

#define MMC_CSD_SECTORSIZE(csd) ((csd[5] >> 10) & 0x1f)

/* ER_GRP_SIZE 37-41 = Erase group size (MMC)*/

#define MMC_CSD_ERGRPSIZE(csd) ((csd[5] >> 5) & 0x1f)

/* WP_GRP_SIZE 32-38 = Write protect group size (SD) */

#define SD_CSD_WPGRPSIZE(csd) (csd[5] & 0x7f)

/* WP_GRP_SIZE 32-36 = Write protect group size (MMC) */

#define MMC_CSD_WPGRPSIZE(csd) (csd[5] & 0x1f)

/* WP_GRP_EN 31-31 = Write protect group enable */

#define MMCSD_WPGRPEN(csd) (csd[6] >> 15)

/* DFLT_ECC 29-30 = Manufacturer default ECC (MMC) */

#define MMC_CSD_DFLTECC(csd) ((csd[6] >> 13) & 3)

/* R2W_FACTOR 26-28 = Write speed factor */

#define MMCSD_CSD_R2WFACTOR(csd) ((csd[6] >> 10) & 7)

/* WRITE_BL_LEN 22-25 = Max. write data block length */

#define MMCSD_CSD_WRITEBLLEN(csd) ((csd[6] >> 6) & 0x0f)

/* WRITE_BL_PARTIAL 21-21 = Partial blocks for write allowed */

#define MMCSD_CSD_WRITEBLPARTIAL(csd) ((csd[6] >> 5) & 1)

/* Reserved 16-20 */

/* FILE_FORMAT_GROUP 15-15 = File format group */

#define MMCSD_CSD_FILEFORMATGRP(csd) (csd[7] >> 15)

/* COPY 14-14 = Copy flag (OTP) */

#define MMCSD_CSD_COPY(csd) ((csd[7] >> 14) & 1)

/* PERM_WRITE_PROTECT 13-13 = Permanent write protection */

#define MMCSD_CSD_PERMWRITEPROTECT(csd) ((csd[7] >> 13) & 1)

/* TMP_WRITE_PROTECT 12-12 = Temporary write protection */

#define MMCSD_CSD_TMPWRITEPROTECT(csd) ((csd[7] >> 12) & 1)

/* FILE_FORMAT 10-11 = File format */

#define MMCSD_CSD_FILEFORMAT(csd) ((csd[7] >> 10) & 3)

/* ECC 8-9 =  ECC (MMC)  */

#define MMC_CSD_ECC(csd) ((csd[7] >> 8) & 3)

/* CRC 1-7 = CRC */

#define MMCSD_CSD_CRC(csd) ((csd[7] >> 1) & 0x7f)

/* Reserved 0-0 */

#else /* CONFIG_MMCSD_BE16 */

/* CSD_STRUCTURE 126-127 */

#define MMCSD_CSD_CSDSTRUCT(csd) (csd[0] >> 6)

/* SPEC_VERS 122-125 = (word 0, bits 13:10) (MMC) Spec version */

#define MMC_CSD_SPECVERS(csd) ((csd[0] >> 2) & 0x0f)

/* Reserved 120-155 */

/* TAAC 112-119 = Data read access-time-1
 *   TIME_VALUE 3-6 = Time mantissa
 *   TIME_UNIT  0-2 = Time exponent
 */

#define MMCSD_CSD_TAAC_TIMEVALUE(csd) ((csd[1] >> 3) & 0x0f)
#define MMCSD_CSD_TAAC_TIMEUNIT(csd) (csd[1] & 7)

#define SD20_CSD_TAC_TIMEVALUE(csd) (1)
#define SD20_CSD_TAC_TIMEUNIT(csd) (6)

/* NSAC 111:104 = Data read access-time-2 in CLK cycle(NSAC*100) */

#define MMCSD_CSD_NSAC(csd) (csd[2])
#define SD20_CSD_NSAC(csd) (0)

/* TRAN_SPEED 96-103 = Max. data transfer rate
 *   TIME_VALUE 3-6 = Rate exponent
 *   TRANSFER_RATE_UNIT 0-2 = Rate mantissa
 */

#define MMCSD_CSD_TRANSPEED_TIMEVALUE(csd) ((csd[3] >> 3) & 0x0f)
#define MMCSD_CSD_TRANSPEED_TRANSFERRATEUNIT(csd) (csd[3] & 7)

#define SD20_CSD_TRANSPEED_TIMEVALUE(csd) MMCSD_CSD_TRANSPEED_TIMEVALUE(csd)
#define SD20_CSD_TRANSPEED_TRANSFERRATEUNIT(csd) MMCSD_CSD_TRANSPEED_TRANSFERRATEUNIT(csd)

/* CCC 84-95 = Card command classes */

#define MMCSD_CSD_CCC(csd) (((uint16_t)csd[4] << 4) | ((uint16_t)csd[5] >> 4))
#define SD20_CSD_CCC(csd) MMCSD_CSD_CCC(csd)

 /* READ_BL_LEN 80-83 = Max. read data block length */

#define MMCSD_CSD_READBLLEN(csd) (csd[5] & 0x0f)
#define SD20_CSD_READBLLEN(csd) (9)

/* READ_BL_PARTIAL 79-79 = Partial blocks for read allowed */

#define MMCSD_CSD_READBLPARTIAL(csd) (csd[6] >> 7)
#define SD20_CSD_READBLPARTIAL(csd) (0)

/* WRITE_BLK_MISALIGN 78-78 = Write block misalignment */

#define MMCSD_CSD_WRITEBLKMISALIGN(csd) ((csd[6] >> 6) & 1)
#define SD20_CSD_WRITEBLKMISALIGN(csd) (0)

/* READ_BLK_MISALIGN 77:77 = Read block misalignment */

#define MMCSD_CSD_READBLKMISALIGN(csd) ((csd[6] >> 5) & 1)
#define SD20_CSD_READBLKMISALIGN(csd) (0)

/* DSR_IMP 76-76 = DSR implemented */

#define MMCSD_CSD_DSRIMP(csd) ((csd[6] >> 4) & 1)
#define SD20_CSD_DSRIMP(csd) MMCSD_CSD_DSRIMP(csd)

/* C_SIZE 62-73 Device size */

#define MMCSD_CSD_CSIZE(csd) (((csd[6] & 3) << 10) | (csd[7] << 2) | (csd[8] >> 6))
#define SD20_CSD_CSIZE(csd)  ((((uint32_t)csd[7] & 0x3f) << 16) | (csd[8] << 8) | csd[9])

/* VDD_R_CURR_MIN 59-61 = Max. read current at Vcc min */

#define MMCSD_CSD_VDDRCURRMIN(csd) ((csd[8] >> 3) & 7)
#define SD20_CSD_VDDRCURRMIN(csd) (7)

/* VDD_R_CURR_MAX 56-58 =  Max. read current at Vcc max */

#define MMCSD_CSD_VDDRCURRMAX(csd) (csd[8] & 7)
#define SD20_CSD_VDDRCURRMAX(csd) (6)

/* VDD_W_CURR_MIN 53-55 = Max. write current at Vcc min */

#define MMCSD_CSD_VDDWCURRMIN(csd) ((csd[9] >> 5) & 7)
#define SD20_CSD_VDDWCURRMIN(csd) (7)

/* VDD_W_CURR_MAX 50-52 = Max. write current at Vcc max */

#define MMCSD_CSD_VDDWCURRMAX(csd) ((csd[9] >> 2) & 7)
#define SD20_CSD_VDDWCURRMAX(csd) (6)

/* C_SIZE_MULT 47-49 Device size multiplier */

#define MMCSD_CSD_CSIZEMULT(csd) (((csd[9] & 3) << 1) | (csd[10] >> 7))
#define SD20_CSD_CSIZEMULT(csd) (10-2)

/* ER_BLK_EN 46-46 = Erase single block enable (SD) */

#define SD_CSD_SDERBLKEN(csd) ((csd[10] >> 6) & 1)
#define SD20_CSD_SDERBLKEN(csd) (1)

/* SECTOR_SIZE 39-45 = Erase sector size (SD) */

#define SD_CSD_SECTORSIZE(csd) (((csd[10] & 0x3f) << 1) | (csd[11] >> 7))
#define SD20_CSD_SECTORSIZE(csd) (0x7f)

/* SECTOR_SIZE 42-46 = Erase sector size (MMC) */

#define MMC_CSD_SECTORSIZE(csd) ((csd[10] >> 2) & 0x1f)

/* ER_GRP_SIZE 37-41 = Erase group size (MMC)*/

#define MMC_CSD_ERGRPSIZE(csd) (((csd[10] & 3) << 3) | (csd[11] > 5))

/* WP_GRP_SIZE 32-38 = Write protect group size (SD) */

#define SD_CSD_WPGRPSIZE(csd) (csd[11] & 0x7f)
#define SD20_CSD_WPGRPSIZE(csd) (0)

/* WP_GRP_SIZE 32-36 = Write protect group size (MMC) */

#define MMC_CSD_WPGRPSIZE(csd) (csd[11] & 0x1f)

/* WP_GRP_EN 31-31 = Write protect group enable */

#define MMCSD_WPGRPEN(csd) (csd[12] >> 7)
#define SD20_WPGRPEN(csd) (0)

/* DFLT_ECC 29-30 = Manufacturer default ECC (MMC) */

#define MMC_CSD_DFLTECC(csd) ((csd[12] >> 5) & 3)

/* R2W_FACTOR 26-28 = Write speed factor */

#define MMCSD_CSD_R2WFACTOR(csd) ((csd[12] >> 2) & 7)
#define SD20_CSD_R2WFACTOR(csd) (2)

/* WRITE_BL_LEN 22-25 = Max. write data block length */

#define MMCSD_CSD_WRITEBLLEN(csd) (((csd[12] & 3) << 2) | (csd[13] >> 6))
#define SD20_CSD_WRITEBLLEN(csd) (9)

/* WRITE_BL_PARTIAL 21-21 = Partial blocks for write allowed */

#define MMCSD_CSD_WRITEBLPARTIAL(csd) ((csd[13] >> 5) & 1)
#define SD20_CSD_WRITEBLPARTIAL(csd) (0)

/* Reserved 16-20 */

/* FILE_FORMAT_GROUP 15-15 = File format group */

#define MMCSD_CSD_FILEFORMATGRP(csd) (csd[14] >> 7)
#define SD20_CSD_FILEFORMATGRP(csd) (0)

/* COPY 14-14 = Copy flag (OTP) */

#define MMCSD_CSD_COPY(csd) ((csd[14] >> 6) & 1)
#define SD20_CSD_COPY(csd) MMCSD_CSD_COPY(csd)

/* PERM_WRITE_PROTECT 13-13 = Permanent write protection */

#define MMCSD_CSD_PERMWRITEPROTECT(csd) ((csd[14] >> 5) & 1)
#define SD20_CSD_PERMWRITEPROTECT(csd) MMCSD_CSD_PERMWRITEPROTECT(csd)

/* TMP_WRITE_PROTECT 12-12 = Temporary write protection */

#define MMCSD_CSD_TMPWRITEPROTECT(csd)  ((csd[14] >> 4) & 1)
#define SD20_CSD_TMPWRITEPROTECT(csd)  MMCSD_CSD_TMPWRITEPROTECT(csd)

/* FILE_FORMAT 10-11 = File format */

#define MMCSD_CSD_FILEFORMAT(csd) ((csd[14] >> 2) & 3)
#define SD20_CSD_FILEFORMAT(csd) (0)

/* ECC 8-9 =  ECC (MMC)  */

#define MMC_CSD_ECC(csd) (csd[14] & 3)

/* CRC 1-7 = CRC */

#define MMCSD_CSD_CRC(csd) (csd[15] >> 1)
#define SD20_CSD_CRC(csd) MMCSD_CSD_CRC(csd)

/* Reserved 0-0 */

#endif /* CONFIG_MMCSD_BE16 */

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
#endif /* __DRIVERS_MMCSD_MMCSD_CSD_H */
