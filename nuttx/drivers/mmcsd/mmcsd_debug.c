/****************************************************************************
 * drivers/mmcsd/mmcsd_debug.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include "mmcsd_csd.h"
#include "mmcsd_internal.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* This needs to match the logic in include/debug.h */

#ifdef CONFIG_CPP_HAVE_VARARGS
#  define message(format, arg...) syslog(format, ##arg)
#else
#  define message syslog
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mmcsd_dmpcsd
 *
 * Description:
 *   Dump the contents of the CSD
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_VERBOSE) && defined(CONFIG_DEBUG_FS)
void mmcsd_dmpcsd(FAR const uint8_t *csd, uint8_t cardtype)
{
  bool mmc = (cardtype == MMCSD_CARDTYPE_MMC);
  bool sd2 = (MMCSD_CSD_CSDSTRUCT(csd) == 1);

  fvdbg("CSD\n");
  fvdbg("  CSD_STRUCTURE:           1.%d\n",   MMCSD_CSD_CSDSTRUCT(csd));
  if (mmc)
    {
      fvdbg("  MMC SPEC_VERS:           %d\n", MMC_CSD_SPECVERS(csd));
    }
  fvdbg("  TAAC:\n",
      sd2 ? SD20_CSD_TAC_TIMEVALUE(csd) : MMCSD_CSD_TAAC_TIMEVALUE(csd));
  fvdbg("    TIME_VALUE:            0x%02x\n",
      sd2 ? SD20_CSD_TAC_TIMEVALUE(csd) : MMCSD_CSD_TAAC_TIMEVALUE(csd));
  fvdbg("    TIME_UNIT:             0x%02x\n",
      sd2 ? SD20_CSD_TAC_TIMEUNIT(csd) : MMCSD_CSD_TAAC_TIMEUNIT(csd));
  fvdbg("  NSAC:                    0x%02x\n",
      sd2 ? SD20_CSD_NSAC(csd) : MMCSD_CSD_NSAC(csd));
  fvdbg("  TRAN_SPEED:\n");
  fvdbg("    TIME_VALUE:            0x%02x\n",
      sd2 ? SD20_CSD_TRANSPEED_TIMEVALUE(csd) : MMCSD_CSD_TRANSPEED_TIMEVALUE(csd));
  fvdbg("    RATE_UNIT:             0x%02x\n",
      sd2 ? SD20_CSD_TRANSPEED_TRANSFERRATEUNIT(csd) : MMCSD_CSD_TRANSPEED_TRANSFERRATEUNIT(csd));
  fvdbg("  CCC:                     0x%03x\n",
      sd2 ? SD20_CSD_CCC(csd) : MMCSD_CSD_CCC(csd));
  fvdbg("  READ_BL_LEN:             %d\n",
      sd2 ? SD20_CSD_READBLLEN(csd) : MMCSD_CSD_READBLLEN(csd));
  fvdbg("  READ_BL_PARTIAL:         %d\n",
      sd2 ? SD20_CSD_READBLPARTIAL(csd) : MMCSD_CSD_READBLPARTIAL(csd));
  fvdbg("  WRITE_BLK_MISALIGN:      %d\n",
      sd2 ? SD20_CSD_WRITEBLKMISALIGN(csd) : MMCSD_CSD_WRITEBLKMISALIGN(csd));
  fvdbg("  READ_BLK_MISALIGN:       %d\n",
      sd2 ? SD20_CSD_READBLKMISALIGN(csd) : MMCSD_CSD_READBLKMISALIGN(csd));
  fvdbg("  DSR_IMP:                 %d\n",
      sd2 ? SD20_CSD_DSRIMP(csd) : MMCSD_CSD_DSRIMP(csd));
  fvdbg("  C_SIZE:                  %d\n",
      sd2 ? SD20_CSD_CSIZE(csd) : MMCSD_CSD_CSIZE(csd));
  fvdbg("  VDD_R_CURR_MIN:          %d\n",
      sd2 ? SD20_CSD_VDDRCURRMIN(csd) : MMCSD_CSD_VDDRCURRMIN(csd));
  fvdbg("  VDD_R_CURR_MAX:          %d\n",
      sd2 ? SD20_CSD_VDDRCURRMAX(csd) : MMCSD_CSD_VDDRCURRMAX(csd));
  fvdbg("  VDD_W_CURR_MIN:          %d\n",
      sd2 ? SD20_CSD_VDDWCURRMIN(csd) : MMCSD_CSD_VDDWCURRMIN(csd));
  fvdbg("  VDD_W_CURR_MAX:          %d\n",
      sd2 ? SD20_CSD_VDDWCURRMAX(csd) : MMCSD_CSD_VDDWCURRMAX(csd));
  fvdbg("  C_SIZE_MULT:             %d\n",
      sd2 ? SD20_CSD_CSIZEMULT(csd) : MMCSD_CSD_CSIZEMULT(csd));
  if (mmc)
    {
      fvdbg("  MMC SECTOR_SIZE:        %d\n", MMC_CSD_SECTORSIZE(csd));
      fvdbg("  MMC ER_GRP_SIZE:        %d\n", MMC_CSD_ERGRPSIZE(csd));
      fvdbg("  MMC WP_GRP_SIZE:        %d\n",  MMC_CSD_WPGRPSIZE(csd));
      fvdbg("  MMC DFLT_ECC:           %d\n",  MMC_CSD_DFLTECC(csd));
    }
  else
    {
      fvdbg("  SD ER_BLK_EN:            %d\n",
          sd2 ? SD20_CSD_SDERBLKEN(csd) : SD_CSD_SDERBLKEN(csd));
      fvdbg("  SD SECTOR_SIZE:          %d\n",
          sd2 ? SD20_CSD_SECTORSIZE(csd) : SD_CSD_SECTORSIZE(csd));
      fvdbg("  SD WP_GRP_SIZE:          %d\n",
          sd2 ? SD_CSD_WPGRPSIZE(csd) : SD_CSD_WPGRPSIZE(csd));
    }
  fvdbg("  WP_GRP_EN:               %d\n",
      sd2 ? SD20_WPGRPEN(csd) : MMCSD_WPGRPEN(csd));
  fvdbg("  R2W_FACTOR:              %d\n",
      sd2 ? SD20_CSD_R2WFACTOR(csd) : MMCSD_CSD_R2WFACTOR(csd));
  fvdbg("  WRITE_BL_LEN:            %d\n",
      sd2 ? SD20_CSD_WRITEBLLEN(csd) : MMCSD_CSD_WRITEBLLEN(csd));
  fvdbg("  WRITE_BL_PARTIAL:        %d\n",
      sd2 ? SD20_CSD_WRITEBLPARTIAL(csd) : MMCSD_CSD_WRITEBLPARTIAL(csd));
  fvdbg("  FILE_FORMAT_GROUP:       %d\n",
      sd2 ? SD20_CSD_FILEFORMATGRP(csd) : MMCSD_CSD_FILEFORMATGRP(csd));
  fvdbg("  COPY:                    %d\n",
      sd2 ? SD20_CSD_COPY(csd) : MMCSD_CSD_COPY(csd));
  fvdbg("  PERM_WRITE_PROTECT:      %d\n",
      sd2 ? SD20_CSD_PERMWRITEPROTECT(csd) : MMCSD_CSD_PERMWRITEPROTECT(csd));
  fvdbg("  TMP_WRITE_PROTECT:       %d\n",
      sd2 ?SD20_CSD_TMPWRITEPROTECT(csd) : MMCSD_CSD_TMPWRITEPROTECT(csd));
  fvdbg("  FILE_FORMAT:             %d\n",
      sd2 ? SD20_CSD_FILEFORMAT(csd) : MMCSD_CSD_FILEFORMAT(csd));
  if (mmc)
    {
      fvdbg("  MMC ECC:                 %d\n",
          sd2 ? MMC_CSD_ECC(csd) : MMC_CSD_ECC(csd));
    }
  fvdbg("  CRC:                     %02x\n",
      sd2 ? SD20_CSD_CRC(csd) : MMCSD_CSD_CRC(csd));
}
#endif
