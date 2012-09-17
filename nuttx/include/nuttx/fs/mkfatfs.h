/****************************************************************************
 * include/nuttx/fs/mkfatfs.h
 *
 *   Copyright (C) 2008-2009, 2012 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_FS_MKFATFS_H
#define __INCLUDE_NUTTX_FS_MKFATFS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MKFATFS_DEFAULT_NFATS        2     /* 2: Default number of FATs */
#define MKFATFS_DEFAULT_FATTYPE      0     /* 0: Autoselect FAT size */
#define MKFATFS_DEFAULT_CLUSTSHIFT   0xff  /* 0xff: Autoselect cluster size */
#define MKFATFS_DEFAULT_VOLUMELABEL  { ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ' }
#define MKFATFS_DEFAULT_BKUPBOOT     0     /* 0: Determine sector number of the backup boot sector */
#define MKFATFS_DEFAULT_ROOTDIRENTS  0     /* 0: Autoselect number of root directory entries */
#define MKFATFS_DEFAULT_RSVDSECCOUNT 0     /* 0: Autoselect number reserved sectors (usually 32) */
#define MKFATFS_DEFAULT_HIDSEC       0     /* No hidden sectors */
#define MKFATFS_DEFAULT_VOLUMEID     0     /* No volume ID */
#define MKFATFS_DEFAULT_NSECTORS     0     /* 0: Use all sectors on device */

#define FAT_FORMAT_INITIALIZER \
{ \
  MKFATFS_DEFAULT_NFATS, \
  MKFATFS_DEFAULT_FATTYPE, \
  MKFATFS_DEFAULT_CLUSTSHIFT, \
  MKFATFS_DEFAULT_VOLUMELABEL, \
  MKFATFS_DEFAULT_BKUPBOOT, \
  MKFATFS_DEFAULT_ROOTDIRENTS, \
  MKFATFS_DEFAULT_RSVDSECCOUNT, \
  MKFATFS_DEFAULT_HIDSEC, \
  MKFATFS_DEFAULT_VOLUMEID, \
  MKFATFS_DEFAULT_NSECTORS \
}

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* These are input parameters for the format.  On return, these values may be
 * overwritted with actual values used in the format.
 */

struct fat_format_s
{
   uint8_t  ff_nfats;           /* Number of FATs */
   uint8_t  ff_fattype;         /* FAT size: 0 (autoselect), 12, 16, or 32 */
   uint8_t  ff_clustshift;      /* Log2 of sectors per cluster: 0-5, 0xff (autoselect) */
   uint8_t  ff_volumelabel[11]; /* Volume label */
   uint16_t ff_backupboot;      /* Sector number of the backup boot sector (0=use default)*/
   uint16_t ff_rootdirentries;  /* Number of root directory entries */
   uint16_t ff_rsvdseccount;    /* Reserved sectors */
   uint32_t ff_hidsec;          /* Count of hidden sectors preceding fat */
   uint32_t ff_volumeid;        /* FAT volume id */
   uint32_t ff_nsectors;        /* Number of sectors from device to use: 0: Use all */
};

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: mkfatfs
 *
 * Description:
 *   Make a FAT file system image on the specified block device
 *
 * Inputs:
 *   pathname - the full path to a registered block driver
 *   fmt - Describes characteristics of the desired filesystem
 *
 * Return:
 *   Zero (OK) on success; -1 (ERROR) on failure with errno set appropriately:
 *
 *   EINVAL - NULL block driver string, bad number of FATS in 'fmt', bad FAT
 *     size in 'fmt', bad cluster size in 'fmt'
 *   ENOENT - 'pathname' does not refer to anything in the filesystem.
 *   ENOTBLK - 'pathname' does not refer to a block driver
 *   EACCESS - block driver does not support write or geometry methods
 *
 * Assumptions:
 *   - The caller must assure that the block driver is not mounted and not in
 *     use when this function is called.  The result of formatting a mounted
 *     device is indeterminate (but likely not good).
 *
 ****************************************************************************/
EXTERN int mkfatfs(FAR const char *pathname, FAR struct fat_format_s *fmt);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_FS_MKFATFS_H */
