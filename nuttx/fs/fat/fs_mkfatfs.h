/****************************************************************************
 * fs/fat/fs_mkfat.h
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

#ifndef __FS_FAT_FS_MKATFS_H
#define __FS_FAT_FS_MKATFS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Only the "hard drive" media type is used */

#define FAT_DEFAULT_MEDIA_TYPE         0xf8

/*  Default hard driver geometry */

#define FAT_DEFAULT_SECPERTRK          63
#define FAT_DEFAULT_NUMHEADS           255

/* FSINFO is always at this sector */

#define FAT_DEFAULT_FSINFO_SECTOR      1

/* FAT32 foot cluster number */

#define FAT32_DEFAULT_ROOT_CLUSTER     2

/* Macros to simplify direct block driver access */

#define DEV_OPEN() \
    var->fb_inode->u.i_bops->open ? \
    var->fv_inode->u.i_bops->open(var->fv_inode) : \
    0
#define DEV_CLOSE() \
    var->fb_inode->u.i_bops->close ? \
    var->fv_inode->u.i_bops->close(var->fv_inode) : \
    0
#define DEV_READ(buf, sect, nsect) \
    var->fv_inode->u.i_bops->read(var->fv_inode, buf, sect, nsect)
#define DEV_WRITE(buf, sect, nsect) \
    var->fv_inode->u.i_bops->write(var->fv_inode, buf, sect, nsect)
#define DEV_GEOMETRY(geo) \
    var->fv_inode->u.i_bops->geometry(var->fv_inode, &geo)
#define DEV_IOCTL(cmd, arg) \
    var->fv_inode->u.i_bops->ioctl(var->fv_inode, cmd, arg)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure (plus the user-provided struct fat_format_s) describes
 * the format FAT file system.  All "global" variables used in the format
 * logic are contained in this structure so that is possible to format two
 * block devices concurrently.
 */

struct fat_var_s
{
   struct inode  *fv_inode;          /* The block driver "handle" */
   uint8_t        fv_jump[3];        /* 3-byte boot jump instruction */
   uint8_t        fv_sectshift;      /* Log2 of fv_sectorsize */
   uint8_t        fv_nrootdirsects;  /* Number of root directory sectors */
   uint8_t        fv_fattype;        /* FAT size: 0 (not determined), 12, 16, or 32 */
   uint16_t       fv_bootcodesize;   /* Size of array at fv_bootcode */
   uint32_t       fv_createtime;     /* Creation time */
   uint32_t       fv_sectorsize;     /* Size of one hardware sector */
   uint32_t       fv_nfatsects;      /* Number of sectors in each FAT */
   uint32_t       fv_nclusters;      /* Number of clusters */
   uint8_t       *fv_sect;           /* Allocated working sector buffer */
   const uint8_t *fv_bootcode;       /* Points to boot code to put into MBR */
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
 * Name: mkfatfs_configfatfs
 *
 * Description:
 *   Based on the geometry of the block device and upon the caller-selected
 *   values, configure the FAT filesystem for the device.
 *
 * Input:
 *    fmt  - Caller specified format parameters
 *    var  - Holds disk geomtry data.  Also, the location to return FAT
 *           configuration data
 *
 * Return:
 *    Zero on success; negated errno on failure
 *
 ****************************************************************************/
EXTERN int mkfatfs_configfatfs(FAR struct fat_format_s *fmt,
                               FAR struct fat_var_s *var);

/****************************************************************************
 * Name: mkfatfs_writefat
 *
 * Description:
 *   Write the configured fat filesystem to the block device
 *
 * Input:
 *    fmt  - Caller specified format parameters
 *    var  - Other format parameters that are not caller specifiable. (Most
 *           set by mkfatfs_configfatfs()).
 *
 * Return:
 *    Zero on success; negated errno on failure
 *
 ****************************************************************************/
EXTERN int mkfatfs_writefatfs(FAR struct fat_format_s *fmt,
                              FAR struct fat_var_s *var);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __FS_FAT_FS_MKATFS_H */
