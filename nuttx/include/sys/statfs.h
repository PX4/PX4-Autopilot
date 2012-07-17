/****************************************************************************
 * include/sys/statfs.h
 *
 *   Copyright (C) 2007-2009, 2011 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_SYS_STATFS_H
#define __INCLUDE_SYS_STATFS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* struct statfs file system types. */

#define ADFS_SUPER_MAGIC      0xadf5
#define AFFS_SUPER_MAGIC      0xadff
#define BEFS_SUPER_MAGIC      0x42465331
#define BFS_MAGIC             0x1badface
#define CIFS_MAGIC_NUMBER     0xff534d42
#define CODA_SUPER_MAGIC      0x73757245
#define COH_SUPER_MAGIC       0x012ff7b7
#define CRAMFS_MAGIC          0x28cd3d45
#define DEVFS_SUPER_MAGIC     0x1373
#define EFS_SUPER_MAGIC       0x00414a53
#define EXT_SUPER_MAGIC       0x137d
#define EXT2_OLD_SUPER_MAGIC  0xef51
#define EXT2_SUPER_MAGIC      0xef53
#define EXT3_SUPER_MAGIC      0xef53
#define HFS_SUPER_MAGIC       0x4244
#define HPFS_SUPER_MAGIC      0xf995e849
#define HUGETLBFS_MAGIC       0x958458f6
#define ISOFS_SUPER_MAGIC     0x9660
#define JFFS2_SUPER_MAGIC     0x72b6
#define JFS_SUPER_MAGIC       0x3153464a
#define MINIX_SUPER_MAGIC     0x137f /* orig. minix */
#define MINIX_SUPER_MAGIC2    0x138f /* 30 char minix */
#define MINIX2_SUPER_MAGIC    0x2468 /* minix V2 */
#define MINIX2_SUPER_MAGIC2   0x2478 /* minix V2, 30 char names */
#define MSDOS_SUPER_MAGIC     0x4d44
#define NCP_SUPER_MAGIC       0x564c
#define NFS_SUPER_MAGIC       0x6969
#define NTFS_SB_MAGIC         0x5346544e
#define OPENPROM_SUPER_MAGIC  0x9fa1
#define PROC_SUPER_MAGIC      0x9fa0
#define QNX4_SUPER_MAGIC      0x002f
#define REISERFS_SUPER_MAGIC  0x52654973
#define ROMFS_MAGIC           0x7275
#define SMB_SUPER_MAGIC       0x517B
#define SYSV2_SUPER_MAGIC     0x012ff7b6
#define SYSV4_SUPER_MAGIC     0x012FF7B5
#define TMPFS_MAGIC           0x01021994
#define UDF_SUPER_MAGIC       0x15013346
#define UFS_MAGIC             0x00011954
#define USBDEVICE_SUPER_MAGIC 0x9fa2
#define VXFS_SUPER_MAGIC      0xa501fcf5
#define XENIX_SUPER_MAGIC     0x012ff7b4
#define XFS_SUPER_MAGIC       0x58465342
#define _XIAFS_SUPER_MAGIC    0x012fd16d

/* NuttX specific file-systems */

#define BINFS_MAGIC           0x4242
#define NXFFS_MAGIC           0x4747

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

struct statfs
{
  uint32_t f_type;     /* Type of filesystem (see definitions above) */
  size_t   f_namelen;  /* Maximum length of filenames */
  size_t   f_bsize;    /* Optimal block size for transfers */
  off_t    f_blocks;   /* Total data blocks in the file system of this size */
  off_t    f_bfree;    /* Free blocks in the file system */
  off_t    f_bavail;   /* Free blocks avail to non-superuser */
  off_t    f_files;    /* Total file nodes in the file system */
  off_t    f_ffree;    /* Free file nodes in the file system */
};

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

/* Inspired by Linux statfs() which was, in turn, inspired by
 * the BSD statfs(). None of these implementations agree in the
 * form of the struct statfs.
 */

EXTERN int statfs(const char *path, struct statfs *buf);
EXTERN int fstatfs(int fd, struct statfs *buf);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_SYS_STATFS_H */
