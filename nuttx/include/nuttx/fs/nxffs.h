/****************************************************************************
 * include/nuttx/fs/nxffs.h
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_FS_NXFFS_H
#define __INCLUDE_NUTTX_FS_NXFFS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/fs.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* If the erased state of FLASH memory is anything other than 0xff, then this
 * configuration should be provided.
 */

#ifndef CONFIG_NXFFS_ERASEDSTATE
#  define CONFIG_NXFFS_ERASEDSTATE 0xff
#endif

#if CONFIG_NXFFS_ERASEDSTATE != 0xff && CONFIG_NXFFS_ERASEDSTATE != 0x00
#  error "CONFIG_NXFFS_ERASEDSTATE must be either 0x00 or 0xff"
#endif

/* Don't bother trying to pack things closer together than this. */

#ifndef CONFIG_NXFFS_PACKTHRESHOLD
#  define CONFIG_NXFFS_PACKTHRESHOLD 32
#endif

/* This is how big an inode name is permitted to be. */

#ifndef CONFIG_NXFFS_MAXNAMLEN
#  define CONFIG_NXFFS_MAXNAMLEN 255
#endif

/* Clean-up can either mean packing files together toward the end of the file
 * or, if file are deleted at the end of the file, clean up can simply mean
 * erasing the end of FLASH memory so that it can be re-used again.  However,
 * doing this can also harm the life of the FLASH part because it can mean
 * that the tail end of the FLASH is re-used too often.
 *
 * This threshold determines if/when it is worth erased the tail end of FLASH
 * and making it available for re-use (and possible over-wear).
 */

#ifndef CONFIG_NXFFS_TAILTHRESHOLD
#  define CONFIG_NXFFS_TAILTHRESHOLD (8*1024)
#endif

/* At present, only a single pre-allocated NXFFS volume is supported.  This
 * is because here can be only a single NXFFS volume mounted at any time.
 * This has to do with the fact that we bind to an MTD driver (instead of a
 * block driver) and bypass all of the normal mount operations.
 */

#undef CONFIG_NXFSS_PREALLOCATED
#define CONFIG_NXFSS_PREALLOCATED 1

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: nxffs_initialize
 *
 * Description:
 *   Initialize to provide NXFFS on an MTD interface
 *
 * Input Parameters:
 *   mtd - The MTD device that supports the FLASH interface.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

EXTERN int nxffs_initialize(FAR struct mtd_dev_s *mtd);

/****************************************************************************
 * Name: nxffs_dump
 *
 * Description:
 *   Dump a summary of the contents of an NXFFS file system.  CONFIG_DEBUG
 *   and CONFIG_DEBUG_FS must be enabled for this function to do anything.
 *
 * Input Parameters:
 *   mtd - The MTD device that provides the interface to NXFFS-formatted
 *     media.
 *   verbose - FALSE: only show errors
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

EXTERN int nxffs_dump(FAR struct mtd_dev_s *mtd, bool verbose);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_FS_NXFFS_H */
