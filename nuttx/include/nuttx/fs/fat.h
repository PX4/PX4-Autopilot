/****************************************************************************
 * include/nuttx/fs/fat.h
 *
 *   Copyright (C) 2007-2009, 2012 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_FS_FAT_H
#define __INCLUDE_NUTTX_FS_FAT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* File attribute bits in FAT directory entry */

#define FATATTR_READONLY  0x01
#define FATATTR_HIDDEN    0x02
#define FATATTR_SYSTEM    0x04
#define FATATTR_VOLUMEID  0x08
#define FATATTR_DIRECTORY 0x10
#define FATATTR_ARCHIVE   0x20

#define FATATTR_LONGNAME \
  (FATATTR_READONLY|FATATTR_HIDDEN|FATATTR_SYSTEM|FATATTR_VOLUMEID)

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

typedef uint8_t fat_attrib_t;

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
 * Name: fat_getattrib and fat_setattrib
 *
 * Description:
 *   Non-standard functions to get and set FAT file/directory attributes
 *
 ****************************************************************************/

EXTERN int fat_getattrib(const char *path, fat_attrib_t *attrib);
EXTERN int fat_setattrib(const char *path, fat_attrib_t setbits, fat_attrib_t clearbits);

/****************************************************************************
 * Name: fat_dma_alloc and fat_dma_free
 *
 * Description:
 *   The FAT file system allocates two I/O buffers for data transfer, each
 *   are the size of one device sector.  One of the buffers is allocated
 *   once for each FAT volume that is mounted; the other buffers are
 *   allocated each time a FAT file is opened.
 *
 *   Some hardware, however, may require special DMA-capable memory in
 *   order to perform the the transfers.  If CONFIG_FAT_DMAMEMORY is defined
 *   then the architecture-specific hardware must provide the funtions
 *   fat_dma_alloc() and fat_dma_free() as prototyped below:  fat_dma_alloc()
 *   will allocate DMA-capable memory of the specified size; fat_dma_free()
 *   is the corresponding function that will be called to free the DMA-
 *   capable memory.
 *
 *   This functions may be simple wrappers around gran_alloc() and gran_free()
 *   (See nuttx/gran.h).
 *
 ****************************************************************************/

#ifdef CONFIG_FAT_DMAMEMORY
EXTERN FAR void *fat_dma_alloc(size_t size);
EXTERN void fat_dma_free(FAR void *memory, size_t size);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_FS_FAT_H */
