/****************************************************************************
 * fs/mmap/rammap.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References: Linux/Documentation/filesystems/romfs.txt
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

#ifndef __FS_MMAP_RAMMAP_H
#define __FS_MMAP_RAMMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <semaphore.h>

#ifdef CONFIG_FS_RAMMAP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure describes one file that has been copied to memory and
 * managed as a share-able "memory mapped" file.  This functionality is
 * intended to provide a substitute for memory mapped files for architectures
 * that do not have MMUs and, hence, cannot support on demand paging of
 * blocks of a file.
 *
 * This copied file has many of the properties of a standard memory mapped
 * file except:
 *
 * - All of the file must be present in memory.  This limits the size of
 *   files that may be memory mapped (especially on MCUs with no significant
 *   RAM resources).
 * - All mapped files are read-only.  You can write to the in-memory image,
 *   but the file contents will not change.
 * - There are not access privileges.
 */

struct fs_rammap_s
{
  struct fs_rammap_s *flink;       /* Implements a singly linked list */
  FAR void           *addr;        /* Start of allocated memory */
  size_t              length;      /* Length of region */
  off_t               offset;      /* File offset */
};

/* This structure defines all "mapped" files */

struct fs_allmaps_s
{
  bool                initialized; /* True: This structure has been initialized */
  sem_t               exclsem;     /* Provides exclusive access the list */
  struct fs_rammap_s *head;        /* List of mapped files */
};

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/* This is the list of all mapped files */

extern struct fs_allmaps_s g_rammaps;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rammap_initialize
 *
 * Description:
 *   Verified that this capability has been initialized.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

extern void rammap_initialize(void);

/****************************************************************************
 * Name: rammmap
 *
 * Description:
 *   Support simulation of memory mapped files by copying files into RAM.
 *
 * Parameters:
 *   fd      file descriptor of the backing file -- required.
 *   length  The length of the mapping.  For exception #1 above, this length
 *           ignored:  The entire underlying media is always accessible.
 *   offset  The offset into the file to map
 *
 * Returned Value:
 *   On success, rammmap() returns a pointer to the mapped area. On error, the
 *   value MAP_FAILED is returned, and errno is set  appropriately.
 *
 *     EBADF
 *      'fd' is not a valid file descriptor.
 *     EINVAL
 *       'length' or 'offset' are invalid
 *     ENOMEM
 *       Insufficient memory is available to map the file.
 *
 ****************************************************************************/

extern FAR void *rammap(int fd, size_t length, off_t offset);

#endif /* CONFIG_FS_RAMMAP */
#endif /* __FS_MMAP_RAMMAP_H */
