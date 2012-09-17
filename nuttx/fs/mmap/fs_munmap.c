/****************************************************************************
 * fs/mmap/fs_munmap.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
#include <sys/mman.h>

#include <stdint.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/kmalloc.h>

#include "fs_internal.h"
#include "fs_rammap.h"

#ifdef CONFIG_FS_RAMMAP

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Name: munmap
 *
 * Description:
 *
 *   munmap() system call deletes mappings for the specified address range.
 *   All memory starting with 'start' and continuing for a length of 'length'
 *   bytes are removed.
 *
 *   NuttX operates in a flat open address space.  Therefore, it generally
 *   does not require mmap() and, hence, munmap functionality.  There are
 *   two exceptions where mmap() is available:
 *
 *   1. mmap() is the API that is used to support direct access to random
 *     access media under the following very restrictive conditions:
 *
 *     a. The filesystem supports the FIOC_MMAP ioctl command.  Any file
 *        system that maps files contiguously on the media should support
 *        this ioctl. (vs. file system that scatter files over the media
 *        in non-contiguous sectors).  As of this writing, ROMFS is the
 *        only file system that meets this requirement.
 *     b. The underlying block driver supports the BIOC_XIPBASE ioctl
 *        command that maps the underlying media to a randomly accessible
 *        address. At  present, only the RAM/ROM disk driver does this.
 *
 *     munmap() is still not required in this first case.  In this first
 *     The mapped address is a static address in the MCUs address space
 *     does not need to be munmapped.  Support for munmap() in this case
 *     provided by the simple definition in sys/mman.h:
 *
 *        #define munmap(start, length)
 *
 *   2. If CONFIG_FS_RAMMAP is defined in the configuration, then mmap() will
 *      support simulation of memory mapped files by copying files whole
 *      into RAM.  munmap() is required in this case to free the allocated
 *      memory holding the shared copy of the file.
 *
 * Parameters:
 *   start   The start address of the mapping to delete.  For this
 *           simplified munmap() implementation, the *must* be the start
 *           address of the memory region (the same address returned by
 *           mmap()).
 *   length  The length region to be umapped.
 *
 * Returned Value:
 *   On success, munmap() returns 0, on failure -1, and errno is set
 *   (probably to EINVAL).
 *
 ****************************************************************************/

int munmap(FAR void *start, size_t length)
{
  FAR struct fs_rammap_s *prev;
  FAR struct fs_rammap_s *curr;
  FAR void *newaddr;
  unsigned int offset;
  int ret;
  int err;

  /* Find a region containing this start and length in the list of regions */

  rammap_initialize();
  ret = sem_wait(&g_rammaps.exclsem);
  if (ret < 0)
    {
      return ERROR;
    }

  /* Seach the list of regions */

  for (prev = NULL, curr = g_rammaps.head; curr; prev = curr, curr = curr->flink)
    {
      /* Does this region include any part of the specified range? */

      if ((uintptr_t)start < (uintptr_t)curr->addr + curr->length &&
          (uintptr_t)start + length >= (uintptr_t)curr->addr)
        {
          break;
        }
    }

  /* Did we find the region */

  if (!curr)
    {
      fdbg("Region not found\n");
      err = EINVAL;
      goto errout_with_semaphore;
    }

  /* Get the offset from the beginning of the region and the actual number
   * of bytes to "unmap".  All mappings must extend to the end of the region.
   * There is no support for free a block of memory but leaving a block of
   * memory at the end.  This is a consequence of using realloc() to
   * simulate the unmapping.
   */

  offset = start - curr->addr;
  if (offset + length < curr->length)
    {
      fdbg("Cannot umap without unmapping to the end\n");
      err = ENOSYS;
      goto errout_with_semaphore;
    }

  /* Okay.. the region is beging umapped to the end.  Make sure the length
   * indicates that.
   */

  length = curr->length - offset;

  /* Are we unmapping the entire region (offset == 0)? */

  if (length >= curr->length)
    {
      /* Yes.. remove the mapping from the list */

      if (prev)
        {
          prev->flink = curr->flink;
        }
      else
        {
          g_rammaps.head = curr->flink;
        }

      /* Then free the region */

      kfree(curr);
    }

  /* No.. We have been asked to "unmap' only a portion of the memory
   * (offset > 0).
   */

  else
    {
      newaddr = krealloc(curr->addr, sizeof(struct fs_rammap_s) + length);
      DEBUGASSERT(newaddr == (FAR void*)(curr->addr));
      curr->length = length;
    }

  sem_post(&g_rammaps.exclsem);
  return OK;

errout_with_semaphore:
  sem_post(&g_rammaps.exclsem);
  errno = err;
  return ERROR;
}

#endif /* CONFIG_FS_RAMMAP */
