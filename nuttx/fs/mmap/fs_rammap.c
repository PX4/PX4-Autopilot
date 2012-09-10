/****************************************************************************
 * fs/mmap/fs_rammmap.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>

#include "fs_internal.h"
#include "fs_rammap.h"

#ifdef CONFIG_FS_RAMMAP

/****************************************************************************
 * Global Data
 ****************************************************************************/

/* This is the list of all mapped files */

struct fs_allmaps_s g_rammaps;

/****************************************************************************
 * Global Functions
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

void rammap_initialize(void)
{
  if (!g_rammaps.initialized)
    {
      sem_init(&g_rammaps.exclsem, 0, 1);
      g_rammaps.initialized = true;
    }
}

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

FAR void *rammap(int fd, size_t length, off_t offset)
{
  FAR struct fs_rammap_s *map;
  FAR uint8_t *alloc;
  FAR uint8_t *rdbuffer;
  ssize_t nread;
  off_t fpos;
  int err;
  int ret;

  /* There is a major design flaw that I have not yet thought of fix for:
   * The goal is to have a single region of memory that represents a single
   * file and can be shared by many threads.  That is, given a filename a
   * thread should be able to open the file, get a file descriptor, and
   * call mmap() to get a memory region.  Different file descriptors opened
   * with the same file path should get the same memory region when mapped.
   *
   * The design flaw is that I don't have sufficient knowledge to know that
   * these different file descriptors map to the same file.  So, for the time
   * being, a new memory region is created each time that rammap() is called.
   * Not very useful!
   */

  /* Allocate a region of memory of the specified size */

  alloc = (FAR uint8_t *)kmalloc(sizeof(struct fs_rammap_s) + length);
  if (!alloc)
    {
      fdbg("Region allocation failed, length: %d\n", (int)length);
      err = ENOMEM;
      goto errout;
    }

  /* Initialize the region */

  map         = (FAR struct fs_rammap_s *)alloc;
  memset(map, 0, sizeof(struct fs_rammap_s));
  map->addr   = alloc + sizeof(struct fs_rammap_s);
  map->length = length;
  map->offset = offset;

  /* Seek to the specified file offset */

  fpos = lseek(fd, offset,  SEEK_SET);
  if (fpos == (off_t)-1)
    {
      /* Seek failed... errno has already been set, but EINVAL is probably
       * the correct response.
       */

      fdbg("Seek to position %d failed\n", (int)offset);
      err = EINVAL;
      goto errout_with_region;
    }

  /* Read the file data into the memory region */

  rdbuffer = map->addr;
  while (length > 0)
    {
      nread = read(fd, rdbuffer, length);
      if (nread < 0)
        {
           /* Handle the special case where the read was interrupted by a
            * signal.
            */

           err = get_errno();
           if (err != EINTR)
             {
               /* All other read errors are bad.  errno is already set.
                * (but maybe should be forced to EINVAL?).  NOTE that if
                * FS DEBUG is enabled, then the following fdbg() macro will
                * destroy the errno value.
                */

               fdbg("Read failed: offset=%d errno=%d\n", (int)offset, err);
#ifdef CONFIG_DEBUG_FS
               goto errout_with_region;
#else
               goto errout_with_errno;
#endif
             }
        }

      /* Check for end of file. */

      if (nread == 0)
        {
          break;
        }

      /* Increment number of bytes read */

      rdbuffer += nread;
      length   -= nread;
    }

  /* Zero any memory beyond the amount read from the file */

  memset(rdbuffer, 0, length);

  /* Add the buffer to the list of regions */

  rammap_initialize();
  ret = sem_wait(&g_rammaps.exclsem);
  if (ret < 0)
    {
      goto errout_with_errno;
    }

  map->flink  = g_rammaps.head;
  g_rammaps.head = map;

  sem_post(&g_rammaps.exclsem);
  return map->addr;

errout_with_region:
  kfree(alloc);
errout:
  set_errno(err);
  return MAP_FAILED;

errout_with_errno:
  kfree(alloc);
  return MAP_FAILED;
}

#endif /* CONFIG_FS_RAMMAP */
