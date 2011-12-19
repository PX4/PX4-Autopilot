/****************************************************************************
 * drivers/bch/bchlib_write.c
 *
 *   Copyright (C) 2008-2009, 2011 Gregory Nutt. All rights reserved.
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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/fs.h>

#include "bch_internal.h"

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
 * Name: bchlib_write
 *
 * Description:
 *   Write to the block device set-up by bchlib_setup as if it were a character
 *   device.
 *
 ****************************************************************************/

ssize_t bchlib_write(FAR void *handle, FAR const char *buffer, size_t offset, size_t len)
{
  FAR struct bchlib_s *bch = (FAR struct bchlib_s *)handle;
  size_t   nsectors;
  size_t   sector;
  uint16_t sectoffset;
  size_t   nbytes;
  size_t   byteswritten;
  int      ret;

  /* Get rid of this special case right away */

  if (len < 1)
    {
      return 0;
    }

  /* Convert the file position into a sector number an offset. */

  sector     = offset / bch->sectsize;
  sectoffset = offset - sector * bch->sectsize;

  if (sector >= bch->nsectors)
    {
      return -EFBIG;
    }

  /* Write the initial partial sector */

  byteswritten = 0;
  if (sectoffset > 0)
    {
      /* Read the full sector into the sector buffer */

      bchlib_readsector(bch, sector);

      /* Copy the tail end of the sector from the user buffer */

      if (sectoffset + len > bch->sectsize)
        {
          nbytes = bch->sectsize - sectoffset;
        }
      else 
        {
          nbytes = len;
        }

      memcpy(&bch->buffer[sectoffset], buffer, nbytes);
      bch->dirty = true;

      /* Adjust pointers and counts */

      sectoffset    = 0;
      sector++;

      if (sector >= bch->nsectors)
        {
          return nbytes;
        }

      byteswritten  = nbytes;
      buffer       += nbytes;
      len          -= nbytes;
    }

  /* Then write all of the full sectors following the partial sector
   * directly from the user buffer.
   */

  if (len >= bch->sectsize )
    {
      nsectors = len / bch->sectsize;
      if (sector + nsectors > bch->nsectors)
        {
          nsectors = bch->nsectors - sector;
        }

      /* Write the contiguous sectors */

      ret = bch->inode->u.i_bops->write(bch->inode, (FAR uint8_t *)buffer,
                                        sector, nsectors);
      if (ret < 0)
        {
          fdbg("Write failed: %d\n", ret);
          return ret;
        }

      /* Adjust pointers and counts */

      sectoffset    = 0;
      sector       += nsectors;

      nbytes        = nsectors * bch->sectsize;
      byteswritten += nbytes;

      if (sector >= bch->nsectors)
        {
          return byteswritten;
        }

      buffer    += nbytes;
      len       -= nbytes;
    }

  /* Then write any partial final sector */

  if (len > 0)
    {
      /* Read the sector into the sector buffer */

      bchlib_readsector(bch, sector);

      /* Copy the head end of the sector from the user buffer */

      memcpy(bch->buffer, buffer, len);
      bch->dirty = true;

      /* Adjust counts */

      byteswritten += len;
    }

  /* Finally, flush any cached writes to the device as well */

  ret = bchlib_flushsector(bch);
  if (ret < 0)
    {
      fdbg("Flush failed: %d\n", ret);
      return ret;
    }

  return byteswritten;
}

