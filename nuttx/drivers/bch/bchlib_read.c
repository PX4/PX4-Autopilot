/****************************************************************************
 * drivers/bch/bchlib_read.c
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

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/fs/fs.h>

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
 * Name: bchlib_read
 *
 * Description:
 *   Read from the block device set-up by bchlib_setup as if it were a character
 *   device.
 *
 ****************************************************************************/

ssize_t bchlib_read(FAR void *handle, FAR char *buffer, size_t offset, size_t len)
{
  FAR struct bchlib_s *bch = (FAR struct bchlib_s *)handle;
  size_t   nsectors;
  size_t   sector;
  uint16_t sectoffset;
  size_t   nbytes;
  size_t   bytesread;
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
      /* Return end-of-file */

      return 0;
    }

  /* Read the initial partial sector */

  bytesread = 0;
  if (sectoffset > 0)
    {
      /* Read the sector into the sector buffer */

      bchlib_readsector(bch, sector);

      /* Copy the tail end of the sector to the user buffer */

      if (sectoffset + len > bch->sectsize)
        {
          nbytes = bch->sectsize - sectoffset;
        }
      else 
        {
          nbytes = len;
        }

      memcpy(buffer, &bch->buffer[sectoffset], nbytes);

      /* Adjust pointers and counts */

      sectoffset = 0;
      sector++;

      if (sector >= bch->nsectors)
        {
          return nbytes;
        }

      bytesread  = nbytes;
      buffer    += nbytes;
      len       -= nbytes;
    }

  /* Then read all of the full sectors following the partial sector directly
   * into the user buffer.
   */

  if (len >= bch->sectsize )
    {
      nsectors = len / bch->sectsize;
      if (sector + nsectors > bch->nsectors)
        {
          nsectors = bch->nsectors - sector;
        }

      ret = bch->inode->u.i_bops->read(bch->inode, (FAR uint8_t *)buffer,
                                       sector, nsectors);
      if (ret < 0)
        {
          fdbg("Read failed: %d\n");
          return ret;
        }

      /* Adjust pointers and counts */

      sectoffset = 0;
      sector    += nsectors;

      nbytes     = nsectors * bch->sectsize;
      bytesread += nbytes;

      if (sector >= bch->nsectors)
        {
          return bytesread;
        }

      buffer    += nbytes;
      len       -= nbytes;
    }

  /* Then read any partial final sector */

  if (len > 0)
    {
      /* Read the sector into the sector buffer */

      bchlib_readsector(bch, sector);

      /* Copy the head end of the sector to the user buffer */

      memcpy(buffer, bch->buffer, len);

      /* Adjust counts */

      bytesread += len;
    }

  return bytesread;
}
