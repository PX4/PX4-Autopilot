/****************************************************************************
 * fs/nxffs/nxffs_blockstats.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/mtd.h>

#include "nxffs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxffs_blockstats
 *
 * Description:
 *   Analyze the NXFFS volume.  This operation must be performed when the
 *   volume is first mounted in order to detect if the volume has been
 *   formatted and contains a usable NXFFS file system.
 *
 * Input Parameters:
 *   volume - Describes the current NXFFS volume.
 *   stats  - On return, will hold nformation describing the state of the
 *     volume.
 *
 * Returned Value:
 *   Negated errnos are returned only in the case of MTD reported failures.
 *   Nothing in the volume data itself will generate errors.
 *
 ****************************************************************************/

int nxffs_blockstats(FAR struct nxffs_volume_s *volume,
                     FAR struct nxffs_blkstats_s *stats)
{
  FAR uint8_t *bptr;     /* Pointer to next block data */
  off_t ioblock;         /* I/O block number */
  int lblock;            /* Logical block index */
  int ret;

  /* Process each erase block */

  memset(stats, 0, sizeof(struct nxffs_blkstats_s));

  for (ioblock = 0; ioblock < volume->nblocks; ioblock += volume->blkper)
    {
      /* Read the full erase block */

      ret = MTD_BREAD(volume->mtd, ioblock, volume->blkper, volume->pack);
      if (ret < volume->blkper)
        {
          fdbg("Failed to read erase block %d: %d\n", ioblock / volume->blkper, -ret);
          return ret;
        }

      /* Process each logical block */

      for (bptr = volume->pack, lblock = 0;
           lblock < volume->blkper;
           bptr += volume->geo.blocksize, lblock++)
        {
          FAR struct nxffs_block_s *blkhdr = (FAR struct nxffs_block_s*)bptr;

          /* Collect statistics */

          stats->nblocks++;
          if (memcmp(blkhdr->magic, g_blockmagic, NXFFS_MAGICSIZE) != 0)
            {
              stats->nunformat++;
            }
          else if (blkhdr->state == BLOCK_STATE_BAD)
            {
              stats->nbad++;
            }
          else if (blkhdr->state == BLOCK_STATE_GOOD)
            {
             stats-> ngood++;
            }
          else
            {
              stats->ncorrupt++;
            }
        }
    }

  fdbg("Number blocks:        %d\n", stats->nblocks);
  fdbg("  Good blocks:        %d\n", stats->ngood);
  fdbg("  Bad blocks:         %d\n", stats->nbad);
  fdbg("  Unformatted blocks: %d\n", stats->nunformat);
  fdbg("  Corrupt blocks:     %d\n", stats->ncorrupt);
  return OK;
}


