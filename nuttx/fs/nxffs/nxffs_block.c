/****************************************************************************
 * fs/nxffs/nxffs_block.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/mtd.h>

#include "nxffs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
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
 * Name: nxffs_verifyblock
 *
 * Description:
 *   Assure the the provided (logical) block number is in the block cache
 *   and that it has a valid block header (i.e., proper magic and
 *   marked good)
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume
 *   block - The (logical) block number to load and verify.
 *
 * Returned Values:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned indicating the nature of the failure.  -ENOENT is returned
 *   if the block is a bad block.
 *
 ****************************************************************************/

int nxffs_verifyblock(FAR struct nxffs_volume_s *volume, off_t block)
{
  FAR struct nxffs_block_s *blkhdr;
  int ret;

  /* Make sure the the block is in the cache */

  ret = nxffs_rdcache(volume, block);
  if (ret < 0)
    {
      /* Perhaps we are at the end of the media */

      fvdbg("Failed to read data into cache: %d\n", ret);
      return ret;
    }

  /* Check if the block has a magic number (meaning that it is not
   * erased) and that it is valid (meaning that it is not marked
   * for deletion)
   */

  blkhdr = (FAR struct nxffs_block_s *)volume->cache;
  if (memcmp(blkhdr->magic, g_blockmagic, NXFFS_MAGICSIZE) == 0)
    {
       /* This does appear to be a block */

      if (blkhdr->state == BLOCK_STATE_GOOD)
        {
          /* The block is valid */

          return OK;
        }
      else if (blkhdr->state == BLOCK_STATE_BAD)
        {
          /* -ENOENT is a special indication that this is a properly marked
           * bad block
           */

          return -ENOENT;
        }
    }

  /* Whatever is here where a block header should be is invalid */

  return -EINVAL;
}

/****************************************************************************
 * Name: nxffs_validblock
 *
 * Description:
 *   Find the next valid (logical) block in the volume.
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume
 *   block  - On entry, this provides the starting block number.  If the
 *     function is succesfful, then this memory location will hold the
 *     block number of the next valid block on return.
 *
 *  Returned Value:
 *    Zero on success otherwise a negated errno value indicating the nature
 *    of the failure.
 *
 ****************************************************************************/

int nxffs_validblock(struct nxffs_volume_s *volume, off_t *block)
{
  off_t i;
  int ret;

  DEBUGASSERT(volume && block);

  /* Loop for each possible block or until a valid block is found */

  for (i = *block; i < volume->nblocks; i++)
    {
      /* Loop until we find a valid block */

      ret = nxffs_verifyblock(volume, i);
      if (ret == OK)
        {
          /* We found it, return the block number */

          *block = i;
          return OK;
        }
    }

  /* ENOSPC is special return value that means that there is no further,
   * valid blocks left in the volume.
   */

  fdbg("No valid block found\n");
  return -ENOSPC;
}
