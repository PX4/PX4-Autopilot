/****************************************************************************
 * fs/nxffs/nxffs_dump.c
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
#include <debug.h>
#include <errno.h>
#include <crc32.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd.h>

#include "nxffs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Re-define fdbg so that the output does not have so much diagnostic info.
 * This should still, however, always agree with the defintion in debug.h.
 */

#undef  fdbg
#define fdbg syslog

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nxffs_blkinfo_s
{
  struct mtd_geometry_s geo;
  FAR uint8_t *buffer;
  off_t nblocks;
  off_t block;
  off_t offset;
  bool verbose;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char g_hdrformat[] = "  BLOCK:OFFS  TYPE  STATE   LENGTH\n";
static const char g_format[]    = "  %5d:%-5d %s %s %5d\n";

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxffs_analyzeinode
 *
 * Description:
 *   Analyze one candidate inode found in the block.
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_FS)
static inline ssize_t nxffs_analyzeinode(FAR struct nxffs_blkinfo_s *blkinfo,
                                         int offset)
{
  FAR struct nxffs_inode_s inode;
  off_t nextblock;
  uint8_t  state;
  uint32_t noffs;
  uint32_t doffs;
  uint32_t utc;
  uint32_t ecrc;
  uint32_t datlen;
  uint32_t crc;
  size_t   spaceleft;

  /* Verify that there is space for an inode header remaining in the block */

  if (offset + SIZEOF_NXFFS_INODE_HDR > blkinfo->geo.blocksize)
    {
      /* No.. then this can't be an inode header */

      return ERROR;
    }

  /* Unpack the header */

  memcpy(&inode, &blkinfo->buffer[offset], SIZEOF_NXFFS_INODE_HDR);
  noffs  = nxffs_rdle32(inode.noffs);
  doffs  = nxffs_rdle32(inode.doffs);
  utc    = nxffs_rdle32(inode.utc);
  ecrc   = nxffs_rdle32(inode.crc);
  datlen = nxffs_rdle32(inode.datlen);

  /* Misc. sanity checks */

  if (noffs < blkinfo->offset + offset + SIZEOF_NXFFS_BLOCK_HDR)
    {
      /* The name begins before the inode header.  This can't can't be
       * a real inode header (or it is a corrupted one).
       */

      return ERROR;
    }
  

  /* Can we verify the inode?  We need to have the inode name in the same
   * block to do that (or get access to the next block)
   */

  if (doffs < blkinfo->offset + offset + SIZEOF_NXFFS_BLOCK_HDR)
    {
      /* The first data block begins before the inode header.  This can't can't
       * be a real inode header (or it is a corrupted one).
       */

      return ERROR;
    }

   spaceleft = (blkinfo->nblocks - blkinfo->block) * blkinfo->geo.blocksize;
   spaceleft -= (offset + SIZEOF_NXFFS_BLOCK_HDR);
   if (datlen > spaceleft)
     {
       /* The data length is greater than what would fit in the rest of FLASH
        * (even ignoring block and data header sizes.
        */

       return ERROR;
     }

  /* The name begins after the inode header.  Does it begin in this block? */

  nextblock = blkinfo->offset + blkinfo->geo.blocksize;
  if (noffs > nextblock)
    {
      /* Not than we cannot verify the inode header */

      if (blkinfo->verbose)
        {
          fdbg(g_format, blkinfo->block, offset, "INODE", "UNVERFD", datlen);
        }
      return ERROR;
    }

  /* The name begins in this block.  Does it also end in this block? */

  if (noffs + inode.namlen > nextblock)
    {
      /* No..  Assume that this is not an inode. */

       return ERROR;
    }

  /* Calculate the CRC */

  state       = inode.state;
  inode.state = CONFIG_NXFFS_ERASEDSTATE;
  nxffs_wrle32(inode.crc, 0);

  crc = crc32((FAR const uint8_t *)&inode, SIZEOF_NXFFS_INODE_HDR);
  crc = crc32part(&blkinfo->buffer[noffs - blkinfo->offset], inode.namlen, crc);

  if (crc != ecrc)
   {
      fdbg(g_format, blkinfo->block, offset, "INODE", "CRC BAD", datlen);
      return ERROR;
   }

  /* If must be a good header */

  if (state == INODE_STATE_FILE)
    {
      if (blkinfo->verbose)
        {
          fdbg(g_format, blkinfo->block, offset, "INODE", "OK     ", datlen);
        }
    }
  else if (state == INODE_STATE_DELETED)
    {
      if (blkinfo->verbose)
        {
          fdbg(g_format, blkinfo->block, offset, "INODE", "DELETED", datlen);
        }
    }
  else
    {
      fdbg(g_format, blkinfo->block, offset, "INODE", "CORRUPT", datlen);
    }

  /* Return the block-relative offset to the next byte after the inode name */

  return noffs + inode.namlen - offset - blkinfo->offset;
}
#endif

/****************************************************************************
 * Name: nxffs_analyzedata
 *
 * Description:
 *   Analyze one candidate inode found in the block.
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_FS)
static inline ssize_t nxffs_analyzedata(FAR struct nxffs_blkinfo_s *blkinfo,
                                        int offset)
{
  struct nxffs_data_s dathdr;
  uint32_t ecrc;
  uint16_t datlen;
  uint32_t crc;

  /* Copy and unpack the data block header */

  memcpy(&dathdr, &blkinfo->buffer[offset], SIZEOF_NXFFS_DATA_HDR);
  ecrc   = nxffs_rdle32(dathdr.crc);
  datlen = nxffs_rdle16(dathdr.datlen);

  /* Sanity checks */

  if (offset + SIZEOF_NXFFS_DATA_HDR + datlen > blkinfo->geo.blocksize)
    {
      /* Data does not fit in within the block, this can't be a data block */

      return ERROR;
    }

  /* Calculate the CRC */

  nxffs_wrle32(dathdr.crc, 0);

  crc = crc32((FAR const uint8_t *)&dathdr, SIZEOF_NXFFS_DATA_HDR);
  crc = crc32part(&blkinfo->buffer[offset + SIZEOF_NXFFS_DATA_HDR], datlen, crc);

  if (crc != ecrc)
   {
      fdbg(g_format, blkinfo->block, offset, "DATA ", "CRC BAD", datlen);
      return ERROR;
   }

  /* If must be a good header */

  if (blkinfo->verbose)
    {
      fdbg(g_format, blkinfo->block, offset, "DATA ", "OK     ", datlen);
    }
  return SIZEOF_NXFFS_DATA_HDR + datlen;
}
#endif

/****************************************************************************
 * Name: nxffs_analyze
 *
 * Description:
 *   Analyze the content of one block.
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_FS)
static inline void nxffs_analyze(FAR struct nxffs_blkinfo_s *blkinfo)
{
  FAR struct nxffs_block_s *blkhdr;
  ssize_t nbytes;
  int hdrndx;
  int datndx;
  int inndx;
  int i;

  /* Verify that there is a header on the block */

  blkhdr = (FAR struct nxffs_block_s *)blkinfo->buffer;
  if (memcmp(blkhdr->magic, g_blockmagic, NXFFS_MAGICSIZE) != 0)
    {
      fdbg(g_format, blkinfo->block, 0, "BLOCK", "NO FRMT",
           blkinfo->geo.blocksize);
    }
  else if (blkhdr->state == BLOCK_STATE_GOOD)
    {
      size_t datsize = blkinfo->geo.blocksize - SIZEOF_NXFFS_BLOCK_HDR;
      size_t nerased = nxffs_erased(blkinfo->buffer + SIZEOF_NXFFS_BLOCK_HDR, datsize);
      if (nerased == datsize)
        {
          if (blkinfo->verbose)
            {
              fdbg(g_format, blkinfo->block, 0, "BLOCK", "ERASED ",
                   blkinfo->geo.blocksize);
            }
          return;
        }
#if 0 /* Too much output, to little information */
      else
        {
          fdbg(g_format, blkinfo->block, 0, "BLOCK", "IN USE ",
               blkinfo->geo.blocksize);
        }
#endif
    }
  else if (blkhdr->state == BLOCK_STATE_BAD)
    {
      fdbg(g_format, blkinfo->block, 0, "BLOCK", "BAD    ",
           blkinfo->geo.blocksize);
    }
  else
    {
      fdbg(g_format, blkinfo->block, 0, "BLOCK", "CORRUPT",
           blkinfo->geo.blocksize);
    }

  /* Serach for Inode and data block headers.  */

  inndx = 0;
  datndx = 0;

  for (i = SIZEOF_NXFFS_BLOCK_HDR; i < blkinfo->geo.blocksize; i++)
    {
      uint8_t ch = blkinfo->buffer[i];

      if (ch == g_inodemagic[inndx])
        {
          inndx++;
          datndx = 0;

          if (inndx == NXFFS_MAGICSIZE)
            {
              hdrndx = i - NXFFS_MAGICSIZE + 1;
              nbytes = nxffs_analyzeinode(blkinfo, hdrndx);
              if (nbytes > 0)
                {
                  i = hdrndx + nbytes - 1;
                }
              inndx = 0;
            }
        }
      else if (ch == g_datamagic[datndx])
        {
          datndx++;
          inndx = 0;

          if (datndx == NXFFS_MAGICSIZE)
            {
              hdrndx = i - NXFFS_MAGICSIZE + 1;
              nbytes = nxffs_analyzedata(blkinfo, hdrndx);
              if (nbytes > 0)
                {
                  i = hdrndx + nbytes - 1;
                }
              datndx = 0;
            }
        }
    }

}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

int nxffs_dump(FAR struct mtd_dev_s *mtd, bool verbose)
{
#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_FS)
  struct nxffs_blkinfo_s blkinfo;
  int ret;

  /* Get the volume geometry. (casting to uintptr_t first eliminates
   * complaints on some architectures where the sizeof long is different
   * from the size of a pointer).
   */

  memset(&blkinfo, 0, sizeof(struct nxffs_blkinfo_s));
  ret = MTD_IOCTL(mtd, MTDIOC_GEOMETRY, (unsigned long)((uintptr_t)&blkinfo.geo));
  if (ret < 0)
    {
      fdbg("ERROR: MTD ioctl(MTDIOC_GEOMETRY) failed: %d\n", -ret);
      goto errout;
    }

  /* Save the verbose output indication */

  blkinfo.verbose = verbose;

  /* Allocate a buffer to hold one block */

  blkinfo.buffer = (FAR uint8_t *)kmalloc(blkinfo.geo.blocksize);
  if (!blkinfo.buffer)
    {
      fdbg("ERROR: Failed to allocate block cache\n");
      ret = -ENOMEM;
      goto errout;
    }

  /* Now read every block on the device */

  fdbg("NXFFS Dump:\n");
  fdbg(g_hdrformat);

  blkinfo.nblocks = blkinfo.geo.erasesize * blkinfo.geo.neraseblocks / blkinfo.geo.blocksize;
  for (blkinfo.block = 0, blkinfo.offset = 0;
       blkinfo.block < blkinfo.nblocks;
       blkinfo.block++, blkinfo.offset += blkinfo.geo.blocksize)
    {
      /* Read the next block */

      ret = MTD_BREAD(mtd, blkinfo.block, 1, blkinfo.buffer);
      if (ret < 0)
        {
          fdbg("ERROR: Failed to read block %d\n", blkinfo.block);
          goto errout_with_block;
        }

      /* Analyze the block */

      nxffs_analyze(&blkinfo);
    }
  fdbg("%d blocks analyzed\n", blkinfo.nblocks);

errout_with_block:
  kfree(blkinfo.buffer);
errout:
  return ret;

#else
  return -ENOSYS;
#endif
}
