/****************************************************************************
 * fs/nxffs/nxffs_pack.c
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
#include <crc32.h>
#include <debug.h>

#include <nuttx/kmalloc.h>

#include "nxffs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/
/* This structure supports access to one inode data stream */

struct nxffs_packstream_s
{
  struct nxffs_entry_s entry;      /* Describes the inode header */
  off_t                fpos;       /* Current file position */
  off_t                blkoffset;  /* Offset to the current data block */
  uint16_t             blklen;     /* Size of this block */
  uint16_t             blkpos;     /* Position in block corresponding to fpos */
};

/* The structure supports the overall packing operation */

struct nxffs_pack_s
{
  /* These describe the source and destination streams */

  struct nxffs_packstream_s src;
  struct nxffs_packstream_s dest;

  /* These describe the state of the current contents of the (destination)
   * volume->pack buffer.
   */

  FAR uint8_t         *iobuffer;   /* I/O block start position */
  off_t                ioblock;    /* I/O block number */
  off_t                block0;     /* First I/O block number in the erase block */
  uint16_t             iooffset;   /* I/O block offset */
};

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxffs_getblock
 *
 * Description:
 *   Return the I/O block number that includes the provided offset.
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume
 *   offset - FLASH offset
 *
 * Returned Value:
 *   The I/O block number.
 *
 ****************************************************************************/

static off_t nxffs_getblock(FAR struct nxffs_volume_s *volume, off_t offset)
{
  return offset / volume->geo.blocksize;
}

/****************************************************************************
 * Name: nxffs_getoffset
 *
 * Description:
 *   Given an I/O block number return the block offset corresponding to the
 *   FLASH offset;
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume
 *   offset - FLASH offset
 *
 * Returned Value:
 *   The I/O block number.
 *
 ****************************************************************************/

static off_t nxffs_getoffset(FAR struct nxffs_volume_s *volume,
                             off_t offset, off_t block)
{
  return offset - block * volume->geo.blocksize;
}

/****************************************************************************
 * Name: nxffs_packtell
 *
 * Description:
 *   Report the current destination position in the pack buffer.
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume
 *   pack   - The volume packing state structure.
 *
 * Returned Value:
 *   The offset from the beginning of FLASH to the current seek position.
 *
 ****************************************************************************/

static off_t nxffs_packtell(FAR struct nxffs_volume_s *volume,
                            FAR struct nxffs_pack_s *pack)
{
  return pack->ioblock * volume->geo.blocksize + pack->iooffset;
}

/****************************************************************************
 * Name: nxffs_packvalid
 *
 * Description:
 *   Check if the current destination block is valid.
 *
 * Input Parameters:
 *   pack   - The volume packing state structure.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static inline bool nxffs_packvalid(FAR struct nxffs_pack_s *pack)
{
  FAR struct nxffs_block_s *blkhdr;

  blkhdr = (FAR struct nxffs_block_s *)pack->iobuffer;
  return (memcmp(blkhdr->magic, g_blockmagic, NXFFS_MAGICSIZE) == 0 &&
          blkhdr->state == BLOCK_STATE_GOOD);
}

/****************************************************************************
 * Name: nxffs_mediacheck
 *
 * Description:
 *   Verify that there is at least one valid block and at least one valid
 *   inode header on the media.  On successful return, the volume packing
 *   structure is initialized and contains the offset to the first valid
 *   inode header is returned.
 *
 * Input Parameters:
 *   volume - The volume to be packed.
 *   pack   - The volume packing state structure.
 *
 * Returned Values:
 *   The offset to the data area on the first valid block.  Zero is return
 *   if there are no valid blocks or if there are no valid inode headers
 *   after the first valid block.
 *
 ****************************************************************************/

static inline off_t nxffs_mediacheck(FAR struct nxffs_volume_s *volume,
                                     FAR struct nxffs_pack_s *pack)
{
  off_t froffset;
  int ret;

  /* Initialize the packing structure to all zero */

  memset(pack, 0, sizeof(struct nxffs_pack_s));

  /* Find the FLASH offset to the first valid block */

  volume->ioblock = 0;
  ret = nxffs_validblock(volume, &volume->ioblock);
  if (ret < 0)
   {
     /* No valid blocks?  Return offset zero. */

     return 0;
   }

  /* The offset to the free location to pack is then just after the block
   * header in this block.
   */

  volume->iooffset = SIZEOF_NXFFS_BLOCK_HDR;
  froffset         = nxffs_iotell(volume);

  /* Get the offset to the first valid inode entry after this free offset */

  ret = nxffs_nextentry(volume, froffset, &pack->src.entry);
  if (ret < 0)
    {
      /* No valid entries on the media -- Return offset zero */

     return 0;
    }

  /* Okay.. the start block and first entry have been found */

  return froffset;
}

/****************************************************************************
 * Name: nxffs_startpos
 *
 * Description:
 *   Find the position in FLASH memory where we should begin packing.  That
 *   position is the place where there is a gap between the last and the next
 *   valid inode.  On entry, the volume packing structure should be as it
 *   was initialized by nxffx_mediacheck.  on successful return, the volume
 *   packing state structure will be updated to begin the packing operation.
 *
 * Input Parameters:
 *   volume - The volume to be packed
 *   pack   - The volume packing state structure.
 *   froffset - On input, this is the location where we should be searching
 *     for the location to begin packing.  On successful return, froffset
 *     will be set the the offset in FLASH where the first inode should be
 *     copied to.  If -ENOSPC is returned -- meaning that the FLASH is full
 *     --  then no packing can be performed. In this case, then the free
 *     flash offset is returned through this location.
 *
 * Returned Values:
 *   Zero on success; Otherwise, a negated errno value is returned to
 *   indicate the nature of the failure.  If -ENOSPC is returned then the
 *   free FLASH offset is also returned.
 *
 ****************************************************************************/

static inline int nxffs_startpos(FAR struct nxffs_volume_s *volume,
                                 FAR struct nxffs_pack_s *pack,
                                 off_t *froffset)
{
  struct nxffs_blkentry_s blkentry;
  off_t offset = *froffset;
  off_t wasted;
  off_t nbytes;
  int ret;

  /* Loop until we find a gap of unused FLASH large enough to warrant
   * compacting.
   */

  for(;;)
    {
      /* Is there wasted space between the offset where the we could have
       * valid data and the offset to the beginning of the first valid
       * inode header?  NOTE:  The threshold check is not accurate, there
       * may or may not be intervening block headers making the separation
       * seem larger than it is.
       */

      DEBUGASSERT(pack->src.entry.hoffset >= offset);
      wasted = pack->src.entry.hoffset - offset;
      if (wasted > CONFIG_NXFFS_PACKTHRESHOLD)
        {
          /* This is where we must begin packing.  Describe the destination
           * inode header (only non-zero entries need to be initialized).
           */

          pack->dest.entry.name    = pack->src.entry.name;
          pack->dest.entry.utc     = pack->src.entry.utc;
          pack->dest.entry.datlen  = pack->src.entry.datlen;

          /* The destination entry now "owns" the name string */

          pack->src.entry.name     = NULL;

          /* Return the FLASH offset to the destination inode header */

          *froffset = offset;
          return OK;
        }

      /* Free the allocated memory in the entry */

      nxffs_freeentry(&pack->src.entry);

      /* Update the offset to the first byte at the end of the last data
       * block.
       */

      nbytes = 0;
      offset = pack->src.entry.doffset;

      while (nbytes < pack->src.entry.datlen)
        {
          /* Read the next data block header */

          ret = nxffs_nextblock(volume, offset, &blkentry);
          if (ret < 0)
            {
              fdbg("Failed to find next data block: %d\n", -ret);
              return ret;
            }

          /* Get the number of blocks and pointer to where the next
           * data block might lie.
           */

          nbytes += blkentry.datlen;
          offset  = blkentry.hoffset + SIZEOF_NXFFS_DATA_HDR + blkentry.datlen;
        }

      /* Make sure there is space at this location for an inode header */

      nxffs_ioseek(volume, offset);
      if (volume->iooffset + SIZEOF_NXFFS_INODE_HDR > volume->geo.blocksize)
        {
          /* No.. not enough space here. Find the next valid block */

          volume->ioblock++;
          ret = nxffs_validblock(volume, &volume->ioblock);
          if (ret < 0)
            {
               /* No valid blocks? Then there is nothing we can do.  Return
                * the end-of-flash indication.
                */

               *froffset = volume->froffset;
               return -ENOSPC;
            }

          volume->iooffset = SIZEOF_NXFFS_BLOCK_HDR;
          offset = nxffs_iotell(volume);
        }

      /* Get the offset to the next valid inode entry */

      ret = nxffs_nextentry(volume, offset, &pack->src.entry);
      if (ret < 0)
        {
          /* No more valid inode entries.  Just return an end-of-flash error
           * indication.  However, there could be many deleted inodes; set
           * volume->froffset to indicate the true free FLASH position.
           */

          *froffset = offset;
          return -ENOSPC;
        }
    }

  /* We won't get here */

  return -ENOSYS;
}

/****************************************************************************
 * Name: nxffs_srcsetup
 *
 * Description:
 *   Given a valid src inode, configure the src data stream.
 *
 * Input Parameters:
 *   volume - The volume to be packed
 *   pack   - The volume packing state structure.
 *   offset - FLASH offset to the data block header (will be zero for zero-
 *     files.
 *
 * Returned Values:
 *   Zero on success; Otherwise, a negated errno value is returned to
 *   indicate the nature of the failure.
 *
 ****************************************************************************/

static int nxffs_srcsetup(FAR struct nxffs_volume_s *volume,
                          FAR struct nxffs_pack_s *pack, off_t offset)
{
  /* Start with the first data block */

  pack->src.blkoffset = offset;
  pack->src.blkpos    = 0;

  /* Zero-length files have no valid data block offset */

  if (offset > 0)
    {
      /* Seek to the data block header, read and verify the block header */

      int ret = nxffs_rdblkhdr(volume, offset, &pack->src.blklen);
      if (ret < 0)
        {
          fdbg("Failed to verify the data block header: %d\n", -ret);
        }
      return ret;
    }

  DEBUGASSERT(pack->src.entry.datlen == 0);
  return OK;
}

/****************************************************************************
 * Name: nxffs_destsetup
 *
 * Description:
 *   Given a valid dest inode, configure the dest data stream.
 *
 * Input Parameters:
 *   volume - The volume to be packed
 *   pack   - The volume packing state structure.
 *
 * Returned Values:
 *   Zero on success; Otherwise, a negated errno value is returned to
 *   indicate the nature of the failure.
 *
 ****************************************************************************/

static int nxffs_destsetup(FAR struct nxffs_volume_s *volume,
                           FAR struct nxffs_pack_s *pack)
{
  size_t mindata;
  int    namlen;
  int    ret;

  /* The destination can be in one of three of states:
   *
   * State 1: The inode position was not yet been found.  This condition can
   * only occur on initial entry into nxffs_packblock() when there we no space
   * for the inode header at the end of the previous block.  We must now be
   * at the beginning of a shiny new I/O block, so we should always have
   * space for a new inode header right here.
   */

  if (pack->dest.entry.hoffset == 0)
    {
      /* Is there room for an inode structure in this block?  */

      if(pack->iooffset + SIZEOF_NXFFS_INODE_HDR > volume->geo.blocksize)
        {
          /* No.. that inode name will not fit in this block. Return an
           * indication that we are at the end of the block and try again
           * later.
           */

          return -ENOSPC;
        }

      /* The inode header will be placed at this position (but not until
       * we are finished.
       */

      pack->dest.entry.hoffset = nxffs_packtell(volume, pack);

      /* Make sure that the initialize state of the inode header memory is
       * erased.  This is important because we may not write to inode header
       * until it has already been written to FLASH.
       */

      memset(&pack->iobuffer[pack->iooffset], CONFIG_NXFFS_ERASEDSTATE,
             SIZEOF_NXFFS_INODE_HDR);

      /* Then set the new FLASH offset */

      pack->iooffset += SIZEOF_NXFFS_INODE_HDR;
    }

  /* State 2: inode position found, inode header not written, inode name
   * position not determined.
   */

  if (pack->dest.entry.noffset == 0)
    {
      /* Find the offset to the string memory.  Will if fit in this block?
       * Note: iooffset has already been incremented to account for the
       * size of the inode header.
       */

      namlen = strlen(pack->dest.entry.name);
      if (pack->iooffset + namlen > volume->geo.blocksize)
        {
          /* No.. that inode name will not fit in this block. Return an
           * indication that we are at the end of the block and try again
           * later.
           */

          return -ENOSPC;
        }

      /* Yes.. Write the inode name to the volume packing buffer now, but do
       * not free the name string memory yet; it will be needed later to\
       * calculate the header CRC.
       */

      memcpy(&pack->iobuffer[pack->iooffset], pack->dest.entry.name, namlen);

      /* Reserve space for the inode name  */

      pack->dest.entry.noffset = nxffs_packtell(volume, pack);
      pack->iooffset += namlen;
    }

  /* State 3: Inode header not-written, inode name written.  Still need the position
   * of the first data block.
   *
   * Deal with the special case where the source inode is a zero length file
   * with no data blocks to be transferred.
   */

  if (pack->src.entry.doffset > 0)
    {
      if (pack->dest.entry.doffset == 0)
        {
          /* Will the data block header plus a minimal amount of data fit in this
           * block? (or the whole file if the file is very small).
           */

          mindata = MIN(NXFFS_MINDATA, pack->dest.entry.datlen);
          if (pack->iooffset + SIZEOF_NXFFS_DATA_HDR + mindata > volume->geo.blocksize)
            {
              /* No.. return an indication that we are at the end of the block
               * and try again later.
               */

              ret = -ENOSPC;
              goto errout;
           }

          /* Yes.. reserve space for the data block header */

          pack->dest.entry.doffset = nxffs_packtell(volume, pack);
          pack->iooffset          += SIZEOF_NXFFS_DATA_HDR;

          /* Initialize the output data stream to start with the first data block */

          pack->dest.blkoffset     = pack->dest.entry.doffset;
          pack->dest.blklen        = 0;
          pack->dest.blkpos        = 0;
        }

      /* State 4:  Starting a new block.  Verify that there is space in the current
       * block for another (minimal sized) block
       */

      if (pack->dest.blkoffset == 0)
        {
          /* Will the data block header plus a minimal amount of data fit in this
           * block? (or the whole file if the file is very small).
           */

          mindata = MIN(NXFFS_MINDATA, pack->dest.entry.datlen);
          if (pack->iooffset + SIZEOF_NXFFS_DATA_HDR + mindata > volume->geo.blocksize)
            {
              /* No.. return an indication that we are at the end of the block
               * and try again later.
               */

              ret = -ENOSPC;
              goto errout;
           }

          /* Yes.. reserve space for the data block header */

          pack->dest.blkoffset = nxffs_packtell(volume, pack);
          pack->iooffset      += SIZEOF_NXFFS_DATA_HDR;
          pack->dest.blklen    = 0;
          pack->dest.blkpos    = 0;
        }
    }

  ret = OK;

errout:
  volume->froffset = nxffs_packtell(volume, pack);
  return ret;
}

/****************************************************************************
 * Name: nxffs_wrinodehdr
 *
 * Description:
 *   Write the destination inode header (only) to FLASH.  Note that the inode
 *   name has already been written to FLASH (thus greatly simplifying the
 *   the complexity of this operation).
 *
 * Input Parameters:
 *   volume - The volume to be packed
 *   pack   - The volume packing state structure.
 *
 * Returned Values:
 *   Zero on success; Otherwise, a negated errno value is returned to
 *   indicate the nature of the failure (not used).
 *
 ****************************************************************************/

static int nxffs_wrinodehdr(FAR struct nxffs_volume_s *volume,
                            FAR struct nxffs_pack_s *pack)
{
  FAR struct nxffs_inode_s *inode;
  off_t ioblock;
  uint16_t iooffset;
  uint32_t crc;
  int namlen;
  int ret;

  /* Get seek positions corresponding to the inode header location */

  ioblock   = nxffs_getblock(volume, pack->dest.entry.hoffset);
  iooffset  = nxffs_getoffset(volume, pack->dest.entry.hoffset, ioblock);

  /* The inode header is not written until all of the inode data has been
   * packed into its new location.  As a result, there are two possibilities:
   *
   * 1. The inode header lies in the current, unwritten erase block,
   * 2. The inode header resides in an earlier erase block and has already
   *    been written to FLASH.
   *
   * Recall that the inode name has already been written to FLASH.  If that
   * were not the case, then there would be other complex possibilities.
   *
   * Case 2: Does the inode header reside in a block before the beginning
   * of the current erase block?
   */

  if (ioblock < pack->block0)
    {
      /* Case 2:  The inode header lies in an earlier erase block that has
       * already been written to FLASH.  In this case, if we are very
       * careful, we can just use the standard routine to write the inode
       * header that is called during the normal file close operation:
       */

      ret = nxffs_wrinode(volume, &pack->dest.entry);
    }
  else
    {
      /* Cases 1:  Both the inode header and name are in the unwritten cache
       * memory.
       *
       * Initialize the inode header.
       */

      iooffset += (ioblock - pack->block0) * volume->geo.blocksize;
      inode     = (FAR struct nxffs_inode_s *)&volume->pack[iooffset];
      memcpy(inode->magic, g_inodemagic, NXFFS_MAGICSIZE);

      nxffs_wrle32(inode->noffs,  pack->dest.entry.noffset);
      nxffs_wrle32(inode->doffs,  pack->dest.entry.doffset);
      nxffs_wrle32(inode->utc,    pack->dest.entry.utc);
      nxffs_wrle32(inode->crc,    0);
      nxffs_wrle32(inode->datlen, pack->dest.entry.datlen);

      /* Get the length of the inode name */

      namlen = strlen(pack->dest.entry.name);
      DEBUGASSERT(namlen < CONFIG_NXFFS_MAXNAMLEN);

      inode->state  = CONFIG_NXFFS_ERASEDSTATE;
      inode->namlen = namlen;

      /* Calculate the CRC */

      crc = crc32((FAR const uint8_t *)inode, SIZEOF_NXFFS_INODE_HDR);
      crc = crc32part((FAR const uint8_t *)pack->dest.entry.name, namlen, crc);

      /* Finish the inode header */

      inode->state = INODE_STATE_FILE;
      nxffs_wrle32(inode->crc, crc);

      /* If any open files reference this inode, then update the open file
       * state.
       */

      ret = nxffs_updateinode(volume, &pack->dest.entry);
      if (ret < 0)
        {
          fdbg("Failed to update inode info: %s\n", -ret);
        }
    }

  /* Reset the dest inode information */

  nxffs_freeentry(&pack->dest.entry);
  memset(&pack->dest, 0, sizeof(struct nxffs_packstream_s));
  return ret;
}

/****************************************************************************
 * Name: nxffs_wrdatthdr
 *
 * Description:
 *   Write the destination data block header to FLASH.
 *
 * Input Parameters:
 *   volume - The volume to be packed
 *   pack   - The volume packing state structure.
 *
 * Returned Values:
 *   Zero on success; Otherwise, a negated errno value is returned to
 *   indicate the nature of the failure.
 *
 ****************************************************************************/

static void nxffs_wrdathdr(FAR struct nxffs_volume_s *volume,
                           FAR struct nxffs_pack_s *pack)
{
  FAR struct nxffs_data_s *dathdr;
  off_t    ioblock;
  uint16_t iooffset;
  uint32_t crc;

  if (pack->dest.blklen > 0)
    {
      /* Get the offset in the block corresponding to the location of the data
       * block header.  NOTE:  This must lie in the same block as we currently have
       * buffered.
       */

      ioblock  = nxffs_getblock(volume, pack->dest.blkoffset);
      iooffset = nxffs_getoffset(volume, pack->dest.blkoffset, ioblock);
      DEBUGASSERT(pack->dest.blkoffset && ioblock == pack->ioblock);

      /* Write the data block header to memory */

      dathdr = (FAR struct nxffs_data_s *)&pack->iobuffer[iooffset];
      memcpy(dathdr->magic, g_datamagic, NXFFS_MAGICSIZE);
      nxffs_wrle32(dathdr->crc, 0);
      nxffs_wrle16(dathdr->datlen, pack->dest.blklen);

      /* Update the entire data block CRC (including the header) */

      crc = crc32(&pack->iobuffer[iooffset], pack->dest.blklen + SIZEOF_NXFFS_DATA_HDR);
      nxffs_wrle32(dathdr->crc, crc);
    }

  /* Setup state to allocate the next data block */

  pack->dest.blkoffset = 0;
  pack->dest.blklen    = 0;
  pack->dest.blkpos    = 0;
}

/****************************************************************************
 * Name: nxffs_packtransfer
 *
 * Description:
 *   Transfer data from the source to the destination buffer.
 *
 * Input Parameters:
 *   volume - The volume to be packed
 *   pack   - The volume packing state structure.
 *
 * Returned Values:
 *   None.
 *
 ****************************************************************************/

static void nxffs_packtransfer(FAR struct nxffs_volume_s *volume,
                               FAR struct nxffs_pack_s *pack)
{
   /* Determine how much data is available in the dest pack buffer */

   uint16_t destlen = volume->geo.blocksize - pack->iooffset;

   /* Dermined how much data is available in the src data block */

   uint16_t srclen = pack->src.blklen - pack->src.blkpos;

   /* Transfer the smaller of the two amounts data */

   uint16_t xfrlen = MIN(srclen, destlen);
   if (xfrlen > 0)
     {
       nxffs_ioseek(volume, pack->src.blkoffset + SIZEOF_NXFFS_DATA_HDR + pack->src.blkpos);
       memcpy(&pack->iobuffer[pack->iooffset], &volume->cache[volume->iooffset], xfrlen);

       /* Increment counts and offset for this data transfer */

       pack->src.fpos    += xfrlen; /* Source data offsets */
       pack->src.blkpos  += xfrlen;
       pack->dest.fpos   += xfrlen; /* Destination data offsets */
       pack->dest.blkpos += xfrlen;
       pack->dest.blklen += xfrlen; /* Destination data block size */
       pack->iooffset    += xfrlen; /* Destination I/O block offset */
       volume->iooffset  += xfrlen; /* Source I/O block offset */
       volume->froffset  += xfrlen; /* Free FLASH offset */
     }
}

/****************************************************************************
 * Name: nxffs_endsrcblock
 *
 * Description:
 *   The end of a source data block has been encountered.  Locate the next
 *   source block and setup to continue the transfer.
 *
 * Input Parameters:
 *   volume - The volume to be packed
 *   pack   - The volume packing state structure.
 *
 * Returned Values:
 *   Zero on success; Otherwise, a negated errno value is returned to
 *   indicate the nature of the failure.
 *
 ****************************************************************************/

static int nxffs_endsrcblock(FAR struct nxffs_volume_s *volume,
                             FAR struct nxffs_pack_s *pack)
{
  struct nxffs_blkentry_s blkentry;
  off_t offset;
  int ret;

  /* Yes.. find the next data block in the source input stream. */

  offset = pack->src.blkoffset + SIZEOF_NXFFS_DATA_HDR + pack->src.blklen;
  ret    = nxffs_nextblock(volume, offset, &blkentry);
  if (ret < 0)
    {
      fdbg("Failed to find next data block: %d\n", -ret);
      return ret;
    }

  /* Set up the source stream */

  pack->src.blkoffset = blkentry.hoffset;
  pack->src.blklen    = blkentry.datlen;
  pack->src.blkpos    = 0;
  return OK;
}

/****************************************************************************
 * Name: nxffs_packblock
 *
 * Description:
 *   Resume packing from the source stream into the newly identified
 *   destination block.
 *
 * Input Parameters:
 *   volume - The volume to be packed
 *   pack   - The volume packing state structure.
 *
 * Returned Values:
 *   Zero on success; Otherwise, a negated errno value is returned to
 *   indicate the nature of the failure.
 *
 ****************************************************************************/

static inline int nxffs_packblock(FAR struct nxffs_volume_s *volume,
                                  FAR struct nxffs_pack_s *pack)
{
  off_t offset;
  int ret;

  /* Are we currently processing a block from the source stream? */

  if (pack->src.blkoffset == 0)
    {
      /* No.. setup the source stream */

      ret = nxffs_srcsetup(volume, pack, pack->src.entry.doffset);
      if (ret < 0)
        {
          fdbg("Failed to configure the src stream: %d\n", -ret);
          return ret;
        }
    }

  /* We enter here on a new block every time, so we always have to setup
   * the dest data stream.  There should never be data block allocated at
   * this point in time.
   */

  DEBUGASSERT(pack->dest.blkoffset == 0 && pack->dest.blkpos == 0);

  ret = nxffs_destsetup(volume, pack);
  if (ret < 0)
    {
      /* -ENOSPC is a special return value which simply means that all of
       * the FLASH has been used up to the end of the current.  We need to
       * return OK in this case and resume at the next block.
       */

      if (ret == -ENOSPC)
        {
          return OK;
        }
      else
        {
          fdbg("Failed to configure the dest stream: %d\n", -ret);
          return ret;
        }
    }

  /* Loop, transferring data from the source block to the destination pack
   * buffer until either (1) the source stream is exhausted, (2) the destination
   * block is full, or (3) an error occurs.
   */

  for (;;)
    {
      /* Transfer data from the source buffer to the destination buffer */

      nxffs_packtransfer(volume, pack);

      /* Now, either the (1) src block has been fully transferred, (2) all
       * of the source data has been transferred, or (3) the the destination
       * block is full, .. or all three.
       *
       * Check if all of the bytes in the source inode have been transferred.
       */

      if (pack->src.fpos >= pack->src.entry.datlen)
        {
          /* Write the final destination data block header and inode
           * headers.
           */

          nxffs_wrdathdr(volume, pack);
          nxffs_wrinodehdr(volume, pack);

          /* Find the next valid source inode */

          offset = pack->src.blkoffset + pack->src.blklen;
          memset(&pack->src, 0, sizeof(struct nxffs_packstream_s));

          ret = nxffs_nextentry(volume, offset, &pack->src.entry);
          if (ret < 0)
            {
              /* No more valid inode entries.  Just return an end-of-flash error
               * indication.
               */

              return -ENOSPC;
            }

          /* Setup the new source stream */

          ret = nxffs_srcsetup(volume, pack, pack->src.entry.doffset);
          if (ret < 0)
            {
              return ret;
            }

          /* Setup the dest stream */

          memset(&pack->dest, 0, sizeof(struct nxffs_packstream_s));
          pack->dest.entry.name   = pack->src.entry.name;
          pack->dest.entry.utc    = pack->src.entry.utc;
          pack->dest.entry.datlen = pack->src.entry.datlen;
          pack->src.entry.name    = NULL;

          /* Is there sufficient space at the end of the I/O block to hold
           * the inode header?
           */

          if (pack->iooffset + SIZEOF_NXFFS_INODE_HDR > volume->geo.blocksize)
            {
              /* No, just return success... we will handle this condition when
               * this function is called on the next I/O block.
               */

              return OK;
            }
 
          /* Configure the destination stream */

          ret = nxffs_destsetup(volume, pack);
          if (ret < 0)
            {
              /* -ENOSPC is a special return value which simply means that all of the
               * has been used up to the end.  We need to return OK in this case and
               * resume at the next block.
               */

              if (ret == -ENOSPC)
                {
                  return OK;
                }
              else
                {
                  fdbg("Failed to configure the dest stream: %d\n", -ret);
                  return ret;
                }
            }
        }

      /* Not at the end of the source data stream.  Check if we are at the
       * end of the current source data block.
       */

      else if (pack->src.blkpos >= pack->src.blklen)
        {
          ret = nxffs_endsrcblock(volume, pack);
          if (ret < 0)
            {
              return ret;
            }
        }

     /* Check if the destination block is full */

     if (pack->iooffset >= volume->geo.blocksize)
       {
         /* Yes.. Write the destination data block header and return success */

         nxffs_wrdathdr(volume, pack);
         return OK;
       }
    }

  return -ENOSYS;
}

/****************************************************************************
 * Name: nxffs_setupwriter
 *
 * Description:
 *   Writing is performed at the end of the free FLASH region.  When we
 *   finish packing the other inodes, we still need to pack the partially
 *   written file at the end of FLASH.  This function performs the setup
 *   necessary to perform that packing phase.
 *
 * Input Parameters:
 *   volume - The volume to be packed
 *   pack   - The volume packing state structure.
 *
 * Returned Values:
 *   If there is an active writer of the volume, its open file instance is
 *   returned.  NULL is returned otherwise.
 *
 ****************************************************************************/

static FAR struct nxffs_wrfile_s *
nxffs_setupwriter(FAR struct nxffs_volume_s *volume,
                  FAR struct nxffs_pack_s *pack)
{
  FAR struct nxffs_wrfile_s *wrfile;

  /* Is there a writer? */

  wrfile = nxffs_findwriter(volume);
  if (wrfile)
    {
      /* Yes...  It is the activity of this write that probably initiated
       * this packing activity.  The writer may have failed in one of several
       * different stages:
       *
       *   hoffset == 0: The write failed early before even FLASH for the inode
       *     header was set aside.
       *   noffset == 0: The write failed after the inode header was set aside,
       *     but before the inode name was written.
       *   doffset == 0: The write failed after writing the inode name, bue
       *     before any data blocks were written to FLASH.
       *
       * If no FLASH has been set aside for the write, then we don't need to
       * do anything here.
       */

      if (wrfile->ofile.entry.hoffset > 0)
        {
          /* Initialize for the packing operation. */

           memset(&pack->dest, 0, sizeof(struct nxffs_packstream_s));
           pack->dest.entry.name   = strdup(wrfile->ofile.entry.name);
           pack->dest.entry.utc    = wrfile->ofile.entry.utc;
           pack->dest.entry.datlen = wrfile->ofile.entry.datlen;

           memset(&pack->src, 0, sizeof(struct nxffs_packstream_s));
           memcpy(&pack->src.entry, &wrfile->ofile.entry, sizeof(struct nxffs_entry_s));
           pack->src.entry.name    = NULL;
           return wrfile;
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: nxffs_packwriter
 *
 * Description:
 *   There is a write in progress at the time that the volume is packed.
 *   This is the normal case because it is the write failures that trigger
 *   the packing operation to begin with.
 *
 *   Writing is performed at the end of the free FLASH region and this
 *   implemenation is restricted to a single writer.  The new inode is not
 *   written to FLASH until the the writer is closed and so will not be
 *   found by nxffs_packblock().
 *
 * Input Parameters:
 *   volume - The volume to be packed
 *   pack   - The volume packing state structure.
 *
 * Returned Values:
 *   Zero on success; Otherwise, a negated errno value is returned to
 *   indicate the nature of the failure.
 *
 ****************************************************************************/

static inline int nxffs_packwriter(FAR struct nxffs_volume_s *volume,
                                   FAR struct nxffs_pack_s *pack,
                                   FAR struct nxffs_wrfile_s *wrfile)
{
  int ret;

  /* Are we currently processing a block from the source stream? */

  if (pack->src.blkoffset == 0)
    {
      /* No.. setup the source stream */

      ret = nxffs_srcsetup(volume, pack, pack->src.entry.doffset);
      if (ret < 0)
        {
          fdbg("Failed to configure the src stream: %d\n", -ret);
          return ret;
        }
    }

  /* We enter here on a new block every time, so we always have to setup
   * the dest data stream.  There should never be data block allocated at
   * this point in time.
   */

  DEBUGASSERT(pack->dest.blkoffset == 0 && pack->dest.blkpos == 0);

  ret = nxffs_destsetup(volume, pack);
  if (ret < 0)
    {
      /* -ENOSPC is a special return value which simply means that all of the
       * has been used up to the end.  We need to return OK in this case and
       * resume at the next block.
       */

      if (ret == -ENOSPC)
        {
          return OK;
        }
      else
        {
          fdbg("Failed to configure the dest stream: %d\n", -ret);
          return ret;
        }
    }

  /* Loop, transferring data from the source block to the destination pack
   * buffer until either (1) the source stream is exhausted, (2) the destination
   * block is full, or (3) an error occurs.
   */

  for (;;)
    {
       /* Transfer data from the source buffer to the destination buffer */

       nxffs_packtransfer(volume, pack);

       /* Now, either the (1) src block has been fully transferred, (2) all
        * of the source data has been transferred, or (3) the the destination
        * block is full, .. or all three.
        *
        * Check if all of the bytes in the source inode have been transferred.
        */

       if (pack->src.fpos >= pack->src.entry.datlen)
         {
           /* Write the final destination data block header and inode
            * headers.
            */

           nxffs_wrdathdr(volume, pack);

           /* Set the new offsets in the open file instance. */

           wrfile->ofile.entry.hoffset = pack->dest.entry.hoffset;
           wrfile->ofile.entry.noffset = pack->dest.entry.noffset;
           wrfile->ofile.entry.doffset = pack->dest.entry.doffset;

           /* Return an end-of-flash error to indicate that all of the write
            * data has been transferred.
            */

           return -ENOSPC;
         }

      /* Not at the end of the source data stream.  Check if we are at the
       * end of the current source data block.
       */

      else if (pack->src.blkpos >= pack->src.blklen)
        {
          ret = nxffs_endsrcblock(volume, pack);
          if (ret < 0)
            {
              return ret;
            }
        }

     /* Check if the destination block is full */

     if (pack->iooffset >= volume->geo.blocksize)
       {
         /* Yes.. Write the destination data block header and return success */

         nxffs_wrdathdr(volume, pack);
         return OK;
       }
    }

  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxffs_pack
 *
 * Description:
 *   Pack and re-write the filesystem in order to free up memory at the end
 *   of FLASH.
 *
 * Input Parameters:
 *   volume - The volume to be packed.
 *
 * Returned Values:
 *   Zero on success; Otherwise, a negated errno value is returned to
 *   indicate the nature of the failure.
 *
 ****************************************************************************/

int nxffs_pack(FAR struct nxffs_volume_s *volume)
{
  struct nxffs_pack_s pack;
  FAR struct nxffs_wrfile_s *wrfile;
  off_t iooffset;
  off_t eblock;
  off_t block;
  bool packed;
  int i;
  int ret;

  /* Get the offset to the first valid inode entry */

  wrfile = NULL;
  packed = false;

  iooffset = nxffs_mediacheck(volume, &pack);
  if (iooffset == 0)
    {
      /* Offset zero is only returned if no valid blocks were found on the
       * FLASH media or if there are no valid inode entries on the FLASH after
       * the first valid block.  There are two possibilities:  (1) there 
       * really is nothing on the FLASH, or (2) there is a file being written
       * to the FLASH now.
       */

      /* Is there a writer? */

      wrfile = nxffs_setupwriter(volume, &pack);
      if (wrfile)
        {
          /* If there is a write, just set ioffset to the offset of data in
           * first block. Setting 'packed' to true will supress normal inode
           * packing operation.  Then we can start compacting the FLASH.
           */

          iooffset = SIZEOF_NXFFS_BLOCK_HDR;
          packed   = true;
          goto start_pack;
        }
      else
        {
          /* No, there is no write in progress.  We just have an empty flash
           * full of deleted files.  In this case, the media needs to be re-
           * formatted.
            */

          ret = nxffs_reformat(volume);
          if (ret == OK)
            {
              /* The free flash offset will be in the first valid block of
               * the FLASH.
               */

              block = 0;
              ret = nxffs_validblock(volume, &block);
              if (ret == OK)
                {
                  /* Set to the offset past the block header in the first
                   * valid block
                   */

                  volume->froffset =
                    block * volume->geo.blocksize + SIZEOF_NXFFS_BLOCK_HDR;
                }
            }

          return ret;
        }
    }

  /* There is a valid format and valid inodes on the media.. setup up to
   * begin the packing operation.
   */

  ret = nxffs_startpos(volume, &pack, &iooffset);
  if (ret < 0)
    {
      /* This is a normal situation if the volume is full */

      if (ret == -ENOSPC)
        {
          /* In the case where the volume is full, nxffs_startpos() will
           * recalculate the free FLASH offset and store it in iooffset.  There
           * may be deleted files at the end of FLASH.  In this case, we don't
           * have to pack any files, we simply have to erase FLASH at the end.
           * But don't do this unless there is some particularly big FLASH
           * savings (otherwise, we risk wearing out these final blocks).
           */

          if (iooffset + CONFIG_NXFFS_TAILTHRESHOLD < volume->froffset)
            {
               /* Setting 'packed' to true will supress normal inode packing
                * operation.
                */

               packed = true;

               /* Writing is performed at the end of the free FLASH region.
                * If we are not packing files, we could still need to pack
                * the partially written file at the end of FLASH.
                */

               wrfile = nxffs_setupwriter(volume, &pack);
             }

          /* Otherwise return OK.. meaning that there is nothing more we can
           * do to recover FLASH space.
           */

          else
            {
              return OK;
            }
        }
      else
        {
          fvdbg("Failed to find a packing position: %d\n", -ret);
          return ret;
        }
    }

  /* Otherwise, begin pack at this src/dest block combination.  Initialize
   * ioblock and iooffset with the position of the first inode header.  In
   * this case, the FLASH offset to the first inode header is return in 
   * iooffset.
   */

start_pack:

  pack.ioblock     = nxffs_getblock(volume, iooffset);
  pack.iooffset    = nxffs_getoffset(volume, iooffset, pack.ioblock);
  volume->froffset = iooffset;

  /* Then pack all erase blocks starting with the erase block that contains
   * the ioblock and through the final erase block on the FLASH.
   */

  for (eblock = pack.ioblock / volume->blkper;
       eblock < volume->geo.neraseblocks;
       eblock++)
    {
      /* Read the erase block into the pack buffer.  We need to do this even
       * if we are overwriting the entire block so that we skip over
       * previously marked bad blocks.
       */

      pack.block0 = eblock * volume->blkper;
      ret = MTD_BREAD(volume->mtd, pack.block0, volume->blkper, volume->pack);
      if (ret < 0)
        {
          fdbg("Failed to read erase block %d: %d\n", eblock, -ret);
          goto errout_with_pack;
        }

      /* Pack each I/O block */

      for (i = 0, block = pack.block0, pack.iobuffer = volume->pack;
           i < volume->blkper;
           i++, block++, pack.iobuffer += volume->geo.blocksize)
        {
           /* The first time here, the ioblock may point to an offset into
            * the erase block.  We just need to skip over those cases.
            */

           if (block >= pack.ioblock)
              {
                /* Set the I/O position.  Note on the first time we get
                 * pack.iooffset will hold the offset in the first I/O block
                 * to the first inode header.  After that, it will always
                 * refer to the first byte after the block header.
                 */

                pack.ioblock = block;

                /* If this is not a valid block or if we have already
                 * finished packing the valid inode entries, then just fall
                 * through, reset the FLASH memory to the erase state, and
                 * write the reset values to FLASH.  (The first block that
                 * we want to process will always be valid -- we have
                 * already verified that).
                 */

                if (nxffs_packvalid(&pack))
                  {
                    /* Have we finished packing inodes? */

                    if (!packed)
                      {
                         DEBUGASSERT(wrfile == NULL);

                         /* Pack inode data into this block */

                         ret = nxffs_packblock(volume, &pack);
                         if (ret < 0)
                           {
                             /* The error -ENOSPC is a special value that simply
                              * means that there is nothing further to be packed.
                              */

                             if (ret == -ENOSPC)
                               {
                                 packed = true;

                                 /* Writing is performed at the end of the free
                                  * FLASH region and this implemenation is restricted
                                  * to a single writer.  The new inode is not
                                  * written to FLASH until the the writer is closed
                                  * and so will not be found by nxffs_packblock().
                                  */

                                 wrfile = nxffs_setupwriter(volume, &pack);
                               }
                             else
                               {
                                 /* Otherwise, something really bad happened */

                                 fdbg("Failed to pack into block %d: %d\n",
                                      block, ret);
                                 goto errout_with_pack;
                               }
                           }
                       }

                     /* If all of the "normal" inodes have been packed, then check if
                      * we need to pack the current, in-progress write operation.
                      */

                     if (wrfile)
                       {
                         DEBUGASSERT(packed == true);

                         /* Pack write data into this block */

                         ret = nxffs_packwriter(volume, &pack, wrfile);
                         if (ret < 0)
                           {
                             /* The error -ENOSPC is a special value that simply
                              * means that there is nothing further to be packed.
                              */

                             if (ret == -ENOSPC)
                               {
                                 wrfile = NULL;
                               }
                             else
                               {
                                 /* Otherwise, something really bad happened */

                                 fdbg("Failed to pack into block %d: %d\n",
                                      block, ret);
                                 goto errout_with_pack;
                               }
                           }
                       }
                   }

                 /* Set any unused portion at the end of the block to the
                  * erased state.
                  */

                 if (pack.iooffset < volume->geo.blocksize)
                   {
                     memset(&pack.iobuffer[pack.iooffset],
                            CONFIG_NXFFS_ERASEDSTATE,
                            volume->geo.blocksize - pack.iooffset);
                   }

                 /* Next time through the loop, pack.iooffset will point to the
                  * first byte after the block header.
                  */

                 pack.iooffset = SIZEOF_NXFFS_BLOCK_HDR;
              }
         }

      /* We now have an in-memory image of how we want this erase block to
       * appear. Now it is safe to erase the block.
       */

      ret = MTD_ERASE(volume->mtd, eblock, 1);
      if (ret < 0)
        {
          fdbg("Failed to erase block %d [%d]: %d\n",
               eblock, pack.block0, -ret);
          goto errout_with_pack;
        }

      /* Write the packed I/O block to FLASH */

      ret = MTD_BWRITE(volume->mtd, pack.block0, volume->blkper, volume->pack);
      if (ret < 0)
        {
          fdbg("Failed to write erase block %d [%]: %d\n",
               eblock, pack.block0, -ret);
          goto errout_with_pack;
        }
    }

errout_with_pack:
  nxffs_freeentry(&pack.src.entry);
  nxffs_freeentry(&pack.dest.entry);
  return ret;
}
