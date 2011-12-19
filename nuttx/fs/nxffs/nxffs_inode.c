/****************************************************************************
 * fs/nxffs/nxffs_inode.c
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
#include <crc32.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
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
 * Name: nxffs_rdentry
 *
 * Description:
 *   Read the inode entry at this offset.  Called only from nxffs_nextentry().
 *
 * Input Parameters:
 *   volume - Describes the current volume.
 *   offset - The byte offset from the beginning of FLASH where the inode
 *     header is expected.
 *   entry  - A memory location to return the expanded inode header
 *     information.
 *
 * Returned Value:
 *   Zero on success.  Otherwise, a negated errno value is returned
 *   indicating the nature of the failure.
 *
 *   On return, the 
 *
 ****************************************************************************/

static int nxffs_rdentry(FAR struct nxffs_volume_s *volume, off_t offset,
                         FAR struct nxffs_entry_s *entry)
{
  struct nxffs_inode_s inode;
  uint32_t ecrc;
  uint32_t crc;
  uint8_t state;
  int namlen;
  int ret;

  DEBUGASSERT(volume && entry);
  memset(entry, 0, sizeof(struct nxffs_entry_s));

  /* Read the header at the FLASH offset */

  nxffs_ioseek(volume, offset);
  memcpy(&inode, &volume->cache[volume->iooffset], SIZEOF_NXFFS_INODE_HDR);

  /* Check if the file state is recognized. */

  state = inode.state;
  if (state != INODE_STATE_FILE && state != INODE_STATE_DELETED)
    {
      /* This can't be a valid inode.. don't bother with the rest */

      ret = -ENOENT;
      goto errout_no_offset;
    }
 
  /* Copy the packed header into the user-friendly buffer */

  entry->hoffset = offset;
  entry->noffset = nxffs_rdle32(inode.noffs);
  entry->doffset = nxffs_rdle32(inode.doffs);
  entry->utc     = nxffs_rdle32(inode.utc);
  entry->datlen  = nxffs_rdle32(inode.datlen);

  /* Modify the packed header and perform the (partial) CRC calculation */

  ecrc           = nxffs_rdle32(inode.crc);
  inode.state    = CONFIG_NXFFS_ERASEDSTATE;
  memset(inode.crc, 0, 4);
  crc            = crc32((FAR const uint8_t *)&inode, SIZEOF_NXFFS_INODE_HDR);

  /* Allocate memory to hold the variable-length file name */

  namlen = inode.namlen;
  entry->name = (FAR char *)kmalloc(namlen + 1);
  if (!entry->name)
    {
      fdbg("Failed to allocate name, namlen: %d\n", namlen);
      ret = -ENOMEM;
      goto errout_no_offset;
    }
  
  /* Seek to the expected location of the name in FLASH */

  nxffs_ioseek(volume, entry->noffset);

  /* Make sure that the block is in memory (the name may not be in the
   * same block as the inode header.
   */
 
  ret = nxffs_rdcache(volume, volume->ioblock);
  if (ret < 0)
    {
      fdbg("nxffsx_rdcache failed: %d\n", -ret);
      goto errout_with_name;
    }

  /* Read the file name from the expected offset in FLASH */

  memcpy(entry->name, &volume->cache[volume->iooffset], namlen);
  entry->name[namlen] = '\0';

  /* Finish the CRC calculation and verify the entry */

  crc = crc32part((FAR const uint8_t *)entry->name, namlen, crc);
  if (crc != ecrc)
    {
      fdbg("CRC entry: %08x CRC calculated: %08x\n", ecrc, crc);
      ret = -EIO;
      goto errout_with_name;
    }

  /* We have a good inode header.. but it still could a deleted file.
   * Check the file state.
   */

  if (state != INODE_STATE_FILE)
    {
      /* It is a deleted file.  But still, the data offset and the
       * start size are good so we can use this information to advance
       * further in FLASH memory and reduce the search time.
       */

      offset = nxffs_inodeend(volume, entry);
      nxffs_freeentry(entry);
      ret = -ENOENT;
      goto errout;
    }

  /* Everything is good.. leave the offset pointing to the valid inode
   * header.
   */

  return OK;

  /* On errors where we are suspicious of the validity of the inode header,
   * we need to increment the file position to just after the "good" magic
   * word.
   */

errout_with_name:
  nxffs_freeentry(entry);
errout_no_offset:
  offset += NXFFS_MAGICSIZE;
errout:
  nxffs_ioseek(volume, offset);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxffs_freeentry
 *
 * Description:
 *   The inode values returned by nxffs_nextentry() include allocated memory
 *   (specifically, the file name string).  This function should be called
 *   to dispose of that memory when the inode entry is no longer needed.
 *
 *   Note that the nxffs_entry_s containing structure is not freed.  The
 *   caller may call kfree upon return of this function if necessary to
 *   free the entry container.
 *
 * Input parameters:
 *   entry  - The entry to be freed.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxffs_freeentry(FAR struct nxffs_entry_s *entry)
{
  if (entry->name)
    {
      kfree(entry->name);
      entry->name = NULL;
    }
}

/****************************************************************************
 * Name: nxffs_nextentry
 *
 * Description:
 *   Search for the next valid inode starting at the provided FLASH offset.
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume.
 *   offset - The FLASH memory offset to begin searching.
 *   entry  - A pointer to memory provided by the caller in which to return
 *     the inode description.
 *  
 * Returned Value:
 *   Zero is returned on success. Otherwise, a negated errno is returned
 *   that indicates the nature of the failure.
 *
 ****************************************************************************/

int nxffs_nextentry(FAR struct nxffs_volume_s *volume, off_t offset,
                    FAR struct nxffs_entry_s *entry)
{
  int nmagic;
  int ch;
  int nerased;
  int ret;

  /* Seek to the first FLASH offset provided by the caller. */

  nxffs_ioseek(volume, offset);

  /* Then begin searching */
  
  nerased = 0;
  nmagic  = 0;
  for (;;)
    {
      /* Read the next character */

      ch = nxffs_getc(volume, SIZEOF_NXFFS_INODE_HDR - nmagic);
      if (ch < 0)
        {
          fvdbg("nxffs_getc failed: %d\n", -ch);
          return ch;
        }

      /* Check for another erased byte */

      else if (ch == CONFIG_NXFFS_ERASEDSTATE)
        {
          /* If we have encountered NXFFS_NERASED number of consecutive
           * erased bytes, then presume we have reached the end of valid
           * data.
           */

          if (++nerased >= NXFFS_NERASED)
            {
              fvdbg("No entry found\n");
              return -ENOENT;
            }
        }
      else
        {
          nerased = 0;

          /* Check for the magic sequence indicating the start of an NXFFS
           * inode. There is the possibility of this magic sequnce occurring
           * in FLASH data.  However, the header CRC should distinguish
           * between real NXFFS inode headers and such false alarms.
           */

          if (ch != g_inodemagic[nmagic])
            {
              /* Ooops... this is the not the right character for the magic
               * Sequence.  Check if we need to restart or to cancel the sequence:
               */

              if (ch == g_inodemagic[0])
                {
                  nmagic = 1;
                }
              else
                {
                  nmagic = 0;
                }
            }
          else if (nmagic < NXFFS_MAGICSIZE - 1)
            {
              /* We have one more character in the magic sequence */

              nmagic++;
            }

          /* We have found the magic sequence in the FLASH data that may
           * indicate the beginning of an NXFFS inode.
           */

          else 
            {
              /* The the FLASH offset where we found the matching magic number */

              offset = nxffs_iotell(volume) - NXFFS_MAGICSIZE;

              /* Try to extract the inode header from that position */

              ret = nxffs_rdentry(volume, offset, entry);
              if (ret == OK)
                {
                  fvdbg("Found a valid fileheader, offset: %d\n", offset);
                  return OK;
                }

              /* False alarm.. keep looking */

              nmagic = 0;
            }
        }
    }

  /* We won't get here, but to keep some compilers happy: */

  return -ENOENT;
}

/****************************************************************************
 * Name: nxffs_findinode
 *
 * Description:
 *   Search for an inode with the provided name starting with the first
 *   valid inode and proceeding to the end FLASH or until the matching
 *   inode is found.
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume
 *   name   - The name of the inode to find
 *   entry  - The location to return information about the inode.
 *
 * Returned Value:
 *   Zero is returned on success. Otherwise, a negated errno is returned
 *   that indicates the nature of the failure.
 *
 ****************************************************************************/

int nxffs_findinode(FAR struct nxffs_volume_s *volume, FAR const char *name,
                    FAR struct nxffs_entry_s *entry)
{
  off_t offset;
  int ret;

  /* Start with the first valid inode that was discovered when the volume
   * was created (or modified after the last file system re-packing).
   */

  offset = volume->inoffset;

  /* Loop, checking each NXFFS inode until either: (1) we find the NXFFS inode
   * with the matching name, or (2) we reach the end of data written on the
   * media.
   */

  for (;;)
   {
      /* Get the next, valid NXFFS inode entry */

      ret = nxffs_nextentry(volume, offset, entry);
      if (ret < 0)
        {
          fvdbg("No inode found: %d\n", -ret);
          return ret;
        }

      /* Is this the NXFFS inode we are looking for? */

      else if (strcmp(name, entry->name) == 0)
        {
          /* Yes, return success with the entry data in 'entry' */

          return OK;
        }

      /* Discard this entry and try the next one.  Here we set the
       * next offset using the raw data length as the offset
       * increment.  This is, of course, not accurate because it
       * does not account for the data headers that enclose the
       * data.  But it is guaranteed to be less than or equal to
       * the correct offset and, hence, better then searching
       * byte-for-byte.
       */

      offset = nxffs_inodeend(volume, entry);
      nxffs_freeentry(entry);
    }

  /* We won't get here, but for some compilers: */

  return -ENOENT;
}

/****************************************************************************
 * Name: nxffs_inodeend
 *
 * Description:
 *   Return an *approximiate* FLASH offset to end of the inode data.  The
 *   returned value is guaranteed to be be less then or equal to the offset
 *   of the thing-of-interest in FLASH.  Parsing for interesting things
 *   can begin at that point.
 *
 *   Assumption:  The inode header has been verified by the caller and is
 *   known to contain valid data.
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume
 *   entry  - Describes the inode.
 *
 * Returned Value:
 *   A FLASH offset to the (approximate) end of the inode data.  No errors
 *   are detected.
 *
 ****************************************************************************/

off_t nxffs_inodeend(FAR struct nxffs_volume_s *volume,
                     FAR struct nxffs_entry_s *entry)
{
  /* A zero length file will have no data blocks */

  if (entry->doffset)
    {
      /* This is the maximum size of one data block.  It is the physcal size
       * of the block minus the minimum number of headers: block sna data
       */

      uint16_t maxsize = volume->geo.blocksize - SIZEOF_NXFFS_BLOCK_HDR - SIZEOF_NXFFS_DATA_HDR;

      /* This is the minimum number of blocks require to span all of the
       * inode data.  One additional block could possibly be required -- we
       * could make this accurate by looking at the size of the first, perhaps
       * partial, data block.
       */

      off_t minblocks = (entry->datlen + maxsize - 1) / maxsize;

      /* And this is our best, simple guess at the end of the inode data */

      return entry->doffset + entry->datlen + minblocks * SIZEOF_NXFFS_DATA_HDR;
    }

  /* Otherwise, return an offset that accounts only for the inode header and
   * the inode name.
   */

  /* All valid inodes will have a name associated with them */

  DEBUGASSERT(entry->noffset);
  return entry->noffset + strlen(entry->name);
}


