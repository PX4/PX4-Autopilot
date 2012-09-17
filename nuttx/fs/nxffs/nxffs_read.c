/****************************************************************************
 * fs/nxffs/nxffs_read.c
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
#include <fcntl.h>
#include <crc32.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
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
 * Name: nxffs_rdseek
 *
 * Description:
 *   Seek to the file position before read or write access.  Note that the
 *   simplier nxffs_ioseek() cannot be used for this purpose.  File offsets
 *   are not easily mapped to FLASH offsets due to intervening block and
 *   data headers.
 *
 * Input Parameters:
 *   volume   - Describes the current volume
 *   entry    - Describes the open inode
 *   fpos     - The desired file position
 *   blkentry - Describes the block entry that we are positioned in
 *
 ****************************************************************************/

static ssize_t nxffs_rdseek(FAR struct nxffs_volume_s *volume,
                            FAR struct nxffs_entry_s *entry,
                            off_t fpos,
                            FAR struct nxffs_blkentry_s *blkentry)
{
  size_t datstart;
  size_t datend;
  off_t offset;
  int ret;

  /* The initial FLASH offset will be the offset to first data block of
   * the inode
   */

  offset = entry->doffset;
  if (offset == 0)
    {
      /* Zero length files will have no data blocks */

      return -ENOSPC;
    }

  /* Loop until we read the data block containing the desired position */

  datend = 0;
  do
    {
      /* Check if the next data block contains the sought after file position */

      ret = nxffs_nextblock(volume, offset, blkentry);
      if (ret < 0)
        {
          fdbg("nxffs_nextblock failed: %d\n", -ret);
          return ret;
        }
 
      /* Get the range of data offsets for this data block */

      datstart  = datend;
      datend   += blkentry->datlen;

      /* Offset to search for the the next data block */

      offset = blkentry->hoffset + SIZEOF_NXFFS_DATA_HDR + blkentry->datlen;
    }
  while (datend <= fpos);

  /* Return the offset to the data within the current data block */

  blkentry->foffset = fpos - datstart;
  nxffs_ioseek(volume, blkentry->hoffset + SIZEOF_NXFFS_DATA_HDR + blkentry->foffset);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: nxffs_read
 *
 * Description:
 *   This is an implementation of the NuttX standard file system read
 *   method.
 *
 ****************************************************************************/

ssize_t nxffs_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  FAR struct nxffs_volume_s *volume;
  FAR struct nxffs_ofile_s *ofile;
  struct nxffs_blkentry_s blkentry;
  ssize_t total;
  size_t available;
  size_t readsize;
  int ret;

  fvdbg("Read %d bytes from offset %d\n", buflen, filep->f_pos);

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover the open file state from the struct file instance */

  ofile = (FAR struct nxffs_ofile_s *)filep->f_priv;

  /* Recover the volume state from the open file */

  volume = (FAR struct nxffs_volume_s *)filep->f_inode->i_private;
  DEBUGASSERT(volume != NULL);

  /* Get exclusive access to the volume.  Note that the volume exclsem
   * protects the open file list.
   */

  ret = sem_wait(&volume->exclsem);
  if (ret != OK)
    {
      ret = -errno;
      fdbg("sem_wait failed: %d\n", ret);
      goto errout;
    }

  /* Check if the file was opened with read access */

  if ((ofile->oflags & O_RDOK) == 0)
    {
      fdbg("File not open for read access\n");
      ret = -EACCES;
      goto errout_with_semaphore;
    }

  /* Loop until all bytes have been read */

  for (total = 0; total < buflen; )
    {
      /* Don't seek past the end of the file */

      if (filep->f_pos >= ofile->entry.datlen)
        {
          /* Return the partial read */

          filep->f_pos = ofile->entry.datlen;
          break;
        }

      /* Seek to the current file offset */

      ret = nxffs_rdseek(volume, &ofile->entry, filep->f_pos, &blkentry);
      if (ret < 0)
        {
          fdbg("nxffs_rdseek failed: %d\n", -ret);
          ret = -EACCES;
          goto errout_with_semaphore;
        }

      /* How many bytes are available at this offset */

      available = blkentry.datlen - blkentry.foffset;

      /* Don't read more than we need to */

      readsize = buflen - total;
      if (readsize > available)
        {
          readsize = available;
        }

      /* Read data from that file offset */

      memcpy(&buffer[total], &volume->cache[volume->iooffset], readsize);

      /* Update the file offset */

      filep->f_pos += readsize;
      total        += readsize;
    }

  sem_post(&volume->exclsem);
  return total;

errout_with_semaphore:
  sem_post(&volume->exclsem);
errout:
  return (ssize_t)ret;
}

/****************************************************************************
 * Name: nxffs_nextblock
 *
 * Description:
 *   Search for the next valid data block starting at the provided
 *   FLASH offset.
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume.
 *   datlen  - A memory location to return the data block length.
 *  
 * Returned Value:
 *   Zero is returned on success. Otherwise, a negated errno is returned
 *   that indicates the nature of the failure.
 *
 ****************************************************************************/

int nxffs_nextblock(FAR struct nxffs_volume_s *volume, off_t offset,
                    FAR struct nxffs_blkentry_s *blkentry)
{
  int nmagic;
  int ch;
  int nerased;
  int ret;

  /* Seek to the first FLASH offset provided by the caller. */

  nxffs_ioseek(volume, offset);

  /* Skip the block header */

  if (volume->iooffset < SIZEOF_NXFFS_BLOCK_HDR)
    {
      volume->iooffset = SIZEOF_NXFFS_BLOCK_HDR;
    }

  /* Then begin searching */
  
  nerased = 0;
  nmagic  = 0;

  for (;;)
    {
      /* Read the next character */

      ch = nxffs_getc(volume, SIZEOF_NXFFS_DATA_HDR - nmagic);
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
           * data block or start of the next inode. There is the possibility
           * of this magic sequnce occurring in FLASH data.  However, the
           * data block CRC should distinguish between real NXFFS data blocks
           * headers and such false alarms.
           */

          if (ch != g_datamagic[nmagic])
            {
              /* Ooops... this is the not the right character for the magic
               * Sequence.  Check if we need to restart or to cancel the sequence:
               */

              if (ch == g_datamagic[0])
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
           * indicate the beginning of an NXFFS data block.
           */

          else 
            {
              /* The offset to the header must be 4 bytes before the current
               * FLASH seek location.
               */

              blkentry->hoffset = nxffs_iotell(volume) - NXFFS_MAGICSIZE;

              /* Read the block header and verify the block at that address */

              ret = nxffs_rdblkhdr(volume, blkentry->hoffset, &blkentry->datlen);
              if (ret == OK)
                {
                  fvdbg("Found a valid data block, offset: %d datlen: %d\n",
                        blkentry->hoffset, blkentry->datlen);
                  return OK;
                }

              /* False alarm.. Restore the volume cache position (that was
               * destroyed by nxfs_rdblkhdr()) and keep looking.
               */

              nxffs_ioseek(volume, blkentry->hoffset + NXFFS_MAGICSIZE);
              nmagic = 0;
            }
        }
    }

  /* We won't get here, but to keep some compilers happy: */

  return -ENOENT;
}

/****************************************************************************
 * Name: nxffs_rdblkhdr
 *
 * Description:
 *   Read and verify the data block header at the specified offset.
 *
 * Input Parameters:
 *   volume - Describes the current volume.
 *   offset - The byte offset from the beginning of FLASH where the data block
 *     header is expected.
 *   datlen  - A memory location to return the data block length.
 *
 * Returned Value:
 *   Zero on success.  Otherwise, a negated errno value is returned
 *   indicating the nature of the failure.
 *
 ****************************************************************************/

int nxffs_rdblkhdr(FAR struct nxffs_volume_s *volume, off_t offset,
                   FAR uint16_t *datlen)
{
  struct nxffs_data_s blkhdr;
  uint32_t ecrc;
  uint32_t crc;
  uint16_t doffset;
  uint16_t dlen;
  int ret;

  /* Make sure the the block containing the data block header is in the cache */

  nxffs_ioseek(volume, offset);
  ret = nxffs_rdcache(volume, volume->ioblock);
  if (ret < 0)
    {
      fvdbg("Failed to read data into cache: %d\n", ret);
      return ret;
    }

  /* Read the header at the FLASH offset */

  doffset = volume->iooffset;
  memcpy(&blkhdr, &volume->cache[doffset], SIZEOF_NXFFS_DATA_HDR);

  /* Extract the data length */

  dlen = nxffs_rdle16(blkhdr.datlen);

  /* Get the offset to the beginning of the data */

  doffset += SIZEOF_NXFFS_DATA_HDR;

  /* Make sure that all of the data fits within the block */

  if ((uint32_t)doffset + (uint32_t)dlen > (uint32_t)volume->geo.blocksize)
    {
      fdbg("Data length=%d is unreasonable at offset=%d\n", dlen, doffset);
      return -EIO;
    }
 
  /* Extract the expected CRC and calculate the CRC of the data block */

  ecrc = nxffs_rdle32(blkhdr.crc);

  nxffs_wrle32(blkhdr.crc, 0);
  crc = crc32((FAR const uint8_t *)&blkhdr, SIZEOF_NXFFS_DATA_HDR);
  crc = crc32part(&volume->cache[doffset], dlen, crc);

  if (crc != ecrc)
    {
      fdbg("CRC failure\n");
      return -EIO;
    }

  /* Looks good! Return the data length and success */

  *datlen = dlen;
  return OK;
}



