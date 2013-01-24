/****************************************************************************
 * fs/nxffs/nxffs_initialize.c
 *
 *   Copyright (C) 2011, 2013 Gregory Nutt. All rights reserved.
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

#include <nuttx/kmalloc.h>
#include <nuttx/mtd.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>

#include "nxffs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/* See fs_mount.c -- this structure is explicitly externed there.
 * We use the old-fashioned kind of initializers so that this will compile
 * with any compiler.
 */

const struct mountpt_operations nxffs_operations =
{
  nxffs_open,        /* open */
  nxffs_close,       /* close */
  nxffs_read,        /* read */
  nxffs_write,       /* write */
  NULL,              /* seek -- Use f_pos in struct file */
  nxffs_ioctl,       /* ioctl */

  NULL,              /* sync -- No buffered data */
  nxffs_dup,         /* dup */

  nxffs_opendir,     /* opendir */
  NULL,              /* closedir */
  nxffs_readdir,     /* readdir */
  nxffs_rewinddir,   /* rewinddir */

  nxffs_bind,        /* bind */
  nxffs_unbind,      /* unbind */
  nxffs_statfs,      /* statfs */

  nxffs_unlink,      /* unlink */
  NULL,              /* mkdir -- no directories */
  NULL,              /* rmdir -- no directories */
  NULL,              /* rename -- cannot rename in place if name is longer */
  nxffs_stat         /* stat */
};

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/* The magic number that appears that the beginning of each NXFFS (logical)
 * block
 */

const uint8_t g_blockmagic[NXFFS_MAGICSIZE] = { 'B', 'l', 'c', 'k' };

/* The magic number that appears that the beginning of each NXFFS inode */

const uint8_t g_inodemagic[NXFFS_MAGICSIZE] = { 'I', 'n', 'o', 'd' };

/* The magic number that appears that the beginning of each NXFFS inode
 * data block.
 */

const uint8_t g_datamagic[NXFFS_MAGICSIZE] = { 'D', 'a', 't', 'a' };

/* If CONFIG_NXFSS_PREALLOCATED is defined, then this is the single, pre-
 * allocated NXFFS volume instance.
 */

#ifdef CONFIG_NXFSS_PREALLOCATED
struct nxffs_volume_s g_volume;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxffs_initialize
 *
 * Description:
 *   Initialize to provide NXFFS on an MTD interface
 *
 * Input Parameters:
 *   mtd - The MTD device that supports the FLASH interface.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int nxffs_initialize(FAR struct mtd_dev_s *mtd)
{
  FAR struct nxffs_volume_s *volume;
  struct nxffs_blkstats_s stats;
  off_t threshold;
  int ret;

  /* If CONFIG_NXFSS_PREALLOCATED is defined, then this is the single, pre-
   * allocated NXFFS volume instance.
   */

#ifdef CONFIG_NXFSS_PREALLOCATED

  volume = &g_volume;
  memset(volume, 0, sizeof(struct nxffs_volume_s));

#else

  /* Allocate a NXFFS volume structure */

  volume = (FAR struct nxffs_volume_s *)kzalloc(sizeof(struct nxffs_volume_s));
  if (!volume)
    {
      ret = -ENOMEM;
      goto errout;
    }
#endif

  /* Initialize the NXFFS volume structure */

  volume->mtd    = mtd;
  volume->cblock = (off_t)-1;
  sem_init(&volume->exclsem, 0, 1);
  sem_init(&volume->wrsem, 0, 1);

  /* Get the volume geometry. (casting to uintptr_t first eliminates
   * complaints on some architectures where the sizeof long is different
   * from the size of a pointer).
   */

  ret = MTD_IOCTL(mtd, MTDIOC_GEOMETRY, (unsigned long)((uintptr_t)&volume->geo));
  if (ret < 0)
    {
      fdbg("MTD ioctl(MTDIOC_GEOMETRY) failed: %d\n", -ret);
      goto errout_with_volume;
    }

  /* Allocate one I/O block buffer to general files system access */

  volume->cache = (FAR uint8_t *)kmalloc(volume->geo.blocksize);
  if (!volume->cache)
    {
      fdbg("Failed to allocate an erase block buffer\n");
      ret = -ENOMEM;
      goto errout_with_volume;
    }

  /* Pre-allocate one, full, in-memory erase block.  This is needed for filesystem
   * packing (but is useful in other places as well). This buffer is not needed
   * often, but is best to have pre-allocated and in-place.
   */

  volume->pack = (FAR uint8_t *)kmalloc(volume->geo.erasesize);
  if (!volume->pack)
    {
      fdbg("Failed to allocate an I/O block buffer\n");
      ret = -ENOMEM;
      goto errout_with_cache;
    }

  /* Get the number of R/W blocks per erase block and the total number o
   * R/W blocks
   */

  volume->blkper  = volume->geo.erasesize / volume->geo.blocksize;
  volume->nblocks = volume->geo.neraseblocks * volume->blkper;
  DEBUGASSERT((off_t)volume->blkper * volume->geo.blocksize == volume->geo.erasesize);

  /* Check if there is a valid NXFFS file system on the flash */

  ret = nxffs_blockstats(volume, &stats);
  if (ret < 0)
    {
      fdbg("Failed to collect block statistics: %d\n", -ret);
      goto errout_with_buffer;
    }

  /* If the proportion of good blocks is low or the proportion of unformatted
   * blocks is high, then reformat the FLASH.
   */

  threshold = stats.nblocks / 5;
  if (stats.ngood < threshold || stats.nunformat > threshold)
    {
      /* Reformat the volume */

      ret = nxffs_reformat(volume);
      if (ret < 0)
        {
          fdbg("Failed to reformat the volume: %d\n", -ret);
          goto errout_with_buffer;
        }

      /* Get statistics on the re-formatted volume */

#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_FS)
      ret = nxffs_blockstats(volume, &stats);
      if (ret < 0)
        {
          fdbg("Failed to collect block statistics: %d\n", -ret);
          goto errout_with_buffer;
        }
#endif
    }

  /* Get the file system limits */

  ret = nxffs_limits(volume);
  if (ret == OK)
    {
      return OK;
    }
  fdbg("Failed to calculate file system limits: %d\n", -ret);

errout_with_buffer:
  kfree(volume->pack);
errout_with_cache:
  kfree(volume->cache);
errout_with_volume:
#ifndef CONFIG_NXFSS_PREALLOCATED
  kfree(volume);
#endif
  return ret;
}

/****************************************************************************
 * Name: nxffs_limits
 *
 * Description:
 *   Recalculate file system limits:  (1) the FLASH offset to the first,
 *   valid inode, and (2) the FLASH offset to the first, unused byte after
 *   the last inode (invalid or not).
 *
 *   The first, lower limit must be recalculated: (1) initially, (2)
 *   whenever the first inode is deleted, or (3) whenever inode is moved
 *   as part of the file system packing operation.
 *
 *   The second, upper limit must be (1) incremented whenever new file
 *   data is written, or (2) recalculated as part of the file system packing
 *   operation.
 *
 * Input Parameters:
 *   volume - Identifies the NXFFS volume
 *
 * Returned Value:
 *   Zero on success. Otherwise, a negated error is returned indicating the
 *   nature of the failure.
 *
 ****************************************************************************/

int nxffs_limits(FAR struct nxffs_volume_s *volume)
{
  FAR struct nxffs_entry_s entry;
  off_t block;
  off_t offset;
  bool noinodes = false;
  int nerased;
  int ret;

  /* Get the offset to the first valid block on the FLASH */

  block = 0;
  ret = nxffs_validblock(volume, &block);
  if (ret < 0)
    {
      fdbg("Failed to find a valid block: %d\n", -ret);
      return ret;
    }

  /* Then find the first valid inode in or beyond the first valid block */

  offset = block * volume->geo.blocksize;
  ret = nxffs_nextentry(volume, offset, &entry);
  if (ret < 0)
    {
      /* The value -ENOENT is special.  This simply means that the FLASH
       * was searched to the end and no valid inode was found... the file
       * system is empty (or, in more perverse cases, all inodes are
       * deleted or corrupted).
       */

      if (ret != -ENOENT)
        {
          fdbg("nxffs_nextentry failed: %d\n", -ret);
          return ret;
        }

      /* Set a flag the just indicates that no inodes were found.  Later,
       * we will set the location of the first inode to be the same as
       * the location of the free FLASH region.
       */

      fvdbg("No inodes found\n");
      noinodes = true;
    }
  else
    {
      /* Save the offset to the first inode */

      volume->inoffset = entry.hoffset;
      fvdbg("First inode at offset %d\n", volume->inoffset);

      /* Discard this entry and set the next offset. */

      offset = nxffs_inodeend(volume, &entry);
      nxffs_freeentry(&entry);
    }

  /* Now, search for the last valid entry */

  if (!noinodes)
    {
      while ((ret = nxffs_nextentry(volume, offset, &entry)) == OK)
        {
          /* Discard the entry and guess the next offset. */

          offset = nxffs_inodeend(volume, &entry);
          nxffs_freeentry(&entry);    
        }
      fvdbg("Last inode before offset %d\n", offset);
    }

  /* No inodes were found after this offset.  Now search for a block of
   * erased flash.
   */

  nxffs_ioseek(volume, offset);
  nerased = 0;
  for (;;)
    {
      int ch = nxffs_getc(volume, 1);
      if (ch < 0)
        {
          /* Failed to read the next byte... this could mean that the FLASH
           * is full?
           */

          if (volume->ioblock + 1 >= volume->nblocks &&
              volume->iooffset + 1 >= volume->geo.blocksize)
            {
              /* Yes.. the FLASH is full.  Force the offsets to the end of FLASH */

              volume->froffset = volume->nblocks * volume->geo.blocksize;
              fvdbg("Assume no free FLASH, froffset: %d\n", volume->froffset);
              if (noinodes)
                {
                  volume->inoffset = volume->froffset;
                  fvdbg("No inodes, inoffset: %d\n", volume->inoffset);
                }
              return OK;
            }

          // No?  Then it is some other failure that we do not know how to handle

          fdbg("nxffs_getc failed: %d\n", -ch);
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
              /* Okay.. we have a long stretch of erased FLASH in a valid
               * FLASH block.  Let's say that this is the beginning of
               * the free FLASH region.
               */

              volume->froffset = offset;
              fvdbg("Free FLASH region begins at offset: %d\n", volume->froffset);
              if (noinodes)
                {
                  volume->inoffset = offset;
                  fvdbg("First inode at offset %d\n", volume->inoffset);
                }
              return OK;
            }
        }
      else
        {
          offset += nerased + 1;
          nerased = 0;
        }
    }

  /* Won't get here */

  return OK;
}

/****************************************************************************
 * Name: nxffs_bind
 *
 * Description:
 *   This function implements a portion of the mount operation. Normmally,
 *   the bind() method allocates and initializes the mountpoint private data
 *   then binds the blockdriver inode to the filesystem private data.  The
 *   final binding of the private data (containing the blockdriver) to the
 *   mountpoint is performed by mount().
 *
 *   For the NXFFS, this sequence is quite different for the following
 *   reasons:
 *
 *   1. A block driver is not used.  Instead, an MTD instance was provided
 *      to nxfs_initialize prior to mounting.  So, in this sense, the NXFFS
 *      file system is already bound.
 *
 *   2. Since the volume was already bound to the MTD driver, all allocations
 *      and initializations have already been performed.  Essentially, all
 *      mount operations have been bypassed and now we just need to provide
 *      the pre-allocated volume instance.
 *
 *   3. The tricky thing is that there is no mechanism to associate multiple
 *      NXFFS volumes to the multiple volumes bound to different MTD drivers.
 *      Hence, the limitation of a single NXFFS volume.
 *
 ****************************************************************************/

int nxffs_bind(FAR struct inode *blkdriver, FAR const void *data,
               FAR void **handle)
{
#ifndef CONFIG_NXFSS_PREALLOCATED
#  error "No design to support dynamic allocation of volumes"
#else

  /* If CONFIG_NXFSS_PREALLOCATED is defined, then this is the single, pre-
   * allocated NXFFS volume instance.
   */

  DEBUGASSERT(g_volume.cache);
  *handle = &g_volume;
#endif
  return OK;
}

/****************************************************************************
 * Name: nxffs_unbind
 *
 * Description: This implements the filesystem portion of the umount
 *   operation.
 *
 ****************************************************************************/

int nxffs_unbind(FAR void *handle, FAR struct inode **blkdriver)
{
#ifndef CONFIG_NXFSS_PREALLOCATED
#  error "No design to support dynamic allocation of volumes"
#else
  return g_volume.ofiles ? -EBUSY : OK;
#endif
}
