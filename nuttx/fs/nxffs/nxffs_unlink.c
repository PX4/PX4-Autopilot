/****************************************************************************
 * fs/nxffs/nxffs_unlink.c
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxffs_rminode
 *
 * Description:
 *   Remove an inode from FLASH.  This is the internal implementation of
 *   the file system unlinke operation.
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume.
 *   name - the name of the inode to be deleted.
 *
 * Returned Value:
 *   Zero is returned if the inode is successfully deleted.  Otherwise, a
 *   negated errno value is returned indicating the nature of the failure.
 *
 ****************************************************************************/

int nxffs_rminode(FAR struct nxffs_volume_s *volume, FAR const char *name)
{
  FAR struct nxffs_ofile_s *ofile;
  FAR struct nxffs_inode_s *inode;
  struct nxffs_entry_s entry;
  int ret;

  /* Check if the file is open */

  ofile = nxffs_findofile(volume, name);
  if (ofile)
    {
      /* We can't remove the inode if it is open */

      fdbg("Inode '%s' is open\n", name);
      ret = -EBUSY;
      goto errout;
    }

  /* Find the NXFFS inode */

  ret = nxffs_findinode(volume, name, &entry);
  if (ret < 0)
    {
      fdbg("Inode '%s' not found\n", name);
      goto errout;
    }

  /* Set the position to the FLASH offset of the file header (nxffs_findinode
   * should have left the block in the cache).
   */

  nxffs_ioseek(volume, entry.hoffset);

  /* Make sure the the block is in the cache */

  ret = nxffs_rdcache(volume, volume->ioblock);
  if (ret < 0)
    {
      fdbg("Failed to read data into cache: %d\n", ret);
      goto errout_with_entry;
    }

  /* Change the file status... it is no longer valid */

  inode = (FAR struct nxffs_inode_s *)&volume->cache[volume->iooffset];
  inode->state = INODE_STATE_DELETED;

  /* Then write the cached block back to FLASH */

  ret = nxffs_wrcache(volume);
  if (ret < 0)
    {
      fdbg("Failed to read data into cache: %d\n", ret);
    }

errout_with_entry:
  nxffs_freeentry(&entry);
errout:
  return ret;
}

/****************************************************************************
 * Name: nxffs_unlink
 *
 * Description: Remove a file
 *
 ****************************************************************************/

int nxffs_unlink(FAR struct inode *mountpt, FAR const char *relpath)
{
  FAR struct nxffs_volume_s *volume;
  int ret;

  fvdbg("Entry\n");

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the NuttX inode structure */

  volume = mountpt->i_private;
  ret = sem_wait(&volume->exclsem);
  if (ret != OK)
    {
      goto errout;
    }

  /* Then remove the NXFFS inode */

  ret = nxffs_rminode(volume, relpath);
  
  sem_post(&volume->exclsem);
errout:
  return ret;
}
