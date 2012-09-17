/****************************************************************************
 * fs/nxffs/nxffs_dirent.c
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
#include <dirent.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/mtd.h>
#include <nuttx/fs/dirent.h>

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
 * Name: nxffs_opendir
 *
 * Description:
 *   Open a directory for read access
 *
 ****************************************************************************/

int nxffs_opendir(FAR struct inode *mountpt, FAR const char *relpath,
                  FAR struct fs_dirent_s *dir)
{
  struct nxffs_volume_s *volume;
  int ret;

  fvdbg("relpath: \"%s\"\n", relpath ? relpath : "NULL");

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover the file system state from the NuttX inode instance */

  volume = mountpt->i_private;
  ret = sem_wait(&volume->exclsem);
  if (ret != OK)
    {
      goto errout;
    }

  /* The requested directory must be the volume-relative "root" directory */

  if (relpath && relpath[0] != '\0')
    {
      ret = -ENOENT;
      goto errout_with_semaphore;
    }

  /* Set the offset to the offset to the first valid inode */

  dir->u.nxffs.nx_offset = volume->inoffset;
  ret = OK;

errout_with_semaphore:
  sem_post(&volume->exclsem);
errout:
  return ret;
}

/****************************************************************************
 * Name: nxffs_readdir
 *
 * Description: Read the next directory entry
 *
 ****************************************************************************/

int nxffs_readdir(FAR struct inode *mountpt, FAR struct fs_dirent_s *dir)
{
  FAR struct nxffs_volume_s *volume;
  FAR struct nxffs_entry_s entry;
  off_t offset;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover the file system state from the NuttX inode instance */

  volume = mountpt->i_private;
  ret = sem_wait(&volume->exclsem);
  if (ret != OK)
    {
      goto errout;
    }

  /* Read the next inode header from the offset */

  offset = dir->u.nxffs.nx_offset;
  ret = nxffs_nextentry(volume, offset, &entry);

  /* If the read was successful, then handle the reported inode.  Note
   * that when the last inode has been reported, the value -ENOENT will
   * be returned.. which is correct for the readdir() method.
   */

  if (ret == OK)
    {
      /* Return the filename and file type */

      fvdbg("Offset %d: \"%s\"\n", entry.hoffset, entry.name);
      dir->fd_dir.d_type = DTYPE_FILE;
      strncpy(dir->fd_dir.d_name, entry.name, NAME_MAX+1);

      /* Discard this entry and set the next offset. */

      dir->u.nxffs.nx_offset = nxffs_inodeend(volume, &entry);
      nxffs_freeentry(&entry);
      ret = OK;
    }

  sem_post(&volume->exclsem);
errout:
  return ret;
}

/****************************************************************************
 * Name: nxffs_rewindir
 *
 * Description:
 *   Reset directory read to the first entry
 *
 ****************************************************************************/

int nxffs_rewinddir(FAR struct inode *mountpt, FAR struct fs_dirent_s *dir)
{
  FAR struct nxffs_volume_s *volume;
  int ret;

  fvdbg("Entry\n");

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover the file system state from the NuttX inode instance */

  volume = mountpt->i_private;
  ret = sem_wait(&volume->exclsem);
  if (ret != OK)
    {
      goto errout;
    }

  /* Reset the offset to the FLASH offset to the first valid inode */

  dir->u.nxffs.nx_offset = volume->inoffset;
  ret = OK;

  sem_post(&volume->exclsem);
errout:
  return ret;
}
