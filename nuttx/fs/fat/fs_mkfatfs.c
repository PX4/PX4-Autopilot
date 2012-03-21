/****************************************************************************
 * fs/fat/fs_writefat.c
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/fat.h>
#include <nuttx/fs/mkfatfs.h>

#include "fs_internal.h"
#include "fs_fat32.h"
#include "fs_mkfatfs.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mkfatfs_getgeometry
 *
 * Description:
 *   Get the sector size and number of sectors of the underlying block
 *   device.
 *
 * Input:
 *   fmt - Caller specified format parameters
 *   var - Other format parameters that are not caller specifiable. (Most
 *     set by mkfatfs_configfatfs()).
 *
 * Return:
 *   Zero on success; negated errno on failure
 *
 ****************************************************************************/

static inline int mkfatfs_getgeometry(FAR struct fat_format_s *fmt,
                                      FAR struct fat_var_s *var)
{
  struct geometry geometry;
  int ret;
 
  /* Get the device geometry */

  ret = DEV_GEOMETRY(geometry);
  if (ret < 0)
    {
      fdbg("geometry() returned %d\n", ret);
      return ret;
    }

  if (!geometry.geo_available || !geometry.geo_writeenabled)
    {
      fdbg("Media is not available\n", ret);
      return -ENODEV;
    }

  /* Check if the user provided maxblocks was provided and, if so, that is it less than
   * the actual number of blocks on the device.
   */

  if (fmt->ff_nsectors != 0)
    {
      if (fmt->ff_nsectors > geometry.geo_nsectors)
        {
          fdbg("User maxblocks (%d) exceeds blocks on device (%d)\n",
               fmt->ff_nsectors, geometry.geo_nsectors);
          return -EINVAL;
        }
    }
  else
    {
      /* Use the actual number of blocks on the device */

      fmt->ff_nsectors = geometry.geo_nsectors;
    }

  /* Verify that we can handle this sector size */

  var->fv_sectorsize = geometry.geo_sectorsize;
  switch (var->fv_sectorsize)
    {
      case 512:
        var->fv_sectshift = 9;
        break;

      case 1024:
        var->fv_sectshift = 10;
        break;

      case 2048:
        var->fv_sectshift = 11;
        break;

      case 4096:
        var->fv_sectshift = 12;
        break;

      default:
        fdbg("Unsupported sector size: %d\n", var->fv_sectorsize);
        return -EPERM;
    }
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mkfatfs
 *
 * Description:
 *   Make a FAT file system image on the specified block device
 *
 * Inputs:
 *   pathname - the full path to a registered block driver
 *   fmt - Describes characteristics of the desired filesystem
 *
 * Return:
 *   Zero (OK) on success; -1 (ERROR) on failure with errno set appropriately:
 *
 *   EINVAL  - NULL block driver string, bad number of FATS in 'fmt', bad FAT
 *     size in 'fmt', bad cluster size in 'fmt'
 *   ENOENT  - 'pathname' does not refer to anything in the filesystem.
 *   ENOTBLK - 'pathname' does not refer to a block driver
 *   EACCES  - block driver does not support wrie or geometry methods
 *
 * Assumptions:
 *   - The caller must assure that the block driver is not mounted and not in
 *     use when this function is called.  The result of formatting a mounted
 *     device is indeterminate (but likely not good).
 *
 ****************************************************************************/
int mkfatfs(FAR const char *pathname, FAR struct fat_format_s *fmt)
{
  struct fat_var_s var;
  int ret;

  /* Initialize */

  memset(&var, 0, sizeof(struct fat_var_s));

  /* Get the filesystem creation time */

  var.fv_createtime = fat_systime2fattime();

  /* Verify format options (only when DEBUG enabled) */

#ifdef CONFIG_DEBUG
  if (!pathname)
    {
      fdbg("No block driver path\n");
      ret = -EINVAL;
      goto errout;
    }

  if (fmt->ff_nfats < 1 || fmt->ff_nfats > 4)
    {
      fdbg("Invalid number of fats: %d\n", fmt->ff_nfats);
      ret = -EINVAL;
      goto errout;
    }

  if (fmt->ff_fattype != 0  && fmt->ff_fattype != 12 &&
      fmt->ff_fattype != 16 && fmt->ff_fattype != 32)
    {
      fdbg("Invalid FAT size: %d\n", fmt->ff_fattype);
      ret = -EINVAL;
      goto errout;
    }
#endif
  var.fv_fattype = fmt->ff_fattype;

  /* The valid range off ff_clustshift is {0,1,..7} corresponding to
   * cluster sizes of {1,2,..128} sectors.  The special value of 0xff
   * means that we should autoselect the cluster sizel.
   */
#ifdef CONFIG_DEBUG
  if (fmt->ff_clustshift > 7 && fmt->ff_clustshift != 0xff)
    {
      fdbg("Invalid cluster shift value: %d\n", fmt->ff_clustshift);
      ret = -EINVAL;
      goto errout;
    }

   if (fmt->ff_rootdirentries != 0 && (fmt->ff_rootdirentries < 16 || fmt->ff_rootdirentries > 32767))
    {
      fdbg("Invalid number of root dir entries: %d\n", fmt->ff_rootdirentries);
      ret = -EINVAL;
      goto errout;
    }

   if (fmt->ff_rsvdseccount != 0 && (fmt->ff_rsvdseccount < 1 || fmt->ff_rsvdseccount > 32767))
    {
      fdbg("Invalid number of reserved sectors: %d\n", fmt->ff_rsvdseccount);
      ret = -EINVAL;
      goto errout;
    }
#endif

  /* Find the inode of the block driver indentified by 'source' */

  ret = open_blockdriver(pathname, 0, &var.fv_inode);
  if (ret < 0)
    {
      fdbg("Failed to open %s\n", pathname);
      goto errout;
    }

  /* Make sure that the inode supports the write and geometry methods at a minimum */

  if (!var.fv_inode->u.i_bops->write || !var.fv_inode->u.i_bops->geometry)
    {
      fdbg("%s does not support write or geometry methods\n", pathname);
      ret = -EACCES;
      goto errout_with_driver;
    }

  /* Determine the volume configuration based upon the input values and upon the
   * reported device geometry.
   */

  ret = mkfatfs_getgeometry(fmt, &var);
  if (ret < 0)
    {
      goto errout_with_driver;
    }

  /* Configure the file system */

  ret = mkfatfs_configfatfs(fmt, &var);
  if (ret < 0)
    {
      goto errout_with_driver;
    }

  /* Allocate a buffer that will be working sector memory */

  var.fv_sect = (uint8_t*)malloc(var.fv_sectorsize);
  if (!var.fv_sect)
    {
      fdbg("Failed to allocate working buffers\n");
      goto errout_with_driver;
    }

  /* Write the filesystem to media */

  ret = mkfatfs_writefatfs(fmt, &var);

errout_with_driver:
  /* Close the driver */

  (void)close_blockdriver(var.fv_inode);

errout:
  /* Release all allocated memory */

  if (var.fv_sect)
    {
      free(var.fv_sect);
    }

    /* Return any reported errors */

    if (ret < 0)
      {
        errno = -ret;
        return ERROR; 
      }
    return OK;
}
