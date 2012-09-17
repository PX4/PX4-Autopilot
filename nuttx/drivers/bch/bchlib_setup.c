/****************************************************************************
 * drivers/bch/bchlib_setup.c
 *
 *   Copyright (C) 2008-2009, 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <sys/types.h>
#include <sys/mount.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>

#include "bch_internal.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bchlib_setup
 *
 * Description:
 *   Setup so that the block driver referenced by 'blkdev' can be accessed
 *   similar to a character device.
 *
 ****************************************************************************/

int bchlib_setup(const char *blkdev, bool readonly, FAR void **handle)
{
  FAR struct bchlib_s *bch;
  struct geometry geo;
  int ret;

  DEBUGASSERT(blkdev);

  /* Allocate the BCH state structure */

  bch = (FAR struct bchlib_s*)kzalloc(sizeof(struct bchlib_s));
  if (!bch)
    {
      fdbg("Failed to allocate BCH structure\n");
      return -ENOMEM;
    }

  /* Open the block driver */

  ret = open_blockdriver(blkdev, readonly ? MS_RDONLY : 0, &bch->inode);
  if (ret < 0)
    {
      fdbg("Failed to open driver %s: %d\n", blkdev, -ret);
      goto errout_with_bch;
    }

  DEBUGASSERT(bch->inode && bch->inode->u.i_bops && bch->inode->u.i_bops->geometry);

  ret = bch->inode->u.i_bops->geometry(bch->inode, &geo);
  if (ret < 0)
    {
      fdbg("geometry failed: %d\n", -ret);
      goto errout_with_bch;
    }

  if (!geo.geo_available)
    {
      fdbg("geometry failed: %d\n", -ret);
      ret = -ENODEV;
      goto errout_with_bch;
    }

  if (!readonly && (!bch->inode->u.i_bops->write || !geo.geo_writeenabled))
    {
      fdbg("write access not supported\n");
      ret = -EACCES;
      goto errout_with_bch;
    }

  /* Save the geometry info and complete initialization of the structure */

  sem_init(&bch->sem, 0, 1);
  bch->nsectors = geo.geo_nsectors;
  bch->sectsize = geo.geo_sectorsize;
  bch->sector   = (size_t)-1;
  bch->readonly = readonly;

  /* Allocate the sector I/O buffer */

  bch->buffer = (FAR uint8_t *)kmalloc(bch->sectsize);
  if (!bch->buffer)
    {
      fdbg("Failed to allocate sector buffer\n");
      ret = -ENOMEM;
      goto errout_with_bch;
    }

  *handle = bch;
  return OK;

errout_with_bch:
  kfree(bch);
  return ret;
}
