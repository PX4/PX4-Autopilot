/****************************************************************************
 * drivers/bch/bchlib_cache.c
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

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
 * Name: bchlib_flushsector
 *
 * Description:
 *   Flush the current contents of the sector buffer (if dirty)
 *
 * Assumptions:
 *   Caller must assume mutual exclusion
 *
 ****************************************************************************/

int bchlib_flushsector(FAR struct bchlib_s *bch)
{
  FAR struct inode *inode;
  ssize_t ret = OK;

  if (bch->dirty)
    {
      inode = bch->inode;
      ret = inode->u.i_bops->write(inode, bch->buffer, bch->sector, 1);
      if (ret < 0)
        {
          fdbg("Write failed: %d\n");
        }
      bch->dirty = false;
    }
  return (int)ret;
}

/****************************************************************************
 * Name: bchlib_readsector
 *
 * Description:
 *   Flush the current contents of the sector buffer (if dirty)
 *
 * Assumptions:
 *   Caller must assume mutual exclusion
 *
 ****************************************************************************/

int bchlib_readsector(FAR struct bchlib_s *bch, size_t sector)
{
  FAR struct inode *inode;
  ssize_t ret = OK;

  if (bch->sector != sector)
    {
      inode = bch->inode;

      (void)bchlib_flushsector(bch);
      bch->sector = (size_t)-1;

      ret = inode->u.i_bops->read(inode, bch->buffer, sector, 1);
      if (ret < 0)
        {
          fdbg("Read failed: %d\n");
        }
      bch->sector = sector;
    }
  return (int)ret;
}

