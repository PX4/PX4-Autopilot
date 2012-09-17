/****************************************************************************
 * drivers/bch/bchdev_driver.c
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
 * Compilation Switches
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <sched.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>

#include "bch_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     bch_open(FAR struct file *filp);
static int     bch_close(FAR struct file *filp);
static ssize_t bch_read(FAR struct file *, FAR char *, size_t);
static ssize_t bch_write(FAR struct file *, FAR const char *, size_t);
static int     bch_ioctl(FAR struct file *filp, int cmd, unsigned long arg);

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct file_operations bch_fops =
{
  bch_open,  /* open */
  bch_close, /* close */
  bch_read,  /* read */
  bch_write, /* write */
  0,         /* seek */
  bch_ioctl  /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , 0        /* poll */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bch_open
 *
 * Description: Open the block device
 *
 ****************************************************************************/

static int bch_open(FAR struct file *filp)
{
  FAR struct inode *inode = filp->f_inode;
  FAR struct bchlib_s *bch;

  DEBUGASSERT(inode && inode->i_private);
  bch = (FAR struct bchlib_s *)inode->i_private;

  /* Increment the reference count */

  bchlib_semtake(bch);
  if (bch->refs == MAX_OPENCNT)
    {
      return -EMFILE;
    }
  else
    {
      bch->refs++;
    }
  bchlib_semgive(bch);

  return OK;
}

/****************************************************************************
 * Name: bch_close
 *
 * Description: close the block device
 *
 ****************************************************************************/

static int bch_close(FAR struct file *filp)
{
  FAR struct inode *inode = filp->f_inode;
  FAR struct bchlib_s *bch;
  int ret = OK;

  DEBUGASSERT(inode && inode->i_private);
  bch = (FAR struct bchlib_s *)inode->i_private;

  /* Flush any dirty pages remaining in the cache */

  bchlib_semtake(bch);
  (void)bchlib_flushsector(bch);

  /* Decrement the reference count (I don't use bchlib_decref() because I
   * want the entire close operation to be atomic wrt other driver operations.
   */

  if (bch->refs == 0)
    {
      ret = -EIO;
    }
  else
    {
      bch->refs--;
    }
  bchlib_semgive(bch);

  return ret;
}

/****************************************************************************
 * Name:bch_read
 ****************************************************************************/

static ssize_t bch_read(FAR struct file *filp, FAR char *buffer, size_t len)
{
  FAR struct inode *inode = filp->f_inode;
  FAR struct bchlib_s *bch;
  int ret;

  DEBUGASSERT(inode && inode->i_private);
  bch = (FAR struct bchlib_s *)inode->i_private;

  bchlib_semtake(bch);
  ret = bchlib_read(bch, buffer, filp->f_pos, len);
  if (ret > 0)
    {
      filp->f_pos += len;
    }
  bchlib_semgive(bch);
  return ret;
}

/****************************************************************************
 * Name:bch_write
 ****************************************************************************/

static ssize_t bch_write(FAR struct file *filp, FAR const char *buffer, size_t len)
{
  FAR struct inode *inode = filp->f_inode;
  FAR struct bchlib_s *bch;
  int ret = -EACCES;

  DEBUGASSERT(inode && inode->i_private);
  bch = (FAR struct bchlib_s *)inode->i_private;

  if (!bch->readonly)
    {
      bchlib_semtake(bch);
      ret = bchlib_write(bch, buffer, filp->f_pos, len);
      if (ret > 0)
        {
          filp->f_pos += len;
        }
      bchlib_semgive(bch);
    }

  return ret;
}

/****************************************************************************
 * Name: bch_ioctl
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int bch_ioctl(FAR struct file *filp, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filp->f_inode;
  FAR struct bchlib_s *bch;
  int ret = -ENOTTY;

  DEBUGASSERT(inode && inode->i_private);
  bch = (FAR struct bchlib_s *)inode->i_private;

  if (cmd == DIOC_GETPRIV)
    {
      FAR struct bchlib_s **bchr = (FAR struct bchlib_s **)((uintptr_t)arg);

      bchlib_semtake(bch);
      if (!bchr && bch->refs < 255)
        {
          ret = -EINVAL;
        }
      else
        {
          bch->refs++;
          *bchr = bch;
        }
      bchlib_semgive(bch);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
