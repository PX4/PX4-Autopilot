/****************************************************************************
 * drivers/pipes/fifo.c
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
#include <stdint.h>
#include <nuttx/fs/fs.h>
#include <errno.h>

#include "pipe_common.h"

#if CONFIG_DEV_PIPE_SIZE > 0

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
 * Private Data
 ****************************************************************************/

static const struct file_operations fifo_fops =
{
  pipecommon_open,  /* open */
  pipecommon_close, /* close */
  pipecommon_read,  /* read */
  pipecommon_write, /* write */
  0,                /* seek */
  0                 /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , pipecommon_poll /* poll */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mkfifo
 *
 * Description:
 *   mkfifo() makes a FIFO device driver file with name 'pathname.'  Unlike
 *   Linux, a NuttX FIFO is not a special file type but simply a device driver
 *   instance.  'mode' specifies the FIFO's permissions. 
 *
 *   Once the FIFO has been created by mkfifo(), any thread can open it for
 *   reading or writing, in the same way as an ordinary file. However, it must
 *   have been opened from both reading and writing before input or output
 *   can be performed.  This FIFO implementation will block all attempts to
 *   open a FIFO read-only until at least one thread has opened the FIFO for
 *   writing.
 *
 *   If all threads that write to the FIFO have closed, subsequent calls to
 *   read() on the FIFO will return 0 (end-of-file).
 *
 * Inputs:
 *   pathname - The full path to the FIFO instance to attach to or to create
 *     (if not already created).
 *   mode - Ignored for now
 *
 * Return:
 *   0 is returned on success; otherwise, -1 is returned with errno set
 *   appropriately.
 *
 ****************************************************************************/

int mkfifo(FAR const char *pathname, mode_t mode)
{
  struct pipe_dev_s *dev;
  int ret;
 
  /* Allocate and initialize a new device structure instance */

  dev = pipecommon_allocdev();
  if (!dev)
    {
      return -ENOMEM;
    }

  ret = register_driver(pathname, &fifo_fops, mode, (void*)dev);
  if (ret != 0)
    {
      pipecommon_freedev(dev);
    }

  return ret;
}

#endif /* CONFIG_DEV_PIPE_SIZE > 0 */
