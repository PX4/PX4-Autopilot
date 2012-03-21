/****************************************************************************
 * examples/pashello/device.c
 *
 *   Copyright (C) 2008, 2012 Gregory Nutt. All rights reserved.
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
#include <string.h>
#include <errno.h>
#include <nuttx/fs/fs.h>

#include "hello.h"
#include "pashello.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t hello_read(struct file *, char *, size_t);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations hello_fops =
{
  0,             /* open */
  0,             /* close */
  hello_read,    /* read */
  0,             /* write */
  0,             /* seek */
  0,             /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  0              /* poll */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static ssize_t hello_read(struct file *filep, char *buffer, size_t len)
{
  off_t offset  = filep->f_pos;  /* Start read position */
  ssize_t nread = 0;             /* Bytes read -- assume EOF */

  /* Make sure that the offset is within the .pex file */

  if (offset < hello_pex_len)
    {
      /* Make sure the read does not extend beyond the .pex file */

      nread = len;
      if (nread + offset > hello_pex_len)
        {
          nread = hello_pex_len - offset;
        }
      memcpy(buffer, &hello_pex[offset], nread);
      filep->f_pos += nread;
    }
  return nread;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void hello_register(void)
{
  (void)register_driver("/dev/hello", &hello_fops, 0444, NULL);
}
