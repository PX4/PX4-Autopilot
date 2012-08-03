/****************************************************************************
 * drivers/dev_null.c
 *
 *   Copyright (C) 2008-2009, 2012 Gregory Nutt. All rights reserved.
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
#include <string.h>
#include <poll.h>
#include <errno.h>
#include <nuttx/fs/fs.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t devzero_read(FAR struct file *, FAR char *, size_t);
static ssize_t devzero_write(FAR struct file *, FAR const char *, size_t);
#ifndef CONFIG_DISABLE_POLL
static int     devzero_poll(FAR struct file *filp, FAR struct pollfd *fds,
                            bool setup);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations devzero_fops =
{
  0,             /* open */
  0,             /* close */
  devzero_read,  /* read */
  devzero_write, /* write */
  0,             /* seek */
  0              /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , devzero_poll /* poll */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devzero_read
 ****************************************************************************/

static ssize_t devzero_read(FAR struct file *filp, FAR char *buffer, size_t len)
{
  memset(buffer, 0, len);
  return len;
}

/****************************************************************************
 * Name: devzero_write
 ****************************************************************************/

static ssize_t devzero_write(FAR struct file *filp, FAR const char *buffer, size_t len)
{
  return len;
}

/****************************************************************************
 * Name: devzero_poll
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
static int devzero_poll(FAR struct file *filp, FAR struct pollfd *fds,
                        bool setup)
{
  if (setup)
    {
      fds->revents |= (fds->events & (POLLIN|POLLOUT));
      if (fds->revents != 0)
        {
          sem_post(fds->sem);
        }
    }
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devzero_register
 *
 * Description:
 *   Register /dev/zero
 *
 ****************************************************************************/

void devzero_register(void)
{
  (void)register_driver("/dev/zero", &devzero_fops, 0666, NULL);
}
