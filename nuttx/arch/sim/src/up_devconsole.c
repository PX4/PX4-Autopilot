/****************************************************************************
 * up_devconsole.c
 *
 *   Copyright (C) 2007-2009 Gregory Nutt. All rights reserved.
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

#include <nuttx/fs/fs.h>

#include "up_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t devconsole_read(struct file *, char *, size_t);
static ssize_t devconsole_write(struct file *, const char *, size_t);
#ifndef CONFIG_DISABLE_POLL
static int     devconsole_poll(FAR struct file *filep, FAR struct pollfd *fds,
                               bool setup);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations devconsole_fops =
{
  .read		= devconsole_read,
  .write	= devconsole_write,
#ifndef CONFIG_DISABLE_POLL
  .poll         = devconsole_poll,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static ssize_t devconsole_read(struct file *filp, char *buffer, size_t len)
{
  return up_hostread(buffer, len);
}

static ssize_t devconsole_write(struct file *filp, const char *buffer, size_t len)
{
  return up_hostwrite(buffer, len);
}

#ifndef CONFIG_DISABLE_POLL
static int devconsole_poll(FAR struct file *filep, FAR struct pollfd *fds,
                           bool setup)
{
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_devconsole(void)
{
  (void)register_driver("/dev/console", &devconsole_fops, 0666, NULL);
}
