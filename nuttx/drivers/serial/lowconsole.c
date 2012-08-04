/****************************************************************************
 * drivers/serial/lowconsole.c
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
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* The architecture must provide up_putc for this driver */

#ifndef CONFIG_ARCH_LOWPUTC
#  error "Architecture must provide up_putc() for this driver"
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t lowconsole_read(struct file *filep, char *buffer, size_t buflen);
static ssize_t lowconsole_write(struct file *filep, const char *buffer, size_t buflen);
static int     lowconsole_ioctl(struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Variables
 ****************************************************************************/

static const struct file_operations g_consoleops =
{
  0,                /* open */
  0,                /* close */
  lowconsole_read,  /* read */
  lowconsole_write, /* write */
  0,                /* seek */
  lowconsole_ioctl  /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , 0               /* poll */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lowconsole_ioctl
 ****************************************************************************/

static int lowconsole_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: lowconsole_read
 ****************************************************************************/

static ssize_t lowconsole_read(struct file *filep, char *buffer, size_t buflen)
{
  return 0;
}

/****************************************************************************
 * Name: lowconsole_write
 ****************************************************************************/

static ssize_t lowconsole_write(struct file *filep, const char *buffer, size_t buflen)
{
  ssize_t ret = buflen;
 
  for (; buflen; buflen--)
    {
      up_putc(*buffer++);
    }
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lowconsole_init
****************************************************************************/

void lowconsole_init(void)
{
  (void)register_driver("/dev/console", &g_consoleops, 0666, NULL);
}
