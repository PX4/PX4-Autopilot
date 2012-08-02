/****************************************************************************
 * drivers/syslog/syslog.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include <nuttx/syslog.h>

#if defined(CONFIG_SYSLOG) && defined(CONFIG_SYSLOG_CHAR)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct syslog_dev_s
{
  int fd; /* File descriptor of the opened SYSLOG character device */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the device structure for the console or syslogging function.  It
 * must be statically initialized because the RAMLOG syslog_putc function
 * could be called before the driver initialization logic executes.
 */

static struct syslog_dev_s g_sysdev      = { -1 };
static const uint8_t       g_syscrlf[2]  = { '\r', '\n' };

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: syslog_initialize
 *
 * Description:
 *   Initialize to use the character device at CONFIG_SYSLOG_DEVPATH as the
 *   SYSLOG.
 *
 ****************************************************************************/

int syslog_initialize(void)
{
  /* Has the device been opened yet */

  if (g_sysdev.fd < 0)
    {
      /* No, try to open the device now: write-only, try to create it if
       * it doesn't exist (it might be a file), if it is a file that already
       * exists, then append new log data to the file.
       */

      g_sysdev.fd = open(CONFIG_SYSLOG_DEVPATH, O_WRONLY | O_CREAT | O_APPEND, 0644);
      if (g_sysdev.fd < 0)
        {
          return -get_errno();
        }
    }

  /* The SYSLOG device is open and ready for writing. */

  return OK;
}

/****************************************************************************
 * Name: syslog_putc
 *
 * Description:
 *   This is the low-level system logging interface.  The debugging/syslogging
 *   interfaces are lib_rawprintf() and lib_lowprinf().  The difference is
 *   the lib_rawprintf() writes to fd=1 (stdout) and lib_lowprintf() uses
 *   a lower level interface that works from interrupt handlers.  This
 *   function is a a low-level interface used to implement lib_lowprintf().
 *
 ****************************************************************************/

int syslog_putc(int ch)
{
  ssize_t nbytes;
  int ret;

  /* Try to initialize the device.  We do this repeatedly because the log
   * device might be something that was not ready the first time that
   * syslog_intialize() was called (such as a USB serial device or a file
   * in an NFS mounted file system.
   */

  ret = syslog_initialize();
  if (ret < 0)
    {
      return ret;
    }

  /* Ignore carriage returns */

  if (ch == '\r')
    {
      return ch;
    }

  /* Pre-pend a newline with a carriage return */

  if (ch == '\n')
    {
      nbytes = write(g_sysdev.fd, g_syscrlf, 2);
    }
  else
    {
      nbytes = write(g_sysdev.fd, &ch, 1);
    }

  if (nbytes < 0)
    {
      return nbytes;
    }

  return ch;
}

#endif /* CONFIG_SYSLOG && CONFIG_SYSLOG_CHAR */
