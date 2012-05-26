/****************************************************************************
 * apps/nshlib/nsh_usbdev.c
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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <assert.h>

#ifdef CONFIG_CDCACM
#  include <nuttx/usb/cdcacm.h>
#endif

#ifdef CONFIG_CDCACM
#  include <nuttx/usb/pl2303.h>
#endif

#include "nsh.h"

#ifdef CONFIG_USBDEV

/****************************************************************************
 * Definitions
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

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nsh_usbconsole
 ****************************************************************************/

#ifdef HAVE_USB_CONSOLE
int nsh_usbconsole(void)
{
  char inch;
  ssize_t nbytes;
  int nlc;
  int fd;
  int ret;

  /* Initialize any USB tracing options that were requested */

  usbtrace_enable(CONFIG_NSH_UBSDEV_TRACEINIT);

  /* Don't start the NSH console until the console device is ready.  Chances
   * are, we get here with no functional console.  The USB console will not
   * be available until the device is connected to the host and until the
   * host-side application opens the connection.
   */

  /* Initialize the USB serial driver */

#if defined(CONFIG_PL2303) || defined(CONFIG_CDCACM)
#ifdef CONFIG_CDCACM
  ret = cdcacm_initialize(CONFIG_NSH_UBSDEV_MINOR, NULL);
#else
  ret = usbdev_serialinitialize(CONFIG_NSH_UBSDEV_MINOR);
#endif
  DEBUGASSERT(ret == OK);
#endif

  /* Open the USB serial device for read/write access */

  do
    {
      /* Try to open the console */

      fd = open(CONFIG_NSH_USBCONDEV, O_RDWR);
      if (fd < 0)
        {
          int errval = errno;

          /* ENOTCONN means that the USB device is not yet connected. Anything
           * else is bad.
           */

          DEBUGASSERT(errval == ENOTCONN);

          /* Sleep a bit and try again */

          sleep(2);
        }
    }
  while (fd < 0);

  /* Now waiting until we successfully read a carriage return a few times. 
   * That is a sure way of know that there is something at the other end of
   * the USB serial connection that is ready to talk with us.  The user needs
   * to hit ENTER a few times to get things started.
   */

  nlc = 0;
  do
    {
      /* Read one byte */

      inch = 0;
      nbytes = read(fd, &inch, 1);

      /* Is it a carriage return (or maybe a newline)? */

      if (nbytes == 1 && (inch == '\n' || inch == '\r'))
        {
          /* Yes.. increment the count */

          nlc++;
        }
      else
        {
          /* No.. Reset the count.  We need to see 3 in a row to continue. */

          nlc = 0;
        }
    }
  while (nlc < 3);

  /* Make sure the stdin, stdout, and stderr are closed */

  (void)fclose(stdin);
  (void)fclose(stdout);
  (void)fclose(stderr);

  /* Dup the fd to create standard fd 0-2 */

  (void)dup2(fd, 0);
  (void)dup2(fd, 1);
  (void)dup2(fd, 2);

  /* We can close the original file descriptor now (unless it was one of 0-2) */

  if (fd > 2)
    {
      close(fd);
    }

  /* fdopen to get the stdin, stdout and stderr streams. The following logic depends
   * on the fact that the library layer will allocate FILEs in order.  And since
   * we closed stdin, stdout, and stderr above, that is what we should get.
   *
   * fd = 0 is stdin  (read-only)
   * fd = 1 is stdout (write-only, append)
   * fd = 2 is stderr (write-only, append)
   */

  (void)fdopen(0, "r");
  (void)fdopen(1, "a");
  (void)fdopen(2, "a");
  return OK;
}

#endif /* HAVE_USB_CONSOLE */
#endif /* CONFIG_USBDEV */
