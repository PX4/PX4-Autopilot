/****************************************************************************
 * examples/nxtext/nxtext_server.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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
#include <sched.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/nx/nx.h>

#ifdef CONFIG_NX_LCDDRIVER
#  include <nuttx/lcd/lcd.h>
#else
#  include <nuttx/fb.h>
#endif

#include "nxtext_internal.h"

#ifdef CONFIG_NX_MULTIUSER

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
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
 * Name: nxtext_server
 ****************************************************************************/

int nxtext_server(int argc, char *argv[])
{
  FAR NX_DRIVERTYPE *dev;
  int ret;

#if defined(CONFIG_EXAMPLES_NXTEXT_EXTERNINIT)
  /* Use external graphics driver initialization */

  message("nxtext_server: Initializing external graphics device\n");
  dev = up_nxdrvinit(CONFIG_EXAMPLES_NXTEXT_DEVNO);
  if (!dev)
    {
      message("nxtext_server: up_nxdrvinit failed, devno=%d\n", CONFIG_EXAMPLES_NXTEXT_DEVNO);
      g_exitcode = NXEXIT_EXTINITIALIZE;
      return ERROR;
    }

#elif defined(CONFIG_NX_LCDDRIVER)
  /* Initialize the LCD device */

  message("nxtext_server: Initializing LCD\n");
  ret = up_lcdinitialize();
  if (ret < 0)
    {
      message("nxtext_server: up_lcdinitialize failed: %d\n", -ret);
      return 1;
    }

  /* Get the device instance */

  dev = up_lcdgetdev(CONFIG_EXAMPLES_NXTEXT_DEVNO);
  if (!dev)
    {
      message("nxtext_server: up_lcdgetdev failed, devno=%d\n", CONFIG_EXAMPLES_NXTEXT_DEVNO);
      return 2;
    }

  /* Turn the LCD on at 75% power */

  (void)dev->setpower(dev, ((3*CONFIG_LCD_MAXPOWER + 3)/4));
#else
  /* Initialize the frame buffer device */

  message("nxtext_server: Initializing framebuffer\n");
  ret = up_fbinitialize();
  if (ret < 0)
    {
      message("nxtext_server: up_fbinitialize failed: %d\n", -ret);
      return 1;
    }

  dev = up_fbgetvplane(CONFIG_EXAMPLES_NXTEXT_VPLANE);
  if (!dev)
    {
      message("nxtext_server: up_fbgetvplane failed, vplane=%d\n", CONFIG_EXAMPLES_NXTEXT_VPLANE);
      return 2;
    }
#endif

  /* Then start the server */

  ret = nx_run(dev);
  gvdbg("nx_run returned: %d\n", errno);
  return 3;
}

/****************************************************************************
 * Name: nxtext_listener
 ****************************************************************************/

FAR void *nxtext_listener(FAR void *arg)
{
  int ret;

  /* Process events forever */

  for (;;)
    {
      /* Handle the next event.  If we were configured blocking, then
       * we will stay right here until the next event is received.  Since
       * we have dedicated a while thread to servicing events, it would
       * be most natural to also select CONFIG_NX_BLOCKING -- if not, the
       * following would be a tight infinite loop (unless we added addition
       * logic with nx_eventnotify and sigwait to pace it).
       */

      ret = nx_eventhandler(g_hnx);
      if (ret < 0)
        {
          /* An error occurred... assume that we have lost connection with
           * the server.
           */

          message("nxtext_listener: Lost server connection: %d\n", errno);
          exit(NXEXIT_LOSTSERVERCONN);
        }

      /* If we received a message, we must be connected */

      if (!g_connected)
        {
          g_connected = true;
          sem_post(&g_semevent);
          message("nxtext_listener: Connected\n");
        }
    }
}

#endif /* CONFIG_NX_MULTIUSER */
