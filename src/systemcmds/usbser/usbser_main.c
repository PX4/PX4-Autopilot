/****************************************************************************
 * examples/cdcacm/cdcacm_main.c
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
#include <string.h>
#include <stdio.h>
#include <debug.h>

#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/cdcacm.h>

#include <systemlib/err.h>

#include "cdcacm.h"

__EXPORT int usbser_main(int argc, char *argv[]);

/* All global variables used by this example are packed into a structure in
 * order to avoid name collisions.
 */

struct cdcacm_state_s g_cdcacm;

static int sercon();
static int serdis();

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int usbser_main(int argc, char *argv[])
{
  if (argc > 1) {
    if (!strcmp(argv[1], "connect")) {
      return sercon();
    } else if (!strcmp(argv[1], "disconnect")) {
      return serdis();
    } else {
      warnx("unknown command");
    }
  }
}

/****************************************************************************
 * sercon_main
 *
 * Description:
 *   This is the main program that configures the CDC/ACM serial device.
 *
 ****************************************************************************/

int sercon()
{
  int ret;

  /* Check if there is a non-NULL USB mass storage device handle (meaning that the
   * USB mass storage device is already configured).
   */

  if (g_cdcacm.handle)
    {
      warnx("ERROR: Already connected");
      return EXIT_FAILURE;
    }

  /* Initialize the USB CDC/ACM serial driver */

  warnx("Registering CDC/ACM serial driver");
  ret = cdcacm_initialize(CONFIG_EXAMPLES_CDCACM_DEVMINOR, &g_cdcacm.handle);
  if (ret < 0)
    {
      warnx("ERROR: Failed to create the CDC/ACM serial device: %d", -ret);
      return EXIT_FAILURE;
    }

  warnx("Successfully registered the CDC/ACM serial driver");
  return EXIT_SUCCESS;
}

/****************************************************************************
 * serdis_main
 *
 * Description:
 *   This is a program entry point that will disconnect the CDC/ACM serial
 *   device.
 *
 ****************************************************************************/

int serdis()
{
  /* First check if the USB mass storage device is already connected */

  if (!g_cdcacm.handle)
    {
      warnx("ERROR: Not connected");
      return EXIT_FAILURE;
    }

  /* Then disconnect the device and uninitialize the USB mass storage driver */

   cdcacm_uninitialize(g_cdcacm.handle);
   g_cdcacm.handle = NULL;
   warnx("Disconnected");
   return EXIT_SUCCESS;
}
