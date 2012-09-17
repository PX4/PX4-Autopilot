/****************************************************************************
 * examples/nxhello/nxhello_main.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <sched.h>
#include <pthread.h>
#include <errno.h>
#include <debug.h>

#ifdef CONFIG_NX_LCDDRIVER
#  include <nuttx/lcd/lcd.h>
#else
#  include <nuttx/fb.h>
#endif

#include <nuttx/arch.h>
#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nxfonts.h>

#include "nxhello.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/
/* If not specified, assume that the hardware supports one video plane */

#ifndef CONFIG_EXAMPLES_NXHELLO_VPLANE
#  define CONFIG_EXAMPLES_NXHELLO_VPLANE 0
#endif

/* If not specified, assume that the hardware supports one LCD device */

#ifndef CONFIG_EXAMPLES_NXHELLO_DEVNO
#  define CONFIG_EXAMPLES_NXHELLO_DEVNO 0
#endif

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

struct nxhello_data_s g_nxhello =
{
  NULL,          /* hnx */
  NULL,          /* hbkgd */
  NULL,          /* hfont */
  0,             /* xres */
  0,             /* yres */
  false,         /* havpos */
  { 0 },         /* sem */
  NXEXIT_SUCCESS /* exit code */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxhello_initialize
 ****************************************************************************/

static inline int nxhello_initialize(void)
{
  FAR NX_DRIVERTYPE *dev;

#if defined(CONFIG_EXAMPLES_NXHELLO_EXTERNINIT)
  /* Use external graphics driver initialization */

  message("nxhello_initialize: Initializing external graphics device\n");
  dev = up_nxdrvinit(CONFIG_EXAMPLES_NXHELLO_DEVNO);
  if (!dev)
    {
      message("nxhello_initialize: up_nxdrvinit failed, devno=%d\n",
              CONFIG_EXAMPLES_NXHELLO_DEVNO);
      g_nxhello.code = NXEXIT_EXTINITIALIZE;
      return ERROR;
    }

#elif defined(CONFIG_NX_LCDDRIVER)
  int ret;

  /* Initialize the LCD device */

  message("nxhello_initialize: Initializing LCD\n");
  ret = up_lcdinitialize();
  if (ret < 0)
    {
      message("nxhello_initialize: up_lcdinitialize failed: %d\n", -ret);
      g_nxhello.code = NXEXIT_LCDINITIALIZE;
      return ERROR;
    }

  /* Get the device instance */

  dev = up_lcdgetdev(CONFIG_EXAMPLES_NXHELLO_DEVNO);
  if (!dev)
    {
      message("nxhello_initialize: up_lcdgetdev failed, devno=%d\n", CONFIG_EXAMPLES_NXHELLO_DEVNO);
      g_nxhello.code = NXEXIT_LCDGETDEV;
      return ERROR;
    }

  /* Turn the LCD on at 75% power */

  (void)dev->setpower(dev, ((3*CONFIG_LCD_MAXPOWER + 3)/4));
#else
  int ret;

  /* Initialize the frame buffer device */

  message("nxhello_initialize: Initializing framebuffer\n");
  ret = up_fbinitialize();
  if (ret < 0)
    {
      message("nxhello_initialize: up_fbinitialize failed: %d\n", -ret);
      g_nxhello.code = NXEXIT_FBINITIALIZE;
      return ERROR;
    }

  dev = up_fbgetvplane(CONFIG_EXAMPLES_NXHELLO_VPLANE);
  if (!dev)
    {
      message("nxhello_initialize: up_fbgetvplane failed, vplane=%d\n", CONFIG_EXAMPLES_NXHELLO_VPLANE);
      g_nxhello.code = NXEXIT_FBGETVPLANE;
      return ERROR;
    }
#endif

  /* Then open NX */

  message("nxhello_initialize: Open NX\n");
  g_nxhello.hnx = nx_open(dev);
  if (!g_nxhello.hnx)
    {
      message("nxhello_initialize: nx_open failed: %d\n", errno);
      g_nxhello.code = NXEXIT_NXOPEN;
      return ERROR;
    }
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxhello_main
 ****************************************************************************/

int nxhello_main(int argc, char *argv[])
{
  nxgl_mxpixel_t color;
  int ret;

  /* Initialize NX */

  ret = nxhello_initialize();
  message("nxhello_main: NX handle=%p\n", g_nxhello.hnx);
  if (!g_nxhello.hnx || ret < 0)
    {
      message("nxhello_main: Failed to get NX handle: %d\n", errno);
      g_nxhello.code = NXEXIT_NXOPEN;
      goto errout;
    }

  /* Get the default font handle */

  g_nxhello.hfont = nxf_getfonthandle(CONFIG_EXAMPLES_NXHELLO_FONTID);
  if (!g_nxhello.hfont)
    {
      message("nxhello_main: Failed to get font handle: %d\n", errno);
      g_nxhello.code = NXEXIT_FONTOPEN;
      goto errout;
    }

  /* Set the background to the configured background color */

  message("nxhello_main: Set background color=%d\n",
          CONFIG_EXAMPLES_NXHELLO_BGCOLOR);

  color = CONFIG_EXAMPLES_NXHELLO_BGCOLOR;
  ret = nx_setbgcolor(g_nxhello.hnx, &color);
  if (ret < 0)
    {
      message("nxhello_main: nx_setbgcolor failed: %d\n", errno);
      g_nxhello.code = NXEXIT_NXSETBGCOLOR;
      goto errout_with_nx;
    }

  /* Get the background window */

  ret = nx_requestbkgd(g_nxhello.hnx, &g_nxhellocb, NULL);
  if (ret < 0)
    {
      message("nxhello_main: nx_setbgcolor failed: %d\n", errno);
      g_nxhello.code = NXEXIT_NXREQUESTBKGD;
      goto errout_with_nx;
    }

  /* Wait until we have the screen resolution.  We'll have this immediately
   * unless we are dealing with the NX server.
   */

  while (!g_nxhello.havepos)
    {
      (void)sem_wait(&g_nxhello.sem);
    }
  message("nxhello_main: Screen resolution (%d,%d)\n", g_nxhello.xres, g_nxhello.yres);

  /* Now, say hello and exit, sleeping a little before each. */

  sleep(1);
  nxhello_hello(g_nxhello.hbkgd);
  sleep(1);

  /* Release background */

  (void)nx_releasebkgd(g_nxhello.hbkgd);

  /* Close NX */

errout_with_nx:
  message("nxhello_main: Close NX\n");
  nx_close(g_nxhello.hnx);
errout:
  return g_nxhello.code;
}
