/****************************************************************************
 * config/sim/src/up_touchscreen.c
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
#include <errno.h>
#include <debug.h>

#include <nuttx/fb.h>
#include <nuttx/input/touchscreen.h>
#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* Pick a background color */

#ifndef CONFIG_EXAMPLES_TOUCHSCREEN_BGCOLOR
#  define CONFIG_EXAMPLES_TOUCHSCREEN_BGCOLOR 0x007b68ee
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sim_touchscreen_s
{
  NXHANDLE hnx;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct sim_touchscreen_s g_simtc;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arch_tcinitialize()
 *
 * Description:
 *   Perform architecuture-specific initialization of the touchscreen
 *   hardware.  This interface must be provided by all configurations
 *   using apps/examples/touchscreen
 *
 ****************************************************************************/

int arch_tcinitialize(int minor)
{
  FAR NX_DRIVERTYPE *dev;
  nxgl_mxpixel_t color;
  int ret;

  /* Initialize the simulated frame buffer device.  We need to create an
   * X11 window to support the mouse-driven touchscreen simulation.
   */

  ivdbg("Initializing framebuffer\n");
  ret = up_fbinitialize();
  if (ret < 0)
    {
      idbg("up_fbinitialize failed: %d\n", -ret);
      goto errout;
    }

  dev = up_fbgetvplane(0);
  if (!dev)
    {
      idbg("up_fbgetvplane 0 failed\n");
      ret = -ENODEV;
      goto errout_with_fb;
    }

  /* Then open NX */

  ivdbg("Open NX\n");
  g_simtc.hnx = nx_open(dev);
  if (!g_simtc.hnx)
    {
      ret = -errno;
      idbg("nx_open failed: %d\n", ret);
      goto errout_with_fb;
    }

  /* Set the background to the configured background color */

  ivdbg("Set background color=%d\n", CONFIG_EXAMPLES_TOUCHSCREEN_BGCOLOR);

  color = CONFIG_EXAMPLES_TOUCHSCREEN_BGCOLOR;
  ret = nx_setbgcolor(g_simtc.hnx, &color);
  if (ret < 0)
    {
      idbg("nx_setbgcolor failed: %d\n", ret);
      goto errout_with_nx;
    }

  /* Finally, initialize the touchscreen simulation on the X window */

  ret = arch_tcinitialize(minor);
  if (ret < 0)
    {
      idbg("arch_tcinitialize failed: %d\n", ret);
      goto errout_with_nx;
    }
  return OK;

errout_with_nx:
  nx_close(g_simtc.hnx);
  goto errout;
errout_with_fb:
  fb_uninitialize();
errout:
  return ret;
}

/****************************************************************************
 * Name: arch_tcuninitialize()
 *
 * Description:
 *   Perform architecuture-specific un-initialization of the touchscreen
 *   hardware.  This interface must be provided by all configurations
 *   using apps/examples/touchscreen
 *
 ****************************************************************************/

void arch_tcuninitialize(void)
{
  /* Shut down the touchscreen driver */

  sim_tcuninitialize();

  /* Close NX */

  nx_close(g_simtc.hnx);
}
