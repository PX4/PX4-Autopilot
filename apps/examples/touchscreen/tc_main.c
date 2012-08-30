/****************************************************************************
 * examples/touchscreen/tc_main.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/input/touchscreen.h>

#include "tc.h"

/****************************************************************************
 * Pre-processor Definitions
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
 * Name: tc_main
 ****************************************************************************/

int tc_main(int argc, char *argv[])
{
  struct touch_sample_s sample;
  ssize_t nbytes;
#if defined(CONFIG_EXAMPLES_TOUCHSCREEN_BUILTIN) || defined(CONFIG_EXAMPLES_TOUCHSCREEN_NSAMPLES)
  long nsamples;
#endif
  int fd;
  int errval = 0;
  int ret;

  /* If this example is configured as an NX add-on, then limit the number of
   * samples that we collect before returning.  Otherwise, we never return
   */

#if defined(CONFIG_EXAMPLES_TOUCHSCREEN_BUILTIN)
  nsamples = 1;
  if (argc > 1)
    {
      nsamples = strtol(argv[1], NULL, 10);
    }
  message("tc_main: nsamples: %d\n", nsamples);
#elif defined(CONFIG_EXAMPLES_TOUCHSCREEN_NSAMPLES)
  message("tc_main: nsamples: %d\n", CONFIG_EXAMPLES_TOUCHSCREEN_NSAMPLES);
#endif

  /* Initialization of the touchscreen hardware is performed by logic
   * external to this test.
   */

  message("tc_main: Initializing external touchscreen device\n");
  ret = arch_tcinitialize(CONFIG_EXAMPLES_TOUCHSCREEN_MINOR);
  if (ret != OK)
    {
      message("tc_main: arch_tcinitialize failed: %d\n", ret);
      errval = 1;
      goto errout;
    }

  /* Open the touchscreen device for reading */

  message("tc_main: Opening %s\n", CONFIG_EXAMPLES_TOUCHSCREEN_DEVPATH);
  fd = open(CONFIG_EXAMPLES_TOUCHSCREEN_DEVPATH, O_RDONLY);
  if (fd < 0)
    {
      message("tc_main: open %s failed: %d\n",
              CONFIG_EXAMPLES_TOUCHSCREEN_DEVPATH, errno);
      errval = 2;
      goto errout_with_tc;
    }

  /* Now loop the appropriate number of times, displaying the collected
   * touchscreen samples.
   */

#if defined(CONFIG_EXAMPLES_TOUCHSCREEN_BUILTIN)
  for (; nsamples > 0; nsamples--)
#elif defined(CONFIG_EXAMPLES_TOUCHSCREEN_NSAMPLES)
  for (nsamples = 0; nsamples < CONFIG_EXAMPLES_TOUCHSCREEN_NSAMPLES; nsamples++)
#else
  for (;;)
#endif
  {
    /* Flush any output before the loop entered or from the previous pass
     * through the loop.
     */

    msgflush();

    /* Read one sample */

    ivdbg("Reading...\n");
    nbytes = read(fd, &sample, sizeof(struct touch_sample_s));
    ivdbg("Bytes read: %d\n", nbytes);

    /* Handle unexpected return values */

    if (nbytes < 0)
      {
        errval = errno;
        if (errval != EINTR)
          {
            message("tc_main: read %s failed: %d\n",
                    CONFIG_EXAMPLES_TOUCHSCREEN_DEVPATH, errval);
            errval = 3;
            goto errout_with_dev;
          }

        message("tc_main: Interrupted read...\n");
      }
    else if (nbytes != sizeof(struct touch_sample_s))
      {
        message("tc_main: Unexpected read size=%d, expected=%d, Ignoring\n",
                nbytes, sizeof(struct touch_sample_s));        
      }

    /* Print the sample data on successful return */

    else
      {
        message("Sample     :\n");
        message("   npoints : %d\n",   sample.npoints);
        message("Point 1    :\n");
        message("        id : %d\n",   sample.point[0].id);
        message("     flags : %02x\n", sample.point[0].flags);
        message("         x : %d\n",   sample.point[0].x);
        message("         y : %d\n",   sample.point[0].y);
        message("         h : %d\n",   sample.point[0].h);
        message("         w : %d\n",   sample.point[0].w);
        message("  pressure : %d\n",   sample.point[0].pressure);
      }
  }

errout_with_dev:
  close(fd);
errout_with_tc:
  arch_tcuninitialize();
errout:
  message("Terminating!\n");
  msgflush();
  return errval;
}
