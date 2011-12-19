/****************************************************************************
 * examples/adc/adc_main.c
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

#include <nuttx/analog/adc.h>

#include "adc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_NSH_BUILTIN_APPS
#  define MAIN_NAME   adc_main
#  define MAIN_STRING "adc_main: "
#else
#  define MAIN_NAME   user_start
#  define MAIN_STRING "user_start: "
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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: user_start/adc_main
 ****************************************************************************/

int MAIN_NAME(int argc, char *argv[])
{
  struct adc_msg_s sample[CONFIG_EXAMPLES_ADC_GROUPSIZE];
  size_t readsize;
  ssize_t nbytes;
#if defined(CONFIG_NSH_BUILTIN_APPS) || defined(CONFIG_EXAMPLES_ADC_NSAMPLES)
  long nsamples;
#endif
  int fd;
  int errval = 0;
  int ret;
  int i;

  /* If this example is configured as an NX add-on, then limit the number of
   * samples that we collect before returning.  Otherwise, we never return
   */

#if defined(CONFIG_NSH_BUILTIN_APPS)
  nsamples = 1;
  if (argc > 1)
    {
      nsamples = strtol(argv[1], NULL, 10);
    }
  message(MAIN_STRING "nsamples: %d\n", nsamples);
#elif defined(CONFIG_EXAMPLES_ADC_NSAMPLES)
  message(MAIN_STRING "nsamples: %d\n", CONFIG_EXAMPLES_ADC_NSAMPLES);
#endif

  /* Initialization of the ADC hardware is performed by logic external to
   * this test.
   */

  message(MAIN_STRING "Initializing external ADC device\n");
  ret = adc_devinit();
  if (ret != OK)
    {
      message(MAIN_STRING "adc_devinit failed: %d\n", ret);
      errval = 1;
      goto errout;
    }

  /* Open the ADC device for reading */

  message(MAIN_STRING "Hardware initialized. Opening the ADC device\n");
  fd = open(CONFIG_EXAMPLES_ADC_DEVPATH, O_RDONLY);
  if (fd < 0)
    {
      message(MAIN_STRING "open %s failed: %d\n",
              CONFIG_EXAMPLES_ADC_DEVPATH, errno);
      errval = 2;
      goto errout_with_dev;
    }

  /* Now loop the appropriate number of times, displaying the collected
   * ADC samples.
   */

  message(MAIN_STRING "Entering the main loop\n");

#if defined(CONFIG_NSH_BUILTIN_APPS)
  for (; nsamples > 0; nsamples--)
#elif defined(CONFIG_EXAMPLES_ADC_NSAMPLES)
  for (nsamples = 0; nsamples < CONFIG_EXAMPLES_ADC_NSAMPLES; nsamples++)
#else
  for (;;)
#endif
  {
    /* Flush any output before the loop entered or from the previous pass
     * through the loop.
     */

    msgflush();

    /* Read CONFIG_EXAMPLES_ADC_GROUPSIZE samples */

    readsize = CONFIG_EXAMPLES_ADC_GROUPSIZE * sizeof(struct adc_msg_s);
    nbytes = read(fd, sample, readsize);
    message("Bytes read: %d\n", nbytes);

    /* Handle unexpected return values */

    if (nbytes < 0)
      {
        errval = errno;
        if (errval != EINTR)
          {
            message(MAIN_STRING "read %s failed: %d\n",
                    CONFIG_EXAMPLES_ADC_DEVPATH, errval);
            errval = 3;
            goto errout_with_dev;
          }

        message(MAIN_STRING "Interrupted read...\n");
      }
    else if (nbytes != readsize)
      {
        message(MAIN_STRING "Unexpected read size=%d, expected=%d, Ignoring\n",
                nbytes, readsize);        
      }

    /* Print the sample data on successful return */

    else
      {
        message("Sample :\n");
        for (i = 0; i < CONFIG_EXAMPLES_ADC_GROUPSIZE; i++)
          {
            message("%d: channel: %d value: %d\n",
                     i, sample[i].am_channel, sample[i].am_data);
          }
      }
  }

errout_with_dev:
  close(fd);

errout:
  message("Terminating!\n");
  msgflush();
  return errval;
}
