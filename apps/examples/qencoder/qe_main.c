/****************************************************************************
 * examples/qe/qe_main.c
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
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <limits.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/sensors/qencoder.h>

#include "qe.h"

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

#ifdef CONFIG_NSH_BUILTIN_APPS
struct qe_example_s g_qeexample;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: qe_devpath
 ****************************************************************************/

static void qe_devpath(FAR const char *devpath)
{
  /* Get rid of any old device path */

  if (g_qeexample.devpath)
    {
      free(g_qeexample.devpath);
    }

  /* The set-up the new device path by copying the string */

  g_qeexample.devpath = strdup(devpath);
}

/****************************************************************************
 * Name: qe_help
 ****************************************************************************/

#ifdef CONFIG_NSH_BUILTIN_APPS
static void qe_help(void)
{
  message("\nUsage: qe [OPTIONS]\n\n");
  message("OPTIONS include:\n");
  message("  [-p devpath] QE device path\n");
  message("  [-n samples] Number of samples\n");
  message("  [-t msec]    Delay between samples (msec)\n");
  message("  [-r]         Reset the position to zero\n");
  message("  [-h]         Shows this message and exits\n\n");
}
#endif

/****************************************************************************
 * Name: arg_string
 ****************************************************************************/

#ifdef CONFIG_NSH_BUILTIN_APPS
static int arg_string(FAR char **arg, FAR char **value)
{
  FAR char *ptr = *arg;

  if (ptr[2] == '\0')
    {
      *value = arg[1];
      return 2;
    }
  else
    {
      *value = &ptr[2];
      return 1;
    }
}
#endif

/****************************************************************************
 * Name: arg_decimal
 ****************************************************************************/

#ifdef CONFIG_NSH_BUILTIN_APPS
static int arg_decimal(FAR char **arg, FAR long *value)
{
  FAR char *string;
  int ret;

  ret = arg_string(arg, &string);
  *value = strtol(string, NULL, 10);
  return ret;
}
#endif

/****************************************************************************
 * Name: parse_args
 ****************************************************************************/

#ifdef CONFIG_NSH_BUILTIN_APPS
static void parse_args(int argc, FAR char **argv)
{
  FAR char *ptr;
  FAR char *str;
  long value;
  int index;
  int nargs;

  g_qeexample.reset  = false;
  g_qeexample.nloops = 1;
  g_qeexample.delay  = CONFIG_EXAMPLES_QENCODER_DELAY;

  for (index = 1; index < argc; )
    {
      ptr = argv[index];
      if (ptr[0] != '-')
        {
          message("Invalid options format: %s\n", ptr);
          exit(0);
        }

      switch (ptr[1])
        {
          case 'n':
            nargs = arg_decimal(&argv[index], &value);
            if (value < 0 || value > INT_MAX)
              {
                message("Sample count out of range: %ld\n", value);
                exit(1);
              }

            g_qeexample.nloops = (unsigned int)value;
            index += nargs;
            break;

          case 'p':
            nargs = arg_string(&argv[index], &str);
            qe_devpath(str);
            index += nargs;
            break;

          case 't':
            nargs = arg_decimal(&argv[index], &value);
            if (value < 0 || value > INT_MAX)
              {
                message("Sample delay out of range: %ld\n", value);
                exit(1);
              }

            g_qeexample.delay = (unsigned int)value;
            index += nargs;
            break;

          case 'r':
            g_qeexample.reset = true;
            index++;
            break;

          case 'h':
            qe_help();
            exit(EXIT_SUCCESS);
        
          default:
            message("Unsupported option: %s\n", ptr);
            qe_help();
            exit(EXIT_FAILURE);
        }
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: qe_main
 ****************************************************************************/

int qe_main(int argc, char *argv[])
{
  int32_t position;
  int fd;
  int exitval = EXIT_SUCCESS;
  int ret;
#if defined(CONFIG_NSH_BUILTIN_APPS) || defined(CONFIG_EXAMPLES_QENCODER_NSAMPLES)
  int nloops;
#endif

  /* Check if we have initialized */

  if (!g_qeexample.initialized)
    {
      /* Initialization of the encoder hardware is performed by logic external to
       * this test.
       */

      message("qe_main: Initializing external encoder(s)\n");
      ret = qe_devinit();
      if (ret != OK)
        {
          message("qe_main: qe_devinit failed: %d\n", ret);
          exitval = EXIT_FAILURE;
          goto errout;
        }

      /* Set the default values */

      qe_devpath(CONFIG_EXAMPLES_QENCODER_DEVPATH);
      g_qeexample.initialized = true;
    }

  /* Parse command line arguments */

#ifdef CONFIG_NSH_BUILTIN_APPS
  parse_args(argc, argv);
#endif

  /* Open the encoder device for reading */

  message("qe_main: Hardware initialized. Opening the encoder device: %s\n",
          g_qeexample.devpath);

  fd = open(g_qeexample.devpath, O_RDONLY);
  if (fd < 0)
    {
      message("qe_main: open %s failed: %d\n", g_qeexample.devpath, errno);
      exitval = EXIT_FAILURE;
      goto errout_with_dev;
    }

  /* Reset the count if so requested */

  if (g_qeexample.reset)
    {
      message("qe_main: Resetting the count...\n");
      ret = ioctl(fd, QEIOC_RESET, 0);
      if (ret < 0)
        {
          message("qe_main: ioctl(QEIOC_RESET) failed: %d\n", errno);
          exitval = EXIT_FAILURE;
          goto errout_with_dev;
        }
    }

  /* Now loop the appropriate number of times, displaying the collected
   * encoder samples.
   */

#if defined(CONFIG_NSH_BUILTIN_APPS)
  message("qe_main: Number of samples: %d\n", g_qeexample.nloops);
  for (nloops = 0; nloops < g_qeexample.nloops; nloops++)
#elif defined(CONFIG_EXAMPLES_QENCODER_NSAMPLES)
  message("qe_main: Number of samples: %d\n", CONFIG_EXAMPLES_QENCODER_NSAMPLES);
  for (nloops = 0; nloops < CONFIG_EXAMPLES_QENCODER_NSAMPLES; nloops++)
#else
  for (;;)
#endif
    {
      /* Flush any output before the loop entered or from the previous pass
       * through the loop.
       */

      msgflush();

      /* Get the positions data using the ioctl */

      ret = ioctl(fd, QEIOC_POSITION, (unsigned long)((uintptr_t)&position));
      if (ret < 0)
        {
          message("qe_main: ioctl(QEIOC_POSITION) failed: %d\n", errno);
          exitval = EXIT_FAILURE;
          goto errout_with_dev;
        }

      /* Print the sample data on successful return */

      else
        {
          message("qe_main: %3d. %d\n", nloops+1, position);
        }

      /* Delay a little bit */

#if defined(CONFIG_NSH_BUILTIN_APPS)
      usleep(g_qeexample.delay * 1000);
#else
      usleep(CONFIG_EXAMPLES_QENCODER_DELAY * 1000);
#endif
    }

errout_with_dev:
  close(fd);

errout:
  message("Terminating!\n");
  msgflush();
  return exitval;
}
