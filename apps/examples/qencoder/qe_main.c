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
#include <fcntl.h>
#include <limits.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/sensors/qencoder.h>

#include "qe.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_NSH_BUILTIN_APPS
#  define MAIN_NAME   qe_main
#  define MAIN_STRING "qe_main: "
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

#ifdef CONFIG_NSH_BUILTIN_APPS
struct qe_example_s g_qeexample;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: qe_help
 ****************************************************************************/

#ifdef CONFIG_NSH_BUILTIN_APPS
static void qe_help(void)
{
  message("\nUsage: qe [OPTIONS]\n\n");
  message("OPTIONS include:\n");
  message("  [-n samples] number of samples\n");
  message("  [-r] reset the count\n");
  message("  [-h] shows this message and exits\n\n");
}
#endif

/****************************************************************************
 * Name: arg_string
 ****************************************************************************/

#ifdef CONFIG_NSH_BUILTIN_APPS
int arg_string(FAR char **arg, FAR char **value)
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
int arg_decimal(FAR char **arg, FAR long *value)
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
void parse_args(int argc, FAR char **argv)
{
  FAR char *ptr;
  long value;
  int index;
  int nargs;

  g_qeexample.reset  = false;
  g_qeexample.nloops = 1;

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

            g_qeexample.nloops = (int)value;
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
 * Name: user_start/qe_main
 ****************************************************************************/

int MAIN_NAME(int argc, char *argv[])
{
  int32_t position;
  int fd;
  int exitval = EXIT_SUCCESS;
  int ret;
#if defined(CONFIG_NSH_BUILTIN_APPS) || defined(CONFIG_EXAMPLES_QENCODER_NSAMPLES)
  int nloops;
#endif

  /* Parse command line arguments */

#ifdef CONFIG_NSH_BUILTIN_APPS
  parse_args(argc, argv);
#endif

  /* Initialization of the encoder hardware is performed by logic external to
   * this test.
   */

  message(MAIN_STRING "Initializing external encoder\n");
  ret = qe_devinit();
  if (ret != OK)
    {
      message(MAIN_STRING "qe_devinit failed: %d\n", ret);
      exitval = EXIT_FAILURE;
      goto errout;
    }

  /* Open the encoder device for reading */

  message(MAIN_STRING "Hardware initialized. Opening the encoder device\n");
  fd = open(CONFIG_EXAMPLES_QENCODER_DEVPATH, O_RDONLY);
  if (fd < 0)
    {
      message(MAIN_STRING "open %s failed: %d\n",
              CONFIG_EXAMPLES_QENCODER_DEVPATH, errno);
      exitval = EXIT_FAILURE;
      goto errout_with_dev;
    }

  /* Reset the count if so requested */

  if (g_qeexample.reset)
    {
      message(MAIN_STRING "Resetting the count...\n");
      ret = ioctl(fd, QEIOC_RESET, 0);
      if (ret < 0)
        {
          message(MAIN_STRING "ioctl(QEIOC_RESET) failed: %d\n", errno);
          exitval = EXIT_FAILURE;
          goto errout_with_dev;
        }
    }

  /* Now loop the appropriate number of times, displaying the collected
   * encoder samples.
   */

#if defined(CONFIG_NSH_BUILTIN_APPS)
  message(MAIN_STRING "Number of samples: %d\n", g_qeexample.nloops);
  for (nloops = 0; nloops < g_qeexample.nloops; nloops++)
#elif defined(CONFIG_EXAMPLES_QENCODER_NSAMPLES)
  message(MAIN_STRING "Number of samples: %d\n", CONFIG_EXAMPLES_QENCODER_NSAMPLES);
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
          message(MAIN_STRING "ioctl(QEIOC_POSITION) failed: %d\n", errno);
          exitval = EXIT_FAILURE;
          goto errout_with_dev;
        }

      /* Print the sample data on successful return */

      else
        {
          message(MAIN_STRING "  %d\n", position);
        }
    }

errout_with_dev:
  close(fd);

errout:
  message("Terminating!\n");
  msgflush();
  return exitval;
}
