/****************************************************************************
 * examples/watchdog/watchdog_main.c
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
#include <errno.h>
#include <debug.h>

#include <nuttx/watchdog.h>

#include "watchdog.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct wdog_example_s
{
  uint32_t pingtime;
  uint32_t pingdelay;
  uint32_t timeout;
};

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
 * Name: wdog_help
 ****************************************************************************/

static void wdog_help(void)
{
  message("Usage: wdog [-h] [-d <pingtime] [-p <pingdelay>] [-t <timeout>]\n");
  message("\nInitialize the watchdog to the <timeout>.  Start the watchdog\n");
  message("timer.  Ping for the watchdog for <pingtime> seconds, then let it expire.\n");
  message("\nOptions include:\n");
  message("  [-d <pingtime>] = Selects the <delay> time in milliseconds.  Default: %d\n",
         CONFIG_EXAMPLES_WATCHDOG_PINGTIME);
  message("  [-p <pingdelay] = Time delay between pings in milliseconds.  Default: %d\n",
         CONFIG_EXAMPLES_WATCHDOG_PINGDELAY);
  message("  [-t timeout] = Time in milliseconds that the example will ping the watchdog\n");
  message("    before letting the watchdog expire. Default: %d\n",
          CONFIG_EXAMPLES_WATCHDOG_TIMEOUT);
  message("  [-h] = Shows this message and exits\n");
}

/****************************************************************************
 * Name: arg_string
 ****************************************************************************/

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

/****************************************************************************
 * Name: arg_decimal
 ****************************************************************************/

static int arg_decimal(FAR char **arg, FAR long *value)
{
  FAR char *string;
  int ret;

  ret = arg_string(arg, &string);
  *value = strtol(string, NULL, 10);
  return ret;
}

/****************************************************************************
 * Name: parse_args
 ****************************************************************************/

static void parse_args(FAR struct wdog_example_s *wdog, int argc, FAR char **argv)
{
  FAR char *ptr;
  long value;
  int index;
  int nargs;
 
  wdog->pingtime  = CONFIG_EXAMPLES_WATCHDOG_PINGTIME;
  wdog->pingdelay = CONFIG_EXAMPLES_WATCHDOG_PINGDELAY;
  wdog->timeout   = CONFIG_EXAMPLES_WATCHDOG_TIMEOUT;

  for (index = 1; index < argc; )
    {
      ptr = argv[index];
      if (ptr[0] != '-')
        {
          message("Invalid options format: %s\n", ptr);
          exit(EXIT_SUCCESS);
        }

      switch (ptr[1])
        {
          case 'd':
            nargs = arg_decimal(&argv[index], &value);
            if (value < 1)
              {
                message("Ping delay out of range: %ld\n", value);
                exit(EXIT_FAILURE);
              }

            wdog->pingdelay = (uint32_t)value;
            index += nargs;
            break;

          case 'p':
            nargs = arg_decimal(&argv[index], &value);
            if (value < 1 || value > 99)
              {
                message("Ping time out of range: %ld\n", value);
                exit(EXIT_FAILURE);
              }

            wdog->pingtime = (uint8_t)value;
            index += nargs;
            break;

          case 't':
            nargs = arg_decimal(&argv[index], &value);
            if (value < 1 || value > INT_MAX)
              {
                message("Duration out of range: %ld\n", value);
                exit(EXIT_FAILURE);
              }

            wdog->timeout = (int)value;
            index += nargs;
            break;

          case 'h':
            wdog_help();
            exit(EXIT_SUCCESS);
        
          default:
            message("Unsupported option: %s\n", ptr);
            wdog_help();
            exit(EXIT_FAILURE);
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wdog_main
 ****************************************************************************/

int wdog_main(int argc, char *argv[])
{
  struct wdog_example_s wdog;
#ifdef CONFIG_DEBUG_WATCHDOG
  struct watchdog_status_s status;
#endif
  long elapsed;
  int fd;
  int ret;

  /* Parse the command line */

  parse_args(&wdog, argc, argv);

  /* Initialization of the WATCHDOG hardware is performed by logic external to
   * this test.
   */

  ret = up_wdginitialize();
  if (ret != OK)
    {
      message("wdog_main: up_wdginitialize failed: %d\n", ret);
      goto errout;
    }

  /* Open the watchdog device for reading */

  fd = open(CONFIG_EXAMPLES_WATCHDOG_DEVPATH, O_RDONLY);
  if (fd < 0)
    {
      message("wdog_main: open %s failed: %d\n",
              CONFIG_EXAMPLES_WATCHDOG_DEVPATH, errno);
      goto errout;
    }

  /* Set the watchdog timeout */

  ret = ioctl(fd, WDIOC_SETTIMEOUT, (unsigned long)wdog.timeout);
  if (ret < 0)
    {
      message("wdog_main: ioctl(WDIOC_SETTIMEOUT) failed: %d\n", errno);
      goto errout_with_dev;
    }

  /* Then start the watchdog timer. */

  ret = ioctl(fd, WDIOC_START, 0);
  if (ret < 0)
    {
      message("wdog_main: ioctl(WDIOC_START) failed: %d\n", errno);
      goto errout_with_dev;
    }

  /* Then ping */

  for (elapsed = 0; elapsed < wdog.pingtime; elapsed += wdog.pingdelay)
    {
      /* Sleep for the requested amount of time */

      usleep(wdog.pingdelay * 1000);

      /* Show watchdog status.  Only if debug is enabled because this
       * could interfere with the timer.
       */

#ifdef CONFIG_DEBUG_WATCHDOG
     ret = ioctl(fd, WDIOC_GETSTATUS, (unsigned long)&status);
     if (ret < 0)
       {
          message("wdog_main: ioctl(WDIOC_GETSTATUS) failed: %d\n", errno);
          goto errout_with_dev;
        }
      message("wdog_main: flags=%08x timeout=%d timeleft=%d\n",
              status.flags, status.timeout, status.timeleft);
#endif

      /* Then ping */

     ret = ioctl(fd, WDIOC_KEEPALIVE, 0);
     if (ret < 0)
       {
          message("wdog_main: ioctl(WDIOC_KEEPALIVE) failed: %d\n", errno);
          goto errout_with_dev;
        }

      message("  ping elapsed=%ld\n", elapsed);
      msgflush();
    }

  /* Then stop pinging */

  for (; ; elapsed += wdog.pingdelay)
    {
      /* Sleep for the requested amount of time */

      usleep(wdog.pingdelay * 1000);

      /* Show watchdog status.  Only if debug is enabled because this
       * could interfere with the timer.
       */

#ifdef CONFIG_DEBUG_WATCHDOG
     ret = ioctl(fd, WDIOC_GETSTATUS, (unsigned long)&status);
     if (ret < 0)
       {
          message("wdog_main: ioctl(WDIOC_GETSTATUS) failed: %d\n", errno);
          goto errout_with_dev;
        }
      message("wdog_main: flags=%08x timeout=%d timeleft=%d\n",
              status.flags, status.timeout, status.timeleft);
#endif

      message("  NO ping elapsed=%ld\n", elapsed);
      msgflush();
    }

  /* We should not get here */

  ret = ioctl(fd, WDIOC_STOP, 0);
  if (ret < 0)
    {
      message("wdog_main: ioctl(WDIOC_STOP) failed: %d\n", errno);
      goto errout_with_dev;
    }

 close(fd);
 msgflush();
 return OK;

errout_with_dev:
  close(fd);
errout:
  msgflush();
  return ERROR;
}
