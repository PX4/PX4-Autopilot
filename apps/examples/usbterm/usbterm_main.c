/****************************************************************************
 * examples/usbterm/usbterm_main.c
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

#include <sys/types.h>
#include <sys/stat.h>

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <apps/readline.h>

#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>

#ifdef CONFIG_CDCACM
#  include <nuttx/usb/cdcacm.h>
#endif

#include "usbterm.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* USB terminal state data */

struct usbterm_globals_s g_usbterm;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: trace_callback
 *
 * Description:
 *   Callback from USB trace instrumentation.
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_TRACE
static int trace_callback(struct usbtrace_s *trace, void *arg)
{
  usbtrace_trprintf((trprintf_t)trmessage, trace->event, trace->value);
  return 0;
}
#endif

/****************************************************************************
 * Name: dumptrace
 *
 * Description:
 *   Dump collected trace data.
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_TRACE
static void dumptrace(void)
{
  (void)usbtrace_enumerate(trace_callback, NULL);
}
#else
#  define dumptrace()
#endif

/****************************************************************************
 * Name: dumptrace
 *
 * Description:
 *   Entry point for the listener thread.
 *
 ****************************************************************************/

FAR void *usbterm_listener(FAR void *parameter)
{
  message("usbterm_listener: Waiting for remote input\n");
  for (;;)
    {
      /* Display the prompt string on the remote USB serial connection */

      fputs("\rusbterm> ", g_usbterm.outstream);
      fflush(g_usbterm.outstream);

     /* Get the next line of input from the remote USB serial connection */

      if (fgets(g_usbterm.inbuffer, CONFIG_EXAMPLES_USBTERM_BUFLEN, g_usbterm.instream))
        {
          /* Echo the line on the local stdout */

          fputs(g_usbterm.inbuffer, stdout);

          /* Display the prompt string on stdout */

          fputs("usbterm> ", stdout);
          fflush(stdout);
        }

      /* If USB tracing is enabled, then dump all collected trace data to stdout */

      dumptrace();
    }

  /* Won't get here */

  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: term_main/user_start
 *
 * Description:
 *   Main entry point for the USB serial terminal example.
 *
 ****************************************************************************/

#ifdef CONFIG_EXAMPLES_USBTERM_BUILTIN
#  define MAIN_NAME usbterm_main
#  define MAIN_STRING "usbterm_main: "
#else
#  define MAIN_NAME user_start
#  define MAIN_STRING "user_start: "
#endif

int MAIN_NAME(int argc, char *argv[])
{
  pthread_attr_t attr;
  int ret;

  /* Initialization of the USB hardware may be performed by logic external to
   * this test.
   */

#ifdef CONFIG_EXAMPLES_USBTERM_DEVINIT
  message(MAIN_STRING "Performing external device initialization\n");
  ret = usbterm_devinit();
  if (ret != OK)
    {
      message(MAIN_STRING "usbterm_devinit failed: %d\n", ret);
      goto errout;
    }
#endif

  /* Initialize the USB serial driver */

  message(MAIN_STRING "Registering USB serial driver\n");
#ifdef CONFIG_CDCACM
  ret = cdcacm_initialize(0, NULL);
#else
  ret = usbdev_serialinitialize(0);
#endif
  if (ret < 0)
    {
      message(MAIN_STRING "ERROR: Failed to create the USB serial device: %d\n", -ret);
      goto errout_with_devinit;
    }
  message(MAIN_STRING "Successfully registered the serial driver\n");

#if CONFIG_USBDEV_TRACE && CONFIG_USBDEV_TRACE_INITIALIDSET != 0
  /* If USB tracing is enabled and tracing of initial USB events is specified,
   * then dump all collected trace data to stdout
   */

  sleep(5);
  dumptrace();
#endif

  /* Then, in any event, configure trace data collection as configured */

  usbtrace_enable(TRACE_BITSET);

  /* Open the USB serial device for writing */

  do
    {
      message(MAIN_STRING "Opening USB serial driver\n");

      g_usbterm.outstream = fopen("/dev/ttyUSB0", "w");
      if (g_usbterm.outstream == NULL)
        {
          int errcode = errno;
          message(MAIN_STRING "ERROR: Failed to open /dev/ttyUSB0 for writing: %d\n",
                  errcode);

          /* ENOTCONN means that the USB device is not yet connected */

          if (errcode == ENOTCONN)
            {
              message(MAIN_STRING "       Not connected. Wait and try again.\n");
              sleep(5);
            }
          else
            {
              /* Give up on other errors */

              goto errout_with_devinit;
            }
        }

      /* If USB tracing is enabled, then dump all collected trace data to stdout */

      dumptrace();
    }
  while (g_usbterm.outstream == NULL);

  /* Open the USB serial device for reading.  Since we are already connected, this
   * should not fail.
   */

  g_usbterm.instream = fopen("/dev/ttyUSB0", "r");
  if (g_usbterm.instream == NULL)
    {
      message(MAIN_STRING "ERROR: Failed to open /dev/ttyUSB0 for reading: %d\n", errno);
      goto errout_with_outstream;
    }

  message(MAIN_STRING "Successfully opened the serial driver\n");

  /* Start the USB term listener thread */

  message(MAIN_STRING "Starting the listener thread\n");
 
  ret = pthread_attr_init(&attr);
  if (ret != OK)
    {
      message(MAIN_STRING "pthread_attr_init failed: %d\n", ret);
      goto errout_with_streams;
    }

  ret = pthread_create(&g_usbterm.listener, &attr,
                       usbterm_listener, (pthread_addr_t)0);
  if (ret != 0)
    {
      message(MAIN_STRING "Error in thread creation: %d\n", ret);
      goto errout_with_streams;
    }

  /* Send messages and get responses -- forever */

  message(MAIN_STRING "Waiting for local input\n");
  for (;;)
    {
      /* Display the prompt string on stdout */

      fputs("usbterm> ", stdout);
      fflush(stdout);

      /* Get the next line of input */

#ifdef CONFIG_EXAMPLES_USBTERM_FGETS
      /* fgets returns NULL on end-of-file or any I/O error */

      if (fgets(g_usbterm.outbuffer, CONFIG_EXAMPLES_USBTERM_BUFLEN, stdin) == NULL)
        {
          printf("ERROR: fgets failed: %d\n", errno);
          return 1;
        }
#else
      ret = readline(g_usbterm.outbuffer, CONFIG_EXAMPLES_USBTERM_BUFLEN, stdin, stdout);

      /* Readline normally returns the number of characters read,
       * but will return 0 on end of file or a negative value
       * if an error occurs.  Either will cause the session to
       * terminate.
       */

      if (ret <= 0)
        {
          printf("ERROR: readline failed: %d\n", ret);
          return 1;
        }
#endif
      else
        {
          /* Send the line of input via USB */

          fputs(g_usbterm.outbuffer, g_usbterm.outstream);

          /* Display the prompt string on the remote USB serial connection */

          fputs("\rusbterm> ", g_usbterm.outstream);
          fflush(g_usbterm.outstream);
        }

      /* If USB tracing is enabled, then dump all collected trace data to stdout */

      dumptrace();
    }

  /* Error exits */

errout_with_streams:
  fclose(g_usbterm.instream);
errout_with_outstream:
  fclose(g_usbterm.outstream);
errout_with_devinit:
#ifdef CONFIG_EXAMPLES_USBTERM_DEVINIT
  usbterm_devuninit();
errout:
#endif
  message(MAIN_STRING "       Aborting\n");
  return 1;
}

