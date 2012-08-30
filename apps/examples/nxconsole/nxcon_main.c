/****************************************************************************
 * examples/nxconsole/nxcon_main.c
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

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
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
#include <nuttx/nx/nxconsole.h>

#include <apps/nsh.h>

#include "nxcon_internal.h"

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

/* All example global variables are retained in a structure to minimize
 * the chance of name collisions.
 */

struct nxcon_state_s g_nxcon_vars;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxcon_initialize
 ****************************************************************************/

static int nxcon_initialize(void)
{
  struct sched_param param;
  pthread_t thread;
  pid_t servrid;
  int ret;

  /* Set the client task priority */

  param.sched_priority = CONFIG_EXAMPLES_NXCON_CLIENTPRIO;
  ret = sched_setparam(0, &param);
  if (ret < 0)
    {
      message("nxcon_initialize: sched_setparam failed: %d\n" , ret);
      return ERROR;
    }

  /* Start the server task */

  message("nxcon_initialize: Starting nxcon_server task\n");
  servrid = task_create("NX Server", CONFIG_EXAMPLES_NXCON_SERVERPRIO,
                        CONFIG_EXAMPLES_NXCON_STACKSIZE, nxcon_server, NULL);
  if (servrid < 0)
    {
      message("nxcon_initialize: Failed to create nxcon_server task: %d\n", errno);
      return ERROR;
    }

  /* Wait a bit to let the server get started */

  sleep(1);

  /* Connect to the server */

  g_nxcon_vars.hnx = nx_connect();
  if (g_nxcon_vars.hnx)
    {
       pthread_attr_t attr;

       /* Start a separate thread to listen for server events.  This is probably
        * the least efficient way to do this, but it makes this example flow more
        * smoothly.
        */

       (void)pthread_attr_init(&attr);
       param.sched_priority = CONFIG_EXAMPLES_NXCON_LISTENERPRIO;
       (void)pthread_attr_setschedparam(&attr, &param);
       (void)pthread_attr_setstacksize(&attr, CONFIG_EXAMPLES_NXCON_STACKSIZE);

       ret = pthread_create(&thread, &attr, nxcon_listener, NULL);
       if (ret != 0)
         {
            printf("nxcon_initialize: pthread_create failed: %d\n", ret);
            return ERROR;
         }

       /* Don't return until we are connected to the server */

       while (!g_nxcon_vars.connected)
         {
           /* Wait for the listener thread to wake us up when we really
            * are connected.
            */

           (void)sem_wait(&g_nxcon_vars.eventsem);
         }
    }
  else
    {
      message("nxcon_initialize: nx_connect failed: %d\n", errno);
      return ERROR;
    }
  return OK;
}

/****************************************************************************
 * Name: nxcon_task
 ****************************************************************************/

static int nxcon_task(int argc, char **argv)
{
  /* If the console front end is selected, then run it on this thread */

#ifdef CONFIG_NSH_CONSOLE
  (void)nsh_consolemain(0, NULL);
#endif

  printf("nxcon_task: Unregister the NX console device\n");
  (void)nxcon_unregister(g_nxcon_vars.hdrvr);

  printf("nxcon_task: Close the window\n");
  (void)nxtk_closewindow(g_nxcon_vars.hwnd);

  /* Disconnect from the server */

  printf("nxcon_task: Disconnect from the server\n");
  nx_disconnect(g_nxcon_vars.hnx);

  return EXIT_SUCCESS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxcon_main
 ****************************************************************************/

int nxcon_main(int argc, char **argv)
{
  nxgl_mxpixel_t color;
  int fd;
  int ret;

  /* General Initialization *************************************************/
  /* Reset all global data */

  message("nxcon_main: Started\n");
  memset(&g_nxcon_vars, 0, sizeof(struct nxcon_state_s));

  /* Call all C++ static constructors */

#if defined(CONFIG_HAVE_CXX) && defined(CONFIG_HAVE_CXXINITIALIZE)
  up_cxxinitialize();
#endif

  /* NSH Initialization *****************************************************/
  /* Initialize the NSH library */

  message("nxcon_main: Initialize NSH\n");
  nsh_initialize();

  /* If the Telnet console is selected as a front-end, then start the
   * Telnet daemon.
   */

#ifdef CONFIG_NSH_TELNET
  ret = nsh_telnetstart();
  if (ret < 0)
    {
     /* The daemon is NOT running.  Report the the error then fail...
      * either with the serial console up or just exiting.
      */

     fprintf(stderr, "ERROR: Failed to start TELNET daemon: %d\n", ret);
   }
#endif
  /* NX Initialization ******************************************************/
  /* Initialize NX */

  message("nxcon_main: Initialize NX\n");
  ret = nxcon_initialize();
  message("nxcon_main: NX handle=%p\n", g_nxcon_vars.hnx);
  if (!g_nxcon_vars.hnx || ret < 0)
    {
      message("nxcon_main: Failed to get NX handle: %d\n", errno);
      goto errout;
    }

  /* Set the background to the configured background color */

  message("nxcon_main: Set background color=%d\n", CONFIG_EXAMPLES_NXCON_BGCOLOR);
  color = CONFIG_EXAMPLES_NXCON_BGCOLOR;
  ret = nx_setbgcolor(g_nxcon_vars.hnx, &color);
  if (ret < 0)
    {
      message("nxcon_main: nx_setbgcolor failed: %d\n", errno);
      goto errout_with_nx;
    }

  /* Window Configuration ***************************************************/
  /* Create a window */

  message("nxcon_main: Create window\n");
  g_nxcon_vars.hwnd = nxtk_openwindow(g_nxcon_vars.hnx, &g_nxconcb, NULL);
  if (!g_nxcon_vars.hwnd)
    {
      message("nxcon_main: nxtk_openwindow failed: %d\n", errno);
      goto errout_with_nx;
    }
  message("nxcon_main: hwnd=%p\n", g_nxcon_vars.hwnd);

  /* Wait until we have the screen resolution.  We'll have this immediately
   * unless we are dealing with the NX server.
   */

  while (!g_nxcon_vars.haveres)
    {
      (void)sem_wait(&g_nxcon_vars.eventsem);
    }
  message("nxcon_main: Screen resolution (%d,%d)\n", g_nxcon_vars.xres, g_nxcon_vars.yres);

  /* Determine the size and position of the window */

  g_nxcon_vars.wndo.wsize.w = g_nxcon_vars.xres / 2 + g_nxcon_vars.xres / 4;
  g_nxcon_vars.wndo.wsize.h = g_nxcon_vars.yres / 2 + g_nxcon_vars.yres / 4;

  g_nxcon_vars.wpos.x       = g_nxcon_vars.xres / 8;
  g_nxcon_vars.wpos.y       = g_nxcon_vars.yres / 8;

  /* Set the window position */

  message("nxcon_main: Set window position to (%d,%d)\n",
          g_nxcon_vars.wpos.x, g_nxcon_vars.wpos.y);

  ret = nxtk_setposition(g_nxcon_vars.hwnd, &g_nxcon_vars.wpos);
  if (ret < 0)
    {
      message("nxcon_main: nxtk_setposition failed: %d\n", errno);
      goto errout_with_hwnd;
    }

  /* Set the window size */

  message("nxcon_main: Set window size to (%d,%d)\n",
          g_nxcon_vars.wndo.wsize.w, g_nxcon_vars.wndo.wsize.h);

  ret = nxtk_setsize(g_nxcon_vars.hwnd, &g_nxcon_vars.wndo.wsize);
  if (ret < 0)
    {
      message("nxcon_main: nxtk_setsize failed: %d\n", errno);
      goto errout_with_hwnd;
    }

  /* Open the toolbar */

  message("nxcon_main: Add toolbar to window\n");
  ret = nxtk_opentoolbar(g_nxcon_vars.hwnd, CONFIG_EXAMPLES_NXCON_TOOLBAR_HEIGHT, &g_nxtoolcb, NULL);
  if (ret < 0)
    {
      message("nxcon_main: nxtk_opentoolbar failed: %d\n", errno);
      goto errout_with_hwnd;
    }

  /* Sleep a little bit to allow the server to catch up */
 
  sleep(2);

  /* NxConsole Configuration ************************************************/
  /* Use the window to create an NX console */

  g_nxcon_vars.wndo.wcolor[0] = CONFIG_EXAMPLES_NXCON_WCOLOR;
  g_nxcon_vars.wndo.fcolor[0] = CONFIG_EXAMPLES_NXCON_FONTCOLOR;
  g_nxcon_vars.wndo.fontid    = CONFIG_EXAMPLES_NXCON_FONTID;

  g_nxcon_vars.hdrvr = nxtk_register(g_nxcon_vars.hwnd, &g_nxcon_vars.wndo, CONFIG_EXAMPLES_NXCON_MINOR);
  if (!g_nxcon_vars.hdrvr)
    {
      message("nxcon_main: nxtk_register failed: %d\n", errno);
      goto errout_with_hwnd;
    }

  /* Open the NxConsole driver */

  fd = open(CONFIG_EXAMPLES_NXCON_DEVNAME, O_WRONLY);
  if (fd < 0)
    {
      message("nxcon_main: open %s read-only failed: %d\n",
              CONFIG_EXAMPLES_NXCON_DEVNAME, errno);
      goto errout_with_driver;
    }

  /* Start Console Task *****************************************************/
  /* Now re-direct stdout and stderr so that they use the NX console driver.
   * Note that stdin is retained (file descriptor 0, probably the the serial console).
    */

   message("nxcon_main: Starting the console task\n");
   msgflush();

  (void)fflush(stdout);
  (void)fflush(stderr);

  (void)fclose(stdout);
  (void)fclose(stderr);

  (void)dup2(fd, 1);
  (void)dup2(fd, 2);

   /* And we can close our original driver file descriptor */

   close(fd);

   /* And start the console task.  It will inherit stdin, stdout, and stderr
    * from this task.
    */
 
   g_nxcon_vars.pid = TASK_CREATE("NxConsole", CONFIG_EXAMPLES_NXCONSOLE_PRIO,
                                  CONFIG_EXAMPLES_NXCONSOLE_STACKSIZE,
                                  nxcon_task, NULL);
   ASSERT(g_nxcon_vars.pid > 0);
   return EXIT_SUCCESS;

  /* Error Exits ************************************************************/

errout_with_driver:
  (void)nxcon_unregister(g_nxcon_vars.hdrvr);

errout_with_hwnd:
  (void)nxtk_closewindow(g_nxcon_vars.hwnd);

errout_with_nx:
  /* Disconnect from the server */

  nx_disconnect(g_nxcon_vars.hnx);
errout:
  return EXIT_FAILURE;
}
