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

#include "nxcon_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NCON_MSG_NLINES 24

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_NX_KBD
static const uint8_t g_pumsg[] = "Pop-Up!";
static const char *g_nxcon_msg[NCON_MSG_NLINES] =
{
  "\nJULIET\n",                           /* Line 1 */
  "Wilt thou be gone?\n",                 /* Line 2 */
  "  It is not yet near day:\n",          /* Line 3 */
  "It was the nightingale,\n",            /* Line 4 */
  "  and not the lark,\n",                /* Line 5 */
  "That pierced the fearful hollow\n",    /* Line 6 */
  "  of thine ear;\n",                    /* Line 7 */
  "Nightly she sings\n",                  /* Line 8 */
  "  on yon pomegranate-tree:\n",         /* Line 9 */
  "Believe me, love,\n",                  /* Line 10 */
  "  it was the nightingale.\n",          /* Line 11 */
  "\nROMEO\n",                            /* Line 12 */
  "It was the lark,\n",                   /* Line 13 */
  "  the herald of the morn,\n",          /* Line 14 */
  "No nightingale:\n",                    /* Line 15 */
  "  look, love, what envious streaks\n", /* Line 16 */
  "Do lace the severing clouds\n",        /* Line 17 */
  "  in yonder east:\n",                  /* Line 18 */
  "Night's candles are burnt out,\n",     /* Line 19 */
  "  and jocund day\n",                   /* Line 20 */
  "Stands tiptoe\n",                      /* Line 21 */
  "  on the misty mountain tops.\n",      /* Line 22 */
  "I must be gone and live,\n",           /* Line 23 */
  "  or stay and die.\n"                  /* Line 24 */
};
#endif

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
 * Name: nxcon_suinitialize
 ****************************************************************************/

#ifndef CONFIG_NX_MULTIUSER
static inline int nxcon_suinitialize(void)
{
  FAR NX_DRIVERTYPE *dev;

#if defined(CONFIG_EXAMPLES_NXCON_EXTERNINIT)
  /* Use external graphics driver initialization */

  message("nxcon_initialize: Initializing external graphics device\n");
  dev = up_nxdrvinit(CONFIG_EXAMPLES_NXCON_DEVNO);
  if (!dev)
    {
      message("nxcon_initialize: up_nxdrvinit failed, devno=%d\n", CONFIG_EXAMPLES_NXCON_DEVNO);
      return ERROR;
    }

#elif defined(CONFIG_NX_LCDDRIVER)
  int ret;

  /* Initialize the LCD device */

  message("nxcon_initialize: Initializing LCD\n");
  ret = up_lcdinitialize();
  if (ret < 0)
    {
      message("nxcon_initialize: up_lcdinitialize failed: %d\n", -ret);
      return ERROR;
    }

  /* Get the device instance */

  dev = up_lcdgetdev(CONFIG_EXAMPLES_NXCON_DEVNO);
  if (!dev)
    {
      message("nxcon_initialize: up_lcdgetdev failed, devno=%d\n",
              CONFIG_EXAMPLES_NXCON_DEVNO);
      return ERROR;
    }

  /* Turn the LCD on at 75% power */

  (void)dev->setpower(dev, ((3*CONFIG_LCD_MAXPOWER + 3)/4));
#else
  int ret;

  /* Initialize the frame buffer device */

  message("nxcon_initialize: Initializing framebuffer\n");
  ret = up_fbinitialize();
  if (ret < 0)
    {
      message("nxcon_initialize: up_fbinitialize failed: %d\n", -ret);
      return ERROR;
    }

  dev = up_fbgetvplane(CONFIG_EXAMPLES_NXCON_VPLANE);
  if (!dev)
    {
      message("nxcon_initialize: up_fbgetvplane failed, vplane=%d\n", CONFIG_EXAMPLES_NXCON_VPLANE);
      return ERROR;
    }
#endif

  /* Then open NX */

  message("nxcon_initialize: Open NX\n");
  g_nxcon_vars.hnx = nx_open(dev);
  if (!g_nxcon_vars.hnx)
    {
      message("nxcon_initialize: nx_open failed: %d\n", errno);
      return ERROR;
    }
  return OK;
}
#endif

/****************************************************************************
 * Name: nxcon_initialize
 ****************************************************************************/

#ifdef CONFIG_NX_MULTIUSER
static inline int nxcon_muinitialize(void)
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
#endif

/****************************************************************************
 * Name: nxcon_initialize
 ****************************************************************************/

static int nxcon_initialize(void)
{
#ifdef CONFIG_NX_MULTIUSER
  return nxcon_muinitialize();
#else
  return nxcon_suinitialize();
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: user_start/nxcon_main
 ****************************************************************************/

#ifdef CONFIG_NSH_BUILTIN_APPS
#  define MAIN_NAME nxcon_main
#  define MAIN_NAME_STRING "nxcon_main"
#else
#  define MAIN_NAME user_start
#  define MAIN_NAME_STRING "user_start"
#endif

int MAIN_NAME(int argc, char **argv)
{
  int exitcode = EXIT_FAILURE;
  nxgl_mxpixel_t color;
  int ndx;
  int ret;
  int fd;

  /* Reset all global data */

  memset(&g_nxcon_vars, 0, sizeof(struct nxcon_state_s));

  /* NX Initialization ******************************************************/
  /* Initialize NX */

  ret = nxcon_initialize();
  message(MAIN_NAME_STRING ": NX handle=%p\n", g_nxcon_vars.hnx);
  if (!g_nxcon_vars.hnx || ret < 0)
    {
      message(MAIN_NAME_STRING ": Failed to get NX handle: %d\n", errno);
      goto errout;
    }

  /* Set the background to the configured background color */

  message(MAIN_NAME_STRING ": Set background color=%d\n", CONFIG_EXAMPLES_NXCON_BGCOLOR);
  color = CONFIG_EXAMPLES_NXCON_BGCOLOR;
  ret = nx_setbgcolor(g_nxcon_vars.hnx, &color);
  if (ret < 0)
    {
      message(MAIN_NAME_STRING ": nx_setbgcolor failed: %d\n", errno);
      goto errout_with_nx;
    }

  /* Window Configuration ***************************************************/
  /* Create a window */

  message(MAIN_NAME_STRING ": Create window\n");
  g_nxcon_vars.hwnd = nxtk_openwindow(g_nxcon_vars.hnx, &g_nxconcb, NULL);
  if (!g_nxcon_vars.hwnd)
    {
      message(MAIN_NAME_STRING ": nxtk_openwindow failed: %d\n", errno);
      goto errout_with_nx;
    }
  message(MAIN_NAME_STRING ": hwnd=%p\n", g_nxcon_vars.hwnd);

  /* Wait until we have the screen resolution.  We'll have this immediately
   * unless we are dealing with the NX server.
   */

  while (!g_nxcon_vars.haveres)
    {
      (void)sem_wait(&g_nxcon_vars.eventsem);
    }
  message(MAIN_NAME_STRING ": Screen resolution (%d,%d)\n", g_nxcon_vars.xres, g_nxcon_vars.yres);

  /* Determine the size and position of the window */

  g_nxcon_vars.wndo.wsize.w = g_nxcon_vars.xres / 2 + g_nxcon_vars.xres / 4;
  g_nxcon_vars.wndo.wsize.h = g_nxcon_vars.yres / 2 + g_nxcon_vars.yres / 4;

  g_nxcon_vars.wpos.x       = g_nxcon_vars.xres / 8;
  g_nxcon_vars.wpos.y       = g_nxcon_vars.yres / 8;

  /* Set the window position */

  message(MAIN_NAME_STRING ": Set window position to (%d,%d)\n",
          g_nxcon_vars.wpos.x, g_nxcon_vars.wpos.y);

  ret = nxtk_setposition(g_nxcon_vars.hwnd, &g_nxcon_vars.wpos);
  if (ret < 0)
    {
      message(MAIN_NAME_STRING ": nxtk_setposition failed: %d\n", errno);
      goto errout_with_hwnd;
    }

  /* Set the window size */

  message(MAIN_NAME_STRING ": Set window size to (%d,%d)\n",
          g_nxcon_vars.wndo.wsize.w, g_nxcon_vars.wndo.wsize.h);

  ret = nxtk_setsize(g_nxcon_vars.hwnd, &g_nxcon_vars.wndo.wsize);
  if (ret < 0)
    {
      message(MAIN_NAME_STRING ": nxtk_setsize failed: %d\n", errno);
      goto errout_with_hwnd;
    }

  /* Open the toolbar */

  message(MAIN_NAME_STRING ": Add toolbar to window\n");
  ret = nxtk_opentoolbar(g_nxcon_vars.hwnd, CONFIG_EXAMPLES_NXCON_TOOLBAR_HEIGHT, &g_nxtoolcb, NULL);
  if (ret < 0)
    {
      message(MAIN_NAME_STRING ": nxtk_opentoolbar failed: %d\n", errno);
      goto errout_with_hwnd;
    }

  /* NxConsole Configuration ************************************************/
  /* Use the window to create an NX console */

  g_nxcon_vars.wndo.wcolor[0] = CONFIG_EXAMPLES_NXCON_WCOLOR;
  g_nxcon_vars.wndo.fcolor[0] = CONFIG_EXAMPLES_NXCON_FONTCOLOR;
  g_nxcon_vars.wndo.fontid    = CONFIG_EXAMPLES_NXCON_FONTID;

  g_nxcon_vars.hdrvr = nxtk_register(g_nxcon_vars.hwnd, &g_nxcon_vars.wndo, CONFIG_EXAMPLES_NXCON_MINOR);
  if (!g_nxcon_vars.hdrvr)
    {
      message(MAIN_NAME_STRING ": nxtk_register failed: %d\n", errno);
      goto errout_with_hwnd;
    }

  /* Open the driver */

  fd = open(CONFIG_EXAMPLES_NXCON_DEVNAME, O_WRONLY);
  if (fd < 0)
    {
      message(MAIN_NAME_STRING ": open %s read-only failed: %d\n",
              CONFIG_EXAMPLES_NXCON_DEVNAME, errno);
      goto errout_with_driver;
    }

  /* Now re-direct stdout and stderr so that they use the NX console driver */

  (void)dup2(fd, 1);
  (void)dup2(fd, 2);

   /* And we can close our original driver fd */

   close(fd);

  /* Test Loop **************************************************************/
  /* Now loop, adding text to the NX console */

  ndx = 0;
  for (;;)
    {
      /* Sleep for one second */

      sleep(1);

      /* Give another line of text to the NX console.*/

      printf(g_nxcon_msg[ndx]);
      if (++ndx >= NCON_MSG_NLINES)
        {
#ifdef CONFIG_NSH_BUILTIN_APPS
          /* If this is an NSH built-in apps, then just return after all
           * of the lines have been presented.
           */

          break;
#else
          /* Otherwise, just reset the index to the first line and continue */

          ndx = 0;
#endif
        }
    }
  exitcode = EXIT_SUCCESS;

  /* Clean-up and Error Exits ***********************************************/

errout_with_driver:
  message(MAIN_NAME_STRING ": Unregister the NX console device\n");
  (void)nxcon_unregister(g_nxcon_vars.hdrvr);

errout_with_hwnd:
  message(MAIN_NAME_STRING ": Close the window\n");
  (void)nxtk_closewindow(g_nxcon_vars.hwnd);

errout_with_nx:
#ifdef CONFIG_NX_MULTIUSER
  /* Disconnect from the server */

  message(MAIN_NAME_STRING ": Disconnect from the server\n");
  nx_disconnect(g_nxcon_vars.hnx);
#else
  /* Close the server */

  message(MAIN_NAME_STRING ": Close NX\n");
  nx_close(g_nxcon_vars.hnx);
#endif
errout:
  return exitcode;
}
