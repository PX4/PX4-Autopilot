/****************************************************************************
 * examples/nxconsole/nxcon_internal.h
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

#ifndef __EXAMPLES_NXCONSOLE_NXCON_INTERNAL_H
#define __EXAMPLES_NXCONSOLE_NXCON_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>

#include <nuttx/rgbcolors.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nxtk.h>
#include <nuttx/nx/nxconsole.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* Need NX graphics support */

#ifndef CONFIG_NX
#  error "NX is not enabled (CONFIG_NX=y)"
#endif

/* Can't do the NxConsole example if the NxConsole driver is not built */

#ifndef CONFIG_NXCONSOLE
#  error "NxConsole is not enabled (CONFIG_NXCONSOLE=y)"
#endif

/* NxConsole requires NX Multi-user mode */

#ifndef CONFIG_NX_MULTIUSER
#  error "Multi-user NX support is required (CONFIG_NX_MULTIUSER=y)"
#endif

/* If there is no NSH console, then why are we running this example? */

#ifndef CONFIG_NSH_CONSOLE
#  warning "Expected CONFIG_NSH_CONSOLE=y"
#endif

/* The NSH telnet console requires networking support (and TCP/IP) */

#ifndef CONFIG_NET
#  undef CONFIG_NSH_TELNET
#endif

/* If not specified, assume that the hardware supports one video plane */

#if CONFIG_NX_NPLANES != 1
#  error "Only CONFIG_NX_NPLANES==1 supported"
#endif

#ifndef CONFIG_EXAMPLES_NXCON_VPLANE
#  define CONFIG_EXAMPLES_NXCON_VPLANE 0
#endif

/* Pixel depth.  If none provided, pick the smallest enabled pixel depth */

#ifndef CONFIG_EXAMPLES_NXCON_BPP
#  if !defined(CONFIG_NX_DISABLE_1BPP)
#    define CONFIG_EXAMPLES_NXCON_BPP 1
#  elif !defined(CONFIG_NX_DISABLE_2BPP)
#    define CONFIG_EXAMPLES_NXCON_BPP 2
#  elif !defined(CONFIG_NX_DISABLE_4BPP)
#    define CONFIG_EXAMPLES_NXCON_BPP 4
#  elif !defined(CONFIG_NX_DISABLE_8BPP)
#    define CONFIG_EXAMPLES_NXCON_BPP 8
#  elif !defined(CONFIG_NX_DISABLE_16BPP)
#    define CONFIG_EXAMPLES_NXCON_BPP 16
//#elif !defined(CONFIG_NX_DISABLE_24BPP)
//#    define CONFIG_NXCONSOLE_BPP 24
#  elif !defined(CONFIG_NX_DISABLE_32BPP)
#    define CONFIG_EXAMPLES_NXCON_BPP 32
#  else
#    error "No pixel depth provided"
#  endif
#endif

/* Background color (default is darker royal blue) */

#ifndef CONFIG_EXAMPLES_NXCON_BGCOLOR
#  if CONFIG_EXAMPLES_NXCON_BPP == 24 || CONFIG_EXAMPLES_NXCON_BPP == 32
#    define CONFIG_EXAMPLES_NXCON_BGCOLOR RGBTO24(39, 64, 139)
#  elif CONFIG_EXAMPLES_NXCON_BPP == 16
#    define CONFIG_EXAMPLES_NXCON_BGCOLOR RGBTO16(39, 64, 139)
#  else
#    define CONFIG_EXAMPLES_NXCON_BGCOLOR RGBTO8(39, 64, 139)
# endif
#endif

/* Window color (lighter steel blue) */

#ifndef CONFIG_EXAMPLES_NXCON_WCOLOR
#  if CONFIG_EXAMPLES_NXCON_BPP == 24 || CONFIG_EXAMPLES_NXCON_BPP == 32
#    define CONFIG_EXAMPLES_NXCON_WCOLOR RGBTO24(202, 225, 255)
#  elif CONFIG_EXAMPLES_NXCON_BPP == 16
#    define CONFIG_EXAMPLES_NXCON_WCOLOR RGBTO16(202, 225, 255)
#  else
#    define CONFIG_EXAMPLES_NXCON_WCOLOR RGBTO8(202, 225, 255)
# endif
#endif

/* Toolbar color (medium grey) */

#ifndef CONFIG_EXAMPLES_NXCON_TBCOLOR
#  if CONFIG_EXAMPLES_NX_BPP == 24 || CONFIG_EXAMPLES_NX_BPP == 32
#    define CONFIG_EXAMPLES_NXCON_TBCOLOR RGBTO24(188, 188, 188)
#  elif CONFIG_EXAMPLES_NX_BPP == 16
#    define CONFIG_EXAMPLES_NXCON_TBCOLOR RGBTO16(188, 188, 188)
#  else
#    define CONFIG_EXAMPLES_NXCON_TBCOLOR RGBTO8(188, 188, 188)
#  endif
#endif

/* Font ID */

#ifndef CONFIG_EXAMPLES_NXCON_FONTID
#  define CONFIG_EXAMPLES_NXCON_FONTID NXFONT_DEFAULT
#endif

/* Font color */

#ifndef CONFIG_EXAMPLES_NXCON_FONTCOLOR
#  if CONFIG_EXAMPLES_NXCON_BPP == 24 || CONFIG_EXAMPLES_NXCON_BPP == 32
#    define CONFIG_EXAMPLES_NXCON_FONTCOLOR RGBTO24(0, 0, 0)
#  elif CONFIG_EXAMPLES_NXCON_BPP == 16
#    define CONFIG_EXAMPLES_NXCON_FONTCOLOR RGBTO16(0, 0, 0)
#  else
#    define CONFIG_EXAMPLES_NXCON_FONTCOLOR RGBTO8(0, 0, 0)
#  endif
#endif

/* Height of the toolbar */

#ifndef CONFIG_EXAMPLES_NXCON_TOOLBAR_HEIGHT
#  define CONFIG_EXAMPLES_NXCON_TOOLBAR_HEIGHT 16
#endif

/* Multi-user NX support */

#ifdef CONFIG_DISABLE_MQUEUE
#  error "The multi-threaded example requires MQ support (CONFIG_DISABLE_MQUEUE=n)"
#endif
#ifdef CONFIG_DISABLE_SIGNALS
#  error "This example requires signal support (CONFIG_DISABLE_SIGNALS=n)"
#endif
#ifdef CONFIG_DISABLE_PTHREAD
#  error "This example requires pthread support (CONFIG_DISABLE_PTHREAD=n)"
#endif
#ifndef CONFIG_NX_BLOCKING
#  error "This example depends on CONFIG_NX_BLOCKING"
#endif
#ifndef CONFIG_EXAMPLES_NXCON_STACKSIZE
#  define CONFIG_EXAMPLES_NXCON_STACKSIZE 2048
#endif
#ifndef CONFIG_EXAMPLES_NXCON_LISTENERPRIO
#  define CONFIG_EXAMPLES_NXCON_LISTENERPRIO 100
#endif
#ifndef CONFIG_EXAMPLES_NXCON_CLIENTPRIO
#  define CONFIG_EXAMPLES_NXCON_CLIENTPRIO 100
#endif
#ifndef CONFIG_EXAMPLES_NXCON_SERVERPRIO
#  define CONFIG_EXAMPLES_NXCON_SERVERPRIO 120
#endif
#ifndef CONFIG_EXAMPLES_NXCON_NOTIFYSIGNO
#  define CONFIG_EXAMPLES_NXCON_NOTIFYSIGNO 4
#endif

/* Graphics Device */

#ifndef CONFIG_EXAMPLES_NXCON_DEVNO
#  define CONFIG_EXAMPLES_NXCON_DEVNO 0
#endif

/* NX Console Device */

#ifndef CONFIG_EXAMPLES_NXCON_MINOR
#  define CONFIG_EXAMPLES_NXCON_MINOR 0
#endif

#ifndef CONFIG_EXAMPLES_NXCON_DEVNAME
#  define CONFIG_EXAMPLES_NXCON_DEVNAME "/dev/nxcon0"
#endif

/* NxConsole task */

#ifndef CONFIG_EXAMPLES_NXCONSOLE_PRIO
#  define CONFIG_EXAMPLES_NXCONSOLE_PRIO SCHED_PRIORITY_DEFAULT
#endif

#ifndef CONFIG_EXAMPLES_NXCONSOLE_STACKSIZE
#  define CONFIG_EXAMPLES_NXCONSOLE_STACKSIZE 2048
#endif

/* Debug ********************************************************************/

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(...) lowsyslog(__VA_ARGS__)
#    define msgflush()
#  else
#    define message(...) printf(__VA_ARGS__)
#    define msgflush() fflush(stdout)
#  endif
#else
#  ifdef CONFIG_DEBUG
#    define message lowsyslog
#    define msgflush()
#  else
#    define message printf
#    define msgflush() fflush(stdout)
#  endif
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* All example global variables are retained in a structure to minimize
 * the chance of name collisions.
 */

struct nxcon_state_s
{
  volatile bool         haveres;   /* True: Have screen resolution */
  volatile bool         connected; /* True: Connected to server */
  sem_t                 eventsem;  /* Control waiting for display events */
  pid_t                 pid;       /* Console task ID */
  NXHANDLE              hnx;       /* The connection handler */
  NXTKWINDOW            hwnd;      /* The window */
  NXCONSOLE             hdrvr;     /* The console driver */
  struct nxcon_window_s wndo;      /* Describes the window */
  nxgl_coord_t          xres;      /* Screen X resolution */
  nxgl_coord_t          yres;      /* Screen Y resolution */
  struct nxgl_size_s    wsize;     /* Window size */
  struct nxgl_point_s   wpos;      /* Window position */
};

/****************************************************************************
 * Public Variables
 ****************************************************************************/
/* All example global variables are retained in a structure to minimize
 * the chance of name collisions.
 */

extern struct nxcon_state_s g_nxcon_vars;

/* NX callback vtables */

extern const struct nx_callback_s g_nxconcb;
extern const struct nx_callback_s g_nxtoolcb;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
/* Board-specific driver intiialization */

#ifdef CONFIG_EXAMPLES_NXCON_EXTERNINIT
extern FAR NX_DRIVERTYPE *up_nxdrvinit(unsigned int devno);
#endif

/* Server thread support */

extern int nxcon_server(int argc, char *argv[]);
extern FAR void *nxcon_listener(FAR void *arg);

#endif /* __EXAMPLES_NXCONSOLE_NXCON_INTERNAL_H */
