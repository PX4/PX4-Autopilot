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

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nxtk.h>
#include <nuttx/nx/nxconsole.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_NX
#  error "NX is not enabled (CONFIG_NX)"
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

/* Background color */

#ifndef CONFIG_EXAMPLES_NXCON_BGCOLOR
#  if CONFIG_EXAMPLES_NXCON_BPP == 24 || CONFIG_EXAMPLES_NXCON_BPP == 32
#    define CONFIG_EXAMPLES_NXCON_BGCOLOR 0x007b68ee
#  elif CONFIG_EXAMPLES_NXCON_BPP == 16
#    define CONFIG_EXAMPLES_NXCON_BGCOLOR 0x7b5d
#  else
#    define CONFIG_EXAMPLES_NXCON_BGCOLOR ' '
# endif
#endif

/* Window color */

#ifndef CONFIG_EXAMPLES_NXCON_WCOLOR
#  if CONFIG_EXAMPLES_NXCON_BPP == 24 || CONFIG_EXAMPLES_NXCON_BPP == 32
#    define CONFIG_EXAMPLES_NXCON_WCOLOR 0x007b68ee
#  elif CONFIG_EXAMPLES_NXCON_BPP == 16
#    define CONFIG_EXAMPLES_NXCON_WCOLOR 0x7b5d
#  else
#    define CONFIG_EXAMPLES_NXCON_WCOLOR ' '
# endif
#endif

/* Font ID */

#ifndef CONFIG_EXAMPLES_NXCON_FONTID
#  define CONFIG_EXAMPLES_NXCON_FONTID NXFONT_DEFAULT
#endif

/* Font color */

#ifndef CONFIG_EXAMPLES_NXCON_FONTCOLOR
#  if CONFIG_EXAMPLES_NXCON_BPP == 24 || CONFIG_EXAMPLES_NXCON_BPP == 32
#    define CONFIG_EXAMPLES_NXCON_FONTCOLOR 0x00000000
#  elif CONFIG_EXAMPLES_NXCON_BPP == 16
#    define CONFIG_EXAMPLES_NXCON_FONTCOLOR 0x0000
#  else
#    define CONFIG_EXAMPLES_NXCON_FONTCOLOR 'F'
#  endif
#endif

/* Height of the toolbar */

#ifndef CONFIG_EXAMPLES_NXCON_TOOLBAR_HEIGHT
#  define CONFIG_EXAMPLES_NXCON_TOOLBAR_HEIGHT 16
#endif

/* Multi-user NX support */

#ifdef CONFIG_NX_MULTIUSER
#  ifdef CONFIG_DISABLE_MQUEUE
#    error "The multi-threaded example requires MQ support (CONFIG_DISABLE_MQUEUE=n)"
#  endif
#  ifdef CONFIG_DISABLE_SIGNALS
#    error "This example requires signal support (CONFIG_DISABLE_SIGNALS=n)"
#  endif
#  ifdef CONFIG_DISABLE_PTHREAD
#    error "This example requires pthread support (CONFIG_DISABLE_PTHREAD=n)"
#  endif
#  ifndef CONFIG_NX_BLOCKING
#    error "This example depends on CONFIG_NX_BLOCKING"
#  endif
#  ifndef CONFIG_EXAMPLES_NXCON_STACKSIZE
#    define CONFIG_EXAMPLES_NXCON_STACKSIZE 2048
#  endif
#  ifndef CONFIG_EXAMPLES_NXCON_LISTENERPRIO
#    define CONFIG_EXAMPLES_NXCON_LISTENERPRIO 100
#  endif
#  ifndef CONFIG_EXAMPLES_NXCON_CLIENTPRIO
#    define CONFIG_EXAMPLES_NXCON_CLIENTPRIO 100
#  endif
#  ifndef CONFIG_EXAMPLES_NXCON_SERVERPRIO
#    define CONFIG_EXAMPLES_NXCON_SERVERPRIO 120
#  endif
#  ifndef CONFIG_EXAMPLES_NXCON_NOTIFYSIGNO
#    define CONFIG_EXAMPLES_NXCON_NOTIFYSIGNO 4
#  endif
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

/* Debug ********************************************************************/

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(...) lib_lowprintf(__VA_ARGS__)
#    define msgflush()
#  else
#    define message(...) printf(__VA_ARGS__)
#    define msgflush() fflush(stdout)
#  endif
#else
#  ifdef CONFIG_DEBUG
#    define message lib_lowprintf
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
  volatile bool             haveres;   /* True: Have screen resolution */
#ifdef CONFIG_NX_MULTIUSER
  bool                      connected; /* True: Connected to server */
#endif
  sem_t                     eventsem;  /* Control waiting for display events */
  NXHANDLE                  hnx;       /* The connection handler */
  NXTKWINDOW                hwnd;      /* The window */
  NXCONSOLE                 hdrvr;     /* The console driver */
  FAR struct nxcon_window_s wndo;      /* Describes the window */
  nxgl_coord_t              xres;      /* Screen X resolution */
  nxgl_coord_t              yres;      /* Screen Y resolution */
  struct nxgl_size_s        wsize;     /* Window size */
  struct nxgl_point_s       wpos;      /* Window position */
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

#ifdef CONFIG_EXAMPLES_NXCON_EXTERNINIT
extern FAR NX_DRIVERTYPE *up_nxdrvinit(unsigned int devno);
#endif
#if defined(CONFIG_NX) && defined(CONFIG_NX_MULTIUSER)
extern int nxcon_server(int argc, char *argv[]);
extern FAR void *nxcon_listener(FAR void *arg);
#endif

#endif /* __EXAMPLES_NXCONSOLE_NXCON_INTERNAL_H */
