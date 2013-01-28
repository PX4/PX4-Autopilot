/****************************************************************************
 * examples/nxhello/nxhello.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

#ifndef __APPS_EXAMPLES_NXHELLO_NXHELLO_H
#define __APPS_EXAMPLES_NXHELLO_NXHELLO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>

#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxfonts.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_NX
#  error "NX is not enabled (CONFIG_NX)"
#endif

#ifndef CONFIG_EXAMPLES_NXHELLO_VPLANE
#    define CONFIG_EXAMPLES_NXHELLO_VPLANE 0
#endif

#ifndef CONFIG_EXAMPLES_NXHELLO_BPP
#  define CONFIG_EXAMPLES_NXHELLO_BPP 32
#endif

#ifndef CONFIG_EXAMPLES_NXHELLO_BGCOLOR
#  if CONFIG_EXAMPLES_NXHELLO_BPP == 24 || CONFIG_EXAMPLES_NXHELLO_BPP == 32
#    define CONFIG_EXAMPLES_NXHELLO_BGCOLOR 0x007b68ee
#  elif CONFIG_EXAMPLES_NXHELLO_BPP == 16
#    define CONFIG_EXAMPLES_NXHELLO_BGCOLOR 0x7b5d
#  else
#    define CONFIG_EXAMPLES_NXHELLO_BGCOLOR ' '
# endif
#endif

#ifndef CONFIG_EXAMPLES_NXHELLO_FONTID
#  define CONFIG_EXAMPLES_NXHELLO_FONTID NXFONT_DEFAULT
#endif

#ifndef CONFIG_EXAMPLES_NXHELLO_FONTCOLOR
#  if CONFIG_EXAMPLES_NXHELLO_BPP == 24 || CONFIG_EXAMPLES_NXHELLO_BPP == 32
#    define CONFIG_EXAMPLES_NXHELLO_FONTCOLOR 0x00000000
#  elif CONFIG_EXAMPLES_NXHELLO_BPP == 16
#    define CONFIG_EXAMPLES_NXHELLO_FONTCOLOR 0x0000
#  else
#    define CONFIG_EXAMPLES_NXHELLO_FONTCOLOR 'F'
#  endif
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

enum exitcode_e
{
  NXEXIT_SUCCESS = 0,
  NXEXIT_EXTINITIALIZE,
  NXEXIT_FBINITIALIZE,
  NXEXIT_FBGETVPLANE,
  NXEXIT_LCDINITIALIZE,
  NXEXIT_LCDGETDEV,
  NXEXIT_NXOPEN,
  NXEXIT_FONTOPEN,
  NXEXIT_NXREQUESTBKGD,
  NXEXIT_NXSETBGCOLOR
};

/* Describes one cached glyph bitmap */

struct nxhello_glyph_s
{
  uint8_t code;                        /* Character code */
  uint8_t height;                      /* Height of this glyph (in rows) */
  uint8_t width;                       /* Width of this glyph (in pixels) */
  uint8_t stride;                      /* Width of the glyph row (in bytes) */
  uint8_t usecnt;                      /* Use count */
  FAR uint8_t *bitmap;                 /* Allocated bitmap memory */
};

/* Describes on character on the display */

struct nxhello_bitmap_s
{
  uint8_t code;                        /* Character code */
  uint8_t flags;                       /* See BMFLAGS_* */
  struct nxgl_point_s pos;             /* Character position */
};

struct nxhello_data_s
{
  /* The NX handles */

  NXHANDLE hnx;
  NXHANDLE hbkgd;
  NXHANDLE hfont;

  /* The screen resolution */

  nxgl_coord_t xres;
  nxgl_coord_t yres;

  volatile bool havepos;
  sem_t sem;
  volatile int code;
};

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/* NXHELLO state data */

extern struct nxhello_data_s g_nxhello;

/* NX callback vtables */

extern const struct nx_callback_s g_nxhellocb;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_EXAMPLES_NXHELLO_EXTERNINIT
extern FAR NX_DRIVERTYPE *up_nxdrvinit(unsigned int devno);
#endif

/* Background window interfaces */

extern void nxhello_hello(NXWINDOW hwnd);

#endif /* __APPS_EXAMPLES_NXHELLO_NXHELLO_H */
