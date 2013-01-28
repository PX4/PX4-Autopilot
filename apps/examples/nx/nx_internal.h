/****************************************************************************
 * examples/nx/nx_internal.h
 *
 *   Copyright (C) 2008-2011 Gregory Nutt. All rights reserved.
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

#ifndef __EXAMPLES_NX_NX_INTERNAL_H
#define __EXAMPLES_NX_NX_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxtk.h>
#include <nuttx/nx/nxfonts.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_NX
#  error "NX is not enabled (CONFIG_NX)"
#endif

#ifndef CONFIG_EXAMPLES_NX_VPLANE
#    define CONFIG_EXAMPLES_NX_VPLANE 0
#endif

#ifndef CONFIG_EXAMPLES_NX_BPP
#  define CONFIG_EXAMPLES_NX_BPP 32
#endif

#ifndef CONFIG_EXAMPLES_NX_BGCOLOR
#  if CONFIG_EXAMPLES_NX_BPP == 24 || CONFIG_EXAMPLES_NX_BPP == 32
#    define CONFIG_EXAMPLES_NX_BGCOLOR 0x007b68ee
#  elif CONFIG_EXAMPLES_NX_BPP == 16
#    define CONFIG_EXAMPLES_NX_BGCOLOR 0x7b5d
#  else
#    define CONFIG_EXAMPLES_NX_BGCOLOR ' '
# endif
#endif

#ifndef CONFIG_EXAMPLES_NX_COLOR1
#  if CONFIG_EXAMPLES_NX_BPP == 24 || CONFIG_EXAMPLES_NX_BPP == 32
#    define CONFIG_EXAMPLES_NX_COLOR1 0x00e6e6fa
#  elif CONFIG_EXAMPLES_NX_BPP == 16
#    define CONFIG_EXAMPLES_NX_COLOR1 0xe73f
#  else
#    define CONFIG_EXAMPLES_NX_COLOR1 '1'
# endif
#endif

#ifndef CONFIG_EXAMPLES_NX_COLOR2
#  if CONFIG_EXAMPLES_NX_BPP == 24 || CONFIG_EXAMPLES_NX_BPP == 32
#    define CONFIG_EXAMPLES_NX_COLOR2 0x00dcdcdc
#  elif CONFIG_EXAMPLES_NX_BPP == 16
#    define CONFIG_EXAMPLES_NX_COLOR2 0xdefb
#  else
#    define CONFIG_EXAMPLES_NX_COLOR2 '2'
# endif
#endif

#ifndef CONFIG_EXAMPLES_NX_TBCOLOR
#  if CONFIG_EXAMPLES_NX_BPP == 24 || CONFIG_EXAMPLES_NX_BPP == 32
#    define CONFIG_EXAMPLES_NX_TBCOLOR 0x00a9a9a9
#  elif CONFIG_EXAMPLES_NX_BPP == 16
#    define CONFIG_EXAMPLES_NX_TBCOLOR 0xad55
#  else
#    define CONFIG_EXAMPLES_NX_TBCOLOR 'T'
#  endif
#endif

#ifndef CONFIG_EXAMPLES_NX_FONTID
#  define CONFIG_EXAMPLES_NX_FONTID NXFONT_DEFAULT
#endif

#ifndef CONFIG_EXAMPLES_NX_FONTCOLOR
#  if CONFIG_EXAMPLES_NX_BPP == 24 || CONFIG_EXAMPLES_NX_BPP == 32
#    define CONFIG_EXAMPLES_NX_FONTCOLOR 0x00000000
#  elif CONFIG_EXAMPLES_NX_BPP == 16
#    define CONFIG_EXAMPLES_NX_FONTCOLOR 0x0000
#  else
#    define CONFIG_EXAMPLES_NX_FONTCOLOR 'F'
#  endif
#endif

#ifndef CONFIG_EXAMPLES_NX_TOOLBAR_HEIGHT
#  define CONFIG_EXAMPLES_NX_TOOLBAR_HEIGHT 16
#endif

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
#  ifndef CONFIG_EXAMPLES_NX_STACKSIZE
#    define CONFIG_EXAMPLES_NX_STACKSIZE 2048
#  endif
#  ifndef CONFIG_EXAMPLES_NX_LISTENERPRIO
#    define CONFIG_EXAMPLES_NX_LISTENERPRIO 100
#  endif
#  ifndef CONFIG_EXAMPLES_NX_CLIENTPRIO
#    define CONFIG_EXAMPLES_NX_CLIENTPRIO 100
#  endif
#  ifndef CONFIG_EXAMPLES_NX_SERVERPRIO
#    define CONFIG_EXAMPLES_NX_SERVERPRIO 120
#  endif
#  ifndef CONFIG_EXAMPLES_NX_NOTIFYSIGNO
#    define CONFIG_EXAMPLES_NX_NOTIFYSIGNO 4
#  endif
#endif

#ifdef CONFIG_EXAMPLES_NX_RAWWINDOWS
#  define NXEGWINDOW NXWINDOW
#else
#  define NXEGWINDOW NXTKWINDOW
#endif

#define NXTK_MAXKBDCHARS 16

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
  NXEXIT_SIGPROCMASK,
  NXEXIT_SCHEDSETPARAM,
  NXEXIT_EVENTNOTIFY,
  NXEXIT_TASKCREATE,
  NXEXIT_PTHREADCREATE,
  NXEXIT_EXTINITIALIZE,
  NXEXIT_FBINITIALIZE,
  NXEXIT_FBGETVPLANE,
  NXEXIT_LCDINITIALIZE,
  NXEXIT_LCDGETDEV,
  NXEXIT_NXOPEN,
  NXEXIT_FONTOPEN,
  NXEXIT_NXOPENTOOLBAR,
  NXEXIT_NXCONNECT,
  NXEXIT_NXSETBGCOLOR,
  NXEXIT_NXOPENWINDOW,
  NXEXIT_NXSETSIZE,
  NXEXIT_NXSETPOSITION,
  NXEXIT_NXLOWER,
  NXEXIT_NXRAISE,
  NXEXIT_NXCLOSEWINDOW,
  NXEXIT_LOSTSERVERCONN
};

/* Describes one cached glyph bitmap */

struct nxeg_glyph_s
{
  uint8_t code;                        /* Character code */
  uint8_t height;                      /* Height of this glyph (in rows) */
  uint8_t width;                       /* Width of this glyph (in pixels) */
  uint8_t stride;                      /* Width of the glyph row (in bytes) */
  FAR uint8_t *bitmap;                 /* Allocated bitmap memory */
};

/* Describes on character on the display */

struct nxeg_bitmap_s
{
  struct nxgl_rect_s bounds;            /* Size/position of bitmap */
  FAR const struct nxeg_glyph_s *glyph; /* The cached glyph */
};

/* Describes the overall state of on one window */

struct nxeg_state_s
{
  uint8_t wnum;                        /* Window number */
  nxgl_mxpixel_t color[CONFIG_NX_NPLANES]; /* Window color */

#ifdef CONFIG_NX_KBD
  uint8_t height;                      /* Max height of a font in pixels */
  uint8_t width;                       /* Max width of a font in pixels */
  uint8_t spwidth;                     /* The width of a space */

  uint8_t nchars;                      /* Number of KBD chars received */
  uint8_t nglyphs;                     /* Number of glyphs cached */

  struct nxeg_bitmap_s bm[NXTK_MAXKBDCHARS];
  struct nxeg_glyph_s  glyph[NXTK_MAXKBDCHARS];
#endif
};

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/* The connecton handle */

extern NXHANDLE g_hnx;

/* NX callback vtables */

extern const struct nx_callback_s g_nxcb;
#ifndef CONFIG_EXAMPLES_NX_RAWWINDOWS
extern const struct nx_callback_s g_tbcb;
#endif

/* The font handle */

extern NXHANDLE g_fonthandle;

/* The screen resolution */

extern nxgl_coord_t g_xres;
extern nxgl_coord_t g_yres;

extern bool b_haveresolution;
#ifdef CONFIG_NX_MULTIUSER
extern bool g_connected;
#endif
extern sem_t g_semevent;

/* Colors used to fill window 1 & 2 */

extern nxgl_mxpixel_t g_color1[CONFIG_NX_NPLANES];
extern nxgl_mxpixel_t g_color2[CONFIG_NX_NPLANES];
#ifndef CONFIG_EXAMPLES_NX_RAWWINDOWS
extern nxgl_mxpixel_t g_tbcolor[CONFIG_NX_NPLANES];
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_EXAMPLES_NX_EXTERNINIT
extern FAR NX_DRIVERTYPE *up_nxdrvinit(unsigned int devno);
#endif

#if defined(CONFIG_NX) && defined(CONFIG_NX_MULTIUSER)
extern int nx_servertask(int argc, char *argv[]);
extern FAR void *nx_listenerthread(FAR void *arg);
#endif

#ifdef CONFIG_NX_KBD
extern void nxeg_kbdin(NXWINDOW hwnd, uint8_t nch, const uint8_t *ch, FAR void *arg);
#ifndef CONFIG_EXAMPLES_NX_RAWWINDOWS
extern void nxeg_tbkbdin(NXWINDOW hwnd, uint8_t nch, const uint8_t *ch, FAR void *arg);
#endif
extern void nxeg_filltext(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                          FAR struct nxeg_state_s *st);
#endif

#endif /* __EXAMPLES_NX_NX_INTERNAL_H */
