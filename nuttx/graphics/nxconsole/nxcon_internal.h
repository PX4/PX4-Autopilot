/****************************************************************************
 * nuttx/graphics/nxconsole/nxcon_internal.h
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

#ifndef __GRAPHICS_NXCONSOLE_NXCON_INTERNAL_H
#define __GRAPHICS_NXCONSOLE_NXCON_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <semaphore.h>

#include <nuttx/fs/fs.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxtk.h>
#include <nuttx/nx/nxfonts.h>
#include <nuttx/nx/nxconsole.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* The maximum number of characters that can be remembered */

#ifndef CONFIG_NXCONSOLE_MXCHARS
#  define CONFIG_NXCONSOLE_MXCHARS 128
#endif

/* Font cache -- this is the number or pre-rendered font glyphs that can be
 * remembered.
 */

#ifdef CONFIG_NXCONSOLE_FONTCACHE
#  ifndef CONFIG_NXCONSOLE_CACHESIZE
#    define CONFIG_NXCONSOLE_CACHESIZE 16
#  endif
#else
#  undef CONFIG_NXCONSOLE_CACHESIZE
#endif

/* Pixel depth */

#ifndef CONFIG_NXCONSOLE_BPP
#  if !defined(CONFIG_NX_DISABLE_1BPP)
#    define CONFIG_NXCONSOLE_BPP 1
#  elif !defined(CONFIG_NX_DISABLE_2BPP)
#    define CONFIG_NXCONSOLE_BPP 2
#  elif !defined(CONFIG_NX_DISABLE_4BPP)
#    define CONFIG_NXCONSOLE_BPP 4
#  elif !defined(CONFIG_NX_DISABLE_8BPP)
#    define CONFIG_NXCONSOLE_BPP 8
#  elif !defined(CONFIG_NX_DISABLE_16BPP)
#    define CONFIG_NXCONSOLE_BPP 16
//#elif !defined(CONFIG_NX_DISABLE_24BPP)
//#    define CONFIG_NXCONSOLE_BPP 24
#  elif !defined(CONFIG_NX_DISABLE_32BPP)
#    define CONFIG_NXCONSOLE_BPP 32
#  else
#    error "No pixel depth provided"
#  endif
#endif

/* Space (in rows) between lines */

#ifndef CONFIG_NXCONSOLE_LINESEPARATION
#  define CONFIG_NXCONSOLE_LINESEPARATION 2
#endif

/* NxConsole Definitions ****************************************************/
/* Bitmap flags */

#define BMFLAGS_NOGLYPH   (1 << 0) /* No glyph available, use space */
#define BM_ISSPACE(bm)    (((bm)->flags & BMFLAGS_NOGLYPH) != 0)

/* Sizes and maximums */

#define MAX_USECNT        255  /* Limit to range of a uint8_t */

/* Device path formats */

#define NX_DEVNAME_FORMAT "/dev/nxcon%d"
#define NX_DEVNAME_SIZE   16

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Describes on set of console window callbacks */

struct nxcon_state_s;
struct nxcon_operations_s
{
  int (*fill)(FAR struct nxcon_state_s *priv,
              FAR const struct nxgl_rect_s *rect,
              nxgl_mxpixel_t wcolor[CONFIG_NX_NPLANES]);
#ifndef CONFIG_NXCONSOLE_NOGETRUN
  int (*move)(FAR struct nxcon_state_s *priv,
              FAR const struct nxgl_rect_s *rect,
              FAR const struct nxgl_point_s *offset);
#endif
  int (*bitmap)(FAR struct nxcon_state_s *priv,
                FAR const struct nxgl_rect_s *dest,
                FAR const void *src[CONFIG_NX_NPLANES],
                FAR const struct nxgl_point_s *origin,
                unsigned int stride);
};

/* Describes one cached glyph bitmap */

struct nxcon_glyph_s
{
  uint8_t code;                        /* Character code */
  uint8_t height;                      /* Height of this glyph (in rows) */
  uint8_t width;                       /* Width of this glyph (in pixels) */
  uint8_t stride;                      /* Width of the glyph row (in bytes) */
#ifdef CONFIG_NXCONSOLE_FONTCACHE
  uint8_t usecnt;                      /* Use count */
#endif
  FAR uint8_t *bitmap;                 /* Allocated bitmap memory */
};

/* Describes on character on the display */

struct nxcon_bitmap_s
{
  uint8_t code;                        /* Character code */
  uint8_t flags;                       /* See BMFLAGS_* */
  struct nxgl_point_s pos;             /* Character position */
};

/* Describes the state of one NX console driver*/

struct nxcon_state_s
{
  FAR const struct nxcon_operations_s *ops; /* Window operations */
  FAR void *handle;                         /* The window handle */
  FAR struct nxcon_window_s wndo;           /* Describes the window and font */
  NXHANDLE font;                            /* The current font handle */
  sem_t exclsem;                            /* Forces mutually exclusive access */
  struct nxgl_point_s fpos;                 /* Next display position */

  uint16_t maxchars;                        /* Size of the bm[] array */
  uint16_t nchars;                          /* Number of chars in the bm[] array */

  uint8_t minor;                            /* Device minor number */
  uint8_t fheight;                          /* Max height of a font in pixels */
  uint8_t fwidth;                           /* Max width of a font in pixels */
  uint8_t spwidth;                          /* The width of a space */
#ifdef CONFIG_NXCONSOLE_FONTCACHE
  uint8_t maxglyphs;                        /* Size of the glyph[] array */
#endif

  /* Font cache data storage */

  struct nxcon_bitmap_s bm[CONFIG_NXCONSOLE_MXCHARS];

  /* Glyph cache data storage */

#ifdef CONFIG_NXCONSOLE_FONTCACHE
  struct nxcon_glyph_s  glyph[CONFIG_NXCONSOLE_CACHESIZE];
#else
  /* A glyph cache of size one -- all fonts will be re-rendered on each use */

  struct nxcon_glyph_s glyph;
#endif
};

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/* This is the common NX driver file operations */

extern const struct file_operations g_nxcon_drvrops;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
/* Common device registration */

FAR struct nxcon_state_s *nxcon_register(NXCONSOLE handle,
    FAR struct nxcon_window_s *wndo, FAR const struct nxcon_operations_s *ops,
    int minor);

/* Generic text display helpers */

void nxcon_home(FAR struct nxcon_state_s *priv);
void nxcon_newline(FAR struct nxcon_state_s *priv);
void nxcon_putc(FAR struct nxcon_state_s *priv, uint8_t ch);
void nxcon_fillchar(FAR struct nxcon_state_s *priv,
     FAR const struct nxgl_rect_s *rect, FAR const struct nxcon_bitmap_s *bm);

/* Scrolling support */

void nxcon_scroll(FAR struct nxcon_state_s *priv, int scrollheight);

#endif /* __GRAPHICS_NXCONSOLE_NXCON_INTERNAL_H */
