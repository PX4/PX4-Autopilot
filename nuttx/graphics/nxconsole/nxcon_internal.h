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
/* NxConsole Definitions ****************************************************/
/* Bitmap flags */

#define BMFLAGS_NOGLYPH    (1 << 0) /* No glyph available, use space */
#define BM_ISSPACE(bm)     (((bm)->flags & BMFLAGS_NOGLYPH) != 0)

/* Sizes and maximums */

#define MAX_USECNT         255  /* Limit to range of a uint8_t */

/* Device path formats */

#define NX_DEVNAME_FORMAT  "/dev/nxcon%d"
#define NX_DEVNAME_SIZE    16

/* Semaphore protection */

#define NO_HOLDER          (pid_t)-1

/* VT100 escape sequence processing */

#define VT100_MAX_SEQUENCE 3

/****************************************************************************
 * Public Types
 ****************************************************************************/
/* Identifies the state of the VT100 escape sequence processing */

enum nxcon_vt100state_e
{
  VT100_NOT_CONSUMED = 0, /* Character is not part of a VT100 escape sequence */
  VT100_CONSUMED,         /* Character was consumed as part of the VT100 escape processing */
  VT100_PROCESSED,        /* The full VT100 escape sequence was processed */
  VT100_ABORT             /* Invalid/unsupported character in buffered escape sequence */
};

/* Describes on set of console window callbacks */

struct nxcon_state_s;
struct nxcon_operations_s
{
  int (*fill)(FAR struct nxcon_state_s *priv,
              FAR const struct nxgl_rect_s *rect,
              nxgl_mxpixel_t wcolor[CONFIG_NX_NPLANES]);
#ifndef CONFIG_NX_WRITEONLY
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
  uint8_t usecnt;                      /* Use count */
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
#ifdef CONFIG_DEBUG
  pid_t holder;                             /* Deadlock avoidance */
#endif
  uint8_t minor;                            /* Device minor number */

  /* Text output support */

  uint8_t fheight;                          /* Max height of a font in pixels */
  uint8_t fwidth;                           /* Max width of a font in pixels */
  uint8_t spwidth;                          /* The width of a space */
  uint8_t maxglyphs;                        /* Size of the glyph[] array */

  uint16_t maxchars;                        /* Size of the bm[] array */
  uint16_t nchars;                          /* Number of chars in the bm[] array */

  struct nxgl_point_s fpos;                 /* Next display position */

  /* VT100 escape sequence processing */

  char seq[VT100_MAX_SEQUENCE];             /* Buffered characters */
  uint8_t nseq;                             /* Number of buffered characters */

  /* Font cache data storage */

  struct nxcon_bitmap_s cursor;
  struct nxcon_bitmap_s bm[CONFIG_NXCONSOLE_MXCHARS];

  /* Glyph cache data storage */

  struct nxcon_glyph_s  glyph[CONFIG_NXCONSOLE_CACHESIZE];

  /* Keyboard input support */

#ifdef CONFIG_NXCONSOLE_NXKBDIN
  sem_t waitsem;                            /* Supports waiting for input data */
  uint8_t nwaiters;                         /* Number of threads waiting for data */
  uint8_t head;                             /* rxbuffer head/input index */
  uint8_t tail;                             /* rxbuffer tail/output index */

  uint8_t rxbuffer[CONFIG_NXCONSOLE_KBDBUFSIZE];

  /* The following is a list if poll structures of threads waiting for
   * driver events. The 'struct pollfd' reference for each open is also
   * retained in the f_priv field of the 'struct file'.
   */

#ifndef CONFIG_DISABLE_POLL
  struct pollfd *fds[CONFIG_RAMLOG_NPOLLWAITERS];
#endif
#endif /* CONFIG_NXCONSOLE_NXKBDIN */
};

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/* This is the common NX driver file operations */

extern const struct file_operations g_nxcon_drvrops;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
/* Semaphore helpers */

#ifdef CONFIG_DEBUG
int nxcon_semwait(FAR struct nxcon_state_s *priv);
int nxcon_sempost(FAR struct nxcon_state_s *priv);
#else
#  define nxcon_semwait(p) sem_wait(&p->exclsem)
#  define nxcon_sempost(p) sem_post(&p->exclsem)
#endif

/* Common device registration */

FAR struct nxcon_state_s *nxcon_register(NXCONSOLE handle,
    FAR struct nxcon_window_s *wndo, FAR const struct nxcon_operations_s *ops,
    int minor);

#ifdef CONFIG_NXCONSOLE_NXKBDIN
ssize_t nxcon_read(FAR struct file *filep, FAR char *buffer, size_t len);
#ifndef CONFIG_DISABLE_POLL
int nxcon_poll(FAR struct file *filep, FAR struct pollfd *fds, bool setup);
#endif
#endif

/* VT100 Terminal emulation */

enum nxcon_vt100state_e nxcon_vt100(FAR struct nxcon_state_s *priv, char ch);

/* Generic text display helpers */

void nxcon_home(FAR struct nxcon_state_s *priv);
void nxcon_newline(FAR struct nxcon_state_s *priv);
FAR const struct nxcon_bitmap_s *nxcon_addchar(NXHANDLE hfont,
    FAR struct nxcon_state_s *priv, uint8_t ch);
int nxcon_hidechar(FAR struct nxcon_state_s *priv,
    FAR const struct nxcon_bitmap_s *bm);
int nxcon_backspace(FAR struct nxcon_state_s *priv);
void nxcon_fillchar(FAR struct nxcon_state_s *priv,
    FAR const struct nxgl_rect_s *rect, FAR const struct nxcon_bitmap_s *bm);

void nxcon_putc(FAR struct nxcon_state_s *priv, uint8_t ch);
void nxcon_showcursor(FAR struct nxcon_state_s *priv);
void nxcon_hidecursor(FAR struct nxcon_state_s *priv);

/* Scrolling support */

void nxcon_scroll(FAR struct nxcon_state_s *priv, int scrollheight);

#endif /* __GRAPHICS_NXCONSOLE_NXCON_INTERNAL_H */
