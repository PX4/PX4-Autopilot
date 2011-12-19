/****************************************************************************
 * examples/nxtext/nxtext_bkgd.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxfonts.h>

#include "nxtext_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void nxbg_redraw(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                        bool morem, FAR void *arg);
static void nxbg_position(NXWINDOW hwnd, FAR const struct nxgl_size_s *size,
                          FAR const struct nxgl_point_s *pos,
                          FAR const struct nxgl_rect_s *bounds,
                          FAR void *arg);
#ifdef CONFIG_NX_MOUSE
static void nxbg_mousein(NXWINDOW hwnd, FAR const struct nxgl_point_s *pos,
                         uint8_t buttons, FAR void *arg);
#endif

#ifdef CONFIG_NX_KBD
static void nxbg_kbdin(NXWINDOW hwnd, uint8_t nch, FAR const uint8_t *ch,
                       FAR void *arg);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct nxtext_state_s  g_bgstate;
static struct nxtext_bitmap_s g_bgbm[CONFIG_EXAMPLES_NXTEXT_BMCACHE];
static struct nxtext_glyph_s  g_bgglyph[CONFIG_EXAMPLES_NXTEXT_GLCACHE];

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Background window call table */

const struct nx_callback_s g_bgcb =
{
  nxbg_redraw,   /* redraw */
  nxbg_position  /* position */
#ifdef CONFIG_NX_MOUSE
  , nxbg_mousein /* mousein */
#endif
#ifdef CONFIG_NX_KBD
  , nxbg_kbdin   /* my kbdin */
#endif
};

/* Background window handle */

NXHANDLE g_bgwnd;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbg_redrawrect
 ****************************************************************************/

static void nxbg_redrawrect(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect)
{
  int ret;
  int i;

  ret = nx_fill(hwnd, rect, g_bgstate.wcolor);
  if (ret < 0)
    {
      message("nxbg_redrawrect: nx_fill failed: %d\n", errno);
    }

  /* Fill each character on the display (Only the characters within rect
   * will actually be redrawn).
   */

  for (i = 0; i < g_bgstate.nchars; i++)
    {
      nxtext_fillchar(hwnd, rect, &g_bgstate, g_bghfont, &g_bgstate.bm[i]);
    }
}

/****************************************************************************
 * Name: nxbg_redraw
 ****************************************************************************/

static void nxbg_redraw(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                        bool more, FAR void *arg)
{
  gvdbg("hwnd=%p rect={(%d,%d),(%d,%d)} more=%s\n",
         hwnd, rect->pt1.x, rect->pt1.y, rect->pt2.x, rect->pt2.y,
         more ? "true" : "false");

  nxbg_redrawrect(hwnd, rect);
}

/****************************************************************************
 * Name: nxbg_position
 ****************************************************************************/

static void nxbg_position(NXWINDOW hwnd, FAR const struct nxgl_size_s *size,
                          FAR const struct nxgl_point_s *pos,
                          FAR const struct nxgl_rect_s *bounds,
                          FAR void *arg)
{
  FAR struct nxtext_state_s *st = (FAR struct nxtext_state_s *)arg;

  /* Report the position */

  gvdbg("hwnd=%p size=(%d,%d) pos=(%d,%d) bounds={(%d,%d),(%d,%d)}\n",
        hwnd, size->w, size->h, pos->x, pos->y,
        bounds->pt1.x, bounds->pt1.y, bounds->pt2.x, bounds->pt2.y);

  /* Have we picked off the window bounds yet? */

  if (!b_haveresolution)
    {
      /* Save the background window handle */

      g_bgwnd = hwnd;

      /* Save the background window size */

      st->wsize.w = size->w;
      st->wsize.h = size->h;

      /* Save the window limits (these should be the same for all places and all windows */

      g_xres = bounds->pt2.x + 1;
      g_yres = bounds->pt2.y + 1;

      b_haveresolution = true;
      sem_post(&g_semevent);
      gvdbg("Have xres=%d yres=%d\n", g_xres, g_yres);
    }
}

/****************************************************************************
 * Name: nxbg_mousein
 ****************************************************************************/

#ifdef CONFIG_NX_MOUSE
static void nxbg_mousein(NXWINDOW hwnd, FAR const struct nxgl_point_s *pos,
                         uint8_t buttons, FAR void *arg)
{
  message("nxbg_mousein: hwnd=%p pos=(%d,%d) button=%02x\n",
          hwnd,  pos->x, pos->y, buttons);
}
#endif

/****************************************************************************
 * Name: nxbg_kbdin
 ****************************************************************************/

#ifdef CONFIG_NX_KBD
static void nxbg_kbdin(NXWINDOW hwnd, uint8_t nch, FAR const uint8_t *ch,
                       FAR void *arg)
{
  gvdbg("hwnd=%p nch=%d\n", hwnd, nch);
  nxbg_write(hwnd, ch, nch);
}
#endif

/****************************************************************************
 * Name: nxbg_movedisplay
 *
 * Description:
 *   This function implements the data movement for the scroll operation.  If
 *   we can read the displays framebuffer memory, then the job is pretty
 *   easy.  However, many displays (such as SPI-based LCDs) are often read-
 *   only.
 ****************************************************************************/

#ifdef CONFIG_EXAMPLES_NXTEXT_NOGETRUN
static inline void nxbg_movedisplay(NXWINDOW hwnd, int bottom, int lineheight)
{
  FAR struct nxtext_bitmap_s *bm;
  struct nxgl_rect_s rect;
  nxgl_coord_t row;
  int ret;
  int i;

  /* Move each row, one at a time.  They could all be moved at once (by calling
   * nxbg_redrawrect), but the since the region is cleared, then re-written, the
   * effect would not be good.  Below the region is also cleared and re-written,
   * however, in much smaller chunks.
   */

  rect.pt1.x = 0;
  rect.pt2.x = g_bgstate.wsize.w - 1;

  for (row = LINE_SEPARATION; row < bottom; row += lineheight)
    {
      /* Create a bounding box the size of one row of characters */

      rect.pt1.y = row;
      rect.pt2.y = row + lineheight - 1;

      /* Clear the region */

      ret = nx_fill(hwnd, &rect, g_bgstate.wcolor);
      if (ret < 0)
        {
          message("nxbg_movedisplay: nx_fill failed: %d\n", errno);
        }

      /* Fill each character that might lie within in the bounding box */

      for (i = 0; i < g_bgstate.nchars; i++)
        {
          bm = &g_bgstate.bm[i];
          if (bm->pos.y <= rect.pt2.y && bm->pos.y + g_bgstate.fheight >= rect.pt1.y)
            {
              nxtext_fillchar(hwnd, &rect, &g_bgstate, g_bghfont, bm);
            }
        }
    }

  /* Finally, clear the bottom part of the display */

  rect.pt1.y = bottom;
  rect.pt2.y = g_bgstate.wsize.h- 1;

  ret = nx_fill(hwnd, &rect, g_bgstate.wcolor);
  if (ret < 0)
    {
      message("nxbg_movedisplay: nx_fill failed: %d\n", errno);
    }
}
#else
static inline void nxbg_movedisplay(NXWINDOW hwnd, int bottom, int lineheight)
{
  struct nxgl_rect_s rect;
  struct nxgl_point_s offset;
  int ret;

  /* Move the display in the range of 0-height up one lineheight.  The
   * line at the bottom will be reset to the background color automatically.
   *
   * The source rectangle to be moved.
   */

  rect.pt1.x = 0;
  rect.pt1.y = lineheight + LINE_SEPARATION;
  rect.pt2.x = g_bgstate.wsize.w - 1;
  rect.pt2.y = g_bgstate.wsize.h - 1;

  /* The offset that determines how far to move the source rectangle */

  offset.x   = 0;
  offset.y   = -lineheight;

  /* Move the source rectangle */

  ret = nx_move(hwnd, &rect, &offset);
  if (ret < 0)
    {
      message("nxbg_movedisplay: nx_move failed: %d\n", errno);
    }
}
#endif

/****************************************************************************
 * Name: nxbg_scroll
 ****************************************************************************/

static inline void nxbg_scroll(NXWINDOW hwnd, int lineheight)
{
  int i;
  int j;

  /* Adjust the vertical position of each character */

  for (i = 0; i < g_bgstate.nchars; )
    {
      FAR struct nxtext_bitmap_s *bm = &g_bgstate.bm[i];

      /* Has any part of this character scrolled off the screen? */

      if (bm->pos.y < lineheight + LINE_SEPARATION)
        {
          /* Yes... Delete the character by moving all of the data */

          for (j = i; j < g_bgstate.nchars-1; j++)
            {
              memcpy(&g_bgstate.bm[j], &g_bgstate.bm[j+1], sizeof(struct nxtext_bitmap_s));
            }

          /* Decrement the number of cached characters ('i' is not incremented
           * in this case because it already points to the next character)
           */

          g_bgstate.nchars--;
        }

      /* No.. just decrement its vertical position (moving it "up" the
       * display by one line).
       */

      else
        {
          bm->pos.y -= lineheight;

          /* We are keeping this one so increment to the next character */
 
          i++;
        }
    }

  /* And move the next display position up by one line as well */

  g_bgstate.fpos.y -= lineheight;

  /* Move the display in the range of 0-height up one lineheight. */

  nxbg_movedisplay(hwnd, g_bgstate.fpos.y, lineheight);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbg_getstate
 *
 * Description:
 *   Initialize the background window state structure.
 *
 ****************************************************************************/

FAR struct nxtext_state_s *nxbg_getstate(void)
{
  FAR const struct nx_font_s *fontset;

  /* Initialize the color (used for redrawing the window) */

  memset(&g_bgstate, 0, sizeof(struct nxtext_state_s));
  g_bgstate.wcolor[0] = CONFIG_EXAMPLES_NXTEXT_BGCOLOR;
  g_bgstate.fcolor[0] = CONFIG_EXAMPLES_NXTEXT_BGFONTCOLOR;

  /* Get information about the font set being used and save this in the
   * state structure
   */

  fontset             = nxf_getfontset(g_bghfont);
  g_bgstate.fheight   = fontset->mxheight;
  g_bgstate.fwidth    = fontset->mxwidth;
  g_bgstate.spwidth   = fontset->spwidth;

  /* Set up the text caches */

  g_bgstate.maxchars  = CONFIG_EXAMPLES_NXTEXT_BMCACHE;
  g_bgstate.maxglyphs = CONFIG_EXAMPLES_NXTEXT_GLCACHE;
  g_bgstate.bm        = g_bgbm;
  g_bgstate.glyph     = g_bgglyph;

  /* Set the first display position */

  nxtext_home(&g_bgstate);
  return &g_bgstate;
}

/****************************************************************************
 * Name: nxbg_write
 *
 * Description:
 *   Put a sequence of bytes in the window.
 *
 ****************************************************************************/

void nxbg_write(NXWINDOW hwnd, FAR const uint8_t *buffer, size_t buflen)
{
  int lineheight = (g_bgstate.fheight + LINE_SEPARATION);

  while (buflen-- > 0)
    {
      /* Will another character fit on this line? */

      if (g_bgstate.fpos.x + g_bgstate.fwidth > g_bgstate.wsize.w)
        {
          /* No.. move to the next line */

          nxtext_newline(&g_bgstate);

          /* If we were about to output a newline character, then don't */

          if (*buffer == '\n')
            {
              buffer++;
              continue;
            }
        }

      /* Check if we need to scroll up (handling a corner case where
       * there may be more than one newline).
       */

      while (g_bgstate.fpos.y >= g_bgstate.wsize.h - lineheight)
        {
          nxbg_scroll(hwnd, lineheight);
        }

      /* Finally, we can output the character */

      nxtext_putc(hwnd, &g_bgstate, g_bghfont, (uint8_t)*buffer++);
    }
}
