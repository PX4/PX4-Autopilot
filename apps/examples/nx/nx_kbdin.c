/****************************************************************************
 * examples/nx/nx_kbdin.c
 *
 *   Copyright (C) 2008, 2010-2011 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxtk.h>
#include <nuttx/nx/nxfonts.h>

#include "nx_internal.h"

#ifdef CONFIG_NX_KBD

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Select renderer -- Some additional logic would be required to support
 * pixel depths that are not directly addressable (1,2,4, and 24).
 */

#if CONFIG_EXAMPLES_NX_BPP == 1
#  define RENDERER nxf_convert_1bpp
#elif CONFIG_EXAMPLES_NX_BPP == 2
#  define RENDERER nxf_convert_2bpp
#elif CONFIG_EXAMPLES_NX_BPP == 4
#  define RENDERER nxf_convert_4bpp
#elif CONFIG_EXAMPLES_NX_BPP == 8
#  define RENDERER nxf_convert_8bpp
#elif CONFIG_EXAMPLES_NX_BPP == 16
#  define RENDERER nxf_convert_16bpp
#elif CONFIG_EXAMPLES_NX_BPP == 24
#  define RENDERER nxf_convert_24bpp
#elif  CONFIG_EXAMPLES_NX_BPP == 32
#  define RENDERER nxf_convert_32bpp
#else
#  error "Unsupported CONFIG_EXAMPLES_NX_BPP"
#endif

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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxeg_fillchar
 ****************************************************************************/

static void nxeg_fillchar(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                          FAR const struct nxeg_bitmap_s *bm)
{
  FAR void *src = (FAR void *)bm->glyph->bitmap;
  struct nxgl_rect_s intersection;
  int ret;

  /* Handle the special case of spaces which have no glyph bitmap */

  if (src)
    {
      /* Get the intersection of the redraw region and the character bitmap */

      nxgl_rectintersect(&intersection, rect, &bm->bounds);
      if (!nxgl_nullrect(&intersection))
        {
#ifndef CONFIG_EXAMPLES_NX_RAWWINDOWS
          ret = nxtk_bitmapwindow((NXTKWINDOW)hwnd, &intersection, (FAR const void **)&src,
                                  &bm->bounds.pt1,
                                  (unsigned int)bm->glyph->stride);
          if (ret < 0)
            {
              message("nxeg_fillchar: nxtk_bitmapwindow failed: %d\n", errno);
            }
#else
          ret = nx_bitmap((NXWINDOW)hwnd, &intersection, &src,
                          &bm->bounds.pt1,
                          (unsigned int)bm->glyph->stride);
          if (ret < 0)
            {
              message("nxeg_fillchar: nx_bitmapwindow failed: %d\n", errno);
            }
#endif
        }
    }
}

/****************************************************************************
 * Name: nxeg_renderglyph
 ****************************************************************************/

static inline FAR const struct nxeg_glyph_s *
nxeg_renderglyph(FAR struct nxeg_state_s *st,
                 FAR const struct nx_fontbitmap_s *bm, uint8_t ch)
{
  FAR struct nxeg_glyph_s *glyph = NULL;
  FAR nxgl_mxpixel_t *ptr;
#if CONFIG_EXAMPLES_NX_BPP < 8
  nxgl_mxpixel_t pixel;
#endif
  int bmsize;
  int row;
  int col;
  int ret;

  /* Make sure that there is room for another glyph */

  message("nxeg_renderglyph: ch=%02x\n", ch);
  if (st->nglyphs < NXTK_MAXKBDCHARS)
    {
      /* Allocate the glyph */

      glyph         = &st->glyph[st->nglyphs];
      glyph->code   = ch;

      /* Get the dimensions of the glyph */

      glyph->width  = bm->metric.width + bm->metric.xoffset;
      glyph->height = bm->metric.height + bm->metric.yoffset;

      /* Allocate memory to hold the glyph with its offsets */

      glyph->stride = (glyph->width * CONFIG_EXAMPLES_NX_BPP + 7) / 8;
      bmsize        =  glyph->stride * glyph->height;
      glyph->bitmap = (FAR uint8_t *)malloc(bmsize);

      if (glyph->bitmap)
        {
          /* Initialize the glyph memory to the background color */

#if CONFIG_EXAMPLES_NX_BPP < 8
          pixel  = st->color[0];
#  if CONFIG_EXAMPLES_NX_BPP == 1
          /* Pack 1-bit pixels into a 2-bits */

          pixel &= 0x01;
          pixel  = (pixel) << 1 |pixel;
#  endif
#  if CONFIG_EXAMPLES_NX_BPP < 4
          /* Pack 2-bit pixels into a nibble */

          pixel &= 0x03;
          pixel  = (pixel) << 2 |pixel;
#  endif

          /* Pack 4-bit nibbles into a byte */

          pixel &= 0x0f;
          pixel  = (pixel) << 4 | pixel;

          ptr    = (FAR nxgl_mxpixel_t *)glyph->bitmap;
          for (row = 0; row < glyph->height; row++)
            {
              for (col = 0; col < glyph->stride; col++)
                {
                  /* Transfer the packed bytes into the buffer */

                  *ptr++ = pixel;
                }
            }

#elif CONFIG_EXAMPLES_NX_BPP == 24
# error "Additional logic is needed here for 24bpp support"

#else /* CONFIG_EXAMPLES_NX_BPP = {8,16,32} */

          ptr = (FAR nxgl_mxpixel_t *)glyph->bitmap;
          for (row = 0; row < glyph->height; row++)
            {
              /* Just copy the color value into the glyph memory */

              for (col = 0; col < glyph->width; col++)
                {
                  *ptr++ = st->color[0];
                }
            }
#endif

          /* Then render the glyph into the allocated memory */

          ret = RENDERER((FAR nxgl_mxpixel_t*)glyph->bitmap,
                          glyph->height, glyph->width, glyph->stride,
                          bm, CONFIG_EXAMPLES_NX_FONTCOLOR);
          if (ret < 0)
            {
              /* Actually, the RENDERER never returns a failure */

              message("nxeg_renderglyph: RENDERER failed\n");
              free(glyph->bitmap);
              glyph->bitmap = NULL;
              glyph         = NULL;
            }
          else
            {
               /* Make it permanent */

               st->nglyphs++;
            }
        }
    }

  return glyph;
}

/****************************************************************************
 * Name: nxeg_addspace
 ****************************************************************************/

static inline FAR const struct nxeg_glyph_s *
nxeg_addspace(FAR struct nxeg_state_s *st, uint8_t ch)
{
  FAR struct nxeg_glyph_s *glyph = NULL;

  /* Make sure that there is room for another glyph */

  if (st->nglyphs < NXTK_MAXKBDCHARS)
    {
      /* Allocate the NULL glyph */

      glyph        = &st->glyph[st->nglyphs];
      memset(glyph, 0, sizeof(struct nxeg_glyph_s));

      glyph->code  = ' ';
      glyph->width = st->spwidth;

      st->nglyphs++;
    }
  return glyph;
}

/****************************************************************************
 * Name: nxeg_findglyph
 ****************************************************************************/

static FAR const struct nxeg_glyph_s *
nxeg_findglyph(FAR struct nxeg_state_s *st, uint8_t ch)
{
  int i;

  /* First, try to find the glyph in the cache of pre-rendered glyphs */

   for (i = 0; i < st->nglyphs; i++)
    {
      if (st->glyph[i].code == ch)
        {
          return &st->glyph[i];
        }
    }
  return NULL;
}

/****************************************************************************
 * Name: nxeg_getglyph
 ****************************************************************************/

static FAR const struct nxeg_glyph_s *
nxeg_getglyph(FAR struct nxeg_state_s *st, uint8_t ch)
{
  FAR const struct nxeg_glyph_s *glyph;
  FAR const struct nx_fontbitmap_s *bm;

  /* First, try to find the glyph in the cache of pre-rendered glyphs */

  glyph = nxeg_findglyph(st, ch);
  if (!glyph)
    {
      /* No, it is not cached... Does the code map to a glyph? */

      bm = nxf_getbitmap(g_fonthandle, ch);
      if (!bm)
        {
          /* No, there is no glyph for this code.  Use space */

          glyph = nxeg_findglyph(st, ' ');
          if (!glyph)
            {
              /* There isn't fake glyph for ' ' yet... create one */

              glyph = nxeg_addspace(st, ' ');
            }
        }
      else
        {
          glyph =  nxeg_renderglyph(st, bm, ch);
        }
    }
  return glyph;
}

/****************************************************************************
 * Name: nxeg_addchar
 ****************************************************************************/

static FAR const struct nxeg_bitmap_s *
nxeg_addchar(FAR struct nxeg_state_s *st, uint8_t ch)
{
  FAR struct nxeg_bitmap_s *bm = NULL;
  FAR struct nxeg_bitmap_s *bmleft;
  nxgl_coord_t leftx;

  /* Is there space for another character on the display? */

  if (st->nchars < NXTK_MAXKBDCHARS)
    {
       /* Yes, setup the bitmap */

       bm = &st->bm[st->nchars];

       /* Find the matching glyph */

       bm->glyph = nxeg_getglyph(st, ch);
       if (!bm->glyph)
         {
           return NULL;
         }

       /* Set up the bounds for the bitmap */

       if (st->nchars <= 0)
         {
            /* The first character is one space from the left */

            leftx  = st->spwidth;
         }
       else
         {
            /* Otherwise, it is to the left of the preceding char */

            bmleft = &st->bm[st->nchars-1];
            leftx  = bmleft->bounds.pt2.x + 1;
         }

       bm->bounds.pt1.x = leftx;
       bm->bounds.pt1.y = 2;
       bm->bounds.pt2.x = leftx + bm->glyph->width - 1;
       bm->bounds.pt2.y = 2 + bm->glyph->height - 1;

       st->nchars++;
    }
  return bm;
}

/****************************************************************************
 * Name: nxeg_addchars
 ****************************************************************************/

static inline void nxeg_addchars(NXWINDOW hwnd, FAR struct nxeg_state_s *st,
                                 uint8_t nch, FAR const uint8_t *ch)
{
  FAR const struct nxeg_bitmap_s *bm;

  while (nch--)
    {
      bm = nxeg_addchar(st, *ch++);
      if (bm)
        {
          nxeg_fillchar(hwnd, &bm->bounds, bm);
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxeg_kbdin
 ****************************************************************************/

void nxeg_kbdin(NXWINDOW hwnd, uint8_t nch, FAR const uint8_t *ch,
                FAR void *arg)
{
  FAR struct nxeg_state_s *st = (FAR struct nxeg_state_s *)arg;
  message("nxeg_kbdin%d: hwnd=%p nch=%d\n", st->wnum, hwnd, nch);
  nxeg_addchars(hwnd, st, nch, ch);
}

/****************************************************************************
 * Name: nxeg_tbkbdin
 ****************************************************************************/

#ifndef CONFIG_EXAMPLES_NX_RAWWINDOWS
void nxeg_tbkbdin(NXWINDOW hwnd, uint8_t nch, const uint8_t *ch, FAR void *arg)
{
  FAR struct nxeg_state_s *st = (FAR struct nxeg_state_s *)arg;
  message("nxeg_tbkbdin: ERROR -- toolbar should not received keyboard input\n");
  message("nxeg_tbkbdin%d: hwnd=%p nch=%d\n", st->wnum, hwnd, nch);
}
#endif

/****************************************************************************
 * Name: nxeg_filltext
 ****************************************************************************/

void nxeg_filltext(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                   FAR struct nxeg_state_s *st)
{
  int i;

  /* Fill each character on the display (Only the characters within rect
   * will actually be redrawn).
   */

  for (i = 0; i < st->nchars; i++)
    {
      nxeg_fillchar(hwnd, rect, &st->bm[i]);
    }
}

#endif /* CONFIG_NX_KBD */
