/****************************************************************************
 * nuttx/graphics/nxconsole/nxcon_font.c
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

#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>

#include "nxcon_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Select renderer -- Some additional logic would be required to support
 * pixel depths that are not directly addressable (1,2,4, and 24).
 */

#if CONFIG_NXCONSOLE_BPP == 1
#  define RENDERER nxf_convert_1bpp
#elif CONFIG_NXCONSOLE_BPP == 2
#  define RENDERER nxf_convert_2bpp
#elif CONFIG_NXCONSOLE_BPP == 4
#  define RENDERER nxf_convert_4bpp
#elif CONFIG_NXCONSOLE_BPP == 8
#  define RENDERER nxf_convert_8bpp
#elif CONFIG_NXCONSOLE_BPP == 16
#  define RENDERER nxf_convert_16bpp
#elif CONFIG_NXCONSOLE_BPP == 24
#  define RENDERER nxf_convert_24bpp
#elif  CONFIG_NXCONSOLE_BPP == 32
#  define RENDERER nxf_convert_32bpp
#else
#  error "Unsupported CONFIG_NXCONSOLE_BPP"
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
 * Name: nxcon_freeglyph
 ****************************************************************************/

static void nxcon_freeglyph(FAR struct nxcon_glyph_s *glyph)
{
  if (glyph->bitmap)
    {
      kfree(glyph->bitmap);
    }
  memset(glyph, 0, sizeof(struct nxcon_glyph_s));
}

/****************************************************************************
 * Name: nxcon_allocglyph
 ****************************************************************************/

static inline FAR struct nxcon_glyph_s *
nxcon_allocglyph(FAR struct nxcon_state_s *priv)
{
  FAR struct nxcon_glyph_s *glyph = NULL;
  FAR struct nxcon_glyph_s *luglyph = NULL;
  uint8_t luusecnt;
  int i;

  /* Search through the glyph cache looking for an unused glyph.  Also, keep
   * track of the least used glyph as well.  We need that if we have to replace
   * a glyph in the cache.
   */

   for (i = 0; i < priv->maxglyphs; i++)
    {
      /* Is this glyph in use? */

      glyph = &priv->glyph[i];
      if (!glyph->usecnt)
        {
          /* No.. return this glyph with a use count of one */

          glyph->usecnt = 1;
          return glyph;
        }

      /* Yes.. check for the least recently used */

      if (!luglyph || glyph->usecnt < luglyph->usecnt)
        {
          luglyph = glyph;
        }
    }

  /* If we get here, the glyph cache is full.  We replace the least used
   * glyph with the one we need now. (luglyph can't be NULL).
   */
   
  luusecnt = luglyph->usecnt;
  nxcon_freeglyph(luglyph);

  /* But lets decrement all of the usecnts so that the new one one be so
   * far behind in the counts as the older ones.
   */

  if (luusecnt > 1)
    {
       uint8_t decr = luusecnt - 1;
 
       for (i = 0; i < priv->maxglyphs; i++)
        {
          /* Is this glyph in use? */

          glyph = &priv->glyph[i];
          if (glyph->usecnt > decr)
            {
              glyph->usecnt -= decr;
            }
        }
    }

  /* Then return the least used glyph */

  luglyph->usecnt = 1;
  return luglyph;
}

/****************************************************************************
 * Name: nxcon_findglyph
 ****************************************************************************/

static FAR struct nxcon_glyph_s *
nxcon_findglyph(FAR struct nxcon_state_s *priv, uint8_t ch)
{
  int i;

  /* First, try to find the glyph in the cache of pre-rendered glyphs */

   for (i = 0; i < priv->maxglyphs; i++)
    {
      FAR struct nxcon_glyph_s *glyph = &priv->glyph[i];
      if (glyph->usecnt > 0 && glyph->code == ch)
        {
          /* Increment the use count (unless it is already at the max) */

          if (glyph->usecnt < MAX_USECNT)
            {
               glyph->usecnt++;
            }

          /* And return the glyph that we found */

          return glyph;
        }
    }
  return NULL;
}

/****************************************************************************
 * Name: nxcon_renderglyph
 ****************************************************************************/

static inline FAR struct nxcon_glyph_s *
nxcon_renderglyph(FAR struct nxcon_state_s *priv,
                   FAR const struct nx_fontbitmap_s *fbm, uint8_t ch)
{
  FAR struct nxcon_glyph_s *glyph = NULL;
  FAR nxgl_mxpixel_t *ptr;
#if CONFIG_NXCONSOLE_BPP < 8
  nxgl_mxpixel_t pixel;
#endif
  int bmsize;
  int row;
  int col;
  int ret;

  /* Allocate the glyph (always succeeds) */

  glyph         = nxcon_allocglyph(priv);
  glyph->code   = ch;

  /* Get the dimensions of the glyph */

  glyph->width  = fbm->metric.width + fbm->metric.xoffset;
  glyph->height = fbm->metric.height + fbm->metric.yoffset;

  /* Get the physical width of the glyph in bytes */

  glyph->stride = (glyph->width * CONFIG_NXCONSOLE_BPP + 7) / 8;

  /* Allocate memory to hold the glyph with its offsets */

  bmsize        =  glyph->stride * glyph->height;
  glyph->bitmap = (FAR uint8_t *)kmalloc(bmsize);

  if (glyph->bitmap)
    {
      /* Initialize the glyph memory to the background color using the
       * hard-coded bits-per-pixel (BPP).
       *
       * TODO:  The rest of NX is configured to support multiple devices
       * with differing BPP.  They logic should be extended to support
       * differing BPP's as well.
       */

#if CONFIG_NXCONSOLE_BPP < 8
      pixel  = priv->wcolor[0];

#  if CONFIG_NXCONSOLE_BPP == 1

      /* Pack 1-bit pixels into a 2-bits */

      pixel &= 0x01;
      pixel  = (pixel) << 1 | pixel;

#  endif
#  if CONFIG_NXCONSOLE_BPP < 4

      /* Pack 2-bit pixels into a nibble */

      pixel &= 0x03;
      pixel  = (pixel) << 2 | pixel;

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

#elif CONFIG_NXCONSOLE_BPP == 24
# error "Additional logic is needed here for 24bpp support"

#else /* CONFIG_NXCONSOLE_BPP = {8,16,32} */

      ptr = (FAR nxgl_mxpixel_t *)glyph->bitmap;
      for (row = 0; row < glyph->height; row++)
        {
          /* Just copy the color value into the glyph memory */

          for (col = 0; col < glyph->width; col++)
            {
              *ptr++ = priv->wndo.wcolor[0];
            }
        }
#endif

      /* Then render the glyph into the allocated memory */

      ret = RENDERER((FAR nxgl_mxpixel_t*)glyph->bitmap,
                      glyph->height, glyph->width, glyph->stride,
                      fbm, priv->wndo.fcolor[0]);
      if (ret < 0)
        {
          /* Actually, the RENDERER never returns a failure */

          gdbg("nxcon_renderglyph: RENDERER failed\n");
          nxcon_freeglyph(glyph);
          glyph = NULL;
        }
    }

  return glyph;
}

/****************************************************************************
 * Name: nxcon_fontsize
 ****************************************************************************/

static int nxcon_fontsize(NXHANDLE hfont, uint8_t ch, FAR struct nxgl_size_s *size)
{
  FAR const struct nx_fontbitmap_s *fbm;

  /* No, it is not cached... Does the code map to a font? */

  fbm = nxf_getbitmap(hfont, ch);
  if (fbm)
    {
      /* Yes.. return the font size */

      size->w = fbm->metric.width + fbm->metric.xoffset;
      size->h = fbm->metric.height + fbm->metric.yoffset;
      return OK;
    }

  return ERROR;
}

/****************************************************************************
 * Name: nxcon_getglyph
 ****************************************************************************/

static FAR struct nxcon_glyph_s *
nxcon_getglyph(NXHANDLE hfont, FAR struct nxcon_state_s *priv, uint8_t ch)
{
  FAR struct nxcon_glyph_s *glyph;
  FAR const struct nx_fontbitmap_s *fbm;

  /* First, try to find the glyph in the cache of pre-rendered glyphs */

  glyph = nxcon_findglyph(priv, ch);
  if (glyph)
    {
      /* We found it in the cache .. return the cached glyph */

      return glyph;
    }

  /* No, it is not cached... Does the code map to a font? */

  fbm = nxf_getbitmap(hfont, ch);
  if (fbm)
    {
      /* Yes.. render the glyph */

      glyph = nxcon_renderglyph(priv, fbm, ch);
    }

  return glyph;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxcon_addchar
 *
 * Description:
 *   This is part of the nxcon_putc logic.  It creates and positions a
 *   the character and renders (or re-uses) a glyph for font.
 *
 ****************************************************************************/

FAR const struct nxcon_bitmap_s *
nxcon_addchar(NXHANDLE hfont, FAR struct nxcon_state_s *priv, uint8_t ch)
{
  FAR struct nxcon_bitmap_s *bm = NULL;
  FAR struct nxcon_glyph_s *glyph;

  /* Is there space for another character on the display? */

  if (priv->nchars < priv->maxchars)
    {
      /* Yes, setup the bitmap information */

      bm        = &priv->bm[priv->nchars];
      bm->code  = ch;
      bm->flags = 0;
      bm->pos.x = priv->fpos.x;
      bm->pos.y = priv->fpos.y;

      /* Find (or create) the matching glyph */

      glyph = nxcon_getglyph(hfont, priv, ch);
      if (!glyph)
        {
          /* No, there is no font for this code.  Just mark this as a space. */

          bm->flags |= BMFLAGS_NOGLYPH;

          /* Set up the next character position */

          priv->fpos.x += priv->spwidth;
        }
      else
        {
          /* Set up the next character position */

          priv->fpos.x += glyph->width;
        }

      /* Success.. increment nchars to retain this character */

      priv->nchars++;
    }

  return bm;
}

/****************************************************************************
 * Name: nxcon_hidechar
 *
 * Description:
 *   Erase a character from the window.
 *
 ****************************************************************************/
int nxcon_hidechar(FAR struct nxcon_state_s *priv,
                   FAR const struct nxcon_bitmap_s *bm)
{
  struct nxgl_rect_s bounds;
  struct nxgl_size_s fsize;
  int ret;

  /* Get the size of the font glyph.  If nxcon_fontsize, then the
   * character will have been rendered as a space, and no display
   * modification is required (not an error).
   */

  ret = nxcon_fontsize(priv->font, bm->code, &fsize);
  if (ret < 0)
    {
      /* It was rendered as a space. */

      return OK;
    }

  /* Construct a bounding box for the glyph */

  bounds.pt1.x = bm->pos.x;
  bounds.pt1.y = bm->pos.y;
  bounds.pt2.x = bm->pos.x + fsize.w - 1;
  bounds.pt2.y = bm->pos.y + fsize.h - 1;

  /* Fill the bitmap region with the background color, erasing the
   * character from the display.  NOTE:  This region might actually
   * be obscured... NX will handle that case.
   */

  return priv->ops->fill(priv, &bounds, priv->wndo.wcolor);
}

/****************************************************************************
 * Name: nxcon_backspace
 *
 * Description:
 *   Remove the last character from the window.
 *
 ****************************************************************************/

int nxcon_backspace(FAR struct nxcon_state_s *priv)
{
  FAR struct nxcon_bitmap_s *bm;
  int ndx;
  int ret = -ENOENT;

  /* Is there a character on the display? */

  if (priv->nchars > 0)
    {
      /* Yes.. Get the index to the last bitmap on the display */

      ndx = priv->nchars - 1;
      bm  = &priv->bm[ndx];

      /* Erase the character from the display */

      ret = nxcon_hidechar(priv, bm);

      /* The current position to the location where the last character was */
    
      priv->fpos.x = bm->pos.x;
      priv->fpos.y = bm->pos.y;

      /* Decrement nchars to discard this character */

      priv->nchars = ndx;
    }

  return ret;
}

/****************************************************************************
 * Name: nxcon_home
 *
 * Description:
 *   Set the next character position to the top-left corner of the display.
 *
 ****************************************************************************/

void nxcon_home(FAR struct nxcon_state_s *priv)
{
  /* The first character is one space from the left */

  priv->fpos.x = priv->spwidth;

  /* And CONFIG_NXCONSOLE_LINESEPARATION lines from the top */

  priv->fpos.y = CONFIG_NXCONSOLE_LINESEPARATION;
}

/****************************************************************************
 * Name: nxcon_newline
 *
 * Description:
 *   Set the next character position to the beginning of the next line.
 *
 ****************************************************************************/

void nxcon_newline(FAR struct nxcon_state_s *priv)
{
  /* Carriage return: The first character is one space from the left */

  priv->fpos.x = priv->spwidth;

  /* Linefeed: Down the max font height + CONFIG_NXCONSOLE_LINESEPARATION */

  priv->fpos.y += (priv->fheight + CONFIG_NXCONSOLE_LINESEPARATION);
}

/****************************************************************************
 * Name: nxcon_fillchar
 *
 * Description:
 *   This implements the character display.  It is part of the nxcon_putc
 *   operation but may also be used when redrawing an existing display.
 *
 ****************************************************************************/

void nxcon_fillchar(FAR struct nxcon_state_s *priv,
                    FAR const struct nxgl_rect_s *rect,
                    FAR const struct nxcon_bitmap_s *bm)
{
  FAR struct nxcon_glyph_s *glyph;
  struct nxgl_rect_s bounds;
  struct nxgl_rect_s intersection;
  struct nxgl_size_s fsize;
  int ret;

  /* Handle the special case of spaces which have no glyph bitmap */

  if (BM_ISSPACE(bm))
    {
      return;
    }

  /* Get the size of the font glyph (which may not have been created yet) */

  ret = nxcon_fontsize(priv->font, bm->code, &fsize);
  if (ret < 0)
    {
      /* This would mean that there is no bitmap for the character code and
       * that the font would be rendered as a space.  But this case should
       * never happen here because the BM_ISSPACE() should have already
       * found all such cases.
       */

      return;
    }

  /* Construct a bounding box for the glyph */

  bounds.pt1.x = bm->pos.x;
  bounds.pt1.y = bm->pos.y;
  bounds.pt2.x = bm->pos.x + fsize.w - 1;
  bounds.pt2.y = bm->pos.y + fsize.h - 1;

  /* Should this also be clipped to a region in the window? */

  if (rect)
    {
      /* Get the intersection of the redraw region and the character bitmap */

      nxgl_rectintersect(&intersection, rect, &bounds);
    }
  else
    {
      /* The intersection is the whole glyph */

      nxgl_rectcopy(&intersection, &bounds);
    }

  /* Check for empty intersections */

  if (!nxgl_nullrect(&intersection))
    {
      FAR const void *src;

      /* Find (or create) the glyph that goes with this font */

      glyph = nxcon_getglyph(priv->font, priv, bm->code);
      if (!glyph)
        {
          /* Shouldn't happen */

          return;
        }

      /* Blit the font bitmap into the window */

      src = (FAR const void *)glyph->bitmap;
      ret = priv->ops->bitmap(priv, &intersection, &src,
                              &bm->pos, (unsigned int)glyph->stride);
      DEBUGASSERT(ret >= 0);
    }
}

