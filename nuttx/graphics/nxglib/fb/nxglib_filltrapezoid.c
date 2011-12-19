/****************************************************************************
 * graphics/nxglib/fb/nxglib_filltrapezoid.c
 *
 *   Copyright (C) 2008-2011 Gregory Nutt. All rights reserved.
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
#include <fixedmath.h>

#include <nuttx/fb.h>
#include <nuttx/nx/nxglib.h>

#include "nxglib_bitblit.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#ifndef NXGLIB_SUFFIX
#  error "NXGLIB_SUFFIX must be defined before including this header file"
#endif

/****************************************************************************
 * Private Types
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxglib_filltrapezoid_*bpp
 *
 * Descripton:
 *   Fill a trapezoidal region in the framebuffer memory with a fixed color.
 *   Clip the trapezoid to lie within a boundng box.  This is useful for
 *   drawing complex shapes that can be broken into a set of trapezoids.
 *
 ****************************************************************************/

void NXGL_FUNCNAME(nxgl_filltrapezoid,NXGLIB_SUFFIX)(
  FAR struct fb_planeinfo_s *pinfo,
  FAR const struct nxgl_trapezoid_s *trap,
  FAR const struct nxgl_rect_s *bounds,
  NXGL_PIXEL_T color)
{
  unsigned int stride;
  unsigned int width;
  FAR uint8_t *line;
  int nrows;
  b16_t x1;
  b16_t x2;
  nxgl_coord_t y1;
  nxgl_coord_t y2;
  b16_t dx1dy;
  b16_t dx2dy;

#if NXGLIB_BITSPERPIXEL < 8
  FAR uint8_t *dest;
  uint8_t mpixel = NXGL_MULTIPIXEL(color);
  uint8_t mask;
  int lnlen;
#endif

  /* Get the width of the framebuffer in bytes */

  stride = pinfo->stride;

  /* Get the top run position and the number of rows to draw */

  x1    = trap->top.x1;
  x2    = trap->top.x2;

  /* Calculate the number of rows to render */

  y1    = trap->top.y;
  y2    = trap->bot.y;
  nrows = y2 - y1 + 1;

  /* Calculate the slope of the left and right side of the trapezoid */

  dx1dy = b16divi((trap->bot.x1 - x1), nrows - 1);
  dx2dy = b16divi((trap->bot.x2 - x2), nrows - 1);

  /* Perform vertical clipping */

  if (y1 < bounds->pt1.y)
    {
      /* Is the entire trapezoid "above" the clipping window? */

      if (y2 < bounds->pt1.y)
        {
          /* Yes.. then do nothing */

          return;
        }

      /* Calculate the x values for the new top run */

      int dy = bounds->pt1.y - y1;
      x1    += dy * dx1dy;
      x2    += dy * dx2dy;

      /* Clip and re-calculate the number of rows to render */

      y1     = bounds->pt1.y;
      nrows  = y2 - y1 + 1;
    }

  if (y2 > bounds->pt2.y)
    {
      /* Is the entire trapezoid "below" the clipping window? */

      if (y1 > bounds->pt2.y)
        {
          /* Yes.. then do nothing */

          return;
        }

      /* Clip and re-calculate the number of rows to render */

      y2     = bounds->pt2.y;
      nrows  = y2 - y1 + 1;
    }

  /* Get the address of the first byte on the first line */

  line = pinfo->fbmem + y1 * stride ;

  /* Then fill the trapezoid line-by-line */

  while (nrows--)
    {
      int ix1;
      int ix2;

      /* Handle the special case where the sides cross (as in an hourglass) */

      if (x1 > x2)
        {
          b16_t tmp;
          ngl_swap(x1, x2, tmp);
          ngl_swap(dx1dy, dx2dy, tmp);
        }

      /* Convert the positions to integer and get the run width */

      ix1   = b16toi(x1);
      ix2   = b16toi(x2);
      width = ix2 - ix1 + 1;

      /* Handle some corner cases where we draw nothing.  Otherwise, we will
       * always draw at least one pixel.
       */

      if (x1 <= x2 && ix2 >= bounds->pt1.x && ix1 <= bounds->pt2.x)
        {
          /* Get a clipped copies of the starting and ending X positions.  This
           * clipped truncates "down" and gives the quantized pixel holding the
           * fractional X position
           */

          ix1 = ngl_clipl(ix1, bounds->pt1.x);
          ix2 = ngl_clipr(ix2, bounds->pt2.x);

#if NXGLIB_BITSPERPIXEL < 8
          /* Handle masking of the fractional initial byte */

#ifdef CONFIG_NX_PACKEDMSFIRST
          mask  = (uint8_t)(0xff >> (8 - NXGL_REMAINDERX(ix1));
#else
          mask  = (uint8_t)(0xff << (8 - NXGL_REMAINDERX(ix1)));
#endif
          dest  = line;
          lnlen = width;

          if (lnlen > 1 && mask)
            {
              dest[0] = (dest[0] & ~mask) | (mpixel & mask);
              mask = 0xff;
              dest++;
              lnlen--;
            }

          /* Handle masking of the fractional final byte */

#ifdef CONFIG_NX_PACKEDMSFIRST
          mask &= (uint8_t)(0xff << (8 - NXGL_REMAINDERX(ix2)));
#else
          mask &= (uint8_t)(0xff >> (8 - NXGL_REMAINDERX(ix2)));
#endif
          if (lnlen > 0 && mask)
            {
              dest[lnlen-1] = (dest[lnlen-1] & ~mask) | (mpixel & mask);
              lnlen--;
            }

          /* Handle all of the unmasked bytes in-between */

          if (lnlen > 0)
            {
              NXGL_MEMSET(dest, (NXGL_PIXEL_T)color, lnlen);
            }

#else
          /* Then draw the run from (line + ix1) to (line + ix2) */

          NXGL_MEMSET(line + NXGL_SCALEX(ix1), (NXGL_PIXEL_T)color, width);
#endif
        }

      /* Move to the start of the next line */

      line += stride;

      /* Add the dx/dy value to get the run positions on the next row */

      x1   += dx1dy;
      x2   += dx2dy;
    }
}
