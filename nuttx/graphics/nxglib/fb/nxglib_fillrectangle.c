/****************************************************************************
 * graphics/nxglib/fb/nxglib_fillrectangle.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

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
 * Name: nxgl_fillrectangle_*bpp
 *
 * Descripton:
 *   Fill a rectangle region in the framebuffer memory with a fixed color
 *
 ****************************************************************************/

void NXGL_FUNCNAME(nxgl_fillrectangle,NXGLIB_SUFFIX)
  (FAR struct fb_planeinfo_s *pinfo,
   FAR const struct nxgl_rect_s *rect,
   NXGL_PIXEL_T color)
{
  FAR uint8_t *line;
  unsigned int width;
  unsigned int stride;
  int rows;

#if NXGLIB_BITSPERPIXEL < 8
  FAR uint8_t *dest;
  uint8_t mpixel = NXGL_MULTIPIXEL(color);
  uint8_t leadmask;
  uint8_t tailmask;
  uint8_t mask;
  int lnlen;
#endif

  /* Get the width of the framebuffer in bytes */

  stride = pinfo->stride;

  /* Get the dimensions of the rectange to fill in pixels */

  width  = rect->pt2.x - rect->pt1.x + 1;
  rows   = rect->pt2.y - rect->pt1.y + 1;

  /* Get the address of the first byte in the first line to write */

  line   = pinfo->fbmem + rect->pt1.y * stride + NXGL_SCALEX(rect->pt1.x);

#if NXGLIB_BITSPERPIXEL < 8
# ifdef CONFIG_NX_PACKEDMSFIRST

  /* Get the mask for pixels that are ordered so that they pack from the
   * MS byte down.
   */

  leadmask = (uint8_t)(0xff >> (8 - NXGL_REMAINDERX(rect->pt1.x)));
  tailmask = (uint8_t)(0xff << (8 - NXGL_REMAINDERX(rect->pt2.x-1)));
# else
  /* Get the mask for pixels that are ordered so that they pack from the
   * LS byte up.
   */

  leadmask = (uint8_t)(0xff << (8 - NXGL_REMAINDERX(rect->pt1.x)));
  tailmask = (uint8_t)(0xff >> (8 - NXGL_REMAINDERX(rect->pt1.x-1)));
# endif
#endif

  /* Then fill the rectangle line-by-line */

  while (rows-- > 0)
    {
#if NXGLIB_BITSPERPIXEL < 8
     /* Handle masking of the fractional initial byte */

     mask  = leadmask;
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

      mask &= tailmask;
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
      /* Draw the entire raster line */

      NXGL_MEMSET(line, (NXGL_PIXEL_T)color, width);
#endif
      line += stride;
    }
}
