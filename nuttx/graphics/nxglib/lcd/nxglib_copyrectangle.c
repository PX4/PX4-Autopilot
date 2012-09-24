/****************************************************************************
 * graphics/nxglib/lcd/nxsglib_copyrectangle.c
 *
 *   Copyright (C) 2010-2011 Gregory Nutt. All rights reserved.
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
#include <assert.h>

#include <nuttx/lcd/lcd.h>
#include <nuttx/nx/nxglib.h>

#include "nxglib_bitblit.h"
#include "nxglib_copyrun.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

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
 * Name: nxgl_copyrectangle_*bpp
 *
 * Descripton:
 *   Copy a rectangular bitmap image into the specific position in the
 *   framebuffer memory.
 *
 ****************************************************************************/

void NXGL_FUNCNAME(nxgl_copyrectangle,NXGLIB_SUFFIX)
(FAR struct lcd_planeinfo_s *pinfo, FAR const struct nxgl_rect_s *dest,
 FAR const void *src, FAR const struct nxgl_point_s *origin,
 unsigned int srcstride)
{
  FAR const uint8_t *sline;
  unsigned int ncols;
  unsigned int row;
  unsigned int xoffset;
#if NXGLIB_BITSPERPIXEL < 8
  unsigned int remainder;
#endif

  /* Get the dimensions of the rectange to fill: width in pixels,
   * height in rows
   */

  ncols = dest->pt2.x - dest->pt1.x + 1;

  /* Set up to copy the image */

  xoffset = dest->pt1.x - origin->x;
  sline = (const uint8_t*)src + NXGL_SCALEX(xoffset) + (dest->pt1.y - origin->y) * srcstride;
#if NXGLIB_BITSPERPIXEL < 8
  remainder = NXGL_REMAINDERX(xoffset);
#endif

  /* Copy the image, one row at a time */

  for (row = dest->pt1.y; row <= dest->pt2.y; row++)
    {
#if NXGLIB_BITSPERPIXEL < 8
      /* if the source pixel is not aligned with a byte boundary, then we will
       * need to copy the image data to the run buffer first.
       */

      if (remainder != 0)
        {
          NXGL_FUNCNAME(nxgl_copyrun,NXGLIB_SUFFIX)(sline, pinfo->buffer, remainder, ncols);
          (void)pinfo->putrun(row, dest->pt1.x, pinfo->buffer, ncols);
        }
      else
#endif
        {
          /* The pixel data is byte aligned.  Copy the image data directly from
           * the image memory.
           */

          (void)pinfo->putrun(row, dest->pt1.x, sline, ncols);
        }

      /* Then adjust the source pointer to refer to the next line in the source
       * image.
       */

      sline += srcstride;
    }
}
