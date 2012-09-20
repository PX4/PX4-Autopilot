/****************************************************************************
 * graphics/nxglib/lcd/nxglib_moverectangle.c
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

#include <nuttx/lcd/lcd.h>
#include <nuttx/nx/nxglib.h>

#include "nxglib_bitblit.h"

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
 * Name: nxgl_moverectangle_*bpp
 *
 * Descripton:
 *   Move a rectangular region from location to another in the
 *   LCD memory.  The source is expressed as a rectangle; the
 *   destination position is expressed as a point corresponding to the
 *   translation of the upper, left-hand corner.
 *
 ****************************************************************************/

void NXGL_FUNCNAME(nxgl_moverectangle,NXGLIB_SUFFIX)
(FAR struct lcd_planeinfo_s *pinfo, FAR const struct nxgl_rect_s *rect,
 FAR struct nxgl_point_s *offset)
{
  unsigned int ncols;
  unsigned int srcrow;
  unsigned int destrow;

  /* Get the width of the rectange to move in pixels. */

  ncols = rect->pt2.x - rect->pt1.x + 1;

  /* Case 1:  The destination position (offset) is above the displayed
   * position (rect)
   */

  if (offset->y < rect->pt1.y)
    {
      /* Copy the rectangle from top down */

      for (srcrow = rect->pt1.y, destrow = offset->y;
           srcrow <= rect->pt2.y;
           srcrow++, destrow++)
        {
          (void)pinfo->getrun(srcrow, rect->pt1.x, pinfo->buffer, ncols);
          (void)pinfo->putrun(destrow, offset->x, pinfo->buffer, ncols);
        }
    }

  /* Case 2: The destination position (offset) is below the displayed
   * position (rect)
   */

  else
    {
      unsigned int dy = rect->pt2.y - rect->pt1.y;

      /* Copy the rectangle from the bottom up */

      for (srcrow = rect->pt2.y, destrow = offset->y + dy;
           srcrow >= rect->pt1.y;
           srcrow--, destrow--)
        {
          (void)pinfo->getrun(srcrow, rect->pt1.x, pinfo->buffer, ncols);
          (void)pinfo->putrun(destrow, offset->x, pinfo->buffer, ncols);
        }
    }
}
