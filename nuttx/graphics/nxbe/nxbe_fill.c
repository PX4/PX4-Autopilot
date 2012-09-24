/****************************************************************************
 * graphics/nxbe/nxbe_fill.c
 *
 *   Copyright (C) 2008-2009, 2011 Gregory Nutt. All rights reserved.
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

#include <nuttx/nx/nxglib.h>

#include "nxbe.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nxbe_fill_s
{
  struct nxbe_clipops_s cops;
  nxgl_mxpixel_t color;
};

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
 * Name: nxbe_clipfill
 *
 * Description:
 *  Called from nxbe_clipper() to performed the fill operation on visible portions
 *  of the rectangle.
 *
 ****************************************************************************/

static void nxbe_clipfill(FAR struct nxbe_clipops_s *cops,
                        FAR struct nxbe_plane_s *plane,
                        FAR const struct nxgl_rect_s *rect)
{
  struct nxbe_fill_s *fillinfo = (struct nxbe_fill_s *)cops;
  plane->fillrectangle(&plane->pinfo, rect, fillinfo->color);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_fill
 *
 * Description:
 *  Fill the specified rectangle in the window with the specified color
 *
 * Input Parameters:
 *   wnd  - The window structure reference
 *   rect - The location to be filled
 *   col  - The color to use in the fill
 *
 * Return:
 *   None
 *
 ****************************************************************************/

void nxbe_fill(FAR struct nxbe_window_s *wnd,
               FAR const struct nxgl_rect_s *rect,
               nxgl_mxpixel_t color[CONFIG_NX_NPLANES])
{
  struct nxbe_fill_s info;
  struct nxgl_rect_s remaining;
  int i;

#ifdef CONFIG_DEBUG
  if (!wnd || !rect)
    {
      return;
    }
#endif

  /* Offset the rectangle by the window origin to convert it into a
   * bounding box
   */

  nxgl_rectoffset(&remaining, rect, wnd->bounds.pt1.x, wnd->bounds.pt1.y);

  /* Clip to the bounding box to the limits of the window and of the
   * background screen
   */

  nxgl_rectintersect(&remaining, &remaining, &wnd->bounds);
  nxgl_rectintersect(&remaining, &remaining, &wnd->be->bkgd.bounds);

  /* Then clip the bounding box due to other windows above this one.
   * Render the portions of the trapezoid exposed in visible regions.
   */

  if (!nxgl_nullrect(&remaining))
    {
#if CONFIG_NX_NPLANES > 1
      for (i = 0; i < wnd->be->vinfo.nplanes; i++)
#else
      i = 0;
#endif
        {
          info.cops.visible  = nxbe_clipfill;
          info.cops.obscured = nxbe_clipnull;
          info.color         = color[i];

          nxbe_clipper(wnd->above, &remaining, NX_CLIPORDER_DEFAULT,
                       &info.cops, &wnd->be->plane[i]);
        }
    }
}
