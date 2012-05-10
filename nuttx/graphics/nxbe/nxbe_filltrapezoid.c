/****************************************************************************
 * graphics/nxbe/nxbe_filltrapezoid.c
 *
 *   Copyright (C) 2008-2009, 2012 Gregory Nutt. All rights reserved.
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

#include <fixedmath.h>
#include <nuttx/nx/nxglib.h>

#include "nxbe.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nxbe_filltrap_s
{
  struct nxbe_clipops_s cops;
  struct nxgl_trapezoid_s trap;
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
 * Name: nxbe_clipfilltrapezoid
 *
 * Description:
 *  Called from nxbe_clipper() to performed the fill operation on visible portions
 *  of the rectangle.
 *
 ****************************************************************************/

static void nxbe_clipfilltrapezoid(FAR struct nxbe_clipops_s *cops,
                                   FAR struct nxbe_plane_s *plane,
                                   FAR const struct nxgl_rect_s *rect)
{
  struct nxbe_filltrap_s *fillinfo = (struct nxbe_filltrap_s *)cops;
  plane->filltrapezoid(&plane->pinfo, &fillinfo->trap, rect, fillinfo->color);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_filltrapezoid
 *
 * Description:
 *  Fill the specified rectangle in the window with the specified color
 *
 * Input Parameters:
 *   wnd  - The window structure reference
 *   clip - Clipping region (in relative window coordinates)
 *   rect - The location to be filled (in relative window coordinates)
 *   col  - The color to use in the fill
 *
 * Return:
 *   None
 *
 ****************************************************************************/

void nxbe_filltrapezoid(FAR struct nxbe_window_s *wnd,
                        FAR const struct nxgl_rect_s *clip,
                        FAR const struct nxgl_trapezoid_s *trap,
                        nxgl_mxpixel_t color[CONFIG_NX_NPLANES])
{
  struct nxbe_filltrap_s info;
  struct nxgl_rect_s remaining;
  int i;

#ifdef CONFIG_DEBUG
  if (!wnd || !trap)
    {
      return;
    }
#endif

  /* Offset the trapezoid by the window origin to position it within
   * the framebuffer region
   */

  nxgl_trapoffset(&info.trap, trap, wnd->bounds.pt1.x, wnd->bounds.pt1.y);

  /* Create a bounding box that contains the trapezoid */

  remaining.pt1.x = b16toi(ngl_min(info.trap.top.x1, info.trap.bot.x1));
  remaining.pt1.y = info.trap.top.y;
  remaining.pt2.x = b16toi(ngl_max(info.trap.top.x2, info.trap.bot.x2));
  remaining.pt2.y = info.trap.bot.y;

  /* Clip to any user specified clipping window */

  if (clip)
    {
      struct nxgl_rect_s tmp;
      nxgl_rectoffset(&tmp, clip, wnd->bounds.pt1.x, wnd->bounds.pt1.y);
      nxgl_rectintersect(&remaining, &remaining, &tmp);
    }

  /* Clip to the limits of the window and of the background screen */

  nxgl_rectintersect(&remaining, &remaining, &wnd->bounds);
  nxgl_rectintersect(&remaining, &remaining, &wnd->be->bkgd.bounds);

  if (!nxgl_nullrect(&remaining))
    {
      info.cops.visible  = nxbe_clipfilltrapezoid;
      info.cops.obscured = nxbe_clipnull;

      /* Then process each color plane */

#if CONFIG_NX_NPLANES > 1
      for (i = 0; i < wnd->be->vinfo.nplanes; i++)
#else
      i = 0;
#endif
        {
          info.color = color[i];
          nxbe_clipper(wnd->above, &remaining, NX_CLIPORDER_DEFAULT,
                       &info.cops, &wnd->be->plane[i]);
        }
    }
}
