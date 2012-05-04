/****************************************************************************
 * graphics/nxbe/nxbe_move.c
 *
 *   Copyright (C) 2008-2009, 2011-2012 Gregory Nutt. All rights reserved.
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

#include <nuttx/nx/nxglib.h>

#include "nxbe.h"
#include "nxfe.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nxbe_move_s
{
  struct nxbe_clipops_s     cops;
  struct nxgl_point_s       offset;
  FAR struct nxbe_window_s *wnd;
  struct nxgl_rect_s        srcrect;
  uint8_t                   order;
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
 * Name: nxbe_clipmovesrc
 *
 * Description:
 *  Called from nxbe_clipper() to performed the move operation on visible regions
 *  of the rectangle.
 *
 ****************************************************************************/

static void nxbe_clipmovesrc(FAR struct nxbe_clipops_s *cops,
                             FAR struct nxbe_plane_s *plane,
                             FAR const struct nxgl_rect_s *rect)
{
  struct nxbe_move_s *info = (struct nxbe_move_s *)cops;
  struct nxgl_point_s offset;

  if (info->offset.x != 0 || info->offset.y != 0)
    {
      /* Offset is the destination position of the moved rectangle */

      offset.x = rect->pt1.x + info->offset.x;
      offset.y = rect->pt1.y + info->offset.y;

      plane->moverectangle(&plane->pinfo, rect, &offset);
    }
}

/****************************************************************************
 * Name: nxbe_clipmoveobscured
 *
 * Description:
 *  Called from nxbe_clipper() to performed the move operation on obsrured regions
 *  of the rectangle.
 *
 ****************************************************************************/

static void nxbe_clipmoveobscured(FAR struct nxbe_clipops_s *cops,
                                  FAR struct nxbe_plane_s *plane,
                                  FAR const struct nxgl_rect_s *rect)
{
  struct nxbe_move_s *info = (struct nxbe_move_s *)cops;
  struct nxgl_rect_s dst;

  nxgl_rectoffset(&dst, rect, info->offset.x, info->offset.y);
  nxfe_redrawreq(info->wnd, &dst);
}

/****************************************************************************
 * Name: nxbe_clipmovedest
 *
 * Description:
 *  Called from nxbe_clipper() to performed the move operation on visible
 *  regions of the source rectangle.
 *
 ****************************************************************************/

static void nxbe_clipmovedest(FAR struct nxbe_clipops_s *cops,
                              FAR struct nxbe_plane_s *plane,
                              FAR const struct nxgl_rect_s *rect)
{
  struct nxbe_move_s *dstdata = (struct nxbe_move_s *)cops;
  struct nxbe_window_s *wnd = dstdata->wnd;
  struct nxgl_point_s offset = dstdata->offset;
  struct nxgl_rect_s src;
  struct nxgl_rect_s tmprect1;
  struct nxgl_rect_s tmprect2;
  struct nxgl_rect_s nonintersecting[4];
  int i;

  /* Redraw dest regions where the source is outside of the bounds of the
   * background window
   */

  nxgl_rectoffset(&tmprect1, &dstdata->srcrect, offset.x, offset.y);
  nxgl_rectintersect(&tmprect2, &tmprect1, &wnd->be->bkgd.bounds);
  nxgl_nonintersecting(nonintersecting, rect, &tmprect2);

  for (i = 0; i < 4; i++)
    {
      if (!nxgl_nullrect(&nonintersecting[i]))
        {
          nxfe_redrawreq(dstdata->wnd, &nonintersecting[i]);
        }
    }

  /* Clip to determine what is inside the bounds */

  nxgl_rectintersect(&src, rect, &dstdata->srcrect);

  if (!nxgl_nullrect(&src))
    {
      struct nxbe_move_s srcinfo;

      srcinfo.cops.visible  = nxbe_clipmovesrc;
      srcinfo.cops.obscured = nxbe_clipmoveobscured;
      srcinfo.offset        = offset;
      srcinfo.wnd           = wnd;

      nxbe_clipper(dstdata->wnd->above, &src, dstdata->order,
                   &srcinfo.cops, plane);
   }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_move
 *
 * Description:
 *   Move a rectangular region within the window
 *
 * Input Parameters:
 *   wnd    - The window within which the move is to be done
 *   rect   - Describes the rectangular region to move
 *   offset - The offset to move the region
 *
 * Return:
 *   None
 *
 ****************************************************************************/

void nxbe_move(FAR struct nxbe_window_s *wnd, FAR const struct nxgl_rect_s *rect,
               FAR const struct nxgl_point_s *offset)
{
  struct nxbe_move_s info;
  int i;

#ifdef CONFIG_DEBUG
  if (!wnd || !rect)
    {
      return;
    }
#endif

  /* Offset the rectangle by the window origin to create a bounding box */

  nxgl_rectoffset(&info.srcrect, rect, wnd->bounds.pt1.x, wnd->bounds.pt1.y);

  /* Clip to the limits of the window and of the background screen */

  nxgl_rectintersect(&info.srcrect, &info.srcrect, &wnd->bounds);
  nxgl_rectintersect(&info.srcrect, &info.srcrect, &wnd->be->bkgd.bounds);

  if (nxgl_nullrect(&info.srcrect))
    {
      return;
    }

  info.cops.visible  = nxbe_clipmovedest;
  info.cops.obscured = nxbe_clipnull;
  info.offset.x      = offset->x;
  info.offset.y      = offset->y;
  info.wnd           = wnd;

  /* The clip order depends up the direction that the rectangle is being
   * moved.
   */
 
  if (offset->y < 0)
    {
      /* Moving rectangle up */

      if (offset->x < 0)
        {
          /* Moving to upper-left */

          info.order = NX_CLIPORDER_TLRB; /* Top-left-right-bottom */
        }
      else
        {
          /* Moving to upper-right (or just up) */

          info.order = NX_CLIPORDER_TRLB;  /* Top-right-left-bottom */
        }
    }
  else
    {
      /* Moving rectangle down (or just left/right) */

      if (offset->x < 0)
        {
          /* Moving to lower-left */

          info.order = NX_CLIPORDER_BLRT; /* Bottom-left-right-top */
        }
      else
        {
          /* Moving to lower-right */

          info.order = NX_CLIPORDER_BRLT; /* Bottom-right-left-top */
        }
    }

  /* Then perform the move */

#if CONFIG_NX_NPLANES > 1
  for (i = 0; i < wnd->be->vinfo.nplanes; i++)
#else
  i = 0;
#endif
    {
      nxbe_clipper(wnd->above, &info.srcrect, info.order,
                   &info.cops, &wnd->be->plane[i]);
    }
}
