/****************************************************************************
 * graphics/nxbe/nxbe_bitmap.c
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

#include <errno.h>
#include <debug.h>

#include <nuttx/nx/nxglib.h>
#include "nxbe.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nx_bitmap_s
{
  struct nxbe_clipops_s cops;
  FAR const void *src;              /* The start of the source image. */
  struct nxgl_point_s origin;       /* Offset into the source image data */
  unsigned int stride;              /* The width of the full source image in pixels. */
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
 * Name: nxs_clipcopy
 *
 * Description:
 *  Called from nxbe_clipper() to performed the fill operation on visible portions
 *  of the rectangle.
 *
 ****************************************************************************/

static void nxs_clipcopy(FAR struct nxbe_clipops_s *cops,
                         FAR struct nxbe_plane_s *plane,
                         FAR const struct nxgl_rect_s *rect)
{
  struct nx_bitmap_s *bminfo = (struct nx_bitmap_s *)cops;
  plane->copyrectangle(&plane->pinfo, rect, bminfo->src,
                       &bminfo->origin, bminfo->stride);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_bitmap
 *
 * Description:
 *   Copy a rectangular region of a larger image into the rectangle in the
 *   specified window.
 *
 * Input Parameters:
 *   wnd   - The window that will receive the bitmap image
 *   dest   - Describes the rectangular on the display that will receive the
 *            the bit map.
 *   src    - The start of the source image.
 *   origin - The origin of the upper, left-most corner of the full bitmap.
 *            Both dest and origin are in window coordinates, however, origin
 *            may lie outside of the display.
 *   stride - The width of the full source image in pixels.
 *
 * Return:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

void nxbe_bitmap(FAR struct nxbe_window_s *wnd, FAR const struct nxgl_rect_s *dest,
                FAR const void *src[CONFIG_NX_NPLANES],
                FAR const struct nxgl_point_s *origin, unsigned int stride)
{
  struct nx_bitmap_s info;
  struct nxgl_rect_s bounds;
  struct nxgl_point_s offset;
  struct nxgl_rect_s remaining;
  unsigned int deststride;
  int i;

#ifdef CONFIG_DEBUG
  if (!wnd || !dest || !src || !origin)
    {
      return;
    }
#endif

  /* Verify that the destination rectangle begins "below" and to the "right"
   * of the origin
   */

  if (dest->pt1.x < origin->x || dest->pt1.y < origin->y)
    {
      gdbg("Bad dest start position\n");
      return;
    }

  /* Verify that the width of the destination rectangle does not exceed the 
   * width of the source bitmap data (taking into account the bitmap origin)
   */

  deststride = (((dest->pt2.x - origin->x + 1) * wnd->be->plane[0].pinfo.bpp + 7) >> 3);
  if (deststride > stride)
    {
      gdbg("Bad dest width\n");
      return;
    }

  /* Offset the rectangle and image origin by the window origin */

  nxgl_rectoffset(&bounds, dest, wnd->bounds.pt1.x, wnd->bounds.pt1.y);
  nxgl_vectoradd(&offset, origin, &wnd->bounds.pt1);

  /* Clip to the limits of the window and of the background screen */

  nxgl_rectintersect(&remaining, &bounds, &wnd->bounds);
  nxgl_rectintersect(&remaining, &remaining, &wnd->be->bkgd.bounds);
  if (nxgl_nullrect(&remaining))
    {
      return;
    }

  /* Then perform the clipped fill */

#if CONFIG_NX_NPLANES > 1
  for (i = 0; i < wnd->be->vinfo.nplanes; i++)
#else
  i = 0;
#endif
    {
      info.cops.visible  = nxs_clipcopy;
      info.cops.obscured = nxbe_clipnull;
      info.src           = src[i];
      info.origin.x      = offset.x;
      info.origin.y      = offset.y;
      info.stride        = stride;

      nxbe_clipper(wnd->above, &remaining, NX_CLIPORDER_DEFAULT,
                   &info.cops, &wnd->be->plane[i]);
    }
}

