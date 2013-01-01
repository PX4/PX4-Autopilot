/****************************************************************************
 * graphics/nxtk/nxtk_subwindowmove.c
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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

#include <stdlib.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxtk.h>

#include "nxfe.h"
#include "nxtk_internal.h"

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
 * Name: nxtk_subwindowmove
 *
 * Description:
 *   Perform common clipping operations in preparatons for calling nx_move()
 *
 * Input Parameters:
 *   fwnd       - The framed window within which the move is to be done.
 *                This must have been previously created by nxtk_openwindow().
 *   destrect   - The loccation to receive the clipped rectangle relative
 *                to containing window
 *   destoffset - The location to received the clipped offset.
 *   srcrect    - Describes the rectangular region relative to the client
 *                sub-window to move relative to the sub-window
 *   srcoffset  - The offset to move the region
 *   bounds     - The subwindow bounds in absolute screen coordinates.
 *
 * Return:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

void nxtk_subwindowmove(FAR struct nxtk_framedwindow_s *fwnd,
                        FAR struct nxgl_rect_s *destrect,
                        FAR struct nxgl_point_s *destoffset,
                        FAR const struct nxgl_rect_s *srcrect,
                        FAR const struct nxgl_point_s *srcoffset,
                        FAR const struct nxgl_rect_s *bounds)
{
  struct nxgl_rect_s abssrc;

  /* Temporarily, position the src rectangle in absolute screen coordinates */

  nxgl_rectoffset(&abssrc, srcrect, bounds->pt1.x, bounds->pt1.y);

  /* Clip the src rectangle to lie within the client window region */

  nxgl_rectintersect(&abssrc, &abssrc, &fwnd->fwrect);

  /* Clip the source rectangle so that destination area is within the window. */

  destoffset->x = srcoffset->x;
  if (destoffset->x < 0)
    {
      if (abssrc.pt1.x + destoffset->x < bounds->pt1.x)
        {
           abssrc.pt1.x = bounds->pt1.x - destoffset->x;
        }
    }
  else if (abssrc.pt2.x + destoffset->x > bounds->pt2.x)
    {
       abssrc.pt2.x = bounds->pt2.x - destoffset->x;
    }

  destoffset->y = srcoffset->y;
  if (destoffset->y < 0)
    {
      if (abssrc.pt1.y + destoffset->y < bounds->pt1.y)
        {
           abssrc.pt1.y = bounds->pt1.y - destoffset->y;
        }
    }
  else if (abssrc.pt2.y + destoffset->y > bounds->pt2.y)
    {
       abssrc.pt2.y = bounds->pt2.y - destoffset->y;
    }


  /* Then move the rectangle so that is relative to the containing window, not the
   * client subwindow
   */

  nxgl_rectoffset(destrect, &abssrc, -fwnd->wnd.bounds.pt1.x, -fwnd->wnd.bounds.pt1.y);
}
