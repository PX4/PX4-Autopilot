/****************************************************************************
 * graphics/nxbe/nxbe_setsize.c
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
#include "nxfe.h"

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
 * Name: nxbe_setsize
 *
 * Descripton:
 *   This function checks for intersections and redraws the display after
 *   a change in the size of a window.
 *
 ****************************************************************************/

void nxbe_setsize(FAR struct nxbe_window_s *wnd,
                  FAR const struct nxgl_size_s *size)
{
  struct nxgl_rect_s bounds;

#ifdef CONFIG_DEBUG
  if (!wnd)
    {
      return;
    }
#endif

  /* Save the before size of the window's bounding box */

  nxgl_rectcopy(&bounds, &wnd->bounds);

  /* Add the window origin to the supplied size get the new window bounding box */

  wnd->bounds.pt2.x = wnd->bounds.pt1.x + size->w - 1;
  wnd->bounds.pt2.y = wnd->bounds.pt1.y + size->h - 1;

  /* Clip the new bounding box so that lies within the background screen */

  nxgl_rectintersect(&wnd->bounds, &wnd->bounds, &wnd->be->bkgd.bounds);

  /* We need to update the larger of the two rectangles.  That will be the
   * union of the before and after sizes.
   */

  nxgl_rectunion(&bounds, &bounds, &wnd->bounds);

  /* Report the new size/position */

  nxfe_reportposition(wnd);

  /* Then redraw this window AND all windows below it. Having resized the
   * window, we may have exposed previoulsy obscured portions of windows
   * below this one.
   */

  nxbe_redrawbelow(wnd->be, wnd, &bounds);
}
