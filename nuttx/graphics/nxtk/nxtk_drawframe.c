/****************************************************************************
 * graphics/nxtk/nxtk_drawframe.c
 *
 *   Copyright (C) 2008-2009, 2011 Gregory Nutt. All rights reserved.
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

#include <stdlib.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/nx/nxglib.h>
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
 * Name: nxtk_drawframeside
 ****************************************************************************/

static void nxtk_drawframeside(FAR struct nxtk_framedwindow_s *fwnd,
                               FAR const struct nxgl_rect_s *side,
                               FAR const struct nxgl_rect_s *bounds,
                               nxgl_mxpixel_t color[CONFIG_NX_NPLANES])
{
  struct nxgl_rect_s intersection;
  nxgl_rectintersect(&intersection, side, bounds);
  if (!nxgl_nullrect(&intersection))
    {
      nx_fill((NXWINDOW)fwnd, &intersection, color);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxtk_drawframe
 *
 * Description:
 *   Redraw the window frame.
 *
 * Input parameters:
 *   fwnd   - the framed window whose frame needs to be re-drawn.  This must
 *            have been previously created by nxtk_openwindow().
 *   bounds - Only draw the ports of the frame within this bounding box.
 *            (window relative coordinates).
 *
 * Returned value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_drawframe(FAR struct nxtk_framedwindow_s *fwnd,
                   FAR const struct nxgl_rect_s *bounds)
{
  struct nxgl_rect_s frame;
  struct nxgl_size_s wndsize;
  struct nxgl_size_s tbsize;

  /* Get the size of the rectangle */

  nxgl_rectsize(&wndsize, &fwnd->wnd.bounds);
  nxgl_rectsize(&tbsize, &fwnd->tbrect);

  /* Draw the top.  Thickness: CONFIG_NXTK_BORDERWIDTH-1, Color:
   * CONFIG_NXTK_BORDERCOLOR1
   */

  frame.pt1.x = 0;
  frame.pt2.x = wndsize.w - 1;

  frame.pt1.y = 0;
#if CONFIG_NXTK_BORDERWIDTH > 1
  frame.pt2.y = CONFIG_NXTK_BORDERWIDTH - 2;
#else
  frame.pt2.y = CONFIG_NXTK_BORDERWIDTH - 1;
#endif
  nxtk_drawframeside(fwnd, &frame, bounds, g_bordercolor1);

  /* Draw a single line under the toolbar, color CONFIG_NXTK_BORDERCOLOR2 */

#if CONFIG_NXTK_BORDERWIDTH > 1
  frame.pt1.y += tbsize.h + CONFIG_NXTK_BORDERWIDTH - 1;
  frame.pt2.y  = frame.pt1.y;
  nxtk_drawframeside(fwnd, &frame, bounds, g_bordercolor2);
#endif

  /* Draw the bottom.  First, thickness: CONFIG_NXTK_BORDERWIDTH-1, 
   * Color: CONFIG_NXTK_BORDERCOLOR1
   */

  frame.pt1.y = wndsize.h - CONFIG_NXTK_BORDERWIDTH;
#if CONFIG_NXTK_BORDERWIDTH > 1
  frame.pt2.y = wndsize.h - 2;
#else
  frame.pt2.y = frame.pt1.y;
#endif
  nxtk_drawframeside(fwnd, &frame, bounds, g_bordercolor1);

  /* Then a single line at the very bottom, Color: CONFIG_NXTK_BORDERCOLOR2 */

#if CONFIG_NXTK_BORDERWIDTH > 1
  frame.pt1.y = wndsize.h - 1;
  frame.pt2.y = frame.pt1.y;
  nxtk_drawframeside(fwnd, &frame, bounds, g_bordercolor2);
#endif

  /* Draw the outer left side.  Thickness: CONFIG_NXTK_BORDERWIDTH-1,
   * Color: CONFIG_NXTK_BORDERCOLOR1
   */

  frame.pt1.y = 0;
  frame.pt2.y = wndsize.h - 2;

  frame.pt1.x = 0;
#if CONFIG_NXTK_BORDERWIDTH > 1
  frame.pt2.x = CONFIG_NXTK_BORDERWIDTH - 2;
#else
  frame.pt2.x = frame.pt1.x;
#endif
  nxtk_drawframeside(fwnd, &frame, bounds, g_bordercolor1);

  /* Draw the outer right side. Thickness: 1, Color: CONFIG_NXTK_BORDERCOLOR2 */

#if CONFIG_NXTK_BORDERWIDTH > 1
  frame.pt1.x = wndsize.w - 1;
  frame.pt2.x = frame.pt1.x;
  nxtk_drawframeside(fwnd, &frame, bounds, g_bordercolor2);
#endif

  /* Draw the inner left side,  Thickness: 1, Color: CONFIG_NXTK_BORDERCOLOR2.
   * This segment stops at the bottom of the toolbar.  If there is a
   * tool bar, then we have to continue this to the top of the display
   * using g_bordercolor1 (see below)
   */

#if CONFIG_NXTK_BORDERWIDTH > 1
  frame.pt1.y = CONFIG_NXTK_BORDERWIDTH - 1 + tbsize.h;
#else
  frame.pt1.y = CONFIG_NXTK_BORDERWIDTH + tbsize.h;
#endif
  frame.pt2.y = wndsize.h - CONFIG_NXTK_BORDERWIDTH - 1;
#if CONFIG_NXTK_BORDERWIDTH > 1
  frame.pt1.x = CONFIG_NXTK_BORDERWIDTH - 1;
  frame.pt2.x = frame.pt1.x;
  nxtk_drawframeside(fwnd, &frame, bounds, g_bordercolor2);
#endif

  /* Draw the inner left side,  Thickness:  CONFIG_NXTK_BORDERWIDTH-1,
   * Color: CONFIG_NXTK_BORDERCOLOR1
   */

#if CONFIG_NXTK_BORDERWIDTH > 1
  frame.pt1.x = wndsize.w - CONFIG_NXTK_BORDERWIDTH;
  frame.pt2.x = wndsize.w - 2;
#else
  frame.pt1.x = wndsize.w - 1;
  frame.pt2.x = frame.pt1.x;
#endif
  nxtk_drawframeside(fwnd, &frame, bounds, g_bordercolor1);

  /* When there is a toolbar, we also have to patch in this tiny
   * line segment -- Is there a better way?
   */

#if CONFIG_NXTK_BORDERWIDTH > 1
  if (tbsize.h > 0)
    {
      frame.pt1.y = 0;
      frame.pt2.y = CONFIG_NXTK_BORDERWIDTH + tbsize.h - 2;

      frame.pt1.x = CONFIG_NXTK_BORDERWIDTH - 1;
      frame.pt2.x = frame.pt1.x;
      nxtk_drawframeside(fwnd, &frame, bounds, g_bordercolor1);
    }
#endif

  return OK;
}
