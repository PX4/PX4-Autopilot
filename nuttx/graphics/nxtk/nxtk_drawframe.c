/****************************************************************************
 * graphics/nxtk/nxtk_drawframe.c
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
  nxgl_coord_t thickness;

  /* Shiny edge: 
   *   Thickness: 1
   *   Color:     CONFIG_NXTK_BORDERCOLOR3;
   *   Condition: CONFIG_NXTK_BORDERWIDTH > 2
   * Central part:
   *   Thickness: Varies with CONFIG_NXTK_BORDERWIDTH
   *   Color:     CONFIG_NXTK_BORDERCOLOR1;
   *   Condition: CONFIG_NXTK_BORDERWIDTH > 0
   * Shadow part:
   *   Thickness: 1;
   *   Color:     CONFIG_NXTK_BORDERCOLOR2;
   *   Condition: CONFIG_NXTK_BORDERWIDTH > 1
   */

#if CONFIG_NXTK_BORDERWIDTH > 2
  thickness = CONFIG_NXTK_BORDERWIDTH - 2;
#elif CONFIG_NXTK_BORDERWIDTH > 1
  thickness = CONFIG_NXTK_BORDERWIDTH - 1;
#else
  thickness = CONFIG_NXTK_BORDERWIDTH;
#endif

  /* Get the size of the rectangle */

  nxgl_rectsize(&wndsize, &fwnd->wnd.bounds);
  nxgl_rectsize(&tbsize, &fwnd->tbrect);

  /* Draw the top ***********************************************************/

#if CONFIG_NXTK_BORDERWIDTH > 0
  frame.pt1.x = 0;
  frame.pt2.x = wndsize.w - 1;
  frame.pt1.y = 0;

  /* Draw the shiny edge */

#if CONFIG_NXTK_BORDERWIDTH > 2
  frame.pt2.y = 0;
  nxtk_drawframeside(fwnd, &frame, bounds, g_bordercolor3);
  frame.pt1.y = 1;
#endif

  /* Draw the central part */

  frame.pt2.y = frame.pt1.y + thickness - 1;
  nxtk_drawframeside(fwnd, &frame, bounds, g_bordercolor1);

  /* Draw a single line under the toolbar, color CONFIG_NXTK_BORDERCOLOR2 */

#if CONFIG_NXTK_BORDERWIDTH > 1
  frame.pt1.y += tbsize.h + thickness;
  frame.pt2.y  = frame.pt1.y;
  nxtk_drawframeside(fwnd, &frame, bounds, g_bordercolor2);
#endif

  /* Draw the bottom ********************************************************/

#if CONFIG_NXTK_BORDERWIDTH > 0
  frame.pt1.y = wndsize.h - CONFIG_NXTK_BORDERWIDTH;

  /* Draw the shiny edge */

#if CONFIG_NXTK_BORDERWIDTH > 2
  frame.pt2.y = frame.pt1.y;
  nxtk_drawframeside(fwnd, &frame, bounds, g_bordercolor3);
  frame.pt1.y ++;
#endif

  /* Draw the central part */

  frame.pt2.y = frame.pt1.y + thickness - 1;
  nxtk_drawframeside(fwnd, &frame, bounds, g_bordercolor1);

  /* Then a single line at the very bottom, Color: CONFIG_NXTK_BORDERCOLOR2 */

#if CONFIG_NXTK_BORDERWIDTH > 1
  frame.pt1.y = wndsize.h - 1;
  frame.pt2.y = frame.pt1.y;
  nxtk_drawframeside(fwnd, &frame, bounds, g_bordercolor2);
#endif
#endif

  /* Draw left and right outer edges *****************************************/

  /* Draw the shiny left out edge */

#if CONFIG_NXTK_BORDERWIDTH > 1
  frame.pt1.x = 0;
  frame.pt1.y = 1;
#if CONFIG_NXTK_BORDERWIDTH > 2
  frame.pt2.x = frame.pt1.x;
  frame.pt2.y = wndsize.h - 2;
  nxtk_drawframeside(fwnd, &frame, bounds, g_bordercolor3);
#endif

  /* Draw the shadowed right outer edge */

  frame.pt1.x = wndsize.w - 1;
  frame.pt2.x = frame.pt1.x;
  nxtk_drawframeside(fwnd, &frame, bounds, g_bordercolor2);
#endif

  /* Draw left and right central regions *************************************/

#if CONFIG_NXTK_BORDERWIDTH > 2
  frame.pt1.x = 1;
  frame.pt1.y = 1;
  frame.pt2.x = frame.pt1.x + thickness - 1;
  frame.pt2.y = wndsize.h - 2;
#else
  frame.pt1.x = 0;
  frame.pt1.y = 0;
  frame.pt2.x = frame.pt1.x + thickness - 1;
  frame.pt2.y = wndsize.h - 1;
#endif
  nxtk_drawframeside(fwnd, &frame, bounds, g_bordercolor1);

#if CONFIG_NXTK_BORDERWIDTH > 2
  frame.pt1.x = wndsize.w - thickness - 1;
  frame.pt2.x = wndsize.w - 2;
#else
  frame.pt1.x = wndsize.w - thickness;
  frame.pt2.x = wndsize.w - 1;
#endif
  nxtk_drawframeside(fwnd, &frame, bounds, g_bordercolor1);
#endif

  /* Draw left and right inner sides *****************************************/
  /* This segment stops at the bottom of the toolbar.  If there is a
   * tool bar, then we have to continue this to the top of the display
   * using g_bordercolor1 (see below)
   */

  /* Draw the shadowed left inner edge */

#if CONFIG_NXTK_BORDERWIDTH > 1
#if CONFIG_NXTK_BORDERWIDTH > 2
  frame.pt1.x = thickness + 1;
  frame.pt1.y = tbsize.h + thickness + 1;
  frame.pt2.x = frame.pt1.x;
  frame.pt2.y = wndsize.h - thickness - 2;
#else
  frame.pt1.x = thickness;
  frame.pt1.y = tbsize.h + thickness;
  frame.pt2.x = frame.pt1.x;
  frame.pt2.y = wndsize.h - thickness - 1;
#endif
  nxtk_drawframeside(fwnd, &frame, bounds, g_bordercolor2);

  /* Draw the shiny right inner edge */

#if CONFIG_NXTK_BORDERWIDTH > 2
  frame.pt1.x = wndsize.w - thickness - 2;
  frame.pt2.x = frame.pt1.x;
  nxtk_drawframeside(fwnd, &frame, bounds, g_bordercolor3);
#endif
#endif

  /* Fix up the little line-seqments at the top inner edges that need to match
   * the color of the toolbar.
   */

#if CONFIG_NXTK_BORDERWIDTH > 1
  if (tbsize.h > 0)
    {
      /* Draw the right side */

#if CONFIG_NXTK_BORDERWIDTH > 2
      frame.pt1.x = thickness + 1;
      frame.pt1.y = 1;
      frame.pt2.x = frame.pt1.x;
      frame.pt2.y = tbsize.h + thickness;
#else
      frame.pt1.x = thickness;
      frame.pt1.y = 0;
      frame.pt2.x = frame.pt1.x;
      frame.pt2.y = tbsize.h + thickness - 1;
#endif
      nxtk_drawframeside(fwnd, &frame, bounds, g_bordercolor1);

      /* Draw the left size */

#if CONFIG_NXTK_BORDERWIDTH > 2
      frame.pt1.x = wndsize.w - thickness - 2;
      frame.pt2.x = frame.pt1.x;
      nxtk_drawframeside(fwnd, &frame, bounds, g_bordercolor1);
#endif
    }
#endif

  return OK;
}
