/****************************************************************************
 * graphics/nxtk/nxtk_subwindowclip.c
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
 * Name: nxtk_subwindowclip
 *
 * Description:
 *   We are given a 'src' rectangle in sub-window, relative coordinates
 *   (i.e., (0,0) is the top left corner of the sub-window).  This function
 *   will (1) clip that src rectangle so that it lies within the sub-window
 *   bounds, and then (2) move the rectangle to that it is relative to the
 *   containing window (i.e., (0,0) is the top left corner of the containing
 *   window).
 *
 * Input parameters:
 *   fwnd   - The framed window to be used
 *   dest   - The locaton to put the result
 *   src    - The src rectangle in relative sub-window coordinates
 *   bounds - The subwindow bounds in absolute screen coordinates.
 *
 * Returned value:
 *   None
 *
 ****************************************************************************/

void nxtk_subwindowclip(FAR struct nxtk_framedwindow_s *fwnd,
                        FAR struct nxgl_rect_s *dest,
                        FAR const struct nxgl_rect_s *src,
                        FAR const struct nxgl_rect_s *bounds)
{
  struct nxgl_rect_s tmp;

  /* Temporarily, position the src rectangle in absolute screen coordinates */

  nxgl_rectoffset(&tmp, src, bounds->pt1.x, bounds->pt1.y);

  /* Clip the src rectangle to lie within the client window region */

  nxgl_rectintersect(&tmp, &tmp, bounds);

  /* Then move the rectangle so that is relative to the containing window, not the
   * client subwindow
   */

  nxgl_rectoffset(dest, &tmp, -fwnd->wnd.bounds.pt1.x, -fwnd->wnd.bounds.pt1.y);
}
