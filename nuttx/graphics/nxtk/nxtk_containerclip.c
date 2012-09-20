/****************************************************************************
 * graphics/nxtk/nxtk_containerclip.c
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
 * Name: nxtk_containerclip
 *
 * Description:
 *   We are given a 'src' rectangle in containing window, relative coordinates
 *   (i.e., (0,0) is the top left corner of the outer, containing window).
 *   This function will (1) clip that src rectangle so that it lies within
 *   the sub-window bounds, and then (2) move the rectangle to that it is
 *   relative to the sub-window (i.e., (0,0) is the top left corner of the
 *   sub-window).
 *
 * Input parameters:
 *   fwnd   - The framed window to be used
 *   dest   - The locaton to put the result
 *   src    - The src rectangle in relative container-window coordinates
 *   bounds - The subwindow bounds in absolute screen coordinates.
 *
 * Returned value:
 *   None
 *
 ****************************************************************************/

void nxtk_containerclip(FAR struct nxtk_framedwindow_s *fwnd,
                        FAR struct nxgl_rect_s *dest,
                        FAR const struct nxgl_rect_s *src,
                        FAR const struct nxgl_rect_s *bounds)
{
  struct nxgl_rect_s relbounds;

  /* The 'src' rectangle is relative to the containing window. Convert
   * the sub-window to the same origin.
   */

  nxgl_rectoffset(&relbounds, bounds, -fwnd->wnd.bounds.pt1.x,
                  -fwnd->wnd.bounds.pt1.y);

  /* The interection then leaves the portion of the containing window that
   * needs to be updated window that needs to be updated.
   */

  nxgl_rectintersect(dest, src, &relbounds);

  /* Offset this so that is relative to client subwindow origin */

  nxgl_rectoffset(dest, dest, fwnd->wnd.bounds.pt1.x - bounds->pt1.x,
                  fwnd->wnd.bounds.pt1.y - bounds->pt1.y);
}
