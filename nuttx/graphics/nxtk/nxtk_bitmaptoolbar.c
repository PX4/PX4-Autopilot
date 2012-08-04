/****************************************************************************
 * graphics/nxtk/nxtk_bitmaptoolbar.c
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

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxtk.h>

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
 * Name: nxtk_bitmaptoolbar
 *
 * Description:
 *   Copy a rectangular region of a larger image into the rectangle in the
 *   specified toolbar sub-window.
 *
 * Input Parameters:
 *   hfwnd  - The sub-window twhose toolbar will receive the bitmap image
 *   dest   - Describes the rectangular region on in the toolbar sub-window
 *            will receive the bit map.
 *   src    - The start of the source image.
 *   origin - The origin of the upper, left-most corner of the full bitmap.
 *            Both dest and origin are in sub-window coordinates, however, the
 *            origin may lie outside of the sub-window display.
 *   stride - The width of the full source image in pixels.
 *
 * Return:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_bitmaptoolbar(NXTKWINDOW hfwnd, FAR const struct nxgl_rect_s *dest,
                       FAR const void *src[CONFIG_NX_NPLANES],
                       FAR const struct nxgl_point_s *origin, unsigned int stride)
{
  FAR struct nxtk_framedwindow_s *fwnd = (FAR struct nxtk_framedwindow_s *)hfwnd;
  struct nxgl_point_s wndorigin;
  struct nxgl_rect_s clipdest;

#ifdef CONFIG_DEBUG
  if (!hfwnd || !dest || !src || !origin)
    {
      errno = EINVAL;
      return ERROR;
    }
#endif

  /* Clip the rectangle so that it lies within the sub-window bounds
   * then move the rectangle to that it is relative to the containing
   * window.
   */

  nxtk_subwindowclip(fwnd, &clipdest, dest, &fwnd->tbrect);

  /* Now, move the bitmap origin so that it is relative to the containing
   * window, not the sub-window.
   *
   * Temporarily, position the origin in absolute screen coordinates
   */

  nxgl_vectoradd(&wndorigin, origin, &fwnd->tbrect.pt1);

  /* Then move the origin so that is relative to the containing window, not the
   * client subwindow
   */

  nxgl_vectsubtract(&wndorigin, &wndorigin, &fwnd->wnd.bounds.pt1);

  /* Then copy the bitmap */

  nx_bitmap((NXWINDOW)hfwnd, &clipdest, src, &wndorigin, stride);
  return OK;
}
