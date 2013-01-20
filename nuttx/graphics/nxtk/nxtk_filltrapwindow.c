/****************************************************************************
 * graphics/nxtk/nxtk_filltrapwindow.c
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
 * Name: nxtk_filltrapwindow
 *
 * Description:
 *  Fill the specified rectangle in the client window with the specified color
 *
 * Input Parameters:
 *   hfwnd - The window handle returned by nxtk_openwindow
 *   trap  - The trapezoidal region to be filled
 *   color - The color to use in the fill
 *
 * Return:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_filltrapwindow(NXTKWINDOW hfwnd, FAR const struct nxgl_trapezoid_s *trap,
                        nxgl_mxpixel_t color[CONFIG_NX_NPLANES])
{
  FAR struct nxtk_framedwindow_s *fwnd = (FAR struct nxtk_framedwindow_s *)hfwnd;
  struct nxgl_rect_s relclip;
  struct nxgl_trapezoid_s reltrap;

#ifdef CONFIG_DEBUG
  if (!hfwnd || !trap || !color)
    {
      errno = EINVAL;
      return ERROR;
    }
#endif

  /* Move the trapezoid from window contents area to window area */

  nxgl_trapoffset(&reltrap, trap,
                  fwnd->fwrect.pt1.x - fwnd->wnd.bounds.pt1.x,
                  fwnd->fwrect.pt1.y - fwnd->wnd.bounds.pt1.y);
  
  /* Perform the fill, clipping to the client window */
  nxgl_rectoffset(&relclip, &fwnd->fwrect, -fwnd->wnd.bounds.pt1.x, -fwnd->wnd.bounds.pt1.y);
  
  return nx_filltrapezoid((NXWINDOW)hfwnd, &relclip, &reltrap, color);
}
