/****************************************************************************
 * graphics/nxtk/nxtk_setsubwindows.c
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
#include <semaphore.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/nx/nx.h>
#include "nxtk_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
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
 * Name: nxtk_setsubwindows
 *
 * Description:
 *   Give the window dimensions, border width, and toolbar height,
 *   calculate the new dimensions of the toolbar region and client window
 *   region
 *
 ****************************************************************************/

void nxtk_setsubwindows(FAR struct nxtk_framedwindow_s *fwnd)
{
  nxgl_coord_t fullheight;
  nxgl_coord_t bdrheight = 0;
  nxgl_coord_t tbtop     = fwnd->wnd.bounds.pt1.y;
  nxgl_coord_t tbheight  = 0;
  nxgl_coord_t fwtop     = fwnd->wnd.bounds.pt1.y;
  nxgl_coord_t fwheight  = 0;
  nxgl_coord_t fullwidth;
  nxgl_coord_t bdrwidth;
  nxgl_coord_t fwwidth;
  nxgl_coord_t fwleft;

  /* Divide up the vertical dimension of the window */

  fullheight = fwnd->wnd.bounds.pt2.y - fwnd->wnd.bounds.pt1.y + 1;

  /* Is it tall enough for a border? */

  if (fullheight > 0)
    {
      /* Get the border height */

      bdrheight = ngl_min(2 * CONFIG_NXTK_BORDERWIDTH, fullheight);

      /* Position the toolbar and client window just under the top border */

#if CONFIG_NXTK_BORDERWIDTH > 1
      tbtop += CONFIG_NXTK_BORDERWIDTH - 1;
      fwtop = tbtop + 1;
#else
      tbtop += CONFIG_NXTK_BORDERWIDTH;
      fwtop = tbtop;
#endif

      /* Is it big enough for any part of the toolbar? */

      if (fullheight > 2 * CONFIG_NXTK_BORDERWIDTH)
        {
           /* Yes.. get the height of the toolbar */

          tbheight  = fwnd->tbheight;
          if (tbheight >= fullheight - bdrheight)
            {
              tbheight = fullheight - bdrheight;
            }
          else
            {
              /* And the client window gets whatever is left */

              fwheight = fullheight - bdrheight - tbheight;
            }

          /* Position the client window just under the toolbar */

          fwtop += tbheight;
        }
    }

  /* Divide up the horizontal dimensions of the window */

  fullwidth = fwnd->wnd.bounds.pt2.x - fwnd->wnd.bounds.pt1.x + 1;
  bdrwidth  = ngl_min(2 * CONFIG_NXTK_BORDERWIDTH, fullwidth);
  fwwidth   = fullwidth - bdrwidth;
  fwleft    = fwnd->wnd.bounds.pt1.x + bdrwidth / 2;

  /* Realize the positions/dimensions */

  fwnd->tbrect.pt1.x = fwleft;
  fwnd->tbrect.pt1.y = tbtop;
  fwnd->tbrect.pt2.x = fwleft + fwwidth - 1;
  fwnd->tbrect.pt2.y = tbtop + tbheight - 1;

  fwnd->fwrect.pt1.x = fwleft;
  fwnd->fwrect.pt1.y = fwtop;
  fwnd->fwrect.pt2.x = fwleft + fwwidth - 1;
  fwnd->fwrect.pt2.y = fwtop + fwheight - 1;
}
