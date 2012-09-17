/****************************************************************************
 * graphics/nxbe/nxbe_redraw.c
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

#include <errno.h>
#include <debug.h>

#include <nuttx/nx/nxglib.h>

#include "nxbe.h"
#include "nxfe.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nxbe_redraw_s
{
  struct nxbe_clipops_s cops;
  FAR struct nxbe_window_s *wnd;
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
 * Name: nxbe_clipredraw
 ****************************************************************************/

static void nxbe_clipredraw(FAR struct nxbe_clipops_s *cops,
                           FAR struct nxbe_plane_s *plane,
                           FAR const struct nxgl_rect_s *rect)
{
  FAR struct nxbe_window_s *wnd = ((struct nxbe_redraw_s *)cops)->wnd;
  if (wnd)
    {
      nxfe_redrawreq(wnd, rect);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_redraw
 *
 * Descripton:
 *   Re-draw the visible portions of the rectangular region for the
 *   specified window
 *
 ****************************************************************************/

void nxbe_redraw(FAR struct nxbe_state_s *be,
                 FAR struct nxbe_window_s *wnd,
                 FAR const struct nxgl_rect_s *rect)
{
  struct nxbe_redraw_s info;
  struct nxgl_rect_s remaining;
#if CONFIG_NX_NPLANES > 1
  int i;
#endif

  /* Clip to the limits of the window and of the background screen */

  nxgl_rectintersect(&remaining, rect, &be->bkgd.bounds);
  nxgl_rectintersect(&remaining, &remaining, &wnd->bounds);
  if (!nxgl_nullrect(&remaining))
    {
      /* Now, request to re-draw any visible rectangular regions not obscured
       * by windows above this one.
       */

      info.cops.visible  = nxbe_clipredraw;
      info.cops.obscured = nxbe_clipnull;
      info.wnd           = wnd;

#if CONFIG_NX_NPLANES > 1
      for (i = 0; i < be->vinfo.nplanes; i++)
        {
          nxbe_clipper(wnd->above, &remaining, NX_CLIPORDER_DEFAULT,
                       &info.cops, &be->plane[i]);
        }
#else
      nxbe_clipper(wnd->above, &remaining, NX_CLIPORDER_DEFAULT,
                   &info.cops, &be->plane[0]);
#endif
    }
}
