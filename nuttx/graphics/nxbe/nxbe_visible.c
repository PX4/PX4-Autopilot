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

#include <stdbool.h>
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

struct nxbe_visible_s
{
  struct nxbe_clipops_s cops;
  bool visible;
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
 * Name: nxbe_clipvisible
 ****************************************************************************/

static void nxbe_clipvisible(FAR struct nxbe_clipops_s *cops,
                             FAR struct nxbe_plane_s *plane,
                             FAR const struct nxgl_rect_s *rect)
{
  FAR struct nxbe_visible_s *info = (FAR struct nxbe_visible_s *)cops;
  info->visible = true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_visible
 *
 * Descripton:
 *   Return true if the point, pt, in window wnd is visible.  pt is in
 *   absolute screen coordinates
 *
 ****************************************************************************/

bool nxbe_visible(FAR struct nxbe_window_s *wnd,
                  FAR const struct nxgl_point_s *pos)
{
  struct nxbe_visible_s info;

  /* Check if the absolute position lies within the window */

  if (!nxgl_rectinside(&wnd->bounds, pos))
    {
      return false;
    }

  /* If this is the top window, then the psition is visible */

  if (!wnd->above)
    {
      return true;
    }

  /* The position within the window range, but the window is not at
   * the top.  We will have to work harder to determine if the point
   * visible
   */

  info.cops.visible  = nxbe_clipvisible;
  info.cops.obscured = nxbe_clipnull;
  info.visible       = false;

  nxbe_clipper(wnd->above, &wnd->bounds, NX_CLIPORDER_DEFAULT,
               &info.cops, &wnd->be->plane[0]);

  return info.visible;
}
