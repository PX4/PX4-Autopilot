/****************************************************************************
 * graphics/nxtk/nxtk_internal.h
 *
 *   Copyright (C) 2008-2009, 2011-1021 Gregory Nutt. All rights reserved.
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

#ifndef __GRAPHICS_NXTK_NXTK_INTERNAL_H
#define __GRAPHICS_NXTK_NXTK_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/nx/nxtk.h>
#include "nxbe.h"
#include "nxfe.h"

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This is the internal representation of the framed window object */

struct nxtk_framedwindow_s
{
  struct nxbe_window_s wnd;      /* The raw NX window */

  /* The toolbar region and callbacks */

  nxgl_coord_t tbheight;
  struct nxgl_rect_s tbrect;
  FAR const struct nx_callback_s *tbcb;
  FAR void *tbarg;

  /* Window data region and callbacks */

  struct nxgl_rect_s fwrect;
  FAR const struct nx_callback_s *fwcb;
  FAR void *fwarg;

  /* Initial mouse down location */

  uint8_t mbutton;
  struct nxgl_point_s mpos;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
# define EXTERN extern "C"
extern "C" {
#else
# define EXTERN extern
#endif

/* That is the callback for the framed window */

extern FAR const struct nx_callback_s g_nxtkcb;

/* Frame border colors */

extern nxgl_mxpixel_t g_bordercolor1[CONFIG_NX_NPLANES];
extern nxgl_mxpixel_t g_bordercolor2[CONFIG_NX_NPLANES];
extern nxgl_mxpixel_t g_bordercolor3[CONFIG_NX_NPLANES];

/****************************************************************************
 * Public Function Prototypes
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

EXTERN void nxtk_setsubwindows(FAR struct nxtk_framedwindow_s *fwnd);

/****************************************************************************
 * Name: nxtk_subwindowclip
 *
 * Description:
 *   Clip the src rectangle so that it lies within the sub-window bounds
 *   then move the rectangle to that it is relative to the containing
 *   window.
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

EXTERN void nxtk_subwindowclip(FAR struct nxtk_framedwindow_s *fwnd,
                               FAR struct nxgl_rect_s *dest,
                               FAR const struct nxgl_rect_s *src,
                               FAR const struct nxgl_rect_s *bounds);

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

EXTERN void nxtk_containerclip(FAR struct nxtk_framedwindow_s *fwnd,
                               FAR struct nxgl_rect_s *dest,
                               FAR const struct nxgl_rect_s *src,
                              FAR const struct nxgl_rect_s *bounds);

/****************************************************************************
 * Name: nxtk_subwindowmove
 *
 * Description:
 *   Perform common clipping operations in preparatons for calling nx_move()
 *
 * Input Parameters:
 *   fwnd       - The framed window within which the move is to be done.
 *                This must have been previously created by nxtk_openwindow().
 *   destrect   - The loccation to receive the clipped rectangle relative
 *                to containing window
 *   destoffset - The location to received the clipped offset.
 *   srcrect    - Describes the rectangular region relative to the client
 *                sub-window to move relative to the sub-window
 *   srcoffset  - The offset to move the region
 *   bounds     - The subwindow bounds in absolute screen coordinates.
 *
 * Return:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

EXTERN void nxtk_subwindowmove(FAR struct nxtk_framedwindow_s *fwnd,
                               FAR struct nxgl_rect_s *destrect,
                               FAR struct nxgl_point_s *destoffset,
                               FAR const struct nxgl_rect_s *srcrect,
                               FAR const struct nxgl_point_s *srcoffset,
                               FAR const struct nxgl_rect_s *bounds);

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

EXTERN int nxtk_drawframe(FAR struct nxtk_framedwindow_s *fwnd,
                          FAR const struct nxgl_rect_s *bounds);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __GRAPHICS_NXTK_NXTK_INTERNAL_H */
