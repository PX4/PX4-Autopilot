/****************************************************************************
 * nuttx/graphics/nxconsole/nxtk_register.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

#include <nuttx/nx/nxtk.h>
#include <nuttx/nx/nxconsole.h>

#include "nxcon_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int nxtkcon_fill(FAR struct nxcon_state_s *priv,
                        FAR const struct nxgl_rect_s *rect,
                        nxgl_mxpixel_t wcolor[CONFIG_NX_NPLANES]);
#ifndef CONFIG_NX_WRITEONLY
static int nxtkcon_move(FAR struct nxcon_state_s *priv,
                        FAR const struct nxgl_rect_s *rect,
                        FAR const struct nxgl_point_s *offset);
#endif
static int nxtkcon_bitmap(FAR struct nxcon_state_s *priv,
                          FAR const struct nxgl_rect_s *dest,
                          FAR const void *src[CONFIG_NX_NPLANES],
                          FAR const struct nxgl_point_s *origin,
                          unsigned int stride);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct nxcon_operations_s g_nxtkops =
{
  nxtkcon_fill,
#ifndef CONFIG_NX_WRITEONLY
  nxtkcon_move,
#endif
  nxtkcon_bitmap
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxtkcon_fill
 *
 * Description:
 *  Fill the specified rectangle in the window with the specified color
 *
 * Input Parameters:
 *   priv   - The driver state structure.
 *   rect  - The location to be filled
 *   color - The color to use in the fill
 *
 * Return:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

static int nxtkcon_fill(FAR struct nxcon_state_s *priv,
                        FAR const struct nxgl_rect_s *rect,
                        nxgl_mxpixel_t wcolor[CONFIG_NX_NPLANES])
{
  return nxtk_fillwindow((NXTKWINDOW)priv->handle, rect, wcolor);
}

/****************************************************************************
 * Name: nxtkcon_move
 *
 * Description:
 *   Move a rectangular region within the window
 *
 * Input Parameters:
 *   priv   - The driver state structure.
 *   rect   - Describes the rectangular region to move
 *   offset - The offset to move the region.  The  rectangular region will be
 *            moved so that the origin is translated by this amount.
 *
 * Return:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

#ifndef CONFIG_NX_WRITEONLY
static int nxtkcon_move(FAR struct nxcon_state_s *priv,
                        FAR const struct nxgl_rect_s *rect,
                        FAR const struct nxgl_point_s *offset)
{
  return nxtk_movewindow((NXTKWINDOW)priv->handle, rect, offset);
}
#endif

/****************************************************************************
 * Name: nxtkcon_bitmap
 *
 * Description:
 *   Copy a rectangular region of a larger image into the rectangle in the
 *   specified window.
 *
 * Input Parameters:
 *   priv   - The driver state structure.
 *   dest   - Describes the rectangular region on the display that will
 *            receive the bit map.
 *   src    - The start of the source image.  This is an array source
 *            images of size CONFIG_NX_NPLANES.
 *   origin - The origin of the upper, left-most corner of the full bitmap.
 *            Both dest and origin are in window coordinates, however, origin
 *            may lie outside of the display.
 *   stride - The width of the full source image in bytes.
 *
 * Return:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

static int nxtkcon_bitmap(FAR struct nxcon_state_s *priv,
                        FAR const struct nxgl_rect_s *dest,
                        FAR const void *src[CONFIG_NX_NPLANES],
                        FAR const struct nxgl_point_s *origin,
                        unsigned int stride)
{
  return nxtk_bitmapwindow((NXTKWINDOW)priv->handle, dest, src, origin, stride);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxtk_register
 *
 * Description:
 *   Register a console device on a framed NX window.  The device will be
 *   registered at /dev/nxtkN where N is the provided minor number.
 *
 * Input Parameters:
 *   hfwnd - A handle that will be used to access the window.  The window must
 *     persist and this handle must be valid for the life of the NX console.
 *   wndo - Describes the window and font to be used
 *   minor - The device minor number
 *
 * Return:
 *   A non-NULL handle is returned on success. 
 *
 ****************************************************************************/

NXCONSOLE nxtk_register(NXTKWINDOW hfwnd, FAR struct nxcon_window_s *wndo, int minor)
{
  return nxcon_register((NXCONSOLE)hfwnd, wndo, &g_nxtkops, minor);
}
