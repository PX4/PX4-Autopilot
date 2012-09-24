/****************************************************************************
 * graphics/nxsu/nx_requestbkgd.c
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

#include <errno.h>
#include <debug.h>

#include <nuttx/nx/nx.h>
#include "nxfe.h"

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
 * Name: nx_requestbkgd
 *
 * Description:
 *   NX normally controls a separate window called the background window.
 *   It repaints the window as necessary using only a solid color fill.  The
 *   background window always represents the entire screen and is always
 *   below other windows.  It is useful for an application to control the
 *   background window in the following conditions:
 *
 *   - If you want to implement a windowless solution.  The single screen
 *     can be used to creat a truly simple graphic environment.  In this
 *     case, you should probably also de-select CONFIG_NX_MULTIUSER as well.
 *   - When you want more on the background than a solid color.  For
 *     example, if you want an image in the background, or animations in the
 *     background, or live video, etc.
 *
 *   This API only requests the handle of the background window.  That
 *   handle will be returned asynchronously in a subsequent position and
 *   redraw callbacks.
 *
 *   Cautions:
 *   - The following should never be called using the background window.
 *     They are guaranteed to cause severe crashes:
 *
 *       nx_setposition, nx_setsize, nx_raise, nx_lower.
 *
 *   - Neither nx_opengbwindow or nx_closebgwindow should be called more than
 *     once.  Multiple instances of the background window are not supported.
 *
 * Input Parameters:
 *   handle - The handle returned by nx_connect
 *   cb     - Callbacks to use for processing background window events
 *   arg    - User provided argument (see nx_openwindow, nx_constructwindow)
 *
 * Return:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nx_requestbkgd(NXHANDLE handle, FAR const struct nx_callback_s *cb,
                   FAR void *arg)
{
  FAR struct nxfe_state_s *fe = (FAR struct nxfe_state_s *)handle;
  FAR struct nxbe_state_s *be = &fe->be;

#ifdef CONFIG_DEBUG
  if (!fe || !cb)
    {
      errno = EINVAL;
      return ERROR;
    }
#endif

  /* Replace the NX background windo callbacks with the client's callbacks */

  be->bkgd.cb  = cb;
  be->bkgd.arg = arg;

  /* Report the size/position of the background window to the client */

  nxfe_reportposition((NXWINDOW)&be->bkgd);

  /* Redraw the background window */

  nxfe_redrawreq(&be->bkgd, &be->bkgd.bounds);

  /* Provide the mouse settings to the client */

#ifdef CONFIG_NX_MOUSE
  nxsu_mousereport(&be->bkgd);
#endif

  /* In this single-user mode, we could return the background window
   * handle here.  However, we cannot do that in the multi-user case
   * because that handle is known only to the server.  Instead, the
   * background window handle is returned to the client via a redraw
   * callback.  So we will do the same in the single-user case for
   * compatibility.
   */

  return OK;
}

