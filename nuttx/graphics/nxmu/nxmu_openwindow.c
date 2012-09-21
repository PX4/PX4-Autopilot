/****************************************************************************
 * graphics/nxmu/nxmu_openwindow.c
 *
 *   Copyright (C) 2008-2011 Gregory Nutt. All rights reserved.
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
 * Name: nxmu_openwindow
 *
 * Description:
 *   Create a new window.
 *
 * Input Parameters:
 *   be  - The back-end status structure
 *   wnd  - The pre-allocated window structure to be initialized [IN/OUT]
 *
 * Return:
 *   None
 *
 ****************************************************************************/

void nxmu_openwindow(FAR struct nxbe_state_s *be, FAR struct nxbe_window_s *wnd)
{
  /* The window structure was allocated in nx_openwindow and all fields have
   * been set to zero cb and conn which were initialized on the client side.
   * On the server side, we need only initialize a few more the non zero fields
   * and insert the new window at the top of the display.
   */

  wnd->be   = be;

  /* Now, insert the new window at the top on the display.  topwind is
   * never NULL (it may point only at the background window, however)
   */

  wnd->above        = NULL;
  wnd->below        = be->topwnd;

  be->topwnd->above = wnd;
  be->topwnd        = wnd;

  /* Report the initial size/position of the window to the client */

  nxfe_reportposition(wnd);

  /* Provide the initial mouse settings to the client */

#ifdef CONFIG_NX_MOUSE
  nxmu_mousereport(wnd);
#endif
}
