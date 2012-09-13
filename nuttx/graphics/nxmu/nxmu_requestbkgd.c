/****************************************************************************
 * graphics/nxmu/nxmu_requestbkgd.c
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
 * Name: nxmu_requestbkgd
 *
 * Description:
 *   Perform the server-side operation for the nx_requestbkgd operation:
 *   Give the client control of the background window connection and receipt
 *   of all background window callbacks.
 *
 *   conn - The client containing connection information [IN]
 *   be   - The server state structure [IN]
 *   cb   - Callbacks used to process window events
 *   arg  - User provided argument (see nx_openwindow, nx_constructwindow)
 *
 * Return:
 *   None
 *
 ****************************************************************************/

void nxmu_requestbkgd(FAR struct nxfe_conn_s *conn,
                      FAR struct nxbe_state_s *be,
                      FAR const struct nx_callback_s *cb,
                      FAR void *arg)
{
#ifdef CONFIG_DEBUG
  if (!conn || !be || !cb)
    {
      errno = EINVAL;
    }
#endif

  /* Set the client's callback vtable and and replace the server
   * connection with the clients connection.
   */

  be->bkgd.cb   = cb;
  be->bkgd.arg  = arg;
  be->bkgd.conn = conn;

  /* Report the size/position of the background window to the client */

  nxfe_reportposition((NXWINDOW)&be->bkgd);

  /* Redraw the background window */

  nxfe_redrawreq(&be->bkgd, &be->bkgd.bounds);

  /* Provide the mouse settings */

#ifdef CONFIG_NX_MOUSE
  nxmu_mousereport(&be->bkgd);
#endif
}

