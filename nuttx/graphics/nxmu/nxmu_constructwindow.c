/****************************************************************************
 * graphics/nxmu/nx_openwindow.c
 *
 *   Copyright (C) 2008, 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * Name: nxfe_constructwindow
 *
 * Description:
 *   This function is the same a nx_openwindow EXCEPT that the client provides
 *   the window structure instance.  nx_constructwindow will initialize the
 *   the pre-allocated window structure for use by NX.  This function is
 *   provided in addition to nx_open window in order to support a kind of
 *   inheritance:  The caller's window structure may include extensions that
 *   are not visible to NX.
 *
 *   NOTE:  wnd must have been allocated using malloc() (or related allocators)
 *   Once provided to nxfe_constructwindow() that memory is owned and managed
 *   by NX.  On certain error conditions or when the window is closed, NX will
 *   free() the window.
 *
 * Input Parameters:
 *   handle - The handle returned by nx_connect
 *   wnd    - The pre-allocated window structure.
 *   cb     - Callbacks used to process window events
 *   arg    - User provided value that will be returned with NX callbacks.
 *
 * Return:
 *   OK on success; ERROR on failure with errno set appropriately.  In the
 *   case of ERROR, NX will have dealloated the pre-allocated window.
 *
 ****************************************************************************/

int nxfe_constructwindow(NXHANDLE handle, FAR struct nxbe_window_s *wnd,
                         FAR const struct nx_callback_s *cb, FAR void *arg)
{
  FAR struct nxfe_conn_s *conn = (FAR struct nxfe_conn_s *)handle;
  struct nxsvrmsg_openwindow_s outmsg;
  int ret;

#ifdef CONFIG_DEBUG
  if (!wnd)
    {
      errno = EINVAL;
      return ERROR;
    }

  if (!conn || !cb)
    {
      free(wnd);
      errno = EINVAL;
      return ERROR;
    }
#endif

  /* Setup only the connection structure, callbacks and client private data
   * reference. The server will set everything else up.
   */

  wnd->conn   = conn;
  wnd->cb     = cb;
  wnd->arg    = arg;

  /* Request initialization the new window from the server */

  outmsg.msgid = NX_SVRMSG_OPENWINDOW;
  outmsg.wnd   = wnd;

  ret = mq_send(conn->cwrmq, &outmsg, sizeof(struct nxsvrmsg_openwindow_s), NX_SVRMSG_PRIO);
  if (ret < 0)
    {
      gdbg("mq_send failed: %d\n", errno);
      free(wnd);
      return ERROR;
    }

  return OK;
}

