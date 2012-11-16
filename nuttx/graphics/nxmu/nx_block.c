/****************************************************************************
 * graphics/nxmu/nx_block.c
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

#include <errno.h>
#include <debug.h>

#include <nuttx/nx/nx.h>

#include "nxbe.h"
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
 * Name: nx_block
 *
 * Description:
 *   This is callback will do to things:  (1) any queue a 'blocked' callback
 *   to the window and then (2) block any further window messaging.
 *
 *   The 'blocked' callback is the response from nx_block (or nxtk_block).
 *   Those blocking interfaces are used to assure that no further messages are
 *   are directed to the window. Receipt of the blocked callback signifies
 *   that (1) there are no further pending callbacks and (2) that the
 *   window is now 'defunct' and will receive no further callbacks.
 *
 *   This callback supports coordinated destruction of a window in multi-
 *   user mode.  In multi-use mode, the client window logic must stay
 *   intact until all of the queued callbacks are processed.  Then the
 *   window may be safely closed.  Closing the window prior with pending
 *   callbacks can lead to bad behavior when the callback is executed.
 *
 *   Multiple user mode only!
 *
 * Input Parameters:
 *   wnd - The window to be blocked
 *   arg - An argument that will accompany the block messages (This is arg2
 *         in the blocked callback).
 *
 * Return:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nx_block(NXWINDOW hwnd, FAR void *arg)
{
  FAR struct nxbe_window_s *wnd = (FAR struct nxbe_window_s *)hwnd;
  struct nxsvrmsg_blocked_s outmsg;
  int ret = OK;

#ifdef CONFIG_DEBUG
  if (!hwnd)
    {
      set_errno(EINVAL);
      return ERROR;
    }
#endif

  /* Ignore additional attempts to block messages (no errors reported) */

  if (!NXBE_ISBLOCKED(wnd))
    {
      /* Mark the window as blocked.  This will stop all messages to the window
       * (EXCEPT the NX_SVRMSG_BLOCKED).  Blocking the messages before sending the
       * blocked message is awkward but assures that no other messages sneak into
       * the message queue before we can set the blocked state.
       */

      NXBE_SETBLOCKED(wnd);

      /* Send the message inicating that the window is blocked (and because of
       * queue also that there are no additional queue messages for the window)
       */

      outmsg.msgid = NX_SVRMSG_BLOCKED;
      outmsg.wnd   = wnd;
      outmsg.arg   = arg;

      /* Send the window message via nxmu_sendserver (vs. nxmu_sendwindow) so
       * that it will not be blocked.
       */

      ret = nxmu_sendserver(wnd->conn, &outmsg, sizeof(struct nxsvrmsg_blocked_s));
    }

  return ret;
}

