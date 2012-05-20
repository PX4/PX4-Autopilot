/****************************************************************************
 * graphics/nxtk/nxtk_block.c
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

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxtk.h>

#include "nxfe.h"

#ifdef CONFIG_NX_MULTIUSER

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
 * Name: nxtk_block
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
 *   hfwnd - The window to be blocked
 *   arg - An argument that will accompany the block messages (This is arg2
 *         in the blocked callback).
 *
 * Return:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_block(NXTKWINDOW hfwnd, FAR void *arg)
{
  return nx_block((NXWINDOW)hfwnd, arg);
}

#endif /* CONFIG_NX_MULTIUSER */
