/****************************************************************************
 * graphics/nxtk/nxtk_openwindow.c
 *
 *   Copyright (C) 2008-2009, 2012 Gregory Nutt. All rights reserved.
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

#include <stdlib.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/nx/nx.h>
#include <nuttx/kmalloc.h>

#include "nxfe.h"
#include "nxtk_internal.h"

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

nxgl_mxpixel_t g_bordercolor1[CONFIG_NX_NPLANES] =
{
  CONFIG_NXTK_BORDERCOLOR1
#if CONFIG_NX_NPLANES > 1
#  error "Multiple plane border colors not defined"
#endif
};

nxgl_mxpixel_t g_bordercolor2[CONFIG_NX_NPLANES] =
{
  CONFIG_NXTK_BORDERCOLOR2
#if CONFIG_NX_NPLANES > 1
#  error "Multiple plane border colors not defined"
#endif
};

nxgl_mxpixel_t g_bordercolor3[CONFIG_NX_NPLANES] =
{
  CONFIG_NXTK_BORDERCOLOR3
#if CONFIG_NX_NPLANES > 1
#  error "Multiple plane border colors not defined"
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxtk_openwindow
 *
 * Description:
 *   Create a new, framed window.
 *
 * Input Parameters:
 *   handle - The handle returned by nx_connect
 *   cb     - Callbacks used to process window events
 *   arg    - User provided value that will be returned with NXTK callbacks.
 *
 * Return:
 *   Success: A non-NULL handle used with subsequent NXTK window accesses
 *   Failure:  NULL is returned and errno is set appropriately
 *
 ****************************************************************************/

NXTKWINDOW nxtk_openwindow(NXHANDLE handle,
                           FAR const struct nx_callback_s *cb,
                           FAR void *arg)
{
  FAR struct nxtk_framedwindow_s *fwnd;
  int ret;

#ifdef CONFIG_DEBUG
  if (!handle || !cb)
    {
      errno = EINVAL;
      return NULL;
    }
#endif

  /* Pre-allocate the window structure */

  fwnd = (FAR struct nxtk_framedwindow_s *)kzalloc(sizeof(struct nxtk_framedwindow_s));
  if (!fwnd)
    {
      errno = ENOMEM;
      return NULL;
    }

  /* Initialize the window structure */

  fwnd->fwcb  = cb;
  fwnd->fwarg = arg;

  /* Then let nxfe_constructwindow do the rest */

  ret = nxfe_constructwindow(handle, &fwnd->wnd, &g_nxtkcb, NULL);
  if (ret < 0)
    {
      /* An error occurred, the window has been freed */

      return NULL;
    }

  /* Return the initialized window reference */

  return (NXTKWINDOW)fwnd;
}

