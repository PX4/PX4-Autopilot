/****************************************************************************
 * graphics/nxsu/nx_open.c
 *
 *   Copyright (C) 2008-2010 Gregory Nutt. All rights reserved.
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

#include <stdbool.h>
#include <stdio.h>
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
 * Private Function Prototypes
 ****************************************************************************/

static void nxsu_bkgdredraw(NXWINDOW hwnd,
                            FAR const struct nxgl_rect_s *rect,
                            bool more, FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct nx_callback_s g_bkgdcb =
{
  nxsu_bkgdredraw,   /* redraw */
  NULL               /* position */
#ifdef CONFIG_NX_MOUSE
  , NULL             /* mousein */
#endif
#ifdef CONFIG_NX_KBD
  , NULL             /* my kbdin */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsu_bkgdredraw
 ****************************************************************************/

static void nxsu_bkgdredraw(NXWINDOW hwnd,
                            FAR const struct nxgl_rect_s *rect,
                            bool more, FAR void *arg)
{
  FAR struct nxbe_window_s *wnd = (FAR struct nxbe_window_s *)hwnd;
  FAR struct nxbe_state_s  *be  = wnd->be;

  gvdbg("BG redraw rect={(%d,%d),(%d,%d)}\n",
        rect->pt1.x, rect->pt1.y, rect->pt2.x, rect->pt2.y);
  nxbe_fill(wnd, &wnd->bounds, be->bgcolor);
}

/****************************************************************************
 * Name: nxsu_setup
 ****************************************************************************/

static inline int nxsu_setup(FAR NX_DRIVERTYPE *dev,
                             FAR struct nxfe_state_s *fe)
{
  int ret;

  /* Configure the framebuffer device */

  ret = nxbe_configure(dev, &fe->be);
  if (ret < 0)
    {
      gdbg("nxbe_configure failed: %d\n", -ret);
      errno = -ret;
      return ERROR;
    }

#if CONFIG_FB_CMAP
  ret = nxbe_colormap(dev);
  if (ret < 0)
    {
      gdbg("nxbe_colormap failed: %d\n", -ret);
      errno = -ret;
      return ERROR;
    }
#endif

  /* Initialize the non-NULL elements of the back-end structure window */
  /* Initialize the background window */

  fe->be.bkgd.be = &fe->be;
  fe->be.bkgd.cb = &g_bkgdcb;

  fe->be.bkgd.bounds.pt2.x = fe->be.vinfo.xres - 1;
  fe->be.bkgd.bounds.pt2.y = fe->be.vinfo.yres - 1;

  /* Complete initialization of the server state structure.  The
   * window list contains only one element:  The background window
   * with nothing else above or below it
   */

  fe->be.topwnd = &fe->be.bkgd;

 /* Initialize the mouse position */

#ifdef CONFIG_NX_MOUSE
  nxsu_mouseinit(fe->be.vinfo.xres, fe->be.vinfo.yres);
#endif
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_open
 *
 * Description:
 *   Create, initialize and return an NX handle for use in subsequent
 *   NX API calls.  nx_open is the single user equivalent of nx_connect
 *   plus nx_run.
 *
 * Input Parameters:
 *   dev - Vtable "object" of the framebuffer/LCD "driver" to use
 *
 * Return:
 *   Success: A non-NULL handle used with subsequent NX accesses
 *   Failure:  NULL is returned and errno is set appropriately
 *
 ****************************************************************************/

NXHANDLE nx_open(FAR NX_DRIVERTYPE *dev)
{
  FAR struct nxfe_state_s *fe;
  int ret;

  /* Sanity checking */

#ifdef CONFIG_DEBUG
  if (!dev)
    {
      errno = EINVAL;
      return NULL;
    }
#endif

  /* Allocate the NX state structure */

  fe = (FAR struct nxfe_state_s *)zalloc(sizeof(struct nxfe_state_s));
  if (!fe)
    {
      errno = ENOMEM;
      return NULL;
    }

  /* Initialize and configure the server */

  ret = nxsu_setup(dev, fe);
  if (ret < 0)
    {
      return NULL; /* nxsu_setup sets errno */
    }

  /* Fill the initial background window */

  nxbe_fill(&fe->be.bkgd, &fe->be.bkgd.bounds, fe->be.bgcolor);
  return (NXHANDLE)fe;
}

