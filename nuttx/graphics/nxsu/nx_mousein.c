/****************************************************************************
 * graphics/nxsu/nx_mousein.c
 *
 *   Copyright (C) 2008-2009, 2011-2012 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nx.h>

#include "nxfe.h"

#ifdef CONFIG_NX_MOUSE

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct nxgl_point_s   g_mpos;
static struct nxgl_point_s   g_mrange;
static uint8_t               g_mbutton;
static struct nxbe_window_s *g_mwnd;

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
 * Name: nxsu_mouseinit
 *
 * Description:
 *   Initialize with the mouse in the center of the display
 *
 ****************************************************************************/

void nxsu_mouseinit(int x, int y)
{
  g_mrange.x = x;
  g_mrange.y = y;
  g_mpos.x   = x / 2;
  g_mpos.y   = y / 2;
  g_mbutton  = 0;
}

/****************************************************************************
 * Name: nxsu_mousereport
 *
 * Description:
 *   Report mouse position info to the specified window
 *
 * Input Parameters:
 *   wnd - The window to receive the mouse report
 *
 * Returned Value:
 *   0: Mouse report sent; >0: Mouse report not sent; <0: An error occurred
 *
 ****************************************************************************/

int nxsu_mousereport(struct nxbe_window_s *wnd)
{
  struct nxgl_point_s relpos;

  /* Does this window support mouse callbacks? */

  if (wnd->cb->mousein)
    {
      /* Yes.. Is the mouse position visible in this window? */

      if (nxbe_visible(wnd, &g_mpos))
        {
          /* Yes... Convert the mouse position to window relative coordinates */

          nxgl_vectsubtract(&relpos, &g_mpos, &wnd->bounds.pt1);
          wnd->cb->mousein((NXWINDOW)wnd, &relpos, g_mbutton, wnd->arg);
          return OK;
        }
    }

  /* No error occurred, but the mouse report was not sent */

  return 1;
}

/****************************************************************************
 * Name: nx_mousein
 *
 * Description:
 *   Used by a thread or interrupt handler that manages some kind of pointing
 *   hardware to report new positional data to the NX server.  That positional
 *   data will be routed by the NX server to the appropriate window client.
 *
 ****************************************************************************/

int nx_mousein(NXHANDLE handle, nxgl_coord_t x, nxgl_coord_t y, uint8_t buttons)
{
  FAR struct nxfe_state_s *fe = (FAR struct nxfe_state_s *)handle;
  struct nxbe_window_s *wnd;
  uint8_t oldbuttons;
  int ret;

  /* Clip x and y to within the bounding rectangle */

  if (x < 0)
    {
      x = 0;
    }
  else if (x >= g_mrange.x)
    {
      x = g_mrange.x - 1;
    }

  if (y < 0)
    {
      y = 0;
    }
  else if (y >= g_mrange.y)
    {
      y = g_mrange.y - 1;
    }

 /* Look any change in values */

  if (x != g_mpos.x || y != g_mpos.y || buttons != g_mbutton)
    {
      /* Update the mouse value */

      oldbuttons = g_mbutton;
      g_mpos.x  = x;
      g_mpos.y  = y;
      g_mbutton = buttons;

      /* If a button is already down, regard this as part of a mouse drag
       * event. Pass all the following events to the window where the drag
       * started in.
       */

      if (oldbuttons && g_mwnd && g_mwnd->cb->mousein)
        {
          struct nxgl_point_s relpos;
          nxgl_vectsubtract(&relpos, &g_mpos, &g_mwnd->bounds.pt1);
          g_mwnd->cb->mousein((NXWINDOW)g_mwnd, &relpos, g_mbutton, g_mwnd->arg);
          return OK;
        }

      /* Pick the window to receive the mouse event.  Start with the top
       * window and go down.  Step with the first window that gets the mouse
       * report
       */

      for (wnd = fe->be.topwnd; wnd; wnd = wnd->below)
        {
          ret = nxsu_mousereport(wnd);
          if (ret == 0)
            {
              break;
            }
        }

      g_mwnd = wnd;
    }
  return OK;
}

#endif /* CONFIG_NX_MOUSE */
