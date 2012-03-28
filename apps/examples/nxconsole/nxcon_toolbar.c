/****************************************************************************
 * examples/nxconsole/nxcon_toolbar.c
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

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxfonts.h>

#include "nxcon_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void nxtool_redraw(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                          bool morem, FAR void *arg);
static void nxtool_position(NXWINDOW hwnd, FAR const struct nxgl_size_s *size,
                            FAR const struct nxgl_point_s *pos,
                            FAR const struct nxgl_rect_s *bounds,
                            FAR void *arg);
#ifdef CONFIG_NX_MOUSE
static void nxtool_mousein(NXWINDOW hwnd, FAR const struct nxgl_point_s *pos,
                           uint8_t buttons, FAR void *arg);
#endif

#ifdef CONFIG_NX_KBD
static void nxtool_kbdin(NXWINDOW hwnd, uint8_t nch, FAR const uint8_t *ch,
                         FAR void *arg);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Background window call table */

const struct nx_callback_s g_nxtoolcb =
{
  nxtool_redraw,   /* redraw */
  nxtool_position  /* position */
#ifdef CONFIG_NX_MOUSE
  , nxtool_mousein /* mousein */
#endif
#ifdef CONFIG_NX_KBD
  , nxtool_kbdin   /* my kbdin */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxtool_redraw
 ****************************************************************************/

static void nxtool_redraw(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                          bool more, FAR void *arg)
{
  nxgl_mxpixel_t color[CONFIG_NX_NPLANES];
  int ret;

  gvdbg("hwnd=%p rect={(%d,%d),(%d,%d)} more=%s\n",
         hwnd, rect->pt1.x, rect->pt1.y, rect->pt2.x, rect->pt2.y,
         more ? "true" : "false");

  color[0] = CONFIG_EXAMPLES_NXCON_TBCOLOR;
  ret = nxtk_filltoolbar(hwnd, rect, color);
  if (ret < 0)
    {
      gdbg("nxtk_filltoolbar failed: %d\n", errno);
    }
}

/****************************************************************************
 * Name: nxtool_position
 ****************************************************************************/

static void nxtool_position(NXWINDOW hwnd, FAR const struct nxgl_size_s *size,
                            FAR const struct nxgl_point_s *pos,
                            FAR const struct nxgl_rect_s *bounds,
                            FAR void *arg)
{
  gvdbg("hwnd=%p size=(%d,%d) pos=(%d,%d) bounds={(%d,%d),(%d,%d)}\n",
        hwnd, size->w, size->h, pos->x, pos->y,
        bounds->pt1.x, bounds->pt1.y, bounds->pt2.x, bounds->pt2.y);
}

/****************************************************************************
 * Name: nxtool_mousein
 ****************************************************************************/

#ifdef CONFIG_NX_MOUSE
static void nxtool_mousein(NXWINDOW hwnd, FAR const struct nxgl_point_s *pos,
                           uint8_t buttons, FAR void *arg)
{
  gvdbg("hwnd=%p pos=(%d,%d) button=%02x\n", hwnd,  pos->x, pos->y, buttons);
}
#endif

/****************************************************************************
 * Name: nxtool_kbdin
 ****************************************************************************/

#ifdef CONFIG_NX_KBD
static void nxtool_kbdin(NXWINDOW hwnd, uint8_t nch, FAR const uint8_t *ch,
                         FAR void *arg)
{
  gvdbg("hwnd=%p nch=%d\n", hwnd, nch);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/
