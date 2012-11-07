/****************************************************************************
 * examples/nxlines/nxlines_bkgd.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <debug.h>
#include <fixedmath.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>

#include "nxlines.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

#ifndef MIN
#  define MIN(a,b) (a < b ? a : b)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void nxlines_redraw(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                        bool morem, FAR void *arg);
static void nxlines_position(NXWINDOW hwnd, FAR const struct nxgl_size_s *size,
                          FAR const struct nxgl_point_s *pos,
                          FAR const struct nxgl_rect_s *bounds,
                          FAR void *arg);
#ifdef CONFIG_NX_MOUSE
static void nxlines_mousein(NXWINDOW hwnd, FAR const struct nxgl_point_s *pos,
                         uint8_t buttons, FAR void *arg);
#endif

#ifdef CONFIG_NX_KBD
static void nxlines_kbdin(NXWINDOW hwnd, uint8_t nch, FAR const uint8_t *ch,
                       FAR void *arg);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Background window call table */

const struct nx_callback_s g_nxlinescb =
{
  nxlines_redraw,   /* redraw */
  nxlines_position  /* position */
#ifdef CONFIG_NX_MOUSE
  , nxlines_mousein /* mousein */
#endif
#ifdef CONFIG_NX_KBD
  , nxlines_kbdin   /* my kbdin */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxlines_redraw
 ****************************************************************************/

static void nxlines_redraw(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                        bool more, FAR void *arg)
{
  gvdbg("hwnd=%p rect={(%d,%d),(%d,%d)} more=%s\n",
         hwnd, rect->pt1.x, rect->pt1.y, rect->pt2.x, rect->pt2.y,
         more ? "true" : "false");
}

/****************************************************************************
 * Name: nxlines_position
 ****************************************************************************/

static void nxlines_position(NXWINDOW hwnd, FAR const struct nxgl_size_s *size,
                          FAR const struct nxgl_point_s *pos,
                          FAR const struct nxgl_rect_s *bounds,
                          FAR void *arg)
{
  /* Report the position */

  gvdbg("hwnd=%p size=(%d,%d) pos=(%d,%d) bounds={(%d,%d),(%d,%d)}\n",
        hwnd, size->w, size->h, pos->x, pos->y,
        bounds->pt1.x, bounds->pt1.y, bounds->pt2.x, bounds->pt2.y);

  /* Have we picked off the window bounds yet? */

  if (!g_nxlines.havepos)
    {
      /* Save the background window handle */

      g_nxlines.hbkgd = hwnd;

      /* Save the window limits */

      g_nxlines.xres = bounds->pt2.x + 1;
      g_nxlines.yres = bounds->pt2.y + 1;

      g_nxlines.havepos = true;
      sem_post(&g_nxlines.sem);
      gvdbg("Have xres=%d yres=%d\n", g_nxlines.xres, g_nxlines.yres);
    }
}

/****************************************************************************
 * Name: nxlines_mousein
 ****************************************************************************/

#ifdef CONFIG_NX_MOUSE
static void nxlines_mousein(NXWINDOW hwnd, FAR const struct nxgl_point_s *pos,
                         uint8_t buttons, FAR void *arg)
{
  message("nxlines_mousein: hwnd=%p pos=(%d,%d) button=%02x\n",
          hwnd,  pos->x, pos->y, buttons);
}
#endif

/****************************************************************************
 * Name: nxlines_kbdin
 ****************************************************************************/

#ifdef CONFIG_NX_KBD
static void nxlines_kbdin(NXWINDOW hwnd, uint8_t nch, FAR const uint8_t *ch,
                       FAR void *arg)
{
  gvdbg("hwnd=%p nch=%d\n", hwnd, nch);

   /* In this example, there is no keyboard so a keyboard event is not
    * expected.
    */

   message("nxlines_kbdin: Unexpected keyboard callback\n");
}
#endif

 /****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxlines_test
 *
 * Description:
 *   Print "Hello, World!" in the center of the display.
 *
 ****************************************************************************/

void nxlines_test(NXWINDOW hwnd)
{
  struct nxgl_point_s center;
  struct nxgl_vector_s vector;
  struct nxgl_vector_s previous;
  nxgl_mxpixel_t color[CONFIG_NX_NPLANES];
  nxgl_coord_t maxradius;
  nxgl_coord_t radius;
  nxgl_coord_t halfx;
  nxgl_coord_t halfy;
  b16_t angle;
  int ret;

  /* Get the maximum radius and center of the circle */

  maxradius = MIN(g_nxlines.yres, g_nxlines.xres) >> 1;
  center.x  = g_nxlines.xres >> 1;
  center.y  = g_nxlines.yres >> 1;

  /* Draw a circular background */

  radius = maxradius - ((CONFIG_EXAMPLES_NXLINES_BORDERWIDTH+1)/2);
  color[0] = CONFIG_EXAMPLES_NXLINES_CIRCLECOLOR;
  ret = nx_fillcircle((NXWINDOW)hwnd, &center, radius, color);
  if (ret < 0)
    {
      message("nxlines_test: nx_fillcircle failed: %d\n", ret);
    }

  /* Draw the circular border */

  color[0] = CONFIG_EXAMPLES_NXLINES_BORDERCOLOR;
  ret = nx_drawcircle((NXWINDOW)hwnd, &center, radius,
                      CONFIG_EXAMPLES_NXLINES_BORDERWIDTH, color);
  if (ret < 0)
    {
      message("nxlines_test: nx_fillcircle failed: %d\n", ret);
    }

  /* Back off the radius to account for the thickness of border line
   * and with a big fudge factor that will (hopefully) prevent the corners
   * of the lines from overwriting the border.  This is overly complicated
   * here because we don't assume anything about the screen resolution or
   * the borderwidth or the line thickness (and there are certainly some
   * smarter ways to do this).
   */

  if (maxradius > (CONFIG_EXAMPLES_NXLINES_BORDERWIDTH + 80))
    {
      radius = maxradius - (CONFIG_EXAMPLES_NXLINES_BORDERWIDTH + 40);
    }
  else if (maxradius > (CONFIG_EXAMPLES_NXLINES_BORDERWIDTH + 60))
    {
      radius = maxradius - (CONFIG_EXAMPLES_NXLINES_BORDERWIDTH + 30);
    }
  else if (maxradius > (CONFIG_EXAMPLES_NXLINES_BORDERWIDTH + 40))
    {
      radius = maxradius - (CONFIG_EXAMPLES_NXLINES_BORDERWIDTH + 20);
    }
  else if (maxradius > (CONFIG_EXAMPLES_NXLINES_BORDERWIDTH + 20))
    {
      radius = maxradius - (CONFIG_EXAMPLES_NXLINES_BORDERWIDTH + 10);
    }
  else
    {
      radius = maxradius - CONFIG_EXAMPLES_NXLINES_BORDERWIDTH;
    }

  /* The loop, showing the line in various orientations */

  angle = 0;
  previous.pt1.x = center.x;
  previous.pt1.y = center.y;
  previous.pt2.x = center.x;
  previous.pt2.y = center.y;

  for (;;)
    {
      /* Determine the position of the line on this pass */

      halfx = b16toi(b16muli(b16sin(angle), radius));
      halfy = b16toi(b16muli(b16cos(angle), radius));

      vector.pt1.x = center.x + halfx;
      vector.pt1.y = center.y + halfy;
      vector.pt2.x = center.x - halfx;
      vector.pt2.y = center.y - halfy;

      message("Angle: %08x vector: (%d,%d)->(%d,%d)\n",
              angle, vector.pt1.x, vector.pt1.y, vector.pt2.x, vector.pt2.y);

      /* Clear the previous line by overwriting it with the circle color */

      color[0] = CONFIG_EXAMPLES_NXLINES_CIRCLECOLOR;
      ret = nx_drawline((NXWINDOW)hwnd, &previous, CONFIG_EXAMPLES_NXLINES_LINEWIDTH, color);
      if (ret < 0)
        {
          message("nxlines_test: nx_drawline failed clearing: %d\n", ret);
        }

      /* Draw the new line */

      color[0] = CONFIG_EXAMPLES_NXLINES_LINECOLOR;
      ret = nx_drawline((NXWINDOW)hwnd, &vector, CONFIG_EXAMPLES_NXLINES_LINEWIDTH, color);
      if (ret < 0)
        {
          message("nxlines_test: nx_drawline failed clearing: %d\n", ret);
        }

      /* Set up for the next time through the loop then sleep for a bit. */

      angle += b16PI / 16;  /* 32 angular positions in full circle */

      /* Check if we have gone all the way around */

      if (angle > (31 *  (2 * b16PI) / 32))
        {
#ifdef CONFIG_NSH_BUILTIN_APPS
          /* If this example was built as an NSH add-on, then exit after we
           * have gone all the way around once.
           */

          return;
#else
          /* Wrap back to zero and continue with the test */

          angle = 0;
#endif
        }

      memcpy(&previous, &vector, sizeof(struct nxgl_vector_s));
      usleep(500*1000);
    }
}
