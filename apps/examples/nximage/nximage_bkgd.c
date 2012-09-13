/****************************************************************************
 * examples/nximage/nximage_bkgd.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nxfonts.h>

#include "nximage.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Select renderer -- Some additional logic would be required to support
 * pixel depths that are not directly addressable (1,2,4, and 24).
 */

#if CONFIG_EXAMPLES_NXIMAGE_BPP == 1
#  define RENDERER nxf_convert_1bpp
#elif CONFIG_EXAMPLES_NXIMAGE_BPP == 2
#  define RENDERER nxf_convert_2bpp
#elif CONFIG_EXAMPLES_NXIMAGE_BPP == 4
#  define RENDERER nxf_convert_4bpp
#elif CONFIG_EXAMPLES_NXIMAGE_BPP == 8
#  define RENDERER nxf_convert_8bpp
#elif CONFIG_EXAMPLES_NXIMAGE_BPP == 16
#  define RENDERER nxf_convert_16bpp
#elif CONFIG_EXAMPLES_NXIMAGE_BPP == 24
#  define RENDERER nxf_convert_24bpp
#elif  CONFIG_EXAMPLES_NXIMAGE_BPP == 32
#  define RENDERER nxf_convert_32bpp
#else
#  error "Unsupported CONFIG_EXAMPLES_NXIMAGE_BPP"
#endif

/* Vertical scaling */

#if defined(CONFIG_EXAMPLES_NXIMAGE_YSCALEp5)

/* Read two rows, output one averaged row */

#define NINPUT_ROWS  2
#define NOUTPUT_ROWS 1

#elif defined(CONFIG_EXAMPLES_NXIMAGE_YSCALE1p5)
/* Read two rows, output three rows */

#define NINPUT_ROWS  2
#define NOUTPUT_ROWS 3

#elif defined(CONFIG_EXAMPLES_NXIMAGE_YSCALE2p0)
/* Read one row, output two rows */

#define NINPUT_ROWS  1
#define NOUTPUT_ROWS 2

#else
/* Read one rows, output one or two rows */

#define NINPUT_ROWS  1
#define NOUTPUT_ROWS 1
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nximage_run_t
{
  nxgl_mxpixel_t run[SCALED_WIDTH];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void nximage_redraw(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                        bool more, FAR void *arg);
static void nximage_position(NXWINDOW hwnd, FAR const struct nxgl_size_s *size,
                          FAR const struct nxgl_point_s *pos,
                          FAR const struct nxgl_rect_s *bounds,
                          FAR void *arg);
#ifdef CONFIG_NX_MOUSE
static void nximage_mousein(NXWINDOW hwnd, FAR const struct nxgl_point_s *pos,
                         uint8_t buttons, FAR void *arg);
#endif

#ifdef CONFIG_NX_KBD
static void nximage_kbdin(NXWINDOW hwnd, uint8_t nch, FAR const uint8_t *ch,
                       FAR void *arg);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char g_hello[] = "Hello, World!";

/* Read one or two rows, output one tow or three rows */

static struct nximage_run_t g_runs[NINPUT_ROWS];

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Background window call table */

const struct nx_callback_s g_nximagecb =
{
  nximage_redraw,   /* redraw */
  nximage_position  /* position */
#ifdef CONFIG_NX_MOUSE
  , nximage_mousein /* mousein */
#endif
#ifdef CONFIG_NX_KBD
  , nximage_kbdin   /* my kbdin */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nximage_redraw
 *
 * Description:
 *   NX re-draw handler
 *
 ****************************************************************************/

static void nximage_redraw(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                        bool more, FAR void *arg)
{
  gvdbg("hwnd=%p rect={(%d,%d),(%d,%d)} more=%s\n",
         hwnd, rect->pt1.x, rect->pt1.y, rect->pt2.x, rect->pt2.y,
         more ? "true" : "false");
}

/****************************************************************************
 * Name: nximage_position
 *
 * Description:
 *   NX position change handler
 *
 ****************************************************************************/

static void nximage_position(NXWINDOW hwnd, FAR const struct nxgl_size_s *size,
                          FAR const struct nxgl_point_s *pos,
                          FAR const struct nxgl_rect_s *bounds,
                          FAR void *arg)
{
  /* Report the position */

  gvdbg("hwnd=%p size=(%d,%d) pos=(%d,%d) bounds={(%d,%d),(%d,%d)}\n",
        hwnd, size->w, size->h, pos->x, pos->y,
        bounds->pt1.x, bounds->pt1.y, bounds->pt2.x, bounds->pt2.y);

  /* Have we picked off the window bounds yet? */

  if (!g_nximage.havepos)
    {
      /* Save the background window handle */

      g_nximage.hbkgd = hwnd;

      /* Save the window limits */

      g_nximage.xres = bounds->pt2.x + 1;
      g_nximage.yres = bounds->pt2.y + 1;

      g_nximage.havepos = true;
      sem_post(&g_nximage.sem);
      gvdbg("Have xres=%d yres=%d\n", g_nximage.xres, g_nximage.yres);
    }
}

/****************************************************************************
 * Name: nximage_mousein
 *
 * Description:
 *   NX mouse input handler
 *
 ****************************************************************************/

#ifdef CONFIG_NX_MOUSE
static void nximage_mousein(NXWINDOW hwnd, FAR const struct nxgl_point_s *pos,
                         uint8_t buttons, FAR void *arg)
{
  message("nximage_mousein: hwnd=%p pos=(%d,%d) button=%02x\n",
          hwnd,  pos->x, pos->y, buttons);
}
#endif

/****************************************************************************
 * Name: nximage_kbdin
 *
 * Description:
 *   NX keyboard input handler
 *
 ****************************************************************************/

#ifdef CONFIG_NX_KBD
static void nximage_kbdin(NXWINDOW hwnd, uint8_t nch, FAR const uint8_t *ch,
                       FAR void *arg)
{
  gvdbg("hwnd=%p nch=%d\n", hwnd, nch);

   /* In this example, there is no keyboard so a keyboard event is not
    * expected.
    */

   message("nximage_kbdin: Unexpected keyboard callback\n");
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nximage_image
 *
 * Description:
 *   Put the NuttX logo in the center of the display.
 *
 ****************************************************************************/

void nximage_image(NXWINDOW hwnd)
{
  FAR const void *state = NULL;
  FAR struct nxgl_point_s pos;
  FAR struct nxgl_rect_s dest;
  FAR const void *src[CONFIG_NX_NPLANES];
  nxgl_coord_t row;
  int ret;
#if defined(CONFIG_EXAMPLES_NXIMAGE_YSCALEp5) || defined(CONFIG_EXAMPLES_NXIMAGE_YSCALE1p5)
  int i;
#endif

  /* Center the image.  Note: these may extend off the display. */

  pos.x = (g_nximage.xres - SCALED_WIDTH) / 2;
  pos.y = (g_nximage.yres - SCALED_HEIGHT) / 2;

  /* Set up the invariant part of the destination bounding box */

  dest.pt1.x = pos.x;
  dest.pt2.x = pos.x + SCALED_WIDTH - 1;
 
  /* Now output the rows */

  for (row = 0; row < IMAGE_HEIGHT; row += NINPUT_ROWS)
    {
      /* Read input row(s) */

     nximage_blitrow(g_runs[0].run, &state);
#if NINPUT_ROWS > 1
     nximage_blitrow(g_runs[1].run, &state);
#endif

      /* Output rows before averaging */

#if defined(CONFIG_EXAMPLES_NXIMAGE_YSCALE1p5) || defined(CONFIG_EXAMPLES_NXIMAGE_YSCALE2p0)

      /* Output row[0] */

      dest.pt1.y = pos.y;
      dest.pt2.y = pos.y;

      src[0] = (FAR const void *)g_runs[0].run;
#if CONFIG_NX_NPLANES > 1
# warning "More logic is needed for the case where CONFIG_NX_PLANES > 1"
#endif
      ret = nx_bitmap((NXWINDOW)hwnd, &dest, src, &pos, SCALED_WIDTH*sizeof(nxgl_mxpixel_t));
      if (ret < 0)
        {
          message("nximage_image: nx_bitmapwindow failed: %d\n", errno);
        }

      /* Increment the vertical position */

      pos.y++;
#endif

      /* Perform averaging */

#if defined(CONFIG_EXAMPLES_NXIMAGE_YSCALEp5) || defined(CONFIG_EXAMPLES_NXIMAGE_YSCALE1p5)

      /* Average row[0] and row[1], output results in row[0] */

      for (i = 0; i < SCALED_WIDTH; i++)
        {
          /* Only average if the corresponding pixels in each row differ */

          nxgl_mxpixel_t pix0 = g_runs[0].run[i];
          nxgl_mxpixel_t pix1 = g_runs[1].run[i];
          if (pix0 != pix1)
            {
              g_runs[0].run[i] = nximage_avgcolor(pix0, pix1);
            }
        }

#endif

      /* Output rows after averaging */

      /* Output row[0] */

      dest.pt1.y = pos.y;
      dest.pt2.y = pos.y;

      src[0] = (FAR const void *)g_runs[0].run;
#if CONFIG_NX_NPLANES > 1
# warning "More logic is needed for the case where CONFIG_NX_PLANES > 1"
#endif
      ret = nx_bitmap((NXWINDOW)hwnd, &dest, src, &pos, SCALED_WIDTH*sizeof(nxgl_mxpixel_t));
      if (ret < 0)
        {
          message("nximage_image: nx_bitmapwindow failed: %d\n", errno);
        }

      /* Increment the vertical position */

      pos.y++;

#if defined(CONFIG_EXAMPLES_NXIMAGE_YSCALE1p5)

      /* Output row[0] and row[1] */

      dest.pt1.y = pos.y;
      dest.pt2.y = pos.y;

      src[0] = (FAR const void *)g_runs[1].run;
#if CONFIG_NX_NPLANES > 1
# warning "More logic is needed for the case where CONFIG_NX_PLANES > 1"
#endif
      ret = nx_bitmap((NXWINDOW)hwnd, &dest, src, &pos, SCALED_WIDTH*sizeof(nxgl_mxpixel_t));
      if (ret < 0)
        {
          message("nximage_image: nx_bitmapwindow failed: %d\n", errno);
        }

      /* Increment the vertical position */

      pos.y++;
#endif
    }
}
