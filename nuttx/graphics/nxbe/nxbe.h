/****************************************************************************
 * graphics/nxbe/nxbe.h
 *
 *   Copyright (C) 2008-2011 Gregory Nutt. All rights reserved.
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

#ifndef __GRAPHICS_NXBE_NXBE_H
#define __GRAPHICS_NXBE_NXBE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_NX_NPLANES
#  define CONFIG_NX_NPLANES      1  /* Max number of color planes supported */
#endif

#ifndef CONFIG_NX_NCOLORS
#  define CONFIG_NX_NCOLORS 256
#endif

/* NXBE Definitions *********************************************************/
/* These are the values for the clipping order provided to nx_clipper */

#define NX_CLIPORDER_TLRB    (0)   /* Top-left-right-bottom */
#define NX_CLIPORDER_TRLB    (1)   /* Top-right-left-bottom */
#define NX_CLIPORDER_BLRT    (2)   /* Bottom-left-right-top */
#define NX_CLIPORDER_BRLT    (3)   /* Bottom-right-left-top */
#define NX_CLIPORDER_DEFAULT NX_CLIPORDER_TLRB

/* Window flags and helper macros */

#define NXBE_WINDOW_BLOCKED  (1 << 0) /* The window is blocked and will not
                                       * receive further input. */

#define NXBE_ISBLOCKED(wnd)  (((wnd)->flags & NXBE_WINDOW_BLOCKED) != 0)
#define NXBE_SETBLOCKED(wnd) do { (wnd)->flags |= NXBE_WINDOW_BLOCKED; } while (0)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Rasterization ************************************************************/

/* A tiny vtable of raster operation function pointers.  The types of the
 * function points must match the rasterizer types exported by nxglib
 */

struct nxbe_plane_s
{
  /* Raster operation callbacks for this bits-per-pixel value */

  void (*setpixel)(FAR NX_PLANEINFOTYPE *pinfo,
                   FAR const struct nxgl_point_s *pos,
                   nxgl_mxpixel_t color);
  void (*fillrectangle)(FAR NX_PLANEINFOTYPE *pinfo,
                        FAR const struct nxgl_rect_s *rect,
                        nxgl_mxpixel_t color);
  void (*getrectangle)(FAR NX_PLANEINFOTYPE *pinfo,
                       FAR const struct nxgl_rect_s *rect,
                       FAR void *dest, unsigned int deststride);
  void (*filltrapezoid)(FAR NX_PLANEINFOTYPE *pinfo,
                        FAR const struct nxgl_trapezoid_s *trap,
                        FAR const struct nxgl_rect_s *bounds,
                        nxgl_mxpixel_t color);
  void (*moverectangle)(FAR NX_PLANEINFOTYPE *pinfo,
                        FAR const struct nxgl_rect_s *rect,
                        FAR struct nxgl_point_s *offset);
  void (*copyrectangle)(FAR NX_PLANEINFOTYPE *pinfo,
                        FAR const struct nxgl_rect_s *dest,
                        FAR const void *src,
                        FAR const struct nxgl_point_s *origin,
                        unsigned int srcstride);

  /* Framebuffer plane info describing destination video plane */

  NX_PLANEINFOTYPE pinfo;
};

/* Clipping *****************************************************************/

/* Clipping callback functions called nxbe_clipper for each visible and
 * obscured region of a rectangle within a window.
 */

struct nxbe_clipops_s
{
  void (*visible)(FAR struct nxbe_clipops_s *cops,
                  FAR struct nxbe_plane_s *plane,
                  FAR const struct nxgl_rect_s *rect);

  void (*obscured)(FAR struct nxbe_clipops_s *cops,
                   FAR struct nxbe_plane_s *plane,
                   FAR const struct nxgl_rect_s *rect);
};

/* Windows ******************************************************************/

/* This structure represents one window. */

struct nxbe_state_s;
struct nxfe_conn_s;
struct nxbe_window_s
{
  /* State information */

  FAR struct nxbe_state_s *be;        /* The back-end state structure */
#ifdef CONFIG_NX_MULTIUSER
  FAR struct nxfe_conn_s *conn;       /* Connection to the window client */
#endif
  FAR const struct nx_callback_s *cb; /* Event handling callbacks */

  /* The following links provide the window's vertical position using a
   * singly linked list.
   */

  FAR struct nxbe_window_s *above;    /* The window "above" this window */
  FAR struct nxbe_window_s *below;    /* The window "below this one */

  /* Window geometry.  The window is described by a rectangle in the
   * absolute screen coordinate system (0,0)->(xres,yres)
   */

  struct nxgl_rect_s bounds;          /* The bounding rectangle of window */

  /* Window flags (see the NXBE_* bit definitions above) */

#ifdef CONFIG_NX_MULTIUSER            /* Currently used only in multi-user mode */
  uint8_t flags;
#endif

  /* Client state information this is provide in window callbacks */

  FAR void *arg;
};

/* Back-end state ***********************************************************/

/* This structure describes the overall back-end window state */

struct nxbe_state_s
{
  /* The window list (with the background window always at the bottom) */

  FAR struct nxbe_window_s *topwnd; /* The window at the top of the display */
  struct nxbe_window_s bkgd;        /* The background window is always at the bottom */

  /* At present, only a solid colored background is supported for refills.  The
   * following provides the background color.  It would be nice to support
   * background bitmap images as well.
   */

  nxgl_mxpixel_t bgcolor[CONFIG_NX_NPLANES];

  /* vinfo describes the video controller and plane[n].pinfo describes color
   * plane 'n' supported by the video controller.  Most common color models
   * fit in one plane, but this array provides future support for hardware
   * with planar YUV types with 3 or 4 color planes.
   */

  struct fb_videoinfo_s vinfo;

  /* Rasterizing functions selected to match the BPP reported in pinfo[] */

  struct nxbe_plane_s plane[CONFIG_NX_NPLANES];
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_colormap
 *
 * Description:
 *   Set the harware color map to the palette expected by NX
 *
 ****************************************************************************/

#ifdef CONFIG_FB_CMAP
EXTERN int nxbe_colormap(FAR NX_DRIVERTYPE *dev);
#endif

/****************************************************************************
 * Name: nx_configure
 *
 * Description:
 *   Configure the back end state structure based on information from the
 *   framebuffer or LCD driver
 *
 ****************************************************************************/

EXTERN int nxbe_configure(FAR NX_DRIVERTYPE *dev,
                          FAR struct nxbe_state_s *be);

/****************************************************************************
 * Name: nxbe_closewindow
 *
 * Description:
 *   Close an existing window
 *
 * Input Parameters:
 *   wnd  - The window to be closed (and deallocated)
 *
 * Return:
 *   None
 *
 ****************************************************************************/

EXTERN void nxbe_closewindow(struct nxbe_window_s *wnd);

/****************************************************************************
 * Name: nxbe_setposition
 *
 * Descripton:
 *   This function checks for intersections and redraws the display after
 *   a change in the position of a window.
 *
 ****************************************************************************/

EXTERN void nxbe_setposition(FAR struct nxbe_window_s *wnd,
                             FAR const struct nxgl_point_s *pos);

/****************************************************************************
 * Name: nxbe_setsize
 *
 * Descripton:
 *   This function checks for intersections and redraws the display after
 *   a change in the size of a window.
 *
 ****************************************************************************/

EXTERN void nxbe_setsize(FAR struct nxbe_window_s *wnd,
                         FAR const struct nxgl_size_s *size);

/****************************************************************************
 * Name: nxbe_raise
 *
 * Description:
 *   Bring the specified window to the top of the display.
 *
 ****************************************************************************/

EXTERN void nxbe_raise(FAR struct nxbe_window_s *wnd);

/****************************************************************************
 * Name: nxbe_lower
 *
 * Description:
 *   Lower the specified window to the bottom of the display.
 *
 ****************************************************************************/

EXTERN void nxbe_lower(FAR struct nxbe_window_s *wnd);

/****************************************************************************
 * Name: nxbe_setpixel
 *
 * Description:
 *  Set a single pixel in the window to the specified color.  This is simply
 *  a degenerate case of nxbe_fill(), but may be optimized in some architectures.
 *
 * Input Parameters:
 *   wnd  - The window structure reference
 *   pos  - The pixel location to be set
 *   col  - The color to use in the set
 *
 * Return:
 *   None
 *
 ****************************************************************************/

EXTERN void nxbe_setpixel(FAR struct nxbe_window_s *wnd,
                          FAR const struct nxgl_point_s *pos,
                          nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);

/****************************************************************************
 * Name: nxbe_fill
 *
 * Description:
 *  Fill the specified rectangle in the window with the specified color
 *
 * Input Parameters:
 *   wnd  - The window structure reference
 *   rect - The location to be filled
 *   col  - The color to use in the fill
 *
 * Return:
 *   None
 *
 ****************************************************************************/

EXTERN void nxbe_fill(FAR struct nxbe_window_s *wnd,
                      FAR const struct nxgl_rect_s *rect,
                      nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);

/****************************************************************************
 * Name: nxbe_filltrapezoid
 *
 * Description:
 *  Fill the specified rectangle in the window with the specified color
 *
 * Input Parameters:
 *   wnd  - The window structure reference
 *   clip - Clipping region (may be null)
 *   rect - The location to be filled
 *   col  - The color to use in the fill
 *
 * Return:
 *   None
 *
 ****************************************************************************/

EXTERN void nxbe_filltrapezoid(FAR struct nxbe_window_s *wnd,
                               FAR const struct nxgl_rect_s *clip,
                               FAR const struct nxgl_trapezoid_s *trap,
                               nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);

/****************************************************************************
 * Name: nxbe_getrectangle
 *
 * Description:
 *  Get the raw contents of graphic memory within a rectangular region. NOTE:
 *  Since raw graphic memory is returned, the returned memory content may be
 *  the memory of windows above this one and may not necessarily belong to
 *  this window unless you assure that this is the top window.
 *
 * Input Parameters:
 *   wnd  - The window structure reference
 *   rect - The location to be copied
 *   plane - Specifies the color plane to get from.
 *   dest - The location to copy the memory region
 *   deststride - The width, in bytes, the the dest memory
 *
 * Return:
 *   None
 *
 ****************************************************************************/

EXTERN void nxbe_getrectangle(FAR struct nxbe_window_s *wnd,
                              FAR const struct nxgl_rect_s *rect,
                              unsigned int plane,
                              FAR uint8_t *dest, unsigned int deststride);

/****************************************************************************
 * Name: nxbe_move
 *
 * Description:
 *   Move a rectangular region within the window
 *
 * Input Parameters:
 *   wnd    - The window within which the move is to be done
 *   rect   - Describes the rectangular region to move
 *   offset - The offset to move the region
 *
 * Return:
 *   None
 *
 ****************************************************************************/

EXTERN void nxbe_move(FAR struct nxbe_window_s *wnd,
                      FAR const struct nxgl_rect_s *rect,
                      FAR const struct nxgl_point_s *offset);

/****************************************************************************
 * Name: nxbe_bitmap
 *
 * Description:
 *   Copy a rectangular region of a larger image into the rectangle in the
 *   specified window.
 *
 * Input Parameters:
 *   wnd   - The window that will receive the bitmap image
 *   dest   - Describes the rectangular on the display that will receive the
 *            the bit map.
 *   src    - The start of the source image.
 *   origin - The origin of the upper, left-most corner of the full bitmap.
 *            Both dest and origin are in window coordinates, however, origin
 *            may lie outside of the display.
 *   stride - The width of the full source image in pixels.
 *
 * Return:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

EXTERN void nxbe_bitmap(FAR struct nxbe_window_s *wnd,
                       FAR const struct nxgl_rect_s *dest,
                       FAR const void *src[CONFIG_NX_NPLANES],
                       FAR const struct nxgl_point_s *origin,
                       unsigned int stride);

/****************************************************************************
 * Name: nxbe_redraw
 *
 * Descripton:
 *   Re-draw the visible portions of the rectangular region for the
 *   specified window
 *
 ****************************************************************************/

EXTERN void nxbe_redraw(FAR struct nxbe_state_s *be,
                        FAR struct nxbe_window_s *wnd,
                        FAR const struct nxgl_rect_s *rect);

/****************************************************************************
 * Name: nxbe_redrawbelow
 *
 * Descripton:
 *   Re-draw the visible portions of the rectangular region for all windows
 *   below (and including) the specified window.  This function is called
 *   whenever a window is closed, moved, lowered or re-sized in order to
 *   expose newly visible portions of lower windows.
 *
 ****************************************************************************/

EXTERN void nxbe_redrawbelow(FAR struct nxbe_state_s *be,
                             FAR struct nxbe_window_s *wnd,
                             FAR const struct nxgl_rect_s *rect);

/****************************************************************************
 * Name: nxbe_visible
 *
 * Descripton:
 *   Return true if the point, pt, in window wnd is visible.  pt is in
 *   absolute screen coordinates
 *
 ****************************************************************************/

EXTERN bool nxbe_visible(FAR struct nxbe_window_s *wnd,
                         FAR const struct nxgl_point_s *pos);

/****************************************************************************
 * Name: nxbe_clipper
 *
 * Descripton:
 *   Perform flexible clipping operations.  Callbacks are executed for
 *   each oscured and visible portions of the window.
 *
 * Input Parameters:
 *   wnd  - The window to be clipped.
 *   rect   - The region of concern within the window
 *   order - Specifies the order to process the parts of the non-intersecting
 *           sub-rectangles.
 *   cops  - The callbacks to handle obscured and visible parts of the
 *           sub-rectangles.
 *   plane  - The raster operations to be used by the callback functions.
 *           These may vary with different color formats.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

EXTERN void nxbe_clipper(FAR struct nxbe_window_s *wnd,
                         FAR const struct nxgl_rect_s *dest, uint8_t order,
                         FAR struct nxbe_clipops_s *cops,
                         FAR struct nxbe_plane_s *plane);

/****************************************************************************
 * Name: nxbe_clipnull
 *
 * Descripton:
 *   The do-nothing clipping callback function
 *
 ****************************************************************************/

EXTERN void nxbe_clipnull(FAR struct nxbe_clipops_s *cops,
                          FAR struct nxbe_plane_s *plane,
                          FAR const struct nxgl_rect_s *rect);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __GRAPHICS_NXBE_NXBE_H */

