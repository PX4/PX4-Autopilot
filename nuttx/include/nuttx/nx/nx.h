/****************************************************************************
 * include/nuttx/nx/nx.h
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

#ifndef _INCLUDE_NUTTX_NX_NX_H
#define _INCLUDE_NUTTX_NX_NX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/nx/nxglib.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Default server MQ name used by nx_run() macro */

#define NX_DEFAULT_SERVER_MQNAME "/dev/nxs"

/* Mouse button bits */

#define NX_MOUSE_NOBUTTONS    0x00
#define NX_MOUSE_LEFTBUTTON   0x01
#define NX_MOUSE_CENTERBUTTON 0x02
#define NX_MOUSE_RIGHTBUTTON  0x04

/****************************************************************************
 * Public Types
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_NX_NPLANES
#  define CONFIG_NX_NPLANES      1  /* Max number of color planes supported */
#endif

/* Check if the underlying graphic device supports read operations */

#if !defined(CONFIG_NX_WRITEONLY) && defined(CONFIG_NX_LCDDRIVER) && defined(CONFIG_LCD_NOGETRUN)
#  define CONFIG_NX_WRITEONLY 1
#endif

/* Handles ******************************************************************/

/* The interface to the NX server is managed using a opaque handle: */

typedef FAR void *NXHANDLE;

/* The interface to a specific window is managed using an opaque handle: */

typedef FAR void *NXWINDOW;

/* NX server callbacks ******************************************************/

/* These define callbacks that must be provided to nx_openwindow.  These
 * callbacks will be invoked as part of the processing performed by
 * nx_eventhandler()
 */

struct nx_callback_s
{
  /**************************************************************************
   * Name: redraw
   *
   * Descripton:
   *   NX requests that the client re-draw the portion of the window within
   *   with rectangle.
   *
   * Input Parameters:
   *   hwnd - Window handle
   *   rect - The rectangle that needs to be re-drawn (in window relative
   *          coordinates)
   *   more - true:  More re-draw requests will follow
   *   arg  - User provided argument (see nx_openwindow, nx_requestbkgd,
   *          nxtk_openwindow, or nxtk_opentoolbar)
   *
   * Returned Value:
   *   None
   *
   **************************************************************************/

  void (*redraw)(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                 bool more, FAR void *arg);

  /**************************************************************************
   * Name: position
   *
   * Descripton:
   *   The size or position of the window has changed (or the window was
   *   just created with zero size.
   *
   * Input Parameters:
   *   hwnd   - Window handle
   *   size   - The size of the window
   *   pos    - The position of the upper left hand corner of the window on
   *            the overall display
   *   bounds - The bounding rectangle that the describes the entire
   *            display
   *   arg  - User provided argument (see nx_openwindow, nx_requestbkgd,
   *          nxtk_openwindow, or nxtk_opentoolbar)
   *
   * Returned Value:
   *   None
   *
   **************************************************************************/

  void (*position)(NXWINDOW hwnd, FAR const struct nxgl_size_s *size,
                   FAR const struct nxgl_point_s *pos,
                   FAR const struct nxgl_rect_s *bounds,
                   FAR void *arg);

  /**************************************************************************
   * Name: mousein
   *
   * Descripton:
   *   New mouse data is available for the window.
   *
   * Input Parameters:
   *   hwnd    - Window handle
   *   pos     - The (x,y) position of the mouse
   *   buttons - See NX_MOUSE_* definitions
   *   arg  - User provided argument (see nx_openwindow, nx_requestbkgd,
   *          nxtk_openwindow, or nxtk_opentoolbar)
   *
   * Returned Value:
   *   None
   *
   **************************************************************************/

#ifdef CONFIG_NX_MOUSE
  void (*mousein)(NXWINDOW hwnd, FAR const struct nxgl_point_s *pos,
                  uint8_t buttons, FAR void *arg);
#endif

  /**************************************************************************
   * Name: kbdin
   *
   * Descripton:
   *   New keyboard/keypad data is available for the window
   *
   * Input Parameters:
   *   hwnd - Window handle
   *   nch  - The number of characters that are available in ch[]
   *   ch   - The array of characters
   *   arg  - User provided argument (see nx_openwindow, nx_requestbkgd,
   *          nxtk_openwindow, or nxtk_opentoolbar)
   *
   * Returned Value:
   *   None
   *
   **************************************************************************/

#ifdef CONFIG_NX_KBD
  void (*kbdin)(NXWINDOW hwnd, uint8_t nch, FAR const uint8_t *ch, FAR void *arg);
#endif

  /**************************************************************************
   * Name: blocked
   *
   * Descripton:
   *   This callback is the response from nx_block (or nxtk_block). Those
   *   blocking interfaces are used to assure that no further messages are
   *   directed to the window. Receipt of the blocked callback signifies
   *   that (1) there are no further pending callbacks and (2) that the
   *   window is now 'defunct' and will receive no further callbacks.
   *
   *   This callback supports coordinated destruction of a window in multi-
   *   user mode.  In multi-use mode, the client window logic must stay
   *   intact until all of the queued callbacks are processed.  Then the
   *   window may be safely closed.  Closing the window prior with pending
   *   callbacks can lead to bad behavior when the callback is executed.
   *
   * Input Parameters:
   *   hwnd - Window handle of the blocked window
   *   arg1 - User provided argument (see nx_openwindow, nx_requestbkgd,
   *          nxtk_openwindow, or nxtk_opentoolbar)
   *   arg2 - User provided argument (see nx_block or nxtk_block)
   *
   * Returned Value:
   *   None
   *
   **************************************************************************/

#ifdef CONFIG_NX_MULTIUSER
  void (*blocked)(NXWINDOW hwnd, FAR void *arg1, FAR void *arg2);
#endif
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
 * Name: nx_runinstance (and nx_run macro)
 *
 * Description:
 *   This is the server entry point.  It does not return; the calling thread
 *   is dedicated to supporting NX server.
 *
 *   NOTE that multiple instances of the NX server may run at the same time,
 *   with different callback and message queue names.  nx_run() is simply
 *   a macro that can be used when only one server instance is required.  In
 *   that case, a default server name is used.
 *
 *   Multiple user mode only!
 *
 * Input Parameters:
 *   mqname - The name for the server incoming message queue
 *   dev     - Vtable "object" of the framebuffer "driver" to use
 *
 * Return:
 *   This function usually does not return.  If it does return, it will
 *   return ERROR and errno will be set appropriately.
 *
 ****************************************************************************/

#ifdef CONFIG_NX_MULTIUSER
EXTERN int nx_runinstance(FAR const char *mqname, FAR NX_DRIVERTYPE *dev);
#  define nx_run(dev) nx_runinstance(NX_DEFAULT_SERVER_MQNAME, dev)
#endif

/****************************************************************************
 * Name:nx_connectinstance (and nx_connect macro)
 *
 * Description:
 *   Open a connection from a client to the NX server.  One one client
 *   connection is normally needed per thread as each connection can host
 *   multiple windows.
 *
 *   NOTES:
 *   - This function returns before the connection is fully instantiated.
 *     it is necessary to wait for the connection event before using the
 *     returned handle.
 *   - Multiple instances of the NX server may run at the same time,
 *     each with different message queue names.
 *   - nx_connect() is simply a macro that can be used when only one
 *     server instance is required.  In that case, a default server name
 *     is used.
 *
 *   Multiple user mode only!
 *
 * Input Parameters:
 *   svrmqname - The name for the server incoming message queue
 *
 * Return:
 *   Success: A non-NULL handle used with subsequent NX accesses
 *   Failure:  NULL is returned and errno is set appropriately
 *
 ****************************************************************************/

#ifdef CONFIG_NX_MULTIUSER
EXTERN NXHANDLE nx_connectinstance(FAR const char *svrmqname);
#  define nx_connect(cb) nx_connectinstance(NX_DEFAULT_SERVER_MQNAME)
#endif

/****************************************************************************
 * Name: nx_open
 *
 * Description:
 *   Create, initialize and return an NX handle for use in subsequent
 *   NX API calls.  nx_open is the single user equivalent of nx_connect
 *   plus nx_run.
 *
 *   Single user mode only!
 *
 * Input Parameters:
 *   dev - Vtable "object" of the framebuffer/LCD "driver" to use
 *   cb - Callbacks used to process received NX server messages
 *
 * Return:
 *   Success: A non-NULL handle used with subsequent NX accesses
 *   Failure:  NULL is returned and errno is set appropriately
 *
 ****************************************************************************/

#ifndef CONFIG_NX_MULTIUSER
EXTERN NXHANDLE nx_open(FAR NX_DRIVERTYPE *dev);
#endif

/****************************************************************************
 * Name: nx_disconnect
 *
 * Description:
 *   Disconnect a client from the NX server and/or free resources reserved
 *   by nx_connect/nx_connectinstance. nx_disconnect is muliti-user equivalent
 *   of nx_close.
 *
 *   Multiple user mode only!
 *
 * Input Parameters:
 *   handle - the handle returned by nx_connect
 *
 * Return:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NX_MULTIUSER
EXTERN void nx_disconnect(NXHANDLE handle);
#endif

/****************************************************************************
 * Name: nx_close
 *
 * Description:
 *   Close the single user NX interface.  nx_close is single-user equivalent
 *   of nx_disconnect.
 *
 *   Single user mode only!
 *
 * Input Parameters:
 *   handle - the handle returned by nx_open
 *
 * Return:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_NX_MULTIUSER
EXTERN void nx_close(NXHANDLE handle);
#endif

/****************************************************************************
 * Name: nx_eventhandler
 *
 * Description:
 *   The client code must call this function periodically to process
 *   incoming messages from the server.  If CONFIG_NX_BLOCKING is defined,
 *   then this function not return until a server message is received.
 *
 *   When CONFIG_NX_BLOCKING is not defined, the client must exercise
 *   caution in the looping to assure that it does not eat up all of
 *   the CPU bandwidth calling nx_eventhandler repeatedly.  nx_eventnotify
 *   may be called to get a signal event whenever a new incoming server
 *   event is avaiable.
 *
 * Input Parameters:
 *   handle - the handle returned by nx_connect
 *
 * Return:
 *     OK: No errors occurred.  If CONFIG_NX_BLOCKING is defined, then
 *         one or more server messages were processed.
 *  ERROR: An error occurred and errno has been set appropriately.  Of
 *         particular interest, it will return errno == EHOSTDOWN when the
 *         server is disconnected.  After that event, the handle can no
 *         longer be used.
 *
 ****************************************************************************/

#ifdef CONFIG_NX_MULTIUSER
EXTERN int nx_eventhandler(NXHANDLE handle);
#else
#  define nx_eventhandler(handle) (OK)
#endif

/****************************************************************************
 * Name: nx_eventnotify
 *
 * Description:
 *   Rather than calling nx_eventhandler periodically, the client may
 *   register to receive a signal when a server event is available.  The
 *   client can then call nv_eventhandler() only when incoming events are
 *   available.
 *
 *   Only one such event is issued.  Upon receipt of the signal, if the client
 *   wishes further notifications, it must call nx_eventnotify again.
 *
 * Input Parameters:
 *   handle - the handle returned by nx_connect
 *
 * Return:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

#if defined(CONFIG_NX_MULTIUSER) && !defined(CONFIG_DISABLE_SIGNALS)
EXTERN int nx_eventnotify(NXHANDLE handle, int signo);
#else
#  define nx_eventnotify(handle, signo) (OK)
#endif

/****************************************************************************
 * Name: nx_openwindow
 *
 * Description:
 *   Create a new window.
 *
 * Input Parameters:
 *   handle - The handle returned by nx_connect or nx_open
 *   cb     - Callbacks used to process window events
 *   arg    - User provided value that will be returned with NX callbacks.
 *
 * Return:
 *   Success: A non-NULL handle used with subsequent NX accesses
 *   Failure:  NULL is returned and errno is set appropriately
 *
 ****************************************************************************/

EXTERN NXWINDOW nx_openwindow(NXHANDLE handle,
                              FAR const struct nx_callback_s *cb,
                              FAR void *arg);

/****************************************************************************
 * Name: nx_closewindow
 *
 * Description:
 *   Destroy a window created by nx_openwindow.
 *
 * Input Parameters:
 *   wnd - The window to be destroyed
 *
 * Return:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

EXTERN int nx_closewindow(NXWINDOW hwnd);

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
 *   user mode.  In multi-use more, the client window logic must stay
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

#ifdef CONFIG_NX_MULTIUSER
EXTERN int nx_block(NXWINDOW hwnd, FAR void *arg);
#endif

/****************************************************************************
 * Name: nx_requestbkgd
 *
 * Description:
 *   NX normally controls a separate window called the background window.
 *   It repaints the window as necessary using only a solid color fill.  The
 *   background window always represents the entire screen and is always
 *   below other windows.  It is useful for an application to control the
 *   background window in the following conditions:
 *
 *   - If you want to implement a windowless solution.  The single screen
 *     can be used to creat a truly simple graphic environment.  In this
 *     case, you should probably also de-select CONFIG_NX_MULTIUSER as well.
 *   - When you want more on the background than a solid color.  For
 *     example, if you want an image in the background, or animations in the
 *     background, or live video, etc.
 *
 *   This API only requests the handle of the background window.  That
 *   handle will be returned asynchronously in a subsequent position and
 *   redraw callbacks.
 *
 *   Cautions:
 *   - The following should never be called using the background window.
 *     They are guaranteed to cause severe crashes:
 *
 *       nx_setposition, nx_setsize, nx_raise, nx_lower.
 *
 *   - Neither nx_requestbkgd or nx_releasebkgd should be called more than
 *     once.  Multiple instances of the background window are not supported.
 *
 * Input Parameters:
 *   handle - The handle returned by nx_connect
 *   cb     - Callbacks to use for processing background window events
 *   arg    - User provided value that will be returned with NX callbacks.
 *
 * Return:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

EXTERN int nx_requestbkgd(NXHANDLE handle,
                          FAR const struct nx_callback_s *cb,
                          FAR void *arg);

/****************************************************************************
 * Name: nx_releasebkgd
 *
 * Description:
 *   Release the background window previously acquired using nx_openbgwindow
 *   and return control of the background to NX.
 *
 * Input Parameters:
 *   hwnd - The handle returned (indirectly) by nx_requestbkgd
 *
 * Return:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

EXTERN int nx_releasebkgd(NXWINDOW hwnd);

/****************************************************************************
 * Name: nx_getposition
 *
 * Description:
 *  Request the position and size information for the selected window.  The
 *  values will be return asynchronously through the client callback function
 *  pointer.
 *
 * Input Parameters:
 *   hwnd   - The window handle
 *
 * Return:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

EXTERN int nx_getposition(NXWINDOW hwnd);

/****************************************************************************
 * Name: nx_setposition
 *
 * Description:
 *  Set the position and size for the selected window
 *
 * Input Parameters:
 *   hwnd  - The window handle
 *   pos   - The new position of the window
 *
 * Return:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

EXTERN int nx_setposition(NXWINDOW hwnd, FAR const struct nxgl_point_s *pos);

/****************************************************************************
 * Name: nx_setsize
 *
 * Description:
 *  Set the size of the selected window
 *
 * Input Parameters:
 *   hwnd   - The window handle
 *   size   - The new size of the window.
 *
 * Return:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

EXTERN int nx_setsize(NXWINDOW hwnd, FAR const struct nxgl_size_s *size);

/****************************************************************************
 * Name: nx_raise
 *
 * Description:
 *   Bring the specified window to the top of the display.
 *
 * Input parameters:
 *   hwnd - the window to be raised
 *
 * Returned value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

EXTERN int nx_raise(NXWINDOW hwnd);

/****************************************************************************
 * Name: nx_lower
 *
 * Description:
 *   Lower the specified window to the bottom of the display.
 *
 * Input parameters:
 *   hwnd - the window to be lowered
 *
 * Returned value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

EXTERN int nx_lower(NXWINDOW hwnd);

/****************************************************************************
 * Name: nx_setpixel
 *
 * Description:
 *  Set a single pixel in the window to the specified color.  This is simply
 *  a degenerate case of nx_fill(), but may be optimized in some architectures.
 *
 * Input Parameters:
 *   wnd  - The window structure reference
 *   pos  - The pixel location to be set
 *   col  - The color to use in the set
 *
 * Return:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

EXTERN int nx_setpixel(NXWINDOW hwnd, FAR const struct nxgl_point_s *pos,
                       nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);

/****************************************************************************
 * Name: nx_fill
 *
 * Description:
 *  Fill the specified rectangle in the window with the specified color
 *
 * Input Parameters:
 *   hwnd  - The window handle
 *   rect  - The location to be filled
 *   color - The color to use in the fill
 *
 * Return:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

EXTERN int nx_fill(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                   nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);

/****************************************************************************
 * Name: nx_getrectangle
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
 * Input Parameters:
 *   hwnd  - The window handle
 *   rect  - The location to be filled
 *   color - The color to use in the fill
 *
 * Return:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

EXTERN int nx_getrectangle(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                           unsigned int plane, FAR uint8_t *dest,
                           unsigned int deststride);

/****************************************************************************
 * Name: nx_filltrapezoid
 *
 * Description:
 *  Fill the specified trapezoidal region in the window with the specified color
 *
 * Input Parameters:
 *   hwnd  - The window handle
 *   clip  - Clipping rectangle relative to window (may be null)
 *   trap  - The trapezoidal region to be filled
 *   color - The color to use in the fill
 *
 * Return:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

EXTERN int nx_filltrapezoid(NXWINDOW hwnd, FAR const struct nxgl_rect_s *clip,
                            FAR const struct nxgl_trapezoid_s *trap,
                            nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);

/****************************************************************************
 * Name: nx_drawline
 *
 * Description:
 *  Fill the specified line in the window with the specified color.  This
 *  is simply a wrapper that uses nxgl_splitline() to break the line into
 *  trapezoids and then calls nx_filltrapezoid() to render the line.
 *
 * Input Parameters:
 *   hwnd   - The window handle
 *   vector - Describes the line to be drawn
 *   width  - The width of the line
 *   color  - The color to use to fill the line
 *
 * Return:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

EXTERN int nx_drawline(NXWINDOW hwnd, FAR struct nxgl_vector_s *vector,
                       nxgl_coord_t width, nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);

/****************************************************************************
 * Name: nx_drawcircle
 *
 * Description:
 *  Draw a circular outline using the specified line thickness and color.
 *
 * Input Parameters:
 *   hwnd   - The window handle
 *   center - A pointer to the point that is the center of the circle
 *   radius - The radius of the circle in pixels.
 *   width  - The width of the line
 *   color  - The color to use to fill the line
 *
 * Return:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

EXTERN int nx_drawcircle(NXWINDOW hwnd, FAR const struct nxgl_point_s *center,
                         nxgl_coord_t radius, nxgl_coord_t width,
                         nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);

/****************************************************************************
 * Name: nx_fillcircle
 *
 * Description:
 *  Fill a circular region using the specified color.
 *
 * Input Parameters:
 *   hwnd   - The window handle
 *   center - A pointer to the point that is the center of the circle
 *   radius - The radius of the circle in pixels.
 *   color  - The color to use to fill the circle
 *
 * Return:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

EXTERN int nx_fillcircle(NXWINDOW hwnd, FAR const struct nxgl_point_s *center,
                         nxgl_coord_t radius,
                         nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);

/****************************************************************************
 * Name: nx_setbgcolor
 *
 * Description:
 *  Set the color of the background
 *
 * Input Parameters:
 *   handle  - The connection handle
 *   color - The color to use in the background
 *
 * Return:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nx_setbgcolor(NXHANDLE handle,
                  nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);

/****************************************************************************
 * Name: nx_move
 *
 * Description:
 *   Move a rectangular region within the window
 *
 * Input Parameters:
 *   hwnd   - The window within which the move is to be done
 *   rect   - Describes the rectangular region to move
 *   offset - The offset to move the region.  The  rectangular region will be
 *            moved so that the origin is translated by this amount.
 *
 * Return:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

EXTERN int nx_move(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                   FAR const struct nxgl_point_s *offset);

/****************************************************************************
 * Name: nx_bitmap
 *
 * Description:
 *   Copy a rectangular region of a larger image into the rectangle in the
 *   specified window.
 *
 * Input Parameters:
 *   hwnd   - The window that will receive the bitmap image
 *   dest   - Describes the rectangular region on the display that will
 *            receive the bit map.
 *   src    - The start of the source image.  This is an array source
 *            images of size CONFIG_NX_NPLANES.
 *   origin - The origin of the upper, left-most corner of the full bitmap.
 *            Both dest and origin are in window coordinates, however, origin
 *            may lie outside of the display.
 *   stride - The width of the full source image in bytes.
 *
 * Return:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

EXTERN int nx_bitmap(NXWINDOW hwnd, FAR const struct nxgl_rect_s *dest,
                     FAR const void *src[CONFIG_NX_NPLANES],
                     FAR const struct nxgl_point_s *origin,
                     unsigned int stride);

/****************************************************************************
 * Name: nx_kbdin
 *
 * Description:
 *   Used by a thread or interrupt handler that manages some kind of keypad
 *   hardware to report text information to the NX server.  That text
 *   data will be routed by the NX server to the appropriate window client.
 *
 ****************************************************************************/

#ifdef CONFIG_NX_KBD
EXTERN int nx_kbdchin(NXHANDLE handle, uint8_t ch);
EXTERN int nx_kbdin(NXHANDLE handle, uint8_t nch, FAR const uint8_t *ch);
#endif

/****************************************************************************
 * Name: nx_mousein
 *
 * Description:
 *   Used by a thread or interrupt handler that manages some kind of pointing
 *   hardware to report new positional data to the NX server.  That positional
 *   data will be routed by the NX server to the appropriate window client.
 *
 ****************************************************************************/

#ifdef CONFIG_NX_MOUSE
EXTERN int nx_mousein(NXHANDLE handle, nxgl_coord_t x, nxgl_coord_t y, uint8_t buttons);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* _INCLUDE_NUTTX_NX_NX_H */

