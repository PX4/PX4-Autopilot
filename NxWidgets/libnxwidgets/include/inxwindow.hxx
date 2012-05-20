/****************************************************************************
 * NxWidgets/libnxwidgets/include/inxwindow.hxx
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
 * 3. Neither the name NuttX, NxWidgets, nor the names of its contributors
 *    me be used to endorse or promote products derived from this software
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

#ifndef __INCLUDE_INXWINDOW_HXX
#define __INCLUDE_INXWINDOW_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <nuttx/nx/nxglib.h>

#include <stdint.h>
#include <stdbool.h>

#ifdef CONFIG_NXCONSOLE_NXKBDIN
#  include <nuttx/nx/nxconsole.h>
#endif

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Pure Virtual Classes
 ****************************************************************************/

#if defined(__cplusplus)

namespace NXWidgets
{
  struct SBitmap;
  class  CWidgetControl;

  /**
   * This class defines common operations on a any NX window.
   * There are three instances that represent an NX window from the
   * perspective of NXWidgets.
   *
   * - There is one widget control instance per NX window,
   * - One CCallback instance per window,
   * - One window instance.
   *
   * There a various kinds of of window instances, but each inherits
   * (1) CCallback and dispatches the Windows callbacks and (2) INxWindow
   * that describes the common window behavior.
   */

  class INxWindow
  {
  public:
    /**
     * A virtual destructor is required in order to override the INxWindow
     * destructor.  We do this because if we delete INxWindow, we want the
     * destructor of the class that inherits from INxWindow to run, not this
     * one.
     */

    virtual ~INxWindow(void) { }

    /**
     * Creates a new window.  Window creation is separate from
     * object instantiation so that window creation failures can
     * be properly reported.
     *
     * @return True if the window was successfully created.
     */
     
    virtual bool open(void) = 0;

    /**
     * Each implementation of INxWindow must provide a method to recover
     * the contained CWidgetControl instance.
     *
     * @return The contained CWidgetControl instance
     */

    virtual CWidgetControl *getWidgetControl(void) const = 0;

    /**
     * Request the position and size information of the window. The values
     * will be returned asynchronously through the client callback method.
     * The GetPosition() method may than be called to obtain the positional
     * data as provided by the callback.
     *
     * @return OK on success; ERROR on failure with errno set appropriately.
     */

    virtual bool requestPosition(void) = 0;

    /**
     * Get the position of the window (as reported by the NX callback).
     *
     * @return The position.
     */

    virtual bool getPosition(FAR struct nxgl_point_s *pPos) = 0;

    /**
     * Get the size of the window (as reported by the NX callback).
     *
     * @return The size.
     */

    virtual bool getSize(FAR struct nxgl_size_s *pSize) = 0;

    /**
     * Set the position and size of the window.
     *
     * @param pPos The new position of the window.
     * @return True on success, false on failure.
     */
     
    virtual bool setPosition(FAR const struct nxgl_point_s *pPos) = 0;

    /**
     * Set the size of the selected window.
     *
     * @param pSize The new size of the window.
     * @return OK on success; ERROR on failure with errno set appropriately.
     */
    
    virtual bool setSize(FAR const struct nxgl_size_s *pSize) = 0;

    /**
     * Bring the window to the top of the display.
     *
     * @return OK on success; ERROR on failure with errno set appropriately.
     */

    virtual bool raise(void) = 0;

    /**
     * Lower the window to the bottom of the display.
     *
     * @return OK on success; ERROR on failure with errno set appropriately.
     */

    virtual bool lower(void) = 0;

    /**
     * Each window implementation also inherits from CCallback.  CCallback,
     * by default, forwards NX keyboard input to the various widgets residing
     * in the window. But NxConsole is a different usage model; In this case,
     * keyboard input needs to be directed to the NxConsole character driver.
     * This method can be used to enable (or disable) redirection of NX
     * keyboard input from the window widgets to the NxConsole
     *
     * @param handle.  The NXCONSOLE handle.  If non-NULL, NX keyboard
     *    input will be directed to the NxConsole driver using this
     *    handle;  If NULL (the default), NX keyboard input will be
     *    directed to the widgets within the window.
     */

#ifdef CONFIG_NXCONSOLE_NXKBDIN
    virtual void redirectNxConsole(NXCONSOLE handle) = 0;
#endif

    /**
     * Set an individual pixel in the window with the specified color.
     *
     * @param pPos The location of the pixel to be filled.
     * @param color The color to use in the fill.
     *
     * @return True on success; false on failure.
     */

    virtual bool setPixel(FAR const struct nxgl_point_s *pPos,
                          nxgl_mxpixel_t color) = 0;

    /**
     * Fill the specified rectangle in the window with the specified color.
     *
     * @param pRect The location to be filled.
     * @param color The color to use in the fill.
     *
     * @return True on success; false on failure.
     */

    virtual bool fill(FAR const struct nxgl_rect_s *pRect,
                      nxgl_mxpixel_t color) = 0;

    /**
     * Get the raw contents of graphic memory within a rectangular region. NOTE:
     * Since raw graphic memory is returned, the returned memory content may be
     * the memory of windows above this one and may not necessarily belong to
     * this window unless you assure that this is the top window.
     *
     * @param rect The location to be copied
     * @param dest - The describes the destination bitmap to receive the
     *   graphics data.
     */

    virtual void getRectangle(FAR const struct nxgl_rect_s *rect,
                              struct SBitmap *dest) = 0;

    /**
     * Fill the specified trapezoidal region in the window with the specified
     * color.
     *
     * @param pClip Clipping rectangle relative to window (may be null).
     * @param pTrap The trapezoidal region to be filled.
     * @param color The color to use in the fill.
     *
     * @return True on success; false on failure.
     */

    virtual bool fillTrapezoid(FAR const struct nxgl_rect_s *pClip,
                               FAR const struct nxgl_trapezoid_s *pTrap,
                               nxgl_mxpixel_t color) = 0;

    /**
     * Fill the specified line in the window with the specified color.
     *
     * @param vector - Describes the line to be drawn
     * @param width  - The width of the line
     * @param color  - The color to use to fill the line
     *
     * @return True on success; false on failure.
     */

    virtual bool drawLine(FAR struct nxgl_vector_s *vector,
                          nxgl_coord_t width,
                          nxgl_mxpixel_t color) = 0;

    /**
     * Draw a filled circle at the specified position, size, and color.
     *
     * @param center The window-relative coordinates of the circle center.
     * @param radius The radius of the rectangle in pixels.
     * @param color The color of the rectangle.
     */

    virtual bool drawFilledCircle(struct nxgl_point_s *center, nxgl_coord_t radius,
                                  nxgl_mxpixel_t color) = 0;

    /**
     * Move a rectangular region within the window.
     *
     * @param pRect Describes the rectangular region to move.
     * @param pOffset The offset to move the region.
     *
     * @return True on success; false on failure.
     */

    virtual bool move(FAR const struct nxgl_rect_s *pRect,
                      FAR const struct nxgl_point_s *pOffset) = 0;

    /**
     * Copy a rectangular region of a larger image into the rectangle in the
     * specified window.  The source image is treated as an opaque image.
     *
     * @param pDest Describes the rectangular on the display that will receive
     * the bitmap.
     * @param pSrc The start of the source image.
     * @param pOrigin the pOrigin of the upper, left-most corner of the full
     * bitmap. Both pDest and pOrigin are in window coordinates, however,
     * pOrigin may lie outside of the display.
     * @param stride The width of the full source image in bytes.
     *
     * @return True on success; false on failure.
     */

    virtual bool bitmap(FAR const struct nxgl_rect_s *pDest,
                        FAR const void *pSrc,
                        FAR const struct nxgl_point_s *pOrigin,
                        unsigned int stride) = 0;
  };
}

#endif // __cplusplus

#endif // __INCLUDE_INXWINDOW_HXX

