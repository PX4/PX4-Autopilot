/****************************************************************************
 * NxWidgets/libnxwidgets/src/cbgwindow.cxx
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

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "cwidgetcontrol.hxx"
#include "cbgwindow.hxx"
#include "cbitmap.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Method Implementations
 ****************************************************************************/

using namespace NXWidgets;

/**
 * Constructor.  Obtains the background window from server and wraps
 * the window as CBgWindow.
 *
 * @param hNxServer Handle to the NX server.
 * @param widgetControl Controlling widget for this window.
 */
  
CBgWindow::CBgWindow(NXHANDLE hNxServer, CWidgetControl *pWidgetControl)
  : CCallback(pWidgetControl), m_hNxServer(hNxServer), m_hWindow(0),
    m_widgetControl(pWidgetControl)
{
  // Create the CGraphicsPort instance for this window

  m_widgetControl->createGraphicsPort(static_cast<INxWindow*>(this));
}

/**
 * Destructor.  Returns the background window to the server.
 */
     
CBgWindow::~CBgWindow(void)
{
  // Release the background.  We do not release the widget control
  // instance.  The lifetime of that instance is owned by he-who-
  // constructed-us.

  (void)nx_releasebkgd(m_hWindow);
}

/**
 * Creates a new window.  Window creation is separate from
 * object instantiation so that failures can be reported.
 *
 * @return True if the window was successfully opened.
 */

bool CBgWindow::open(void)
{
  // Get the C-callable callback vtable

  FAR struct nx_callback_s *vtable = getCallbackVTable();

  // Request the background the window

  int ret = nx_requestbkgd(m_hNxServer, vtable,
                          (FAR void *)static_cast<CCallback*>(this));
  if (ret < 0)
    {
      return false;
    }

  // Window handle (picked off by the callback logic)

  m_hWindow = m_widgetControl->getWindowHandle();
  return true;
}

/**
 * Each implementation of INxWindow must provide a method to recover
 * the contained CWidgetControl instance.
 *
 * @return The contained CWidgetControl instance
 */

CWidgetControl *CBgWindow::getWidgetControl(void) const
{
  return m_widgetControl;
}

/**
 * Request the position and size information of the window. The values
 * will be returned asynchronously through the client callback method.
 * The GetPosition() method may than be called to obtain the positional
 * data as provided by the callback.
 *
 * @return Always returns true.
 */
     
bool CBgWindow::requestPosition(void)
{
  // The background window is always at {0,0} and the size never changes.

  return true;
}

/**
 * Get the position of the window (as reported by the NX callback). NOTE:
 * The background window is always positioned at {0,0}
 *
 * @return The position.
 */

bool CBgWindow::getPosition(FAR struct nxgl_point_s *pPos)
{
  // The background window is always at {0,0}

  pPos->x = 0;
  pPos->y = 0;
  return true;
}

/**
 * Get the size of the window (as reported by the NX callback).  NOTE:
 * The size of the background window is always the entire display.
 *
 * @return The size.
 */

bool CBgWindow::getSize(FAR struct nxgl_size_s *pSize)
{
  // The size is always the full size of the display

  return m_widgetControl->getWindowSize(pSize);
}

/**
 * Set the position and size of the window.
 *
 * @param pPos The new position of the window.
 * @return Always returns false.
 */
     
bool CBgWindow::setPosition(FAR const struct nxgl_point_s *pPos)
{
  // The position of the background cannot be changed

  return false;
}

/**
 * Set the size of the selected window.  NOTE:  The size of the
 * background window is always the entire display and cannot be
 * changed.
 *
 * @param pSize The new size of the window.
 * @return Always returns false.
 */
    
bool CBgWindow::setSize(FAR const struct nxgl_size_s *pSize)
{
  // The position of the background cannot be changed

  return false;
}

/**
 * Bring the window to the top of the display.  NOTE:  The background
 * window cannot be raised.
 *
 * @return Always returns false.
 */

bool CBgWindow::raise(void)
{
  // The background cannot be raised

  return false;
}

/**
 * Lower the window to the bottom of the display.  NOTE:  The background
 * window is always at the bottom of the window hierarchy.
 *
 * @return Always returns false.
 */

bool CBgWindow::lower(void)
{
  // The background cannot be lowered

  return false;
}

/**
 * Set an individual pixel in the window with the specified color.
 *
 * @param pPos The location of the pixel to be filled.
 * @param color The color to use in the fill.
 *
 * @return True on success; false on failure.
 */

bool CBgWindow::setPixel(FAR const struct nxgl_point_s *pPos,
                         nxgl_mxpixel_t color)
{
  // Set an individual pixel to the specified color

  return nx_setpixel(m_hWindow, pPos, &color) == OK;
}

/**
 * Fill the specified rectangle in the window with the specified color.
 *
 * @param pRect The location to be filled.
 * @param color The color to use in the fill.
 *
 * @return True on success; false on failure.
 */

bool CBgWindow::fill(FAR const struct nxgl_rect_s *pRect,
                     nxgl_mxpixel_t color)
{
  // Fill a rectangular region with a solid color

  return nx_fill(m_hWindow, pRect, &color) == OK;
}

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

void CBgWindow::getRectangle(FAR const struct nxgl_rect_s *rect, struct SBitmap *dest)
{
  // Get a rectangule region from the window

  (void)nx_getrectangle(m_hWindow, rect, 0, (FAR uint8_t*)dest->data, dest->stride);
}

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

bool CBgWindow::fillTrapezoid(FAR const struct nxgl_rect_s *pClip,
                              FAR const struct nxgl_trapezoid_s *pTrap,
                              nxgl_mxpixel_t color)
{
  // Fill a trapezoidal region with a solid color

  return nx_filltrapezoid(m_hWindow, pClip, pTrap, &color) == OK;
}

/**
 * Fill the specified line in the window with the specified color.
 *
 * @param vector - Describes the line to be drawn
 * @param width  - The width of the line
 * @param color  - The color to use to fill the line
 *
 * @return True on success; false on failure.
 */

bool CBgWindow::drawLine(FAR struct nxgl_vector_s *vector,
                         nxgl_coord_t width, nxgl_mxpixel_t color)
{
  // Draw a line with the specified color

  return nx_drawline(m_hWindow, vector, width, &color) == OK;
}

/**
 * Draw a filled circle at the specified position, size, and color.
 *
 * @param center The window-relative coordinates of the circle center.
 * @param radius The radius of the rectangle in pixels.
 * @param color The color of the rectangle.
 */

bool CBgWindow::drawFilledCircle(struct nxgl_point_s *center, nxgl_coord_t radius,
                                 nxgl_mxpixel_t color)
{
  return nx_fillcircle(m_hWindow, center, radius, &color) == OK;
}

/**
 * Move a rectangular region within the window.
 *
 * @param pRect Describes the rectangular region to move.
 * @param pOffset The offset to move the region.
 *
 * @return True on success; false on failure.
 */

bool CBgWindow::move(FAR const struct nxgl_rect_s *pRect,
                     FAR const struct nxgl_point_s *pOffset)
{
  // Move a rectangular region of the display

  return nx_move(m_hWindow, pRect, pOffset) == OK;
}

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

bool CBgWindow::bitmap(FAR const struct nxgl_rect_s *pDest,
                       FAR const void *pSrc,
                       FAR const struct nxgl_point_s *pOrigin,
                       unsigned int stride)
{
  // Copy a rectangular bitmap image in a region on the display

  return nx_bitmap(m_hWindow, pDest, &pSrc, pOrigin, stride) == OK;
}
