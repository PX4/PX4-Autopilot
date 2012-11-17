/****************************************************************************
 * NxWidgets/libnxwidgets/src/cnxtkwindow.cxx
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
#include <sched.h>
#include <cassert>

#include "cwidgetcontrol.hxx"
#include "cnxtkwindow.hxx"
#include "cnxtoolbar.hxx"
#include "cbitmap.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Method Implementations
 ****************************************************************************/

using namespace NXWidgets;

/**
 * Constructor.
 *
 * @param hNxServer Handle to the NX server.
 * @param widgetControl Controlling widget for this window.
 */

CNxTkWindow::CNxTkWindow(NXHANDLE hNxServer, CWidgetControl *widgetControl)
  : CCallback(widgetControl)
{
  // Save construction values

  m_hNxServer     = hNxServer;
  m_widgetControl = widgetControl;

  // Nullify uninitilized pointers and values

  m_hNxTkWindow   = (NXTKWINDOW  )0;
  m_toolbar       = (CNxToolbar *)0;
  m_toolbarHeight = 0;

  // Create the CGraphicsPort instance for this window

  m_widgetControl->createGraphicsPort(static_cast<INxWindow*>(this));
}

/**
 * Destructor.
 */

CNxTkWindow::~CNxTkWindow(void)
{
  // It would be a disaster if toolbar instance persists beyond
  // the window!

  DEBUGASSERT(!m_toolbar);

  // Release the window.  We do not release the widget control
  // instance.  The lifetime of that instance is owned by he-who-
  // constructed-us.

  (void)nxtk_closewindow(m_hNxTkWindow);

  delete m_widgetControl;
}

/**
 * Creates a new framed window. Window creation is separate from
 * object instantiation so that failures can be reported.
 *
 * @return True if the framed window was successfully created.
 */

bool CNxTkWindow::open(void)
{
  // Get the C-callable callback vtable

  FAR struct nx_callback_s *vtable = getCallbackVTable();

  // Create the window

  m_hNxTkWindow = nxtk_openwindow(m_hNxServer, vtable,
                                 (FAR void *)static_cast<CCallback*>(this));
  return m_hNxTkWindow != NULL;
}

/**
 * Each implementation of INxWindow must provide a method to recover
 * the contained CWidgetControl instance.
 *
 * @return The contained CWidgetControl instance
 */

CWidgetControl *CNxTkWindow::getWidgetControl(void) const
{
  return m_widgetControl;
}

/**
 * Open a toolbar on the framed window.  This method both instantiates
 * the toolbar object AND calls the INxWindow::open() method to
 * create the toolbar.  The toolbar is ready for use upon return.
 *
 * @param height.  The height in rows of the tool bar
 * @param widgetControl. The controlling widget for this window.  If
 *   none is provided, then a new, vanilla CWidgetControl will be created
 *   for the tool bar.
 * @param height Height of the toolbar
 */

CNxToolbar *CNxTkWindow::openToolbar(nxgl_coord_t height, CWidgetControl *widgetControl)
{
  if (m_hNxTkWindow && !m_toolbar)
    {
      // Create a new widget control if none was provided

      CWidgetControl *allocControl = (CWidgetControl *)0;
      if (!widgetControl)
        {
          // NOTE: This constructor would accept the toolbar "style" as a argument.
          // However, we will explicitly set the style below to handle the case
          // where the user has provided a custom widget control

          allocControl = new CWidgetControl();
          if (!allocControl)
            {
              return (CNxToolbar *)0;
            }

          // Use the allocated widget control

          widgetControl = allocControl;
        }

      // Get current window style from the NXTK window's widget control

      CWidgetStyle style;
      m_widgetControl->getWidgetStyle(&style);

      // Set the background color(s) to the color of the toolbar

      style.colors.background         = CONFIG_NXTK_BORDERCOLOR1;
      style.colors.selectedBackground = CONFIG_NXTK_BORDERCOLOR1;

      widgetControl->setWidgetStyle(&style);

      // And create the toolbar

      m_toolbar = new CNxToolbar(this, m_hNxTkWindow,
                                 widgetControl, height);
      if (!m_toolbar)
        {
          if (allocControl)
            {
              delete allocControl;
            }
          return (CNxToolbar *)0;
        }

      // Create the new toolbar.  Toolbar creation is separate from
      // object instantiation so that failures can be reported.

      if (!m_toolbar->open())
        {
          // Failed to create the toolbar. Clean-up and return NULL

          delete m_toolbar;
          if (allocControl)
            {
              delete allocControl;
            }
          return (CNxToolbar *)0;
        }

      // Save the height of the toolbar.  We will need this because it will change
      // how we report the size of drawable part of the window.

      m_toolbarHeight = height;

      // Provide parent widget control information to new widget control instance.
      // This information is reported by an NX callback for "normal" windows.  But
      // the toolbar widget control does not get NX callbacks and has to get the
      // window size through the setWindowBounds method.

      // Disable preemption so that we can be assured that all of the following
      // values are synchronized.

      sched_lock();

      // Get the physical bounding box of the window in display coordinates

      struct nxgl_rect_s windowBounds;
      m_widgetControl->getWindowBoundingBox(&windowBounds);

      // Get the position of the parent window in display coordinates

      struct nxgl_point_s windowPos;
      m_widgetControl->getWindowPosition(&windowPos);

      // Get the bounding box of the toolbar in parent window coordinates

      struct nxgl_rect_s toolbarBounds;
      nxtk_toolbarbounds(m_hNxTkWindow, &toolbarBounds);

      // Get the toolbar size

      struct nxgl_size_s toolbarSize;
      nxgl_rectsize(&toolbarSize, &toolbarBounds);

      // Get the toolbar position in display coordinates by adding the window position

      struct nxgl_point_s toolbarPos;
      nxgl_vectoradd(&toolbarPos, &toolbarBounds.pt1, &windowPos);

      // Perform the fake NX callback

      widgetControl->geometryEvent(m_hNxTkWindow, &toolbarSize,
                                   &toolbarPos, &windowBounds);
      sched_unlock();
    }

  return m_toolbar;
}

/**
 * Request the position and size information of the window. The values
 * will be returned asynchronously through the client callback method.
 * The GetPosition() method may than be called to obtain the positional
 * data as provided by the callback.
 *
 * @return True on success, false on any failure.
 */

bool CNxTkWindow::requestPosition(void)
{
  // Request the window position

  return nxtk_getposition(m_hNxTkWindow) == OK;
}

/**
 * Get the position of the window in the physical display coordinates
 * (as reported by the NX callback).
 *
 * @return The position.
 */

bool CNxTkWindow::getPosition(FAR struct nxgl_point_s *pos)
{
  return m_widgetControl->getWindowPosition(pos);
}

/**
 * Get the size of the window drawable region.
 *
 * @return The size.
 */

bool CNxTkWindow::getSize(FAR struct nxgl_size_s *size)
{
  // Get the size of the NXTK window (this will exclude the thickness of
  // the frame and the height of the toolbar, if any).

  return m_widgetControl->getWindowSize(size);
}

/**
 * Set the position and size of the window.
 *
 * @param pos The new position of the window.
 * @return True on success, false on any failure.
 */

bool CNxTkWindow::setPosition(FAR const struct nxgl_point_s *pos)
{
  // Set the window size and position

  return nxtk_setposition(m_hNxTkWindow, pos) == OK;
}

/**
 * Set the size of the selected window.
 *
 * @param size The new size of the window.
 * @return True on success, false on any failure.
 */

bool CNxTkWindow::setSize(FAR const struct nxgl_size_s *size)
{
  // Set the window size

  return nxtk_setsize(m_hNxTkWindow, size) == OK;
}

/**
 * Bring the window to the top of the display.
 *
 * @return True on success, false on any failure.
 */

bool CNxTkWindow::raise(void)
{
  // Raise the window to the top of the display

  return nxtk_raise(m_hNxTkWindow) == OK;
}

/**
 * Lower the window to the bottom of the display.
 *
 * @return True on success, false on any failure.
 */

bool CNxTkWindow::lower(void)
{
  // Lower the window to the bottom of the display

  return nxtk_lower(m_hNxTkWindow) == OK;
}

/**
 * Set an individual pixel in the window with the specified color.
 *
 * @param pos The location of the pixel to be filled.
 * @param color The color to use in the fill.
 *
 * @return True on success; false on failure.
 */

bool CNxTkWindow::setPixel(FAR const struct nxgl_point_s *pos,
                           nxgl_mxpixel_t color)
{
#if 0
  // Set an individual pixel to the specified color

  return nxtk_setpixel(m_hNxTkWindow, pos, &color) == OK;
#else
  // REVISIT
  return false;
#endif
}

/**
 * Fill the specified rectangle in the window with the specified color.
 *
 * @param pRect The location to be filled.
 * @param color The color to use in the fill.
 *
 * @return True on success; false on failure.
 */

bool CNxTkWindow::fill(FAR const struct nxgl_rect_s *pRect,
                       nxgl_mxpixel_t color)
{
  // Fill a rectangular region with a solid color

  return nxtk_fillwindow(m_hNxTkWindow, pRect, &color) == OK;
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

void CNxTkWindow::getRectangle(FAR const struct nxgl_rect_s *rect, struct SBitmap *dest)
{
  // Get a rectangule region from the window

  (void)nxtk_getwindow(m_hNxTkWindow, rect, 0, (FAR uint8_t*)dest->data, dest->stride);
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

bool CNxTkWindow::fillTrapezoid(FAR const struct nxgl_rect_s *pClip,
                                FAR const struct nxgl_trapezoid_s *pTrap,
                                nxgl_mxpixel_t color)
{
  // Fill a trapezoidal region with a solid color

  return nxtk_filltrapwindow(m_hNxTkWindow, pTrap, &color) == OK;
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

bool CNxTkWindow::drawLine(FAR struct nxgl_vector_s *vector,
                           nxgl_coord_t width, nxgl_mxpixel_t color)
{
  // Draw a line with the specified color

  return nxtk_drawlinewindow(m_hNxTkWindow, vector, width, &color) == OK;
}

/**
 * Draw a filled circle at the specified position, size, and color.
 *
 * @param center The window-relative coordinates of the circle center.
 * @param radius The radius of the rectangle in pixels.
 * @param color The color of the rectangle.
 */

bool CNxTkWindow::drawFilledCircle(struct nxgl_point_s *center, nxgl_coord_t radius,
                                   nxgl_mxpixel_t color)
{
  return nxtk_fillcirclewindow(m_hNxTkWindow, center, radius, &color) == OK;
}

/**
 * Move a rectangular region within the window.
 *
 * @param pRect Describes the rectangular region to move.
 * @param pOffset The offset to move the region.
 *
 * @return True on success; false on failure.
 */

bool CNxTkWindow::move(FAR const struct nxgl_rect_s *pRect,
                       FAR const struct nxgl_point_s *pOffset)
{
  // Move a rectangular region of the display

  return nxtk_movewindow(m_hNxTkWindow, pRect, pOffset) == OK;
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

bool CNxTkWindow::bitmap(FAR const struct nxgl_rect_s *pDest,
                         FAR const void *pSrc,
                         FAR const struct nxgl_point_s *pOrigin,
                         unsigned int stride)
{
  // Copy a rectangular bitmap image in a region on the display

  return nxtk_bitmapwindow(m_hNxTkWindow, pDest, &pSrc, pOrigin, stride) == OK;
}
