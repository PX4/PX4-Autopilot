/****************************************************************************
 * NxWidgets/libnxwidgets/include/cscrollingpanel.hxx
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
 ****************************************************************************
 *
 * Portions of this package derive from Woopsi (http://woopsi.org/) and
 * portions are original efforts.  It is difficult to determine at this
 * point what parts are original efforts and which parts derive from Woopsi.
 * However, in any event, the work of  Antony Dzeryn will be acknowledged
 * in most NxWidget files.  Thanks Antony!
 *
 *   Copyright (c) 2007-2011, Antony Dzeryn
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * * Neither the names "Woopsi", "Simian Zombie" nor the
 *   names of its contributors may be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY Antony Dzeryn ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Antony Dzeryn BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_CSCROLLINGPANEL_HXX
#define __INCLUDE_CSCROLLINGPANEL_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/nx/nxglib.h>

#include "cnxwidget.hxx"
#include "iscrollable.hxx"
#include "cwidgetstyle.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
 
/****************************************************************************
 * Implementation Classes
 ****************************************************************************/
 
#if defined(__cplusplus)

namespace NXWidgets
{
  /**
   * Class containing a scrollable region.  Responds to mouse movement.  Can
   * contain sub-widgets which will also be scrolled.
   */

  class CScrollingPanel : public CNxWidget, public IScrollable
  {
  protected:
    CWidgetControl *m_widgetControl;         /**< Widget control instance */
    int32_t         m_canvasX;               /**< X coordinate of the virtual
                                                  canvas. */
    int32_t         m_canvasY;               /**< Y coordinate of the virtual
                                                 canvas. */
    int32_t         m_canvasWidth;           /**< Width of the virtual canvas. */
    int32_t         m_canvasHeight;          /**< Height of the virtual canvas. */
    bool            m_allowVerticalScroll;   /**< True if vertical scrolling is
                                                  allowed. */
    bool            m_allowHorizontalScroll; /**< True if horizontal scrolling is
                                                  allowed. */
    bool            m_isContentScrolled;     /**< True if the content drawn to the
                                                  panel is scrolled(ie. everything
                                                  drawn in the draw() method);
                                                  false if just child objects are scrolled. */

    /**
     * Draw the area of this widget that falls within the clipping region.
     * Called by the redraw() function to draw all visible regions.
     *
     * @param port The CGraphicsPort to draw to.
     * @see redraw()
     */

    virtual void drawContents(CGraphicsPort *port);

    /**
     * Draw the area of this widget that falls within the clipping region.
     * Called by the redraw() function to draw all visible regions.
     *
     * @param port The CGraphicsPort to draw to.
     * @see redraw()
     */

    virtual void drawBorder(CGraphicsPort *port);

    /**
     * Scrolls the panel to match the drag.
     *
     * @param x The x coordinate of the mouse.
     * @param y The y coordinate of the mouse.
     * @param vX The horizontal drag distance.
     * @param vY The vertical drag distance.
     */

    virtual void onDrag(nxgl_coord_t x, nxgl_coord_t y,
                        nxgl_coord_t vX, nxgl_coord_t vY);
    
    /**
     * Starts the dragging system.
     *
     * @param x The x coordinate of the click.
     * @param y The y coordinate of the click.
     */

    virtual void onClick(nxgl_coord_t x, nxgl_coord_t y);

    /**
     * Scroll all child widgets by the specified amounts.  Actually uses
     * the widget's moveTo() function to reposition them.
     *
     * @param dx The horizontal distance to scroll.
     * @param dy The vertical distance to scroll.
     * @param do_redraw Redraw widgets after moving.
     */

    void scrollChildren(int32_t dx, int32_t dy, bool do_redraw);

    /**
     * Destructor.
     */

    virtual ~CScrollingPanel(void) { }

    /**
     * Copy constructor is protected to prevent usage.
     */

    inline CScrollingPanel(const CScrollingPanel &scrollingPanel)
    : CNxWidget(scrollingPanel) { }

  public:

    /**
     * Constructor.
     *
     * @param pWidgetControl The widget control for the display.
     * @param x The x coordinate of the widget.
     * @param y The y coordinate of the widget.
     * @param width The width of the widget.
     * @param height The height of the widget.
     * @param flags The usual widget flags.
     * @param style The style that the widget should use.  If this is not
     *   specified, the widget will use the values stored in the global
     *   g_defaultWidgetStyle object.  The widget will copy the properties of
     *   the style into its own internal style object.
     */

    CScrollingPanel(CWidgetControl *pWidgetControl,
                    nxgl_coord_t x, nxgl_coord_t y,
                    nxgl_coord_t width, nxgl_coord_t height,
                    uint32_t flags,
                    CWidgetStyle *style = (CWidgetStyle *)NULL);

    /**
     * Scroll the panel by the specified amounts.
     *
     * @param dx The horizontal distance to scroll.
     * @param dy The vertical distance to scroll.
     */

    virtual void scroll(int32_t dx, int32_t dy);
    
    /**
     * Reposition the panel's scrolling region to the specified coordinates.
     *
     * @param x The new x coordinate of the scrolling region.
     * @param y The new y coordinate of the scrolling region.
     */

    virtual void jump(int32_t x, int32_t y);

    /**
     * Returns true if vertical scrolling is allowed.
     *
     * @return True if vertical scrolling is allowed.
     */

    inline bool allowsVerticalScroll(void) const
    {
      return m_allowVerticalScroll;
    }

    /**
     * Returns true if horizontal scrolling is allowed.
     *
     * @return True if horizontal scrolling is allowed.
     */

    inline bool allowsHorizontalScroll(void) const
    {
      return m_allowHorizontalScroll;
    }

    /**
     * Gets the x coordinate of the virtual canvas.
     *
     * @return The x coordinate of the virtual canvas.
     */

    virtual inline const int32_t getCanvasX(void) const
    {
      return m_canvasX;
    }
    
    /**
     * Gets the y coordinate of the virtual canvas.
     *
     * @return The y coordinate of the virtual canvas.
     */

    virtual inline const int32_t getCanvasY(void) const
    {
      return m_canvasY;
    }

    /**
     * Gets the width of the virtual canvas.
     *
     * @return The width of the virtual canvas.
     */

    virtual inline const int32_t getCanvasWidth(void) const
    {
      return m_canvasWidth;
    }
    
    /**
     * Gets the height of the virtual canvas.
     *
     * @return The height of the virtual canvas.
     */

    virtual inline const int32_t getCanvasHeight(void) const
    {
      return m_canvasHeight;
    }

    /**
     * Set whether or not horizontal scrolling is allowed.
     *
     * @param allow True to allow horizontal scrolling; false to deny it.
     */

    inline void setAllowsVerticalScroll(bool allow)
    {
      m_allowVerticalScroll = allow;
    }

    /**
     * Set whether or not horizontal scrolling is allowed.
     *
     * @param allow True to allow horizontal scrolling; false to deny it.
     */

    inline void setAllowsHorizontalScroll(bool allow)
    {
      m_allowHorizontalScroll = allow;
    }

    /**
     * Set whether or not the content of the panel is scrolled.
     * Content is anything drawn to the panel in the draw() method.
     * This property is disabled by default, which will result in
     * faster scrolling of child objects.
     * If the panel contains no child objects, just draw() method
     * content, consider using a SuperBitmap class instead.
     *
     * @param scrolled True to enable content scrolling; false to disable it.
     */

    inline void setContentScrolled(bool scrolled)
    {
      m_isContentScrolled = scrolled;
    }

    /**
     * Check if the content of the panel, drawn via the draw() method,
     * is scrolled.
     *
     * @return True if the content is scrolled; false if not.
     */

    inline bool IsContentScrolled(void)
    {
      return m_isContentScrolled;
    }

    /**
     * Sets the width of the virtual canvas.
     *
     * @param width The width of the virtual canvas.
     */

    virtual inline void setCanvasWidth(const int32_t width)
    {
      m_canvasWidth = width;
    }
    
    /**
     * Sets the height of the virtual canvas.
     *
     * @param height The height of the virtual canvas.
     */

    virtual inline void setCanvasHeight(const int32_t height)
    {
      m_canvasHeight = height;
    }
  };
}

#endif // __cplusplus

#endif // __INCLUDE_CSCROLLINGPANEL_HXX

