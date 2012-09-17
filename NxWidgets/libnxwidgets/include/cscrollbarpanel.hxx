/****************************************************************************
 * NxWidgets/libnxwidgets/include/cscrollbarpanel.hxx
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

#ifndef __INCLUDE_CSCROLLBARPANEL_HXX
#define __INCLUDE_CSCROLLBARPANEL_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/nx/nxglib.h>

#include "cnxwidget.hxx"
#include "cscrollingpanel.hxx"
#include "cwidgetstyle.hxx"
#include "cscrollbarvertical.hxx"
#include "cscrollbarhorizontal.hxx"
#include "cwidgeteventhandler.hxx"
#include "iscrollable.hxx"
#include "cgraphicsport.hxx"

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
   * Class containing a scrolling panel bordered by scrollbars.
   */
  class CScollbarPanel : public CNxWidget, public IScrollable,
                         public CWidgetEventHandler
  {
  protected:
    CWidgetControl       *m_widgetControl;          /**< Widget control instance */
    CScrollingPanel      *m_panel;                  /**< Internal panel that
                                                         contains children. */
    CScrollbarHorizontal *m_scrollbarHorizontal;    /**< Horizontal scrollbar. */
    CScrollbarVertical   *m_scrollbarVertical;      /**< Vertical scrollbar. */
    uint8_t               m_scrollbarWidth;         /**< Width of the vertical
                                                         scrollbar. */
    uint8_t               m_scrollbarHeight;        /**< Height of the horizontal
                                                         scrollbar. */
    bool                  m_hasVerticalScrollbar;   /**< Indicates the presence of
                                                         a vertical scrollbar. */
    bool                  m_hasHorizontalScrollbar; /**< Indicates the presence of
                                                         a horizontal scrollbar. */

    /**
     * Creates the child widgets.
     */

    void buildUI(void);

    /**
     * Draw the area of this widget that falls within the clipping region.
     * Called by the redraw() function to draw all visible regions.
     *
     * @param port The CGraphicsPort to draw to.
     * @see redraw()
     */

    virtual void drawContents(CGraphicsPort *port);

    /**
     * Destructor.
     */

    virtual ~CScollbarPanel(void) { }

    /**
     * Copy constructor is protected to prevent usage.
     */

    inline CScollbarPanel(const CScollbarPanel &scrollbarPanel)
    : CNxWidget(scrollbarPanel) { }

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

    CScollbarPanel(CWidgetControl *pWidgetControl,
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
     * Set whether or not horizontal scrolling is allowed.
     *
     * @param allow True to allow horizontal scrolling; false to deny it.
     */

    virtual void setAllowsVerticalScroll(bool allow);

    /**
     * Set whether or not horizontal scrolling is allowed.
     *
     * @param allow True to allow horizontal scrolling; false to deny it.
     */

    virtual void setAllowsHorizontalScroll(bool allow);

    /**
     * Sets the width of the virtual canvas.
     *
     * @param width The width of the virtual canvas.
     */

    virtual void setCanvasWidth(const int32_t width);
    
    /**
     * Sets the height of the virtual canvas.
     *
     * @param height The height of the virtual canvas.
     */

    virtual void setCanvasHeight(const int32_t height);

    /**
     * Returns true if vertical scrolling is allowed.
     *
     * @return True if vertical scrolling is allowed.
     */

    virtual bool allowsVerticalScroll(void) const;

    /**
     * Returns true if horizontal scrolling is allowed.
     *
     * @return True if horizontal scrolling is allowed.
     */

    virtual bool allowsHorizontalScroll(void) const;

    /**
     * Gets the x coordinate of the virtual canvas.
     *
     * @return The x coordinate of the virtual canvas.
     */

    virtual const int32_t getCanvasX(void) const;
    
    /**
     * Gets the y coordinate of the virtual canvas.
     *
     * @return The y coordinate of the virtual canvas.
     */

    virtual const int32_t getCanvasY(void) const;

    /**
     * Gets the width of the virtual canvas.
     *
     * @return The width of the virtual canvas.
     */

    virtual const int32_t getCanvasWidth(void) const;
    
    /**
     * Gets the height of the virtual canvas.
     *
     * @return The height of the virtual canvas.
     */

    virtual const int32_t getCanvasHeight(void) const;

    /**
     * Handle a widget scroll event.
     *
     * @param e The event data.
     */

    void handleScrollEvent(const CWidgetEventArgs &e);

    /**
     * Handle a widget value change event.
     *
     * @param e The event data.
     */

    void handleValueChangeEvent(const CWidgetEventArgs &e);

    /**
     * Gets a pointer to the CScrollingPanel widget contained within
     * this widget.
     *
     * @return A pointer to the CScrollingPanel widget.
     */

    inline CScrollingPanel *getPanel(void)
    {
      return m_panel;
    }
  };
}

#endif // __cplusplus

#endif // __INCLUDE_CSCROLLBARPANEL_HXX

