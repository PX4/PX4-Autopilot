/****************************************************************************
 * NxWidgets/libnxwidgets/src/cscrollbarpanel.cxx
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

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "cscrollbarpanel.hxx"
#include "cbutton.hxx"

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

CScollbarPanel::CScollbarPanel(CWidgetControl *pWidgetControl,
                               nxgl_coord_t x, nxgl_coord_t y,
                               nxgl_coord_t width, nxgl_coord_t height,
                               uint32_t flags, CWidgetStyle *style)
: CNxWidget(pWidgetControl, x, y, width, height, flags, style)
{
  m_scrollbarWidth         = 10;
  m_scrollbarHeight        = 10;
  m_hasVerticalScrollbar   = true;
  m_hasHorizontalScrollbar = true;

  m_borderSize.top         = 0;
  m_borderSize.right       = 0;
  m_borderSize.left        = 0;
  m_borderSize.bottom      = 0;
  m_flags.borderless       = true;

  m_widgetControl          = pWidgetControl;
  m_panel                  = (CScrollingPanel *)NULL;
  m_scrollbarVertical      = (CScrollbarVertical *)NULL;
  m_scrollbarHorizontal    = (CScrollbarHorizontal *)NULL;

  buildUI();
}

/**
 * Scroll the panel by the specified amounts.
 *
 * @param dx The horizontal distance to scroll.
 * @param dy The vertical distance to scroll.
 */

void CScollbarPanel::scroll(int32_t dx, int32_t dy)
{
  m_panel->scroll(dx, dy);
}

/**
 * Reposition the panel's scrolling region to the specified coordinates.
 *
 * @param x The new x coordinate of the scrolling region.
 * @param y The new y coordinate of the scrolling region.
 */

void CScollbarPanel::jump(int32_t x, int32_t y)
{
  m_panel->jump(x, y);
}

/**
 * Set whether or not horizontal scrolling is allowed.
 *
 * @param allow True to allow horizontal scrolling; false to deny it.
 */

void CScollbarPanel::setAllowsVerticalScroll(bool allow)
{
  m_panel->setAllowsVerticalScroll(allow);

  if (m_hasVerticalScrollbar)
    {
      if (allow)
        {
          m_scrollbarVertical->enable();
        }
      else
        {
          m_scrollbarVertical->disable();
        }
    }
}

/**
 * Set whether or not horizontal scrolling is allowed.
 *
 * @param allow True to allow horizontal scrolling; false to deny it.
 */

void CScollbarPanel::setAllowsHorizontalScroll(bool allow)
{
  m_panel->setAllowsHorizontalScroll(allow);

  if (m_hasHorizontalScrollbar)
    {
      if (allow)
        {
          m_scrollbarHorizontal->enable();
        }
      else
        {
          m_scrollbarHorizontal->disable();
        }
    }
}

/**
 * Sets the width of the virtual canvas.
 *
 * @param width The width of the virtual canvas.
 */

void CScollbarPanel::setCanvasWidth(const int32_t width)
{
  m_panel->setCanvasWidth(width);
  m_scrollbarHorizontal->setMaximumValue(width);
}

/**
 * Sets the height of the virtual canvas.
 *
 * @param height The height of the virtual canvas.
 */

void CScollbarPanel::setCanvasHeight(const int32_t height)
{
  m_panel->setCanvasHeight(height);
  m_scrollbarVertical->setMaximumValue(height);
}

/**
 * Returns true if vertical scrolling is allowed.
 *
 * @return True if vertical scrolling is allowed.
 */

bool CScollbarPanel::allowsVerticalScroll(void) const
{
  return m_panel->allowsVerticalScroll();
}

/**
 * Returns true if horizontal scrolling is allowed.
 *
 * @return True if horizontal scrolling is allowed.
 */

bool CScollbarPanel::allowsHorizontalScroll(void) const
{
  return m_panel->allowsHorizontalScroll();
}

/**
 * Gets the x coordinate of the virtual canvas.
 *
 * @return The x coordinate of the virtual canvas.
 */

const int32_t CScollbarPanel::getCanvasX(void) const
{
  return m_panel->getCanvasX();
}

/**
 * Gets the y coordinate of the virtual canvas.
 *
 * @return The y coordinate of the virtual canvas.
 */

const int32_t CScollbarPanel::getCanvasY(void) const
{
  return m_panel->getCanvasY();
}

/**
 * Gets the width of the virtual canvas.
 *
 * @return The width of the virtual canvas.
 */

const int32_t CScollbarPanel::getCanvasWidth(void) const
{
  return m_panel->getCanvasWidth();
}

/**
 * Gets the height of the virtual canvas.
 *
 * @return The height of the virtual canvas.
 */

const int32_t CScollbarPanel::getCanvasHeight(void) const
{
  return m_panel->getCanvasHeight();
}

/**
 * Handle a widget scroll event.
 *
 * @param e The event data.
 */

void CScollbarPanel::handleScrollEvent(const CWidgetEventArgs &e)
{
  if (e.getSource() != NULL)
    {
      if (e.getSource() == m_panel)
        {
          if (m_scrollbarVertical != NULL)
            {
              m_scrollbarVertical->setRaisesEvents(false);
              m_scrollbarVertical->setValue(0 - m_panel->getCanvasY());
              m_scrollbarVertical->setRaisesEvents(true);
            }
          else if (m_scrollbarHorizontal != NULL)
            {
              m_scrollbarHorizontal->setRaisesEvents(false);
              m_scrollbarHorizontal->setValue(0 - m_panel->getCanvasX());
              m_scrollbarHorizontal->setRaisesEvents(true);
            }
        }
    }
}

/**
 * Handle a widget value change event.
 *
 * @param e The event data.
 */

void CScollbarPanel::handleValueChangeEvent(const CWidgetEventArgs &e)
{
  if (e.getSource() != NULL)
    {
      if (e.getSource() == m_scrollbarVertical)
        {
          if (m_panel != NULL)
            {
              m_panel->setRaisesEvents(false);
              m_panel->jump(m_panel->getCanvasX(), 0 - m_scrollbarVertical->getValue());
              m_panel->setRaisesEvents(true);
            }
        }
      else if (e.getSource() == m_scrollbarHorizontal)
        {
          if (m_panel != NULL)
            {
              m_panel->setRaisesEvents(false);
              m_panel->jump(0 - m_scrollbarHorizontal->getValue(), m_panel->getCanvasY());
              m_panel->setRaisesEvents(true);
            }
        }
    }
}

/**
 * Creates the child widgets.
 */

void CScollbarPanel::buildUI(void)
{
  CRect rect;
  getClientRect(rect);

  nxgl_coord_t panelWidth = rect.getWidth();
  if (m_hasVerticalScrollbar)
    {
      panelWidth -= m_scrollbarWidth;
    }

  nxgl_coord_t panelHeight = rect.getHeight();
  if (m_hasHorizontalScrollbar)
    {
      panelHeight -= m_scrollbarHeight;
    }

  m_panel = new CScrollingPanel(m_widgetControl, rect.getX(), rect.getY(),
                                panelWidth, panelHeight, 0, &m_style);
  addWidget(m_panel);

  CRect panelRect;
  m_panel->getClientRect(panelRect);

  // Disable content scrolling by default, as the panel cannot be drawn to
  // without some additional programmer effort

  m_panel->setContentScrolled(false);

  // Adjust scrollbar dimensions based on scrollbar visibility

  nxgl_coord_t verticalScrollHeight = rect.getHeight();
  if (m_hasHorizontalScrollbar)
    {
      verticalScrollHeight -= m_scrollbarHeight;
    }

  nxgl_coord_t horizontalScrollWidth = rect.getWidth();
  if (m_hasVerticalScrollbar)
    {
      horizontalScrollWidth -= m_scrollbarWidth;
    }

  if (m_hasHorizontalScrollbar)
    {
      m_scrollbarHorizontal =
        new CScrollbarHorizontal(m_widgetControl,
                                 rect.getX(), rect.getHeight() - m_scrollbarHeight,
                                 horizontalScrollWidth, m_scrollbarHeight,
                                 &m_style);
      m_scrollbarHorizontal->setMinimumValue(0);
      m_scrollbarHorizontal->setMaximumValue(getCanvasWidth());
      m_scrollbarHorizontal->setPageSize(panelRect.getWidth());
      m_scrollbarHorizontal->addWidgetEventHandler(this);
      addWidget(m_scrollbarHorizontal);
    }

  if (m_hasVerticalScrollbar)
    {
      m_scrollbarVertical =
        new CScrollbarVertical(m_widgetControl,
                              rect.getWidth() - m_scrollbarWidth,
                              rect.getY(), m_scrollbarWidth,
                              verticalScrollHeight, &m_style);
      m_scrollbarVertical->setMinimumValue(0);
      m_scrollbarVertical->setMaximumValue(getCanvasHeight());
      m_scrollbarVertical->setPageSize(panelRect.getHeight());
      m_scrollbarVertical->addWidgetEventHandler(this);
      addWidget(m_scrollbarVertical);
    }
}

/**
 * Draw the area of this widget that falls within the clipping region.
 * Called by the redraw() function to draw all visible regions.
 *
 * @param port The CGraphicsPort to draw to.
 * @see redraw()
 */

void CScollbarPanel::drawContents(CGraphicsPort *port)
{
  port->drawFilledRect(0, 0, getWidth(), getHeight(), getBackgroundColor());
}
