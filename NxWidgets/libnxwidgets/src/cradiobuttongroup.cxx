/****************************************************************************
 * NxWidgets/libnxwidgets/src/cradiobutton.cxx
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include <nuttx/nx/nxglib.h>

#include "cradiobuttongroup.hxx"
#include "cradiobutton.hxx"
#include "cgraphicsport.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * CLabel Method Implementations
 ****************************************************************************/

using namespace NXWidgets;

/**
 * Constructor.  Note that the group determines its width and height
 * from the position and dimensions of its children.
 *
 * @param pWidgetControl The controlling widget for the display.
 * @param x The x coordinate of the group.
 * @param y The y coordinate of the group.
 * @param style The style that the button should use.  If this is not
 *        specified, the button will use the global default widget
 *        style.
 */

CRadioButtonGroup::CRadioButtonGroup(CWidgetControl *pWidgetControl,
                                     nxgl_coord_t x, nxgl_coord_t y,
                                     CWidgetStyle *style)
: CNxWidget(pWidgetControl, x, y, 0, 0, WIDGET_BORDERLESS, style)
{
  m_widgetControl  = pWidgetControl;
  m_selectedWidget = (CRadioButton *)NULL;
}

/**
 * Simple method for adding a new radio button to the group.
 * This should be used in preference to the usual addWidget() method,
 * as this method automatically resizes the group.
 *
 * @param x The x coordinate of the new button, relative to this
 * widget.
 * @param y The y coordinate of the new button, relative to this
 * widget.
 * @param width The width of the new button.
 * @param height The height of the new button.
 */

CRadioButton *CRadioButtonGroup::newRadioButton(nxgl_coord_t x, nxgl_coord_t y,
                                                nxgl_coord_t width, nxgl_coord_t height)
{
  CRadioButton *newButton = new CRadioButton(m_widgetControl, x, y, width, height, &m_style);
  newButton->addWidgetEventHandler(this);
  addWidget(newButton);

  // Do we need to resize?

  nxgl_coord_t newWidth  = getWidth();
  nxgl_coord_t newHeight = getHeight();

  if (newWidth < x + width)
    {
      newWidth = x + width;
    }

  if (newHeight < y + height)
    {
      newHeight = y + height;
    }

  resize(newWidth, newHeight);
  return newButton;
}

/**
 * Gets a pointer to the selected widget.
 *
 * @return Pointer to the selected widget.
 */

const CRadioButton *CRadioButtonGroup::getSelectedWidget(void) const
{
  return (CRadioButton *)m_selectedWidget;
}

/**
 * Gets the index of the selected widget.
 *
 * @return The index of the selected widget.
 */

const int CRadioButtonGroup::getSelectedIndex(void) const
{
  for (int i = 0; i < m_children.size(); i++)
    {
      if (((CRadioButton*)m_children[i]) == m_selectedWidget)
        {
          return i;
        }
    }

  // Nothing selected

  return -1;
}

/**
 * Sets the selected radio button to the supplied widget.
 *
 * @param widget The radio button to select.
 */

void CRadioButtonGroup::setSelectedWidget(CRadioButton* widget)
{
  if (m_selectedWidget != widget)
    {
      if (m_selectedWidget != NULL)
        {
          m_selectedWidget->setState(CRadioButton::RADIO_BUTTON_STATE_OFF);
        }

    m_selectedWidget = widget;

    if (m_selectedWidget != NULL)
      {
        m_selectedWidget->setState(CRadioButton::RADIO_BUTTON_STATE_ON);
      }

    m_widgetEventHandlers->raiseValueChangeEvent();
  }
}

/**
 * Selects the widget at the specified index.
 *
 * @param index The index of the widget to select.
 */

void CRadioButtonGroup::setSelectedIndex(int index)
{
  if (index < m_children.size())
    {
      setSelectedWidget((CRadioButton*)m_children[index]);
      m_widgetEventHandlers->raiseValueChangeEvent();
    }
}

/**
 * Insert the dimensions that this widget wants to have into the rect
 * passed in as a parameter.  All coordinates are relative to the
 * widget's parent.  Value is based on the length of the largest string
 * in the set of options.
 *
 * @param rect Reference to a rect to populate with data.
 */

void CRadioButtonGroup::getPreferredDimensions(CRect& rect) const
{
  nxgl_coord_t width = 0;
  nxgl_coord_t height = 0;

  if (!m_flags.borderless)
    {
      width  = m_borderSize.left + m_borderSize.right;
      height = m_borderSize.top + m_borderSize.bottom;
    }

  nxgl_coord_t widgetX = 0;
  nxgl_coord_t widgetY = 0;

  nxgl_coord_t maxX = 0;
  nxgl_coord_t maxY = 0;

  // Locate largest x and y coords within children

  for (int i = 0; i < m_children.size(); ++i)
    {
      widgetX = m_children[i]->getX() + m_children[i]->getWidth();
      widgetY = m_children[i]->getY() + m_children[i]->getHeight();

      if (widgetX > maxX)
        {
          maxX = widgetX;
        }

      if (widgetY > maxY)
        {
          maxY = widgetY;
        }
    }

  rect.setX(m_rect.getX());
  rect.setY(m_rect.getY());
  rect.setWidth(width + maxX - getX());
  rect.setHeight(height + maxY - getY());
}

/**
 * Handle a mouse click event.
 *
 * @param e The event data.
 */

void CRadioButtonGroup::handleClickEvent(const CWidgetEventArgs &e)
{
  m_widgetEventHandlers->raiseClickEvent(e.getX(), e.getY());
}

/**
 * Handle a mouse double-click event.
 *
 * @param e The event data.
 */

void CRadioButtonGroup::handleDoubleClickEvent(const CWidgetEventArgs &e)
{
  m_widgetEventHandlers->raiseDoubleClickEvent(e.getX(), e.getY());
}

/**
 * Handle a mouse button release event that occurred within the bounds of
 * the source widget.
 * @param e The event data.
 */

void CRadioButtonGroup::handleReleaseEvent(const CWidgetEventArgs &e)
{
  m_widgetEventHandlers->raiseReleaseEvent(e.getX(), e.getY());
}

/**
 * Handle a mouse button release event that occurred outside the bounds of
 * the source widget.
 *
 * @param e The event data.
 */

void CRadioButtonGroup::handleReleaseOutsideEvent(const CWidgetEventArgs &e)
{
  // Child raised a release outside event, but we need to raise a different
  // event if the release occurred within the bounds of this parent widget

  if (checkCollision(e.getX(), e.getY()))
    {
      m_widgetEventHandlers->raiseReleaseEvent(e.getX(), e.getY());
    }
  else
    {
      m_widgetEventHandlers->raiseReleaseOutsideEvent(e.getX(), e.getY());
    }
}

/**
 * Draw the area of this widget that falls within the clipping region.
 * Called by the redraw() function to draw all visible regions.
 * @param port The CGraphicsPort to draw to.
 * @see redraw()
 */

void CRadioButtonGroup::drawContents(CGraphicsPort *port)
{
  port->drawFilledRect(getX(), getY(), getWidth(), getHeight(), getBackgroundColor());
}
