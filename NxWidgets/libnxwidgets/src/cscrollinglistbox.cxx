/****************************************************************************
 * NxWidgets/libnxwidgets/src/cscrollinglistbox.cxx
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

#include "cscrollinglistbox.hxx"
#include "cscrollbarvertical.hxx"
#include "cgraphicsport.hxx"

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
 * @param style The style that the widget should use.  If this is not
 *   specified, the widget will use the values stored in the global
 *   g_defaultWidgetStyle object.  The widget will copy the properties of
 *   the style into its own internal style object.
 */

CScrollingListBox::CScrollingListBox(CWidgetControl *pWidgetControl,
                                     nxgl_coord_t x, nxgl_coord_t y,
                                     nxgl_coord_t width, nxgl_coord_t height,
                                     CWidgetStyle *style)
: CNxWidget(pWidgetControl, x, y, width, height, 0, style)
{
  m_scrollbarWidth = 10;

  setBorderless(true);

  m_listbox = new CListBox(pWidgetControl, 0, 0, width - m_scrollbarWidth,
                           height, &m_style);
  m_listbox->addWidgetEventHandler(this);

  // Create scrollbar

  CRect rect;
  m_listbox->getClientRect(rect);
  m_scrollbar = new CScrollbarVertical(pWidgetControl, width - m_scrollbarWidth,
                                       0, m_scrollbarWidth, height, &m_style);
  m_scrollbar->setMinimumValue(0);
  m_scrollbar->setMaximumValue(0);
  m_scrollbar->setPageSize(rect.getHeight() / m_listbox->getOptionHeight());
  m_scrollbar->addWidgetEventHandler(this);

  // Add children to child array

  addWidget(m_listbox);
  addWidget(m_scrollbar);
}

/**
 * Add a new option to the widget using default colors.
 *
 * @param text Text to show in the option.
 * @param value The value of the option.
 */

void CScrollingListBox::addOption(const CNxString &text, const uint32_t value)
{
  m_listbox->addOption(text, value);
  m_scrollbar->setMaximumValue(m_listbox->getOptionCount() - 1);
}

/**
 * Add an option to the widget.
 *
 * @param option The option to add.
 */

void CScrollingListBox::addOption(CListBoxDataItem *item)
{
  m_listbox->addOption(item);
  m_scrollbar->setMaximumValue(m_listbox->getOptionCount() - 1);
}

/**
 * Add a new option to the widget.
 *
 * @param text Text to show in the option.
 * @param value The value of the option.
 * @param normalTextColor Color to draw the text with when not selected.
 * @param normalBackColor Color to draw the background with when not selected.
 * @param selectedTextColor Color to draw the text with when selected.
 * @param selectedBackColor Color to draw the background with when selected.
 */

void CScrollingListBox::addOption(const CNxString &text, const uint32_t value,
                                  const nxwidget_pixel_t normalTextColor,
                                  const nxwidget_pixel_t normalBackColor,
                                  const nxwidget_pixel_t selectedTextColor,
                                  const nxwidget_pixel_t selectedBackColor)
{
  m_listbox->addOption(text, value, normalTextColor, normalBackColor,
                       selectedTextColor, selectedBackColor);
  m_scrollbar->setMaximumValue(m_listbox->getOptionCount() - 1);
}

/**
 * Remove an option from the widget by its index.
 *
 * @param index The index of the option to remove.
 */

void CScrollingListBox::removeOption(const int index)
{
  m_listbox->removeOption(index);
  m_scrollbar->setMaximumValue(m_listbox->getOptionCount() - 1);

  // Reposition grip if necessary

  if (m_scrollbar->getValue() > m_listbox->getOptionCount())
    {
      m_scrollbar->setValue(0);
    }
}

/**
 * Remove all options from the widget.
 */

void CScrollingListBox::removeAllOptions(void)
{
  m_listbox->removeAllOptions();
  m_scrollbar->setMaximumValue(0);
  m_scrollbar->setValue(0);
}

/**
 * Handles events raised by its sub-widgets.
 *
 * @param e Event arguments.
 */

void CScrollingListBox::handleValueChangeEvent(const CWidgetEventArgs &e)
{
  if (e.getSource() != NULL)
    {
      if (e.getSource() == m_scrollbar)
        {
          if (m_listbox != NULL)
            {
              m_listbox->setRaisesEvents(false);
              m_listbox->jump(0, 0 - (m_scrollbar->getValue() * m_listbox->getOptionHeight()));
              m_listbox->setRaisesEvents(true);
            }
        }
      else if (e.getSource() == m_listbox)
        {
          m_widgetEventHandlers->raiseValueChangeEvent();
        }
    }
}

/**
 * Handle a widget action event.
 *
 * @param e The event data.
 */

void CScrollingListBox::handleActionEvent(const CWidgetEventArgs &e)
{
  if (e.getSource() != NULL)
    {
      if (e.getSource() == m_listbox)
        {
          // Raise action events from list box to event handler

          m_widgetEventHandlers->raiseActionEvent();
        }
    }
}

/**
 * Handles events raised by its sub-widgets.
 *
 * @param e Event arguments.
 */

void CScrollingListBox::handleScrollEvent(const CWidgetEventArgs &e)
{
  if (e.getSource() != NULL)
    {
      if (e.getSource() == m_listbox)
        {
          if (m_scrollbar != NULL)
            {
              m_scrollbar->setRaisesEvents(false);

              int value = ((0 - m_listbox->getCanvasY()) << 16) / m_listbox->getOptionHeight();

              m_scrollbar->setValueWithBitshift(value);
              m_scrollbar->setRaisesEvents(true);
            }
        }
    }
}

/**
 * Handle a mouse button click event.
 *
 * @param e The event data.
 */

void CScrollingListBox::handleClickEvent(const CWidgetEventArgs &e)
{
  m_widgetEventHandlers->raiseClickEvent(e.getX(), e.getY());
}

/**
 * Handles events raised by its sub-widgets.
 *
 * @param e Event arguments.
 */

void CScrollingListBox::handleDoubleClickEvent(const CWidgetEventArgs &e)
{
  m_widgetEventHandlers->raiseDoubleClickEvent(e.getX(), e.getY());
}

/**
 * Handle a mouse button release event that occurred within the bounds of
 * the source widget.
 *
 * @param e The event data.
 */

void CScrollingListBox::handleReleaseEvent(const CWidgetEventArgs &e)
{
  m_widgetEventHandlers->raiseReleaseEvent(e.getX(), e.getY());
}

/**
 * Handle a mouse button release event that occurred outside the bounds of
 * the source widget.
 *
 * @param e The event data.
 */

void CScrollingListBox::handleReleaseOutsideEvent(const CWidgetEventArgs &e)
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
 * Set the font used in the textbox.
 *
 * @param font Pointer to the new font.
 */

void CScrollingListBox::setFont(CNxFont *font)
{
  m_style.font = font;
  m_listbox->setFont(font);
  m_scrollbar->setFont(font);
}

/**
 * Insert the dimensions that this widget wants to have into the rect
 * passed in as a parameter.  All coordinates are relative to the widget's
 * parent.  Value is based on the length of the largest string in the
 * set of options.
 *
 * @param rect Reference to a rect to populate with data.
 */

void CScrollingListBox::getPreferredDimensions(CRect &rect) const
{
  // Get the listbox's preferred dimensions

  m_listbox->getPreferredDimensions(rect);

  // Add on the scrollbar width

  nxgl_coord_t width = rect.getWidth();
  rect.setWidth(width + m_scrollbarWidth);
}

/**
 * Draw the area of this widget that falls within the clipping region.
 * Called by the redraw() function to draw all visible regions.
 *
 * @param port The CGraphicsPort to draw to.
 * @see redraw()
 */

void CScrollingListBox::drawContents(CGraphicsPort *port)
{
  port->drawFilledRect(0, 0, getWidth(), getHeight(), getBackgroundColor());
}

/**
 * Resize the listbox to the new dimensions.
 *
 * @param width The new width.
 * @param height The new height.
 */

void CScrollingListBox::onResize(nxgl_coord_t width, nxgl_coord_t height)
{
  // Resize the children

  m_listbox->resize(width - m_scrollbarWidth, height);
  m_scrollbar->resize(m_scrollbarWidth, height);

  // Adjust scrollbar page size

  CRect rect;
  getClientRect(rect);
  m_scrollbar->setPageSize(rect.getHeight() / m_listbox->getOptionHeight());

  // Move the scrollbar

  m_scrollbar->moveTo(width - m_scrollbarWidth, 0);
}




