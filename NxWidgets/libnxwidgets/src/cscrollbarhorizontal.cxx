/****************************************************************************
 * NxWidgets/libnxwidgets/include/cscrollbarhorizontal.hxx
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

#include <nuttx/nx/nxglib.h>

#include "nxconfig.hxx"
#include "cnxwidget.hxx"
#include "cwidgeteventhandler.hxx"
#include "islider.hxx"
#include "cscrollbarhorizontal.hxx"
#include "cglyphbutton.hxx"
#include "csliderhorizontal.hxx"
#include "cnxtimer.hxx"
#include "glyphs.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
 
/****************************************************************************
 * Method Implementation
 ****************************************************************************/

using namespace NXWidgets;

/**
 * Constructor.
 *
 * @param pWidgetControl The widget control instance for the window.
 * @param x The x coordinate of the slider, relative to its parent.
 * @param y The y coordinate of the slider, relative to its parent.
 * @param width The width of the slider.
 * @param height The height of the slider.
 * @param style The style that the widget should use.  If this is not
 *   specified, the widget will use the values stored in the global
 *   g_defaultWidgetStyle object.  The widget will copy the properties of
 *   the style into its own internal style object.
 */

CScrollbarHorizontal::CScrollbarHorizontal(CWidgetControl *pWidgetControl,
                                           nxgl_coord_t x, nxgl_coord_t y,
                                           nxgl_coord_t width, nxgl_coord_t height,
                                           CWidgetStyle* style) 
: CNxWidget(pWidgetControl, x, y, width, height, WIDGET_BORDERLESS, style)
{
  m_buttonWidth = 10;

  // Create the children

  m_slider = new CSliderHorizontal(pWidgetControl, m_buttonWidth, 0,
                                   width - (m_buttonWidth << 1), height);
  m_slider->addWidgetEventHandler(this);

  // Set up the left glyph button

  m_leftButton = new CGlyphButton(pWidgetControl,
                                  0, 0,
                                  m_buttonWidth, height,
                                  0, 0,
                                  &g_arrowLeft, &g_arrowLeft, &m_style);
  m_leftButton->addWidgetEventHandler(this);

  // Set up the right glyph button

  m_rightButton = new CGlyphButton(pWidgetControl,
                                   width - m_buttonWidth, 0,
                                   m_buttonWidth, height,
                                   0, 0,
                                   &g_arrowRight, &g_arrowRight, &m_style);
  m_rightButton->addWidgetEventHandler(this);

  // Create timer

  m_scrollTimeout   = 10;
  m_timer           = new CNxTimer(pWidgetControl, m_scrollTimeout, true);
  m_timer->addWidgetEventHandler(this);

  addWidget(m_slider);
  addWidget(m_leftButton);
  addWidget(m_rightButton);
  addWidget(m_timer);
}

/**
 * Get the smallest value that the slider can represent.
 *
 * @return The smallest value.
 */

const nxgl_coord_t CScrollbarHorizontal::getMinimumValue(void) const
{
  return m_slider->getMinimumValue();
}

/**
 * Get the largest value that the slider can represent.
 *
 * @return The largest value.
 */

const nxgl_coord_t CScrollbarHorizontal::getMaximumValue(void) const
{
  return m_slider->getMaximumValue();
}

/**
 * Get the current value of the slider.
 *
 * @return The current slider value.
 */

const nxgl_coord_t CScrollbarHorizontal::getValue(void) const
{
  return m_slider->getValue();
}

/**
 * Get the value represented by the height of the grip.
 * For sliders, this would typically be 1 (so each new
 * grip position is worth 1).  For scrollbars, this
 * would be the height of the scrolling widget.
 *
 * @return The page size.
 */

const nxgl_coord_t CScrollbarHorizontal::getPageSize(void) const
{
  return m_slider->getPageSize();
}

/**
 * Set the smallest value that the slider can represent.
 *
 * @param value The smallest value.
 */

void CScrollbarHorizontal::setMinimumValue(const nxgl_coord_t value)
{
  m_slider->setMinimumValue(value);
}

/**
 * Set the largest value that the slider can represent.
 *
 * @param value The largest value.
 */

void CScrollbarHorizontal::setMaximumValue(const nxgl_coord_t value)
{
  m_slider->setMaximumValue(value);
}

/**
 * Set the value that of the slider.  This will reposition
 * and redraw the grip.
 *
 * @param value The new value.
 */

void CScrollbarHorizontal::setValue(const nxgl_coord_t value)
{
  m_slider->setValue(value);
}

/**
 * Set the value that of the slider.  This will reposition and redraw
 * the grip.  The supplied value should be bitshifted left 16 places.
 * This ensures greater accuracy than the standard setValue() method if
 * the slider is being used as a scrollbar.
 *
 * @param value The new value.
 */

void CScrollbarHorizontal::setValueWithBitshift(const int32_t value)
{
  m_slider->setValueWithBitshift(value);
}

/**
 * Set the page size represented by the grip.
 *
 * @param pageSize The page size.
 * @see getPageSize().
 */

void CScrollbarHorizontal::setPageSize(nxgl_coord_t pageSize)
{
  m_slider->setPageSize(pageSize);
}

/**  
 * Process events fired by the grip.
 *
 * @param e The event details.
 */

void CScrollbarHorizontal::handleActionEvent(const CWidgetEventArgs &e)
{
  // Check which widget fired the event

  if (e.getSource() == m_timer)
    {
      // Which widget is clicked?

      if (m_leftButton->isClicked())
        {
          // Move the grip left

          m_slider->setValue(m_slider->getValue() - m_slider->getMinimumStep());
        }
      else if (m_rightButton->isClicked())
        {
          // Move the grip right

          m_slider->setValue(m_slider->getValue() + m_slider->getMinimumStep());
        }
    }
}

/**
 * Process events fired by the grip.
 *
 * @param e The event details.
 */

void CScrollbarHorizontal::handleClickEvent(const CWidgetEventArgs &e)
{
  if (e.getSource() == m_leftButton)
    {
      // Start the timer

      m_timer->start();

      // Move the grip left

      m_slider->setValue(m_slider->getValue() - m_slider->getMinimumStep());
    }
  else if (e.getSource() == m_rightButton)
    {
      // Start the timer

      m_timer->start();

      // Move the grip right

      m_slider->setValue(m_slider->getValue() + m_slider->getMinimumStep());
    }
}

/**
 * Process events fired by the grip.
 *
 * @param e The event details.
 */

void CScrollbarHorizontal::handleReleaseEvent(const CWidgetEventArgs &e)
{
  if ((e.getSource() == m_leftButton) || (e.getSource() == m_rightButton))
    {
      // Stop the timer

      m_timer->stop();
    }
}

/**
 * Process events fired by the grip.
 *
 * @param e The event details.
 */

void CScrollbarHorizontal::handleReleaseOutsideEvent(const CWidgetEventArgs &e)
{
  if ((e.getSource() == m_leftButton) || (e.getSource() == m_rightButton))
    {
      // Stop the timer

      m_timer->stop();
    }
}

/**
 * Process events fired by the grip.
 *
 * @param e The event details.
 */

void CScrollbarHorizontal::handleValueChangeEvent(const CWidgetEventArgs &e)
{
  if (e.getSource() == m_slider)
    {
      m_widgetEventHandlers->raiseValueChangeEvent();
    }
}

/**
 * Resize the scrollbar to the new dimensions.
 *
 * @param width The new width.
 * @param height The new height.
 */

void CScrollbarHorizontal::onResize(nxgl_coord_t width, nxgl_coord_t height)
{
  // Remember current values

  nxgl_coord_t value = getValue();
  bool events = raisesEvents();

  // Disable event raising

  setRaisesEvents(false);

  // Resize and move children

  m_slider->resize(width - (m_buttonWidth << 1), height);
  m_leftButton->moveTo(m_slider->getWidth(), 0);
  m_rightButton->moveTo(m_slider->getWidth() + m_buttonWidth, 0);

  // Set back to current value

  setValue(value);

  // Reset event raising

  setRaisesEvents(events);
}
