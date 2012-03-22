/****************************************************************************
 * NxWidgets/libnxwidgets/src/csliderhorizontal.cxx
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

#include "cwidgetcontrol.hxx"
#include "csliderhorizontal.hxx"
#include "csliderhorizontalgrip.hxx"
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
 * @param pWidgetControl The controlling widget for the display
 * @param x The x coordinate of the slider, relative to its parent.
 * @param y The y coordinate of the slider, relative to its parent.
 * @param width The width of the slider.
 * @param height The height of the slider.
 */

CSliderHorizontal::CSliderHorizontal(CWidgetControl *pWidgetControl,
                                     nxgl_coord_t x, nxgl_coord_t y,
                                     nxgl_coord_t width, nxgl_coord_t height)
: CNxWidget(pWidgetControl, x, y, width, height, WIDGET_DRAGGABLE)
{
  m_minimumValue          = 0;
  m_maximumValue          = 0;
  m_contentSize           = 0;
  m_value                 = 0;
  m_minimumGripWidth      = 10;
  m_pageSize              = 1;

  m_flags.permeable       = false;
  m_flags.borderless      = false;
  m_flags.doubleClickable = false;

  // Create grip

  CRect rect;
  getClientRect(rect);

  m_grip = new CSliderHorizontalGrip(pWidgetControl, rect.getX(), rect.getY(),
                                     rect.getWidth(), rect.getHeight());
  m_grip->addWidgetEventHandler(this);
  addWidget(m_grip);

  m_gutterWidth = rect.getWidth();
}

/**
 * Set the value that of the slider.  This will reposition
 * and redraw the grip.
 *
 * @param value The new value.
 */

void CSliderHorizontal::setValue(const nxgl_coord_t value)
{
  setValueWithBitshift((int32_t)value << 16);
}

/**
 * Set the value that of the slider.  This will reposition and redraw
 * the grip.  The supplied value should be bitshifted left 16 places.
 * This ensures greater accuracy than the standard setValue() method if
 * the slider is being used as a scrollbar.
 *
 * @param value The new value.
 */

void CSliderHorizontal::setValueWithBitshift(const int32_t value)
{
  CRect rect;
  getClientRect(rect);
  
  // Can the grip move?

  if ((rect.getWidth() > m_grip->getWidth()) && (m_maximumValue != m_minimumValue))
    {
      int32_t newValue = value;
      int32_t maxValue = getPhysicalMaximumValueWithBitshift();
    
      // Limit to max/min values

      if (newValue > maxValue)
        {
          newValue = maxValue;
        }

      if (newValue >> 16 < m_minimumValue)
        {
          newValue = m_minimumValue << 16;
        }
    
      uint32_t scrollRatio = newValue / m_contentSize;
      int32_t newGripX     = m_gutterWidth * scrollRatio;
      newGripX            += newGripX & 0x8000;
      newGripX           >>= 16;
      newGripX            += rect.getX();
    
      m_grip->moveTo(newGripX, rect.getY());

      // Update stored value if necessary

      if (m_value != newValue)
        {
          m_value = newValue;
          m_widgetEventHandlers->raiseValueChangeEvent();
        }
    }
}

/**
 * Process events fired by the grip.
 *
 * @param e The event details.
 */

void CSliderHorizontal::handleDragEvent(const CWidgetEventArgs &e)
{
  // Handle grip events

  if ((e.getSource() == m_grip) && (e.getSource() != NULL))
    {
      int32_t newValue = getGripValue() >> 16;

      // Grip has moved - compare values and raise event if the
      // value has changed.  Compare using integer values rather
      // than fixed-point.

      if (m_value >> 16 != newValue)
        {
          m_value = newValue << 16;
          m_widgetEventHandlers->raiseValueChangeEvent();
        }
    }
}

/**
 * Get the smallest value that the slider can move through when
 * dragged.
 *
 * @return The smallest value that the slider can move through when
 * dragged.
 */

nxgl_coord_t CSliderHorizontal::getMinimumStep(void) const
{
  // If the ratio of content to gutter is greater than or equal to one,
  // the minimum step that the slider can represent will be that ratio.

  uint32_t gutterRatio = m_contentSize << 16 / m_gutterWidth;
  gutterRatio         += gutterRatio & 0x8000;
  gutterRatio        >>= 16;

  if (gutterRatio > 0)
    {
      return gutterRatio;
    }

  return 1;
}

/**
 * Get the maximum possible value that the slider can represent.  Useful when
 * using the slider as a scrollbar, as the height of the grip prevents the full
 * range of values being accessed (intentionally).
 * The returned value is bitshfted left 16 places for more accuracy in fixed-point
 * calculations.
 *
 * @return The maximum possible value that the slider can represent.
 */

int32_t CSliderHorizontal::getPhysicalMaximumValueWithBitshift(void) const
{
  uint32_t maxX        = m_gutterWidth - m_grip->getWidth();
  uint32_t scrollRatio = (maxX << 16) / m_gutterWidth;
  int32_t value        = (scrollRatio * m_contentSize);

  return value;
}

/**
 * Get the value represented by the top of the grip.
 * return The value represented by the top of the grip.
 */

const int32_t CSliderHorizontal::getGripValue(void) const
{
  // Calculate the current value represented by the top of the grip

  CRect rect;
  getClientRect(rect);

  uint32_t gripPos     = ((m_grip->getX() - getX()) - rect.getX());
  uint32_t scrollRatio = (gripPos << 16) / m_gutterWidth;
  int32_t value        = (scrollRatio * m_contentSize);

  return value;
}

/**
 * Draw the area of this widget that falls within the clipping region.
 * Called by the redraw() function to draw all visible regions.
 *
 * @param port The CGraphicsPort to draw to.
 * @see redraw()
 */

void CSliderHorizontal::drawContents(CGraphicsPort *port)
{
  CRect rect;
  getRect(rect);

  port->drawFilledRect(rect.getX(), rect.getY(),
                       rect.getWidth(), rect.getHeight(),
                       getSelectedBackgroundColor());
}

/**
 * Draw the area of this widget that falls within the clipping region.
 * Called by the redraw() function to draw all visible regions.
 *
 * @param port The CGraphicsPort to draw to.
 * @see redraw()
 */

void CSliderHorizontal::drawBorder(CGraphicsPort *port)
{
  // Stop drawing if the widget indicates it should not have an outline

  if (!isBorderless())
    {
      port->drawBevelledRect(getX(), getY(), getWidth(), getHeight(),
                             getShadowEdgeColor(), getShineEdgeColor());
    }
}

/**
 * Resize the slider to the new dimensions.
 *
 * @param width The new width.
 * @param height The new height.
 */

void CSliderHorizontal::onResize(nxgl_coord_t width, nxgl_coord_t height)
{
  // Remember current values

  int32_t oldValue = m_value;
  bool events = raisesEvents();

  // Disable event raising

  setRaisesEvents(false);
  resizeGrip();

  // Set back to current value

  setValue(oldValue);

  // Reset event raising

  setRaisesEvents(events);
}

/**
 * Moves the grip towards the mouse.
 *
 * @param x The x coordinate of the click.
 * @param y The y coordinate of the click.
 */

void CSliderHorizontal::onClick(nxgl_coord_t x, nxgl_coord_t y)
{
  // Which way should the grip move?

  if (x > m_grip->getX())
    {
      // Move grip right

      setValueWithBitshift(m_value + (m_pageSize << 16));
    }
  else
    {
      // Move grip left

      setValueWithBitshift(m_value - (m_pageSize << 16));
    }
}

/**
 * Resize and redraw the grip.
 */

void CSliderHorizontal::resizeGrip(void)
{
  // Get available size

  CRect rect;
  getClientRect(rect);

  int32_t gripRatio = (m_pageSize << 16) / m_contentSize;
  int32_t gripSize  = rect.getWidth() * gripRatio;
  gripSize        >>= 16;
  m_gutterWidth     = rect.getWidth();

  if (gripSize < m_minimumGripWidth)
    {
      // TODO: Need to implement scaling here.  If we resize the grip to be
      // artificially larger, we effectively reduce the scale (not just the
      // height) of the gutter.  Each position in the gutter needs to be
      // reduced in value.
    }
  
  m_grip->resize(gripSize, rect.getHeight());
}


