/****************************************************************************
 * NxWidgets/libnxwidgets/src/cprogressbar.cxx
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
#include <cstdio>
#include <cstring>

#include <nuttx/nx/nxglib.h>

#include "cprogressbar.hxx"
#include "cgraphicsport.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * CButton Method Implementations
 ****************************************************************************/

using namespace NXWidgets;

/**
 * Constructor.
 *
 * @param pWidgetControl The widget control for the display.
 * @param x The x coordinate of the progress bar, relative to its parent.
 * @param y The y coordinate of the progress bar, relative to its parent.
 * @param width The width of the progress bar.
 * @param height The height of the progress bar.
 */

CProgressBar::CProgressBar(CWidgetControl *pWidgetControl,
                           nxgl_coord_t x, nxgl_coord_t y,
                           nxgl_coord_t width, nxgl_coord_t height)
: CNxWidget(pWidgetControl, x, y, width, height, 0)
{
  m_minimumValue       = 0;
  m_maximumValue       = 0;
  m_value              = 0;
  m_showPercentageText = true;
  m_flags.borderless   = false;
}

/**
 * Set the value that of the progress bar. 
 *
 * @param value The new value.
 */

void CProgressBar::setValue(const int16_t value)
{
  m_value = value;

  // Limit to max/min values

  if (m_value > m_maximumValue)
    {
      m_value = m_maximumValue;
    }

  if (m_value < m_minimumValue)
    {
      m_value = m_minimumValue;
    }

  redraw();
}

/**
 * Draw the area of this widget that falls within the clipping region.
 * Called by the redraw() function to draw all visible regions.
 *
 * @param port The CGraphicsPort to draw to.
 * @see redraw()
 */

void CProgressBar::drawContents(CGraphicsPort *port)
{
  // Get the drawing window, accounting for the presence of any border

  CRect rect;
  getRect(rect);

  // Calculate ratio of pixels to value range (max fractional value of 255)

  uint32_t ratio = ((uint32_t)rect.getWidth() << 8) / (uint32_t)(m_maximumValue - m_minimumValue);
  
  // Convert value using ratio, rounding up and shifting down

  int16_t barWidth = ((m_value * ratio) + 128) >> 8;

  // Draw filled region

  port->drawFilledRect(rect.getX(), rect.getY(), barWidth, rect.getHeight(), getHighlightColor());
  
  // Draw unfilled background

  if (barWidth < rect.getWidth())
    {
      port->drawFilledRect(rect.getX() + barWidth, rect.getY(),
                           rect.getWidth() - barWidth, rect.getHeight(),
                           getBackgroundColor());
    }

  // Draw completion percentage text

  if (m_showPercentageText)
    {
      char text[6];
      sprintf(text, "%d%%", (100 * m_value) / (m_maximumValue - m_minimumValue));

      struct nxgl_point_s pos;
      pos.x = rect.getX() +
              ((rect.getWidth() - getFont()->getStringWidth(text)) >> 1);
      pos.y = rect.getY() +
              ((rect.getHeight() - getFont()->getHeight()) >> 1);

      // Determine the background and text color

      nxgl_mxpixel_t textColor;

     if (!isEnabled())
        {
          textColor = getDisabledTextColor();
        }
      else
        {
          textColor = getEnabledTextColor();
        }

      // And draw the text using the selected color

      port->drawText(&pos, &rect, getFont(), text, 0, strlen(text),
                     textColor);
    }
}

/**
 * Draw the area of this widget that falls within the clipping region.
 * Called by the redraw() function to draw all visible regions.
 *
 * @param port The CGraphicsPort to draw to.
 * @see redraw()
 */

void CProgressBar::drawBorder(CGraphicsPort *port)
{
  // Stop drawing if the widget indicates it should not have an outline

  if (!isBorderless())
    {
      port->drawBevelledRect(getX(), getY(), getWidth(), getHeight(),
                             getShadowEdgeColor(), getShineEdgeColor());
    }
}
