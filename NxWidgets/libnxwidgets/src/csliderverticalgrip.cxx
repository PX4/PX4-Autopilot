/****************************************************************************
 * NxWidgets/libnxwidgets/src/cslidervertical.cxx
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

#include "nxconfig.hxx"
#include "cwidgetcontrol.hxx"
#include "csliderverticalgrip.hxx"
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
 * @param x The x coordinate of the grip, relative to its parent.
 * @param y The y coordinate of the grip, relative to its parent.
 * @param width The width of the grip.
 * @param height The height of the grip.
 */

CSliderVerticalGrip::CSliderVerticalGrip(CWidgetControl *pWidgetControl,
                                         nxgl_coord_t x, nxgl_coord_t y,
                                         nxgl_coord_t width, nxgl_coord_t height)
: CNxWidget(pWidgetControl, x, y, width, height, WIDGET_DRAGGABLE)
{
}

/**
 * Draw the area of this widget that falls within the clipping region.
 * Called by the redraw() function to draw all visible regions.
 *
 * @param port The CGraphicsPort to draw to.
 * @see redraw()
 */

void CSliderVerticalGrip::drawContents(CGraphicsPort *port)
{
  CRect rect;
  getRect(rect);

  nxwidget_pixel_t color;
  if (!m_flags.clicked)
    {
      color = getSelectedBackgroundColor();
    }
  else
    {
      color = getHighlightColor();
    }

  port->drawFilledRect(rect.getX(), rect.getY(),
                       rect.getWidth(), rect.getHeight(),
                       color);
}

/**
 * Draw the area of this widget that falls within the clipping region.
 * Called by the redraw() function to draw all visible regions.
 *
 * @param port The CGraphicsPort to draw to.
 * @see redraw()
 */

void CSliderVerticalGrip::drawBorder(CGraphicsPort *port)
{
  // Stop drawing if the widget indicates it should not have an outline

  if (!isBorderless())
    {
      if (isClicked())
        {
          port->drawBevelledRect(getX(), getY(), getWidth(), getHeight(),
                                 getShadowEdgeColor(), getShineEdgeColor());
        }
      else
        {
          port->drawBevelledRect(getX(), getY(), getWidth(), getHeight(),
                                 getShineEdgeColor(), getShadowEdgeColor());
        }
    }
}

/**
 * Starts dragging the grip and redraws it.
 *
 * @param x The x coordinate of the click.
 * @param y The y coordinate of the click.
 */

void CSliderVerticalGrip::onClick(nxgl_coord_t x, nxgl_coord_t y)
{
  startDragging(x, y);
  redraw();
}

/**
 * Redraws the grip.
 *
 * @param x The x coordinate of the mouse.
 * @param y The y coordinate of the mouse.
 */

void CSliderVerticalGrip::onRelease(nxgl_coord_t x, nxgl_coord_t y)
{
  redraw();
}

/**
 * Redraws the grip.
 *
 * @param x The x coordinate of the mouse.
 * @param y The y coordinate of the mouse.
 */

void CSliderVerticalGrip::onReleaseOutside(nxgl_coord_t x, nxgl_coord_t y)
{
  redraw();
}

/**
 * Moves the grip to follow the mouse.
 *
 * @param x The x coordinate of the mouse.
 * @param y The y coordinate of the mouse.
 * @param vX The horizontal distance of the drag.
 * @param vY The vertical distance of the drag.
 */

void CSliderVerticalGrip::onDrag(nxgl_coord_t x, nxgl_coord_t y,
                                 nxgl_coord_t vX, nxgl_coord_t vY)
{
  // Work out where we're moving to

  nxgl_coord_t destY = y - m_grabPointY - m_parent->getY();

  // Do we need to move?

  if (destY != m_rect.getY())
    {
      // Get parent rect

      CRect rect;
      m_parent->getClientRect(rect);

      // Prevent grip from moving outside parent

      if (destY < rect.getY())
        {
          destY = rect.getY();
        }
      else
        {
          if (destY + getHeight() > rect.getHeight() + rect.getY())
            {
              destY = (rect.getHeight() + rect.getY()) - getHeight();
            }
        }

      // Move to new location

      moveTo(rect.getX(), destY);
    }
}
