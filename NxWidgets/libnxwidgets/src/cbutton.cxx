/****************************************************************************
 * NxWidgets/libnxwidgets/src/cbutton.cxx
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

#include "cbutton.hxx"
#include "cgraphicsport.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * CButton Method Implementations
 ****************************************************************************/

using namespace NXWidgets;

/**
 * Constructor for buttons that display a string.
 *
 * @param pWidgetControl The controlling widget for the display.
 * @param x The x coordinate of the button, relative to its parent.
 * @param y The y coordinate of the button, relative to its parent.
 * @param width The width of the button.
 * @param height The height of the button.
 * @param text The text for the button to display.
 * @param style The style that the button should use.  If this is not
 * specified, the button will use the values stored in the global
 * g_defaultWidgetStyle object.  The button will copy the properties of
 * the style into its own internal style object.
 */

CButton::CButton(CWidgetControl *pWidgetControl,
                 nxgl_coord_t x, nxgl_coord_t y, nxgl_coord_t width,
                 nxgl_coord_t height, const CNxString &text, CWidgetStyle *style)
: CLabel(pWidgetControl, x, y, width, height, text, style)
{
}

/**
 * Draws the outline of the button.
 *
 * @param port Graphics port to draw to.
 */

void CButton::drawOutline(CGraphicsPort *port)
{
  drawOutline(port, isClicked());
}

/**
 * Draws the outline of the button.
 *
 * @param port       Graphics port to draw to.
 * @param useClicked Present outline using the 'clicked' style
 */

void CButton::drawOutline(CGraphicsPort *port, bool useClicked)
{
  // Stop drawing if the widget indicates it should not have an outline

  if (isBorderless())
    {
      return;
    }
  
  // Work out which colors to use

  nxgl_coord_t color1;
  nxgl_coord_t color2;
  
  if (useClicked)
    {
      // Bevelled into the screen

      color1 = getShadowEdgeColor();
      color2 = getShineEdgeColor();
    }
  else
    {
      // Bevelled out of the screen

      color1 = getShineEdgeColor();
      color2 = getShadowEdgeColor();
    }
  
  port->drawBevelledRect(getX(), getY(), getWidth(), getHeight(), color1, color2);
}

/**
 * Draw the area of this widget that falls within the clipping region.
 * Called by the redraw() function to draw all visible regions.
 *
 * @param port The CGraphicsPort to draw to.
 * @see redraw()
 */

void CButton::drawContents(CGraphicsPort *port)
{
  drawContents(port, isClicked());
}

/**
 * Draw the area of this widget that falls within the clipping region.
 * Called by the redraw() function to draw all visible regions.
 *
 * @param port The CGraphicsPort to draw to.
 * @param useClicked Present contents using the 'clicked' style
 * @see redraw()
 */

void CButton::drawContents(CGraphicsPort *port, bool useClicked)

{
  // Get the draw region (excluding any borders)

  CRect rect;
  getRect(rect);

  // Get the X/Y position of the text within the button

  struct nxgl_point_s pos;
  pos.x = rect.getX() + m_align.x;
  pos.y = rect.getY() + m_align.y;

  // Pick the text color

  nxgl_mxpixel_t textColor;
  if (!isEnabled())
    {
      textColor = getDisabledTextColor();
    }
  else if (useClicked)
    {
      textColor = getSelectedTextColor();
    }
  else
    {
      textColor = getEnabledTextColor();
    }

  // And draw the button text

  port->drawText(&pos, &rect, getFont(), m_text, 0,  m_text.getLength(), textColor);
}

/**
 * Draw the area of this widget that falls within the clipping region.
 * Called by the redraw() function to draw all visible regions.
 *
 * @param port The CGraphicsPort to draw to.
 * @see redraw()
 */

void CButton::drawBorder(CGraphicsPort *port)
{
  drawBorder(port, isClicked());
}

/**
 * Draw the area of this widget that falls within the clipping region.
 * Called by the redraw() function to draw all visible regions.
 *
 * @param port       The CGraphicsPort to draw to.
 * @param useClicked Present border using the 'clicked' style
 * @see redraw()
 */

void CButton::drawBorder(CGraphicsPort *port, bool useClicked)
{
  // Determine the background color

  nxgl_mxpixel_t backColor;

  if (useClicked || m_highlighted)
    {
      backColor = getSelectedBackgroundColor();
    }
  else
    {
      backColor = getBackgroundColor();
    }

  port->drawFilledRect(getX(), getY(), getWidth(), getHeight(), backColor);
  drawOutline(port);
}

/**
 * Redraws the button.
 *
 * @param x The x coordinate of the click.
 * @param y The y coordinate of the click.
 */

void CButton::onClick(nxgl_coord_t x, nxgl_coord_t y)
{
  redraw();
}

/**
 * Raises an action.
 *
 * @param x The x coordinate of the mouse.
 * @param y The y coordinate of the mouse.
 */

void CButton::onPreRelease(nxgl_coord_t x, nxgl_coord_t y)
{
  m_widgetEventHandlers->raiseActionEvent();
}

/**
 * Redraws the button.
 *
 * @param x The x coordinate of the mouse.
 * @param y The y coordinate of the mouse.
 */

void CButton::onRelease(nxgl_coord_t x, nxgl_coord_t y)
{
  redraw();
}

/**
 * Redraws the button.
 *
 * @param x The x coordinate of the mouse.
 * @param y The y coordinate of the mouse.
 */

void CButton::onReleaseOutside(nxgl_coord_t x, nxgl_coord_t y)
{
  redraw();
}
