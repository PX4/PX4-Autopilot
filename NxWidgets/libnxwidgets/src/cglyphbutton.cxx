/****************************************************************************
 * NxWidgets/libnxwidgets/src/cglyphbutton.cxx
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

#include "nxconfig.hxx"
#include "cglyphbutton.hxx"
#include "cgraphicsport.hxx"
#include "cbitmap.hxx"
#include "singletons.hxx"

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
 * @param x The x coordinate of the button.
 * @param y The y coordinate of the button.
 * @param width The width of the button.
 * @param height The height of the button.
 * @param normalGlyph Glyph to display when unclicked.
 * @param clickedGlyph Glyph to display when clicked.
 * @param bitmapX The x coordinate at which the bitmaps will be drawn.
 * @param bitmapY The y coordinate at which the bitmaps will be drawn.
 * @param style The style that the button should use.  If this is not
 *   specified, the button will use the values stored in the global
 *   g_defaultWidgetStyle object.  The button will copy the properties of
 *   the style into its own internal style object.
 */

CGlyphButton::CGlyphButton(CWidgetControl *pWidgetControl,
                           nxgl_coord_t x, nxgl_coord_t y,
                           nxgl_coord_t width, nxgl_coord_t height,
                           nxgl_coord_t bitmapX, nxgl_coord_t bitmapY,
                           FAR const struct SBitmap *normalGlyph,
                           FAR const struct SBitmap *clickedGlyph,
                           CWidgetStyle *style)
: CNxWidget(pWidgetControl, x, y, width, height, 0, style)
{
  // The border should be set to the minimum so that the bitmap
  // image can extend all the way to the border.

  m_borderSize.top        = 1;
  m_borderSize.right      = 1;
  m_borderSize.bottom     = 1;
  m_borderSize.left       = 1;

  // Save the bitmap state data

  m_bitmapX               = bitmapX;
  m_bitmapY               = bitmapY;
  m_bitmapNormal          = normalGlyph;
  m_bitmapClicked         = clickedGlyph;
}

/**
 * Insert the dimensions that this widget wants to have into the rect
 * passed in as a parameter.  All coordinates are relative to the widget's
 * parent.
 * @param rect Reference to a rect to populate with data.
 */

void CGlyphButton::getPreferredDimensions(CRect &rect) const
{
  nxgl_coord_t width  = m_bitmapNormal->width;
  nxgl_coord_t height = m_bitmapNormal->height;

  if (!m_flags.borderless)
    {
      width  = m_borderSize.left + m_borderSize.right;
      height = m_borderSize.top + m_borderSize.bottom;
    }

  rect.setX(m_rect.getX());
  rect.setY(m_rect.getY());
  rect.setWidth(width);
  rect.setHeight(height);
}

/**
 * Draw the area of this widget that falls within the clipping region.
 * Called by the redraw() function to draw all visible regions.
 *
 * @param port The CGraphicsPort to draw to.
 * @see redraw()
 */

void CGlyphButton::drawContents(CGraphicsPort *port)
{
  // Get the drawable region within the Label

  CRect rect;
  getRect(rect);

  // Pick the glyph to show

  FAR const struct SBitmap *bitmap;
  if (m_flags.clicked)
    {
      bitmap = m_bitmapClicked;
    }
  else
    {
      bitmap = m_bitmapNormal;
    }

  // Then draw it -- in greyscale if not enabled

  if (isEnabled())
    {
      port->drawBitmap(rect.getX(), rect.getY(),
                       rect.getWidth(), rect.getHeight(),
                       bitmap, m_bitmapX, m_bitmapY,
                       CONFIG_NXWIDGETS_TRANSPARENT_COLOR);
    }
  else
    {
      port->drawBitmapGreyScale(rect.getX(), rect.getY(),
                                rect.getWidth(), rect.getHeight(),
                                bitmap, m_bitmapX, m_bitmapY);
    }
}

/**
 * Draw the area of this widget that falls within the clipping region.
 * Called by the redraw() function to draw all visible regions.
 *
 * @param port The CGraphicsPort to draw to.
 * @see redraw()
 */

void CGlyphButton::drawBorder(CGraphicsPort *port)
{
  port->drawFilledRect(getX(), getY(), getWidth(),
                       getHeight(), getBackgroundColor());
  drawOutline(port);
}

/**
 * Draws the outline of the button.
 *
 * @param port Graphics port to draw to.
 */

void CGlyphButton::drawOutline(CGraphicsPort *port)
{
  // Don't draw if the widget indicates it should not have an outline

  if (!isBorderless())
    {
      // Determine which colors to use

      nxgl_coord_t color1;
      nxgl_coord_t color2;
  
      if (isClicked())
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
  
      port->drawBevelledRect(getX(), getY(),
                             getWidth(), getHeight(),
                             color1, color2);
    }
}

/**
 * Redraws the button.
 *
 * @param x The x coordinate of the click.
 * @param y The y coordinate of the click.
 */

void CGlyphButton::onClick(nxgl_coord_t x, nxgl_coord_t y)
{
  redraw();
}

/**
 * Raises an action event and redraws the button.
 *
 * @param x The x coordinate of the mouse.
 * @param y The y coordinate of the mouse.
 */

void CGlyphButton::onRelease(nxgl_coord_t x, nxgl_coord_t y)
{
  m_widgetEventHandlers->raiseReleaseEvent(x, y);
  redraw();
}

/**
 * Redraws the button.
 *
 * @param x The x coordinate of the mouse.
 * @param y The y coordinate of the mouse.
 */

void CGlyphButton::onReleaseOutside(nxgl_coord_t x, nxgl_coord_t y)
{
  redraw();
}
