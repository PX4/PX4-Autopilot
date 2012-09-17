/****************************************************************************
 * NxWidgets/libnxwidgets/src/clabel.cxx
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
#include "clabel.hxx"
#include "cgraphicsport.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * CLabel Method Implementations
 ****************************************************************************/

using namespace NXWidgets;

/**
 * Constructor for a label containing a string.
 *
 * @param pWidgetControl The controlling widget for the display
 * @param x The x coordinate of the text box, relative to its parent.
 * @param y The y coordinate of the text box, relative to its parent.
 * @param width The width of the textbox.
 * @param height The height of the textbox.
 * @param text Pointer to a string to display in the textbox.
 * @param style The style that the button should use.  If this is not
 *        specified, the button will use the global default widget
 *        style.
 */

CLabel::CLabel(CWidgetControl *pWidgetControl,
               nxgl_coord_t x, nxgl_coord_t y,
               nxgl_coord_t width, nxgl_coord_t height,
               const CNxString &text, CWidgetStyle *style)
             : CNxWidget(pWidgetControl, x, y, width, height, 0, style)
{
  // Default is centered text

  m_hAlignment        = TEXT_ALIGNMENT_HORIZ_CENTER;
  m_vAlignment        = TEXT_ALIGNMENT_VERT_CENTER;

  // The border thickness is 1 pixel
  
  m_borderSize.top    = 1;
  m_borderSize.right  = 1;
  m_borderSize.bottom = 1;
  m_borderSize.left   = 1;

  m_align.x           = 0;
  m_align.y           = 0;

  m_textChange        = false;
  m_highlighted       = false;
  setText(text);

  calculateTextPositionHorizontal();
  calculateTextPositionVertical();
}

/**
 * Set the horizontal alignment of text within the label.
 *
 * @param alignment The horizontal position of the text.
 */

void CLabel::setTextAlignmentHoriz(TextAlignmentHoriz alignment)
{
  m_hAlignment = alignment;
  calculateTextPositionHorizontal();
  redraw();
}

/**
 * Set the vertical alignment of text within the label.
 *
 * @param alignment The vertical position of the text.
 */

void CLabel::setTextAlignmentVert(TextAlignmentVert alignment)
{
  m_vAlignment = alignment;
  calculateTextPositionVertical();
  redraw();
}

/**
 * Set the text displayed in the label.
 *
 * @param text String to display.
 */

void CLabel::setText(const CNxString &text)
{
  m_text = text;
  onTextChange();
}

/**
 * Append new text to the end of the current text displayed in the
 * label.
 *
 * @param text String to append.
 */

void CLabel::appendText(const CNxString &text)
{
  m_text.append(text);
  onTextChange();
}

/**
 * Insert text at the specified index.
 *
 * @param text The text to insert.
 * @param index Index at which to insert the text.
 */

void CLabel::insertText(const CNxString &text, const int index)
{
  m_text.insert(text, index);
  onTextChange();
}

/**
 * Control the highlight state.
 *
 * @param highlightOn True(1), the label will be highlighted
 */

void CLabel::highlight(bool highlightOn)
{
  if (highlightOn != m_highlighted)
    {
      m_highlighted = highlightOn;
      redraw();
    }
}

/**
 * Insert the dimensions that this widget wants to have into the rect
 * passed in as a parameter.  All coordinates are relative to the
 * widget's parent.
 *
 * @param rect Reference to a rect to populate with data.
 */

void CLabel::getPreferredDimensions(CRect &rect) const
{
  nxgl_coord_t width;
  nxgl_coord_t height;

  if (!m_flags.borderless)
    {
      width  = m_borderSize.left + m_borderSize.right;
      height = m_borderSize.top + m_borderSize.bottom;
    }
  else
    {
      width  = 0;
      height = 0;
    }

  width  += getFont()->getStringWidth(m_text);
  height += getFont()->getHeight();

  rect.setX(m_rect.getX());
  rect.setY(m_rect.getY());
  rect.setWidth(width);
  rect.setHeight(height);
}

/**
 * Sets the font.
 *
 * @param font A pointer to the font to use.
 */

void CLabel::setFont(CNxFont *font)
{
  m_style.font = font;

  // Need to recalculate the text position as the font may have changed size

  calculateTextPositionHorizontal();
  calculateTextPositionVertical();
  redraw();
}

/**
 * Draw the area of this widget that falls within the clipping region.
 * Called by the redraw() function to draw all visible regions.
 * @param port The CGraphicsPort to draw to.
 *
 * @see redraw()
 */
 
void CLabel::drawContents(CGraphicsPort *port)
{
  // Get the drawing area (excluding the border)

  CRect rect;
  getRect(rect);

  // Pick the text and background colors

  nxgl_mxpixel_t textColor;
  nxgl_mxpixel_t backColor;

  if (!isEnabled())
    {
      textColor = getDisabledTextColor();
      backColor = getBackgroundColor();
    }
  else if (m_highlighted)
    {
      textColor = getSelectedTextColor();
      backColor = getSelectedBackgroundColor();
    }
  else
    {
      textColor = getEnabledTextColor();
      backColor = getBackgroundColor();
    }

  // Draw the background (excluding the border)

  port->drawFilledRect(rect.getX(), rect.getY(),
                       rect.getWidth(), rect.getHeight(), backColor);

  // Get the X/Y position of the text within the Label

  struct nxgl_point_s pos;
  pos.x = rect.getX() + m_align.x;
  pos.y = rect.getY() + m_align.y;

  // Add the text using the selected color

  port->drawText(&pos, &rect, getFont(), m_text, 0, m_text.getLength(),
                 textColor);
}

/**
 * Draw the area of this widget that falls within the clipping region.
 * Called by the redraw() function to draw all visible regions.
 *
 * @param port The CGraphicsPort to draw to.
 * @see redraw()
 */

void CLabel::drawBorder(CGraphicsPort *port)
{
  // Check if the widget indicates it should have an outline: That
  // the outline is enabled and the this is not just a text-only
  // redraw

  if (!isBorderless() && !isTextChange())
    {
      port->drawBevelledRect(getX(), getY(), getWidth(), getHeight(),
                             getShadowEdgeColor(), getShineEdgeColor());
    }
}

/**
 * Resize the widget to the new dimensions.
 *
 * @param width The new width.
 * @param height The new height.
 */

void CLabel::onResize(nxgl_coord_t width, nxgl_coord_t height)
{
  calculateTextPositionHorizontal();
  calculateTextPositionVertical();
}

/**
 * Calculate the vertical position of the string based on the font
 *
 * height and the alignment options.
 */

void CLabel::calculateTextPositionVertical(void)
{
  CRect rect;
  getClientRect(rect);

  nxgl_coord_t height = rect.getHeight();

  switch (m_vAlignment)
    {
    case TEXT_ALIGNMENT_VERT_CENTER:
      m_align.y = (height - getFont()->getHeight()) >> 1;
      break;

    case TEXT_ALIGNMENT_VERT_TOP:
      m_align.y = 0;
      break;
 
    case TEXT_ALIGNMENT_VERT_BOTTOM:
      m_align.y = height - getFont()->getHeight();
      break;
    }
}

/**
 * Calculate the position of the string based on its length and the
 * alignment options.
 */

void CLabel::calculateTextPositionHorizontal(void)
{
  CRect rect;
  getClientRect(rect);
  
  nxgl_coord_t width = rect.getWidth();

  switch (m_hAlignment)
    {
    case TEXT_ALIGNMENT_HORIZ_CENTER:
      m_align.x = (width - getFont()->getStringWidth(m_text)) >> 1;
      break;

    case TEXT_ALIGNMENT_HORIZ_LEFT:
      m_align.x = 0;
      break;

    case TEXT_ALIGNMENT_HORIZ_RIGHT:
      m_align.x = width - getFont()->getStringWidth(m_text);
      break;
    }
}

/**
 * Updates the GUI after the text has changed.
 */

void CLabel::onTextChange(void)
{
  calculateTextPositionHorizontal();
  calculateTextPositionVertical();
  m_textChange = true;
  redraw();
  m_textChange = false;
  m_widgetEventHandlers->raiseValueChangeEvent();
}


