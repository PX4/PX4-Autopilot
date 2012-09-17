/****************************************************************************
 * NxWidgets/libnxwidgets/src/ccyclebutton.cxx
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

#include "ccyclebutton.hxx"
#include "cgraphicsport.hxx"
#include "cbitmap.hxx"
#include "glyphs.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * CButton Method Implementations
 ****************************************************************************/

using namespace NXWidgets;

/**
 * Constructor for cycle buttons.
 *
 * @param pWidgetControl The widget control for the display.
 * @param x The x coordinate of the button, relative to its parent.
 * @param y The y coordinate of the button, relative to its parent.
 * @param width The width of the button.
 * @param height The height of the button.
 * @param style The style that the button should use.  If this is not
 *   specified, the button will use the values stored in the global
 *   g_defaultWidgetStyle object.  The button will copy the properties of
 *   the style into its own internal style object.
 */

CCycleButton::CCycleButton(CWidgetControl *pWidgetControl,
                           nxgl_coord_t x, nxgl_coord_t y,
                           nxgl_coord_t width, nxgl_coord_t height,
                           CWidgetStyle *style)
: CButton(pWidgetControl, x, y, width, height, "", style)
{
  // Force text to align left

  nxgl_coord_t glyphSpace = m_borderSize.left - 1;

  // Text x coordinate is width of cycle glyph plus a space plus width
  // of the spacer line (2px) plus a space

  m_align.x = g_cycle.width + 2 + (glyphSpace << 1);

  m_options.addListDataEventHandler(this);
  m_options.setAllowMultipleSelections(false);
}

/**
 * Add a new option to the widget.
 *
 * @param text The text of the option.
 * @param value The value of the option.
 */

void CCycleButton::addOption(const CNxString &text, const uint32_t value)
{
  m_options.addItem(new CListDataItem(text, value));

  // Select the option if this is the first option added

  if (m_options.getItemCount() == 1)
    {
      selectOption(0);
    }
}

/**
 * Remove an option from the widget by its index.
 *
 * @param index The index of the option to remove.
 */

void CCycleButton::removeOption(const int index)
{
  m_options.removeItem(index);
}

/**
 * Remove all options from the widget.
 */

void CCycleButton::removeAllOptions(void)
{
  m_options.removeAllItems();
}

/**
 * Select an option by its index.
 * Redraws the widget and raises a value changed event.
 *
 * @param index The index of the option to select.
 */

void CCycleButton::selectOption(const int index)
{
  m_options.setItemSelected(index, true);
}

/**
 * Get the selected index.  Returns -1 if nothing is selected.  If more than one
 * option is selected, the index of the first selected option is returned.
 *
 * @return The selected index.
 */

const int CCycleButton::getSelectedIndex(void) const
{
  return m_options.getSelectedIndex();
}

/**
 * Sets the selected index.  Specify -1 to select nothing.  Resets any
 * other selected options to deselected.
 * Redraws the widget and raises a value changed event.
 *
 * @param index The selected index.
 */

void CCycleButton::setSelectedIndex(const int index)
{
  m_options.setItemSelected(index, true);
}

/**
 * Get the selected option.  Returns NULL if nothing is selected.
 *
 * @return The selected option.
 */

const CListDataItem *CCycleButton::getSelectedOption(void) const
{
  return m_options.getSelectedItem();
}

/**
 * Sort the options alphabetically by the text of the options.
 */

void CCycleButton::sort(void)
{
  m_options.sort();
}

/**
 * Handles list data changed events.
 *
 * @param e Event arguments.
 */

void CCycleButton::handleListDataChangedEvent(const CListDataEventArgs &e)
{
  redraw();
}

/**
 * Handles list selection changed events.
 *
 * @param e Event arguments.
 */

void CCycleButton::handleListDataSelectionChangedEvent(const CListDataEventArgs &e)
{
  redraw();
  m_widgetEventHandlers->raiseValueChangeEvent();
}

/**
 * Insert the dimensions that this widget wants to have into the rect
 * passed in as a parameter.  All coordinates are relative to the widget's
 * parent.  Value is based on the length of the largest string in the
 * set of options.
 *
 * @param rect Reference to a rect to populate with data.
 */

void CCycleButton::getPreferredDimensions(CRect &rect) const
{
  nxgl_coord_t width = 0;
  nxgl_coord_t height = getFont()->getHeight();

  // Locate longest string in options

  for (int i = 0; i < m_options.getItemCount(); ++i)
    {
      nxgl_coord_t optionWidth =
        getFont()->getStringWidth(m_options.getItem(i)->getText());

      if (optionWidth > width)
        {
          width = optionWidth;
       }
    }

  // Add the border width

  if (!m_flags.borderless)
    {
      width  += (m_borderSize.left + m_borderSize.right);
      height += (m_borderSize.top + m_borderSize.bottom);
    }

  // Add text alignment

  width  += m_align.x;
  height += m_align.y;

  // And returned this preferred size

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

void CCycleButton::drawContents(CGraphicsPort *port)
{
  // Get the drawing region (excluding any border)
 
  CRect rect;
  getRect(rect);

  nxgl_coord_t glyphSpace   = m_borderSize.left - 1;
  nxgl_coord_t glyphYOffset = (rect.getHeight() - g_cycle.height) >> 1;

  nxwidget_pixel_t textColor;
  nxwidget_pixel_t separatorLeftColor;
  nxwidget_pixel_t separatorRightColor;

  if (!isEnabled())
    {
      textColor           = getDisabledTextColor();
      separatorLeftColor  = getShadowEdgeColor();
      separatorRightColor = getShineEdgeColor();
    }
  else if (!isClicked())
    {
      textColor           = getEnabledTextColor();
      separatorLeftColor  = getShadowEdgeColor();
      separatorRightColor = getShineEdgeColor();
    }
  else
    {
      textColor           = getSelectedTextColor();
      separatorLeftColor  = getShineEdgeColor();
      separatorRightColor = getShadowEdgeColor();
    }

  // Draw cycle glyph

  port->drawBitmap(rect.getX(), rect.getY() + glyphYOffset,
                   g_cycle.width, g_cycle.height, &g_cycle,
                   0, 0, CONFIG_NXWIDGETS_TRANSPARENT_COLOR);

  // Draw separator

  nxgl_coord_t x = getX() + glyphSpace + g_cycle.width;
  nxgl_coord_t y = getY();
  nxgl_coord_t w = glyphSpace + g_cycle.width;
  nxgl_coord_t h = rect.getHeight() - 1;

  port->drawLine(x, y, w, h, separatorLeftColor);
  port->drawLine(x+1, y, w+1, h, separatorRightColor);

  // Only draw text if option is selected

  if (m_options.getSelectedItem() != NULL)
    {
      struct nxgl_point_s pos;
      pos.x = getX() + m_align.x;
      pos.y = getY() + m_align.y;

      port->drawText(&pos, &rect, getFont(),
                     m_options.getSelectedItem()->getText(), 0,
                     m_options.getSelectedItem()->getText().getLength(),
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

void CCycleButton::drawBorder(CGraphicsPort *port)
{
  // Determine the background color

  nxwidget_pixel_t backColor;

  if (isClicked() || m_highlighted)
    {
      backColor = getSelectedBackgroundColor();
    }
  else
    {
      backColor = getBackgroundColor();
    }

  // Draw the background (excluding the border)

  port->drawFilledRect(getX(), getY(), getWidth(), getHeight(), backColor);

  // Then add the border.

  drawOutline(port);
}

/**
 * Draws the outline of the button.
 *
 * @param port Graphics port to draw to.
 */

void CCycleButton::drawOutline(CGraphicsPort *port)
{
  // Stop drawing if the widget indicates it should not have an outline

  if (isBorderless())
    {
      return;
    }
  
  // Work out which colors to use

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
  
  port->drawBevelledRect(getX(), getY(), getWidth(), getHeight(), color1, color2);
}

/**
 * Selects the next option in the list and redraws the button.
 *
 * @param x The x coordinate of the mouse.
 * @param y The y coordinate of the mouse.
 */

void CCycleButton::onRelease(nxgl_coord_t x, nxgl_coord_t y)
{
  // Choose next option

  if (m_options.getItemCount() > 1)
    {
      int selectedIndex = m_options.getSelectedIndex();

      if (selectedIndex < m_options.getItemCount() - 1)
        {
          // Move to next option

          selectOption(selectedIndex + 1);
        }
      else
        {
          // Wrap around as there are no more options

          selectOption(0);
        }
    }

  redraw();
}

/**
 * Redraws the button.
 *
 * @param x The x coordinate of the mouse.
 * @param y The y coordinate of the mouse.
 */

void CCycleButton::onReleaseOutside(nxgl_coord_t x, nxgl_coord_t y)
{
  redraw();
}

