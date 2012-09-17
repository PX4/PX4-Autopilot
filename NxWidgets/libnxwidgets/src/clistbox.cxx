/****************************************************************************
 * NxWidgets/libnxwidgets/src/clistbox.cxx
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
#include "clistbox.hxx"
#include "cgraphicsport.hxx"
#include "cnxfont.hxx"
#include "singletons.hxx"

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
 * @param pWidgetControl The controlling widget for the display.
 * @param x The x coordinate of the widget.
 * @param y The y coordinate of the widget.
 * @param width The width of the widget.
 * @param height The height of the widget.
 * @param style The style that the widget should use.  If this is not
 *   specified, the widget will use the values stored in the global
 *   g_defaultWidgetStyle object.  The widget will copy the properties of
 *   the style into its own internal style object.
 */

CListBox::CListBox(CWidgetControl *pWidgetControl,
                   nxgl_coord_t x, nxgl_coord_t y,
                   nxgl_coord_t width, nxgl_coord_t height,
                   CWidgetStyle *style)
: CScrollingPanel(pWidgetControl, x, y, width, height, 0, style)
{
  m_flags.draggable       = true;
  m_flags.doubleClickable = true;
  m_optionPadding         = 2;
  m_options.addListDataEventHandler(this);
  m_lastSelectedIndex     = -1;

  // Disallow horizontal scrolling

  setAllowsHorizontalScroll(false);
}

/**
 * Destructor.
 */

CListBox::~CListBox(void)
{
  m_options.removeListDataEventHandler(this);
}


/**
 * Add a new option to the widget using default colors.
 *
 * @param text Text to show in the option.
 * @param value The value of the option.
 */

void CListBox::addOption(const CNxString &text, const uint32_t value)
{
  addOption(text, value,
            getShadowEdgeColor(), getBackgroundColor(),
            getShadowEdgeColor(), getHighlightColor());
}

/**
 * Add an option to the widget.
 *
 * @param option The option to add.
 */

void CListBox::addOption(CListBoxDataItem *option)
{
  m_options.addItem(option);
}

/**
 * Remove an option from the widget by its index.
 *
 * @param index The index of the option to remove.
 */

void CListBox::removeOption(const int index)
{
  m_options.removeItem(index);
}

/**
 * Remove all options from the widget.
 */

void CListBox::removeAllOptions(void)
{
  m_options.removeAllItems();
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

void CListBox::addOption(const CNxString &text, const uint32_t value,
                         const nxwidget_pixel_t normalTextColor,
                         const nxwidget_pixel_t normalBackColor,
                         const nxwidget_pixel_t selectedTextColor,
                         const nxwidget_pixel_t selectedBackColor)
{
  addOption(new CListBoxDataItem(text, value,
                                 normalTextColor, normalBackColor,
                                 selectedTextColor, selectedBackColor));
}

/**
 * Select an option by its index.
 * Redraws the widget and raises a value changed event.
 *
 * @param index The index of the option to select.
 */

void CListBox::selectOption(const int index)
{
  setOptionSelected(index, true);
}

/**
 * Select an option by its index.
 * Redraws the widget and raises a value changed event.
 *
 * @param index The index of the option to select.
 */

void CListBox::deselectOption(const int index)
{
  setOptionSelected(index, false);
}

/**
 * Select all options.  Does nothing if the listbox does not allow
 * multiple selections. Redraws the widget and raises a value changed
 * event.
 */

void CListBox::selectAllOptions(void)
{
  m_options.selectAllItems();
}

/**
 * Deselect all options.
 * Redraws the widget and raises a value changed event.
 */

void CListBox::deselectAllOptions(void)
{
  m_options.deselectAllItems();
}

/**
 * Get the selected index.  Returns -1 if nothing is selected.  If
 * more than one option is selected, the index of the first selected
 * option is returned.
 *
 * @return The selected index.
 */

const int CListBox::getSelectedIndex(void) const
{
  return m_options.getSelectedIndex();
}

/**
 * Sets the selected index.  Specify -1 to select nothing.  Resets any
 * other selected options to deselected. Redraws the widget and raises
 * a value changed event.
 *
 * @param index The selected index.
 */

void CListBox::setSelectedIndex(const int index)
{
  setOptionSelected(index, true);
}

/**
 * Get the selected option.  Returns NULL if nothing is selected.
 *
 * @return The selected option.
 */

const CListBoxDataItem *CListBox::getSelectedOption(void) const
{
  return (const CListBoxDataItem*)m_options.getSelectedItem();
}

/**
 * Resize the scrolling canvas to encompass all options.
 */

void CListBox::resizeCanvas(void)
{
  // Get client area

  CRect rect;
  getClientRect(rect);

  int oldCanvasHeight = m_canvasHeight;

  // Resize the canvas

  m_canvasHeight = (m_options.getItemCount() * getOptionHeight());

  // Ensure canvas is at least as tall as the widget

  m_canvasHeight = m_canvasHeight < rect.getHeight() ? rect.getHeight() : m_canvasHeight;

  // If resize has left scroll position beyond end of canvas, adjust
  // to compensate

  if (m_canvasY + (m_canvasHeight - getHeight()) < 0)
    {
      scroll(0, -(oldCanvasHeight - (m_canvasHeight - getHeight())));
    }
}

/**
 * Sort the options alphabetically by the text of the options.
 */

void CListBox::sort(void)
{
  m_options.sort();
}

/**
 * Get the height of a single option.
 *
 * @return The height of an option.
 */

const nxgl_coord_t CListBox::getOptionHeight(void) const
{
  return getFont()->getHeight() + (m_optionPadding << 1);
}

/**
 * Handles list data changed events.
 *
 * @param e Event arguments.
 */

void CListBox::handleListDataChangedEvent(const CListDataEventArgs &e)
{
  // Forget the last selected item as it may have changed

  m_lastSelectedIndex = -1;
  resizeCanvas();
  redraw();
}

/**
 * Handles list selection changed events.
 *
 * @param e Event arguments.
 */

void CListBox::handleListDataSelectionChangedEvent(const CListDataEventArgs &e)
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

void CListBox::getPreferredDimensions(CRect &rect) const
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

  nxgl_coord_t maxWidth    = 0;
  nxgl_coord_t optionWidth = 0;

  // Locate longest string in options

  for (int i = 0; i < m_options.getItemCount(); ++i)
    {
      optionWidth = getFont()->getStringWidth(m_options.getItem(i)->getText());

      if (optionWidth > maxWidth)
        {
          maxWidth = optionWidth;
        }
    }

  rect.setX(m_rect.getX());
  rect.setY(m_rect.getY());
  rect.setWidth(width + (m_optionPadding << 1) + maxWidth);
  rect.setHeight(height + getOptionHeight() * 3);
}

/**
 * Check if the click is a double-click.
 *
 * @param x X coordinate of the click.
 * @param y Y coordinate of the click.
 * @return True if the click is a double-click.
 */

bool CListBox::isDoubleClick(nxgl_coord_t x, nxgl_coord_t y)
{
  if (CNxWidget::isDoubleClick(x, y))
    {
      // Calculate which option was clicked

      int selectedIndex = (-m_canvasY + (y - getY())) / getOptionHeight();

      // Has the same option been clicked twice?  Ignore double-clicks that
      // occur on different items

      if (selectedIndex == m_lastSelectedIndex)
        {
          // Process click as a double-click

          return true;
        }
    }

  return false;
}

/**
 * Draw the area of this widget that falls within the clipping region.
 * Called by the redraw() function to draw all visible regions.
 *
 * @param port The CGraphicsPort to draw to.
 * @see redraw()
 */

void CListBox::drawContents(CGraphicsPort *port)
{
  // Get the drawing region (excluding the borders)

  CRect rect;
  getRect(rect);

  // Draw background

  port->drawFilledRect(rect.getX(), rect.getY(),
                       rect.getWidth(), rect.getHeight(),
                       getBackgroundColor());

  // Calculate clipping values

  nxgl_coord_t clipX      = rect.getX();
  nxgl_coord_t clipY      = rect.getY();
  nxgl_coord_t clipHeight = rect.getHeight();

  // Precalc values for option draw loop

  nxgl_coord_t optionHeight = getOptionHeight();
  
  // Ensure that we subtract 1 from topOption to ensure that the option
  // above is drawn if it is partially visible

  int topOption = ((clipY - m_canvasY) / optionHeight) - 1;
  
  // Ensure that we add 2 to the bottom option to ensure that the option
  // below is draw if it is partially visible - subbing 1 from topOption
  // means we need to add an additional 1 to compensate

  int bottomOption = topOption + (clipHeight / optionHeight) + 2;

  // Ensure top options is not negative

  if (topOption < 0)
    {
      topOption = 0;
    }

  // Ensure bottom option does not exceed number of options

  if (bottomOption >= m_options.getItemCount())
    {
      bottomOption = m_options.getItemCount() - 1;
    }

  // Calculate values for loop

  int y = m_canvasY + (topOption * optionHeight);
  int i = topOption;

  const CListBoxDataItem *item = (CListBoxDataItem *)NULL;

  // Loop through all options drawing each ones

  while (i <= bottomOption)
    {
      item = (const CListBoxDataItem*)m_options.getItem(i);
    
      // Is the option selected?

      if (item->isSelected())
        {
          // Draw background

          if (item->getSelectedBackColor() != getBackgroundColor())
            {
              port->drawFilledRect(rect.getX(), rect.getY() + y,
                                   rect.getWidth(), optionHeight,
                                   item->getSelectedBackColor());
            }
    
          // Draw text

          struct nxgl_point_s pos;
          pos.x = rect.getX() + m_optionPadding;
          pos.y = rect.getY() + y + m_optionPadding;
 
          if (isEnabled())
            {
              port->drawText(&pos, &rect, getFont(), item->getText(), 0,
                             item->getText().getLength(),
                             item->getSelectedTextColor());
            }
          else
            {
              port->drawText(&pos, &rect, getFont(), item->getText(), 0,
                             item->getText().getLength(),
                             getDisabledTextColor());
           }
        }
      else
        {
          // Draw background

          if (item->getNormalBackColor() != getBackgroundColor())
            {
              port->drawFilledRect(clipX, y, getWidth(), optionHeight,
                                   item->getNormalBackColor());
            }

          // Draw text

          struct nxgl_point_s pos;
          pos.x = rect.getX() + m_optionPadding;
          pos.y = rect.getY() + y + m_optionPadding;
 
          if (isEnabled())
            {
              port->drawText(&pos, &rect, getFont(), item->getText(), 0,
                             item->getText().getLength(),
                             item->getNormalTextColor());
            }
          else
            {
              port->drawText(&pos, &rect, getFont(), item->getText(), 0,
                             item->getText().getLength(),
                             getDisabledTextColor());
            }
        }

      i++;
      y += optionHeight;
    }
}

/**
 * Draw the area of this widget that falls within the clipping region.
 * Called by the redraw() function to draw all visible regions.
 *
 * @param port The CGraphicsPort to draw to.
 * @see redraw()
 */

void CListBox::drawBorder(CGraphicsPort *port)
{
  // Stop drawing if the widget indicates it should not have an outline

  if (!isBorderless())
    {
      port->drawBevelledRect(0, 0, getWidth(), getHeight(),
                             getShadowEdgeColor(), getShineEdgeColor());
    }
}

/**
 * Determines which item was clicked and selects or deselects it as
 * appropriate.  Also starts the dragging system.
 *
 * @param x The x coordinate of the click.
 * @param y The y coordinate of the click.
 */

void CListBox::onClick(nxgl_coord_t x, nxgl_coord_t y)
{
  // Calculate which option was clicked

  m_lastSelectedIndex = (-m_canvasY + (y - getY())) / getOptionHeight();  
  
  const CListBoxDataItem *item =
    (const CListBoxDataItem*)m_options.getItem(m_lastSelectedIndex);

  // Are we setting or unsetting?

  if (item->isSelected())
    {
      // Deselecting

      m_options.deselectItem(m_lastSelectedIndex);
    }
  else
    {
     // Selecting

     m_options.selectItem(m_lastSelectedIndex);
    }

  startDragging(x, y);
  redraw();
}

/**
 * Selects the clicked item and deselects all others.
 *
 * @param x The x coordinate of the click.
 * @param y The y coordinate of the click.
 */

void CListBox::onDoubleClick(nxgl_coord_t x, nxgl_coord_t y)
{
  // Calculate which option was clicked

  int newSelectedIndex = (-m_canvasY + (y - getY())) / getOptionHeight();  

  // Double-click - select the item exclusively

  deselectAllOptions();
  setSelectedIndex(newSelectedIndex);
  m_widgetEventHandlers->raiseActionEvent();
}

/**
 * Select or deselect an option by its index.  Does not deselect any other selected options.
 * Set index to -1 to select nothing.
 * Redraws the widget and raises a value changed event.
 *
 * @param index The index of the option to select.
 * @param selected True to select the option, false to deselect it.
 */

void CListBox::setOptionSelected(const int index, bool selected)
{
  m_options.setItemSelected(index, selected);
}
