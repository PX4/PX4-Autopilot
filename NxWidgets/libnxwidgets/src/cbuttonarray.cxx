/****************************************************************************
 * NxWidgets/libnxwidgets/include/cbuttonarray.cxx
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/nx/nxglib.h>

#include "cbuttonarray.hxx"
#include "cgraphicsport.hxx"
#include "cwidgetstyle.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
 
/****************************************************************************
 * CButtonArray Method Implementations
 ****************************************************************************/

using namespace NXWidgets;

/**
 * Constructor for an array of buttons.
 *
 * @param pWidgetControl The widget control for the display.
 * @param x The x coordinate of the button array, relative to its parent.
 * @param y The y coordinate of the button array, relative to its parent.
 * @param buttonColumns The number of buttons in one row of the button array
 * @param buttonRows The number of buttons in one column of the button array
 * @param buttonWidth The width of one button
 * @param buttonHeight The height of one button
 * @param style The style that the button should use.  If this is not
 *        specified, the button will use the global default widget
 *        style.
 */

CButtonArray::CButtonArray(CWidgetControl *pWidgetControl,
                           nxgl_coord_t x, nxgl_coord_t y,
                           uint8_t buttonColumns, uint8_t buttonRows,
                           nxgl_coord_t buttonWidth, nxgl_coord_t buttonHeight,
                           CWidgetStyle *style)
: CNxWidget(pWidgetControl, x, y,
            buttonColumns * buttonWidth + 2, buttonRows * buttonHeight + 2,     
            0, style)
{
  // Save configuration

  m_buttonRows        = buttonRows;
  m_buttonColumns     = buttonColumns;
  m_buttonWidth       = buttonWidth;
  m_buttonHeight      = buttonHeight;

  // Make sure the the positional data is valid.

  m_clickX            = x;
  m_clickY            = y;
  m_cursorColumn      = 0;
  m_cursorRow         = 0;

  // Default behavior is to redraw the entire button array

  m_redrawButton      = false;

  // No hightlighted buttons

  m_cursorOn          = false;
  m_cursorChange      = false;

  // At present, There is no border on the array itself

  m_flags.borderless  = true;

  // Make sure that the full size of the text array is allocated

  m_buttonText        = new CNxString[m_buttonRows * m_buttonColumns];
}

/**
 * CButtonArray Destructor.
 */

CButtonArray::~CButtonArray(void)
{
  // Delete the array of CNxString instances

  delete [] m_buttonText;
}

/**
 * Returns the string shown in the label.
 *
 * @param column The column index of the button of interest
* @param row The row index of the button of interest
 * @return The label's text.
 */

const CNxString &CButtonArray::getText(int column, int row) const
{
  return m_buttonText[row * m_buttonColumns + column];
}

/**
 * Set the text displayed in the label.
 *
 * @param column The column index of the button to set
 * @param row The row index of the button to set
 * @param text String to display.
 */

void CButtonArray::setText(int column, int row, const CNxString &text)
{
  m_buttonText[row * m_buttonColumns + column] = text;
  onTextChange();
}

/**
 * Return the position of the last clicked button (0,0 will be returned
 * the no button has every been clicked).  The button at this position
 * is currently clicked then, in addition, return true.
 *
 * @param column The location to return the column index of the button
 *    of interest
 * @param row The location to return the row index of the button of
 *    interest
 * @return True if a button in the array is clicked
 */

bool CButtonArray::isButtonClicked(int &column, int &row) const
{
  // Return the last clicked position

  column = (m_clickX - getX()) / m_buttonWidth;

  // Calculate and return the button row index

  row = (m_clickY - getY()) / m_buttonHeight;

  // Return true if the button is currently clicked

  return isClicked();
}

/**
 * Check if this specific button in the array is clicked
 *
 * @param column The column of the button to check.
 * @param row The row of the button to check.
 * @return True if this button is clicked
 */

bool CButtonArray::isThisButtonClicked(int column, int row) const
{
  // The upper left X/Y position of the button at these indices

  nxgl_coord_t x = getX() + column * m_buttonWidth;
  nxgl_coord_t y = getY() + row * m_buttonHeight;

  // Was the last click on this button?

  if ((m_clickX >= x && m_clickX < x + m_buttonWidth) &&
      (m_clickY >= y && m_clickY < y + m_buttonHeight))
    {
      // Yes.. return true if the button is clicked

      return isClicked();
    }
  else
    {
      // No.. then it can't be clicked

      return false;
    }
}

/**
 * Control the cursor state.
 *
 * @param cursorOn True(1), the current cursor position will be highlighted
 */

void CButtonArray::cursor(bool cursorOn)
{
  if (cursorOn != m_cursorOn)
    {
      // Set the state cursor state

      m_cursorOn     = cursorOn;

      // Update only the effected button

      m_cursorChange = true;
      redraw();
      m_cursorChange = false;
    }
}

/**
 * Return the current cursor position (button indices) and an indication
 * if the button at the cursor is currently hightlighted.
 *
 * @param column The location to return the column index of the button
 *    of interest
 * @param row The location to return the row index of the button of
 *    interest
 * @return True if the cursor is enabled and the button is highlighted
 */

bool CButtonArray::getCursorPosition(int &column, int &row) const
{
  column = m_cursorColumn;
  row    = m_cursorRow;
  return m_cursorOn;
}

/**
 * Set the cursor position (button indices).  Note that the cursor
 * does not have to be enabled to set the position.
 *
 * @param column The column index of the button of interest
 * @param row The row index of the button of interest
 * @return True if the cursor position is valid
 */

bool CButtonArray::setCursorPosition(int column, int row)
{
  // Verify that the cursor position is within range

  if ((unsigned)column < m_buttonColumns && (unsigned)row < m_buttonRows)
    {
      // Verify that the now position is different from the old position

      if (column != m_cursorColumn || row != m_cursorRow)
        {
           m_cursorChange = true;

          // Is the cursor on now?

          bool redrawCursor = m_cursorOn;
          if (redrawCursor)
            {
              // Yes.. clear the old cursor highlight

              m_cursorOn = false;
              redraw();
            }

          // Set the new cursor position

          m_cursorColumn = column;
          m_cursorRow    = row;

          // Do we need to turn the cursor back on?

          if (redrawCursor)
            {
              // Yes.. Set the new cursor hightlight

              m_cursorOn = true;
              redraw();
            }

          m_cursorChange = false;
        }

      return true;
    }

  return false;
}

/**
 * Check if this specific button in the array is at the cursor position
 * and highlighted.
 *
 * @param column The column of the button to check.
 * @param row The row of the button to check.
 * @return True if this button is at the cursor postion and highlighted.
 */

bool CButtonArray::isCursorPosition(int column, int row) const
{
  if (column == m_cursorColumn && row == m_cursorRow)
    {
      return m_cursorOn;
    }

  return false;
}

/**
 * Insert the dimensions that this widget wants to have into the rect
 * passed in as a parameter.  All coordinates are relative to the
 * widget's parent.
 *
 * @param rect Reference to a rect to populate with data.
 */

void CButtonArray::getPreferredDimensions(CRect &rect) const
{
  return getRect(rect);
}

/**
 * Sets the font.
 *
 * @param font A pointer to the font to use.
 */

void CButtonArray::setFont(CNxFont *font)
{
  m_style.font = font;
  redraw();
}

/**
 * Draw the area of this widget that falls within the clipping region.
 * Called by the redraw() function to draw all visible regions.
 *
 * @param port The CGraphicsPort to draw to.
 * @see redraw()
 */

void CButtonArray::drawContents(CGraphicsPort *port)
{
  // Do we draw just the clicked/unclicked button?

  if (m_redrawButton)
    {
      int column;
      int row;
 
      // Just one.  Get the row/column indices from the last click

      (void)posToButton(m_clickX, m_clickY, column, row);

      // And draw that button

      drawButton(port, column, row, isClicked());
    }

  // Do we just draw the hightlighted button?

  else if (m_cursorChange)
    {
      // Do nothing if the highlighted button is also the clicked button

      if (!isThisButtonClicked(m_cursorColumn, m_cursorRow))
        {
          drawButton(port, m_cursorColumn, m_cursorRow, false);
        }
    }

  // Draw all buttons

  else
    {
      // Visit each column

      for (int column = 0; column < m_buttonColumns; column++)
        {
          // Visit each row

          for (int row = 0; row < m_buttonRows; row++)
            {
               // Draw each button

               drawButton(port, column, row, isThisButtonClicked(column, row));
            }
        }
    }
}

/**
 * Draw the area of this widget that falls within the clipping region.
 * Called by the redraw() function to draw all visible regions.
 *
 * @param port The CGraphicsPort to draw to.
 * @see redraw()
 */

void CButtonArray::drawBorder(CGraphicsPort *port)
{
  // This doesn't do anything anymore
}

/**
 * Redraw only one button
 *
 * @param port The CGraphicsPort to draw to.
 * @param column The button column index
 * @param row The button row index
 * @param useClicked Draw the button using the 'clicked' button style,
 *        regardless of the actual button state.
 * @see onClick() and onRelease()
 */

void CButtonArray::drawButton(CGraphicsPort *port, int column, int row, bool useClicked)
{
  // The X/Y position of the button

  nxgl_coord_t x = getX() + column * m_buttonWidth;
  nxgl_coord_t y = getY() + row * m_buttonHeight;

  // Determine which colors to use

  nxwidget_pixel_t borderColor1;
  nxwidget_pixel_t borderColor2;
  nxwidget_pixel_t backColor;
  nxwidget_pixel_t textColor;

  // Pick the background and test colors. Should we use the 'clicked' backgound style?

  if (useClicked || isCursorPosition(column, row))
    {
      // "Selected" text color on unique "Selected" background color
 
      backColor    = getSelectedBackgroundColor();
      textColor    = getSelectedTextColor();
    }
  else
    {
      // Normal background color and "Enabled" text color

      backColor    = getBackgroundColor();
      textColor    = getEnabledTextColor();
    }

  // Pick the border colors.  Should we use the 'clicked' button style?
 
  if (useClicked)
    {
      // Yes.. Bevelled into the screen

      borderColor1 = getShadowEdgeColor();
      borderColor2 = getShineEdgeColor();
    }
  else
    {
      // No.. Bevelled out of the screen

      borderColor1 = getShineEdgeColor();
      borderColor2 = getShadowEdgeColor();
    }

  // Use a special text color if the widget is not enabled

  if (!isEnabled())
    {
      textColor = getDisabledTextColor();
    }

  // Draw the background

  port->drawFilledRect(x, y, m_buttonWidth, m_buttonHeight, backColor);

  // Draw the button outline

  port->drawBevelledRect(x, y, m_buttonWidth, m_buttonHeight,  borderColor1, borderColor2);

  // Get the text for this button

  CNxString *text = &m_buttonText[row * m_buttonColumns + column];

  // Get the text drawing region

  CRect rect;
  rect.setX(x + 1);
  rect.setY(y + 1);
  rect.setWidth(m_buttonWidth - 2);
  rect.setHeight(m_buttonHeight - 2);

  // Get the centered text alignment

  nxgl_coord_t alignY = (m_buttonHeight - getFont()->getHeight() - 2) >> 1;
  nxgl_coord_t alignX = (m_buttonWidth - getFont()->getStringWidth(*text) - 2) >> 1;

  // Get the text display position

  struct nxgl_point_s pos;
  pos.x = x + alignX;
  pos.y = y + alignY;

  // Set the CGraphicsControl background to match the selected background color.
  // This is only necessary if we cannot read from the LCD.  If we cannot read
  // from then the font background is set to this background color.
  // REVISIT:  This begs for a more generalized solution.

#ifdef CONFIG_NX_WRITEONLY
  nxgl_mxpixel_t saveColor = port->getBackColor();
  port->setBackColor(backColor);
#endif

  // And draw the button text.

  port->drawText(&pos, &rect, getFont(), *text, 0, text->getLength(), textColor);

  // Restore the default background color

#ifdef CONFIG_NX_WRITEONLY
  port->setBackColor(saveColor);
#endif
}

/**
 * Redraws the button.
 *
 * @param x The x coordinate of the click.
 * @param y The y coordinate of the click.
 */

void CButtonArray::onClick(nxgl_coord_t x, nxgl_coord_t y)
{
  // Click X,Y is in raw window coordinates

  m_clickX = x;
  m_clickY = y;

  // Redraw only the button that was clicked

  m_redrawButton = true;
  redraw();
  m_redrawButton = false;
}

/**
 * Raises an action.
 *
 * @param x The x coordinate of the mouse.
 * @param y The y coordinate of the mouse.
 */

void CButtonArray::onPreRelease(nxgl_coord_t x, nxgl_coord_t y)
{
  m_widgetEventHandlers->raiseActionEvent();
}

/**
 * Redraws the button.
 *
 * @param x The x coordinate of the mouse.
 * @param y The y coordinate of the mouse.
 */

void CButtonArray::onRelease(nxgl_coord_t x, nxgl_coord_t y)
{
  // Redraw only the button that was released

  m_redrawButton = true;
  redraw();
  m_redrawButton = false;
}

/**
 * Redraws the button.
 *
 * @param x The x coordinate of the mouse.
 * @param y The y coordinate of the mouse.
 */

void CButtonArray::onReleaseOutside(nxgl_coord_t x, nxgl_coord_t y)
{
  redraw();
}

/**
 * Convert an X/Y position to a button column/row index
 *
 * @param x The x position
 * @param y The y position
 * @param column The location to return the column index of the button
 *    of interest
 * @param row The location to return the row index of the button of
 *    interest
 * @return false is the position is invalid
 */

bool CButtonArray::posToButton(nxgl_coord_t x, nxgl_coord_t y, int &column, int &row)
{
  // Get the row/column indices matching the x/y position

  column = (x - getX()) / m_buttonWidth;
  row    = (y - getY()) / m_buttonHeight;
  return ((unsigned)column < m_buttonColumns && (unsigned)row < m_buttonRows);
}

/**
 * Updates the GUI after the text has changed.
 */

void CButtonArray::onTextChange(void)
{
  redraw();
  m_widgetEventHandlers->raiseValueChangeEvent();
}

