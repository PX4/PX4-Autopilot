/****************************************************************************
 * NxWidgets/libnxwidgets/src/cmultilinetextbox.cxx
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
#include "cnxfont.hxx"
#include "ctext.hxx"
#include "cgraphicsport.hxx"
#include "singletons.hxx"
#include "cstringiterator.hxx"
#include "cnxtimer.hxx"
#include "cmultilinetextbox.hxx"

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
 * @param pWidgetControl The widget control for the display.
 * @param x The x coordinate of the text box, relative to its parent.
 * @param y The y coordinate of the text box, relative to its parent.
 * @param width The width of the textbox.
 * @param height The height of the textbox.
 * @param text Pointer to a string to display in the textbox.
 * @param flags Standard widget flag options.
 * @param maxRows The maximum number of rows the textbox can track.  Adding
 *   text beyond this number will cause rows at the start of the text to be
 *   forgotten; text is essentially stored as a queue, and adding to the back
 *   of a full queue causes the front items to be popped off.  Setting this to
 *   0 will make the textbox track only the visible rows.
 * @param style The style that the widget should use.  If this is not
 *   specified, the widget will use the values stored in the global
 *   g_defaultWidgetStyle object.  The widget will copy the properties of
 *   the style into its own internal style object.
 */

CMultiLineTextBox::CMultiLineTextBox(CWidgetControl *pWidgetControl,
                                     nxgl_coord_t x, nxgl_coord_t y,
                                     nxgl_coord_t width, nxgl_coord_t height,
                                     const CNxString &text, uint32_t flags,
                                     nxgl_coord_t maxRows, CWidgetStyle *style)
: CScrollingPanel(pWidgetControl, x, y, width, height, flags, style)
{
  m_hAlignment            = TEXT_ALIGNMENT_HORIZ_CENTER;
  m_vAlignment            = TEXT_ALIGNMENT_VERT_CENTER;
  m_topRow                = 0;

  // The border is one line thick

  m_borderSize.top        = 1;
  m_borderSize.right      = 1;
  m_borderSize.bottom     = 1;
  m_borderSize.left       = 1;

  CRect rect;
  getClientRect(rect);
  m_text                  = new CText(getFont(), "", rect.getWidth());
  m_canvasWidth           = rect.getWidth();

  m_flags.draggable       = true;
  m_flags.doubleClickable = true;
  m_maxRows               = maxRows;

  calculateVisibleRows();

  // Set maximum rows if value not set

  if (m_maxRows == 0)
  {
    m_maxRows            = m_visibleRows + 1;
  }

  m_cursorPos            = 0;
  m_showCursor           = SHOW_CURSOR_NEVER;
  m_wrapCursor           = false;

  setText(text);
}

/**
 * Set the horizontal alignment of text within the textbox.
 *
 * @param alignment The horizontal position of the text.
 */

void CMultiLineTextBox::setTextAlignmentHoriz(TextAlignmentHoriz alignment)
{
  m_hAlignment = alignment;
  redraw();
}

/**
 * Set the vertical alignment of text within the textbox.
 *
 * @param alignment The vertical position of the text.
 */

void CMultiLineTextBox::setTextAlignmentVert(TextAlignmentVert alignment)
{
  m_vAlignment = alignment;
  redraw();
}

/**
 * Returns the number of "pages" that the text spans.  A page
 * is defined as the amount of text that can be displayed within
 * the textbox at one time.
 *
 * @return The page count.
 */

const int CMultiLineTextBox::getPageCount(void) const
{
  if (m_visibleRows > 0)
    {
      return (m_text->getLineCount() / m_visibleRows) + 1;
    }
  else
    {
      return 1;
    }
}

/**
 * Returns the current page.
 *
 * @return The current page.
 * @see getPageCount().
 */

const int CMultiLineTextBox::getCurrentPage(void) const
{
  // Calculate the top line of text

  int topRow = -m_canvasY / m_text->getLineHeight();

  // Return the page on which the top row falls

  if (m_visibleRows > 0)
    {
      return topRow / m_visibleRows;
    }
  else
    {
      return 1;
    }
}

/**
 * Set the text displayed in the textbox.
 *
 * @param text String to display.
 */

void CMultiLineTextBox::setText(const CNxString &text)
{
  bool drawingEnabled = m_flags.drawingEnabled;
  disableDrawing();

  m_text->setText(text);

  cullTopLines();
  limitCanvasHeight();
  jumpToTextBottom();

  if (drawingEnabled)
    {
      enableDrawing();
    }
  redraw();

  m_widgetEventHandlers->raiseValueChangeEvent();
}

/**
 * Append new text to the end of the current text
 * displayed in the textbox.
 *
 * @param text String to append.
 */

void CMultiLineTextBox::appendText(const CNxString &text)
{
  bool drawingEnabled = m_flags.drawingEnabled;
  disableDrawing();

  m_text->append(text);

  cullTopLines();
  limitCanvasHeight();
  jumpToTextBottom();

  if (drawingEnabled)
    {
      enableDrawing();
    }
  redraw();

  m_widgetEventHandlers->raiseValueChangeEvent();
}

/**
 * Remove all characters from the string from the start index onwards.
 *
 * @param startIndex Index to remove from.
 */

void CMultiLineTextBox::removeText(const unsigned int startIndex)
{
  removeText(startIndex, m_text->getLength() - startIndex);
}

/**
 * Remove specified number of characters from the string from the
 * start index onwards.
 *
 * @param startIndex Index to remove from.
 * @param count Number of characters to remove.
 */

void CMultiLineTextBox::removeText(const unsigned int startIndex,
                                   const unsigned int count)
{
  bool drawingEnabled = m_flags.drawingEnabled;
  disableDrawing();

  m_text->remove(startIndex, count);

  limitCanvasHeight();
  limitCanvasY();

  moveCursorToPosition(startIndex);

  if (drawingEnabled)
    {
      enableDrawing();
    }
  redraw();

  m_widgetEventHandlers->raiseValueChangeEvent();
}

/**
 * Set the font used in the textbox.
 *
 * @param font Pointer to the new font.
 */

void CMultiLineTextBox::setFont(CNxFont *font)
{
  bool drawingEnabled = m_flags.drawingEnabled;
  disableDrawing();

  m_style.font = font;
  m_text->setFont(font);

  cullTopLines();
  limitCanvasHeight();
  limitCanvasY();

  if (drawingEnabled)
    {
      enableDrawing();
    }
  redraw();

  m_widgetEventHandlers->raiseValueChangeEvent();
}

/**
 * Get the length of the text string.
 *
 * @return The length of the text string.
 */

const int CMultiLineTextBox::getTextLength(void) const
{
  return m_text->getLength();
}

/**
 * Sets the cursor display mode.
 *
 * @param cursorMode Determines cursor display mode
 */

void CMultiLineTextBox::showCursor(EShowCursor cursorMode)
{
  if (m_showCursor != cursorMode)
   {
      m_showCursor = (uint8_t)cursorMode;
      redraw();
    }
}

/**
 * Move the cursor to the text position specified.  0 indicates the start
 * of the string.  If position is greater than the length of the string,
 * the cursor is moved to the end of the string.
 *
 * @param position The new cursor position.
 */

void CMultiLineTextBox::moveCursorToPosition(const int position)
{
  CGraphicsPort *port = m_widgetControl->getGraphicsPort();

  // Erase existing cursor

  drawCursor(port);

  // Force position to within confines of string

  if (position < 0)
    {
      m_cursorPos = 0;
    }
  else
    {
      int len = (int)m_text->getLength();
      m_cursorPos = len > position ? position : len;
    }

  // Draw cursor in new position

  drawCursor(port);
}

/**
 * Insert text at the specified index.
 *
 * @param text The text to insert.
 * @param index Index at which to insert the text.
 */

void CMultiLineTextBox::insertText(const CNxString &text,
                                   const unsigned int index)
{
  bool drawingEnabled = m_flags.drawingEnabled;
  disableDrawing();

  m_text->insert(text, index);

  cullTopLines();
  limitCanvasHeight();

  moveCursorToPosition(index + text.getLength());

  if (drawingEnabled)
    {
      enableDrawing();
    }
  redraw();

  m_widgetEventHandlers->raiseValueChangeEvent();
}

/**
 * Insert text at the current cursor position.
 *
 * @param text The text to insert.
 */

void CMultiLineTextBox::insertTextAtCursor(const CNxString &text)
{
  insertText(text, getCursorPosition());
}

/**
 * Handle a keyboard press event.
 *
 * @param e The event data.
 */

void CMultiLineTextBox::handleKeyPressEvent(const CWidgetEventArgs &e)
{
  nxwidget_char_t key = e.getKey();

  if (key == KEY_CODE_BACKSPACE)
    {
      // Delete character in front of cursor

      if (m_cursorPos > 0)
        {
          removeText(m_cursorPos - 1, 1);
        }
    }
  else if (key != KEY_CODE_NONE)
    {
      // Not modifier; append value

      insertTextAtCursor(key);
    } 
}

/**
 * Get the coordinates of the cursor relative to the text.
 *
 * @param x Will be populated with the x coordinate of the cursor.
 * @param y Will be populated with the y coordinate of the cursor.
 */

void CMultiLineTextBox::getCursorCoordinates(nxgl_coord_t &x, nxgl_coord_t &y) const
{
  unsigned int cursorRow = 0;

  x = 0;
  y = 0;

  // Only calculate the cursor position if the cursor isn't at the start
  // of the text

  if (m_cursorPos > 0)
    {
      // Calculate the row in which the cursor appears

      cursorRow = m_text->getLineContainingCharIndex(m_cursorPos);

      // Cursor line offset gives us the distance of the cursor from the
      // start of the line

      uint8_t cursorLineOffset = m_cursorPos - m_text->getLineStartIndex(cursorRow);
     
      CStringIterator *iterator = m_text->newStringIterator();
      iterator->moveTo(m_text->getLineStartIndex(cursorRow));
      
      // Sum the width of each char in the row to find the x coordinate

      for (int i = 0; i < cursorLineOffset; ++i)
        {
          x += getFont()->getCharWidth(iterator->getChar());
          iterator->moveToNext();
        }

      delete iterator;
    }

  // Add offset of row to calculated value

  x += getRowX(cursorRow);

  // Calculate y coordinate of the cursor

  y = getRowY(cursorRow);
}

/**
 * Gets the index of the character at the specified x coordinate in the
 * specified row.
 *
 * @param x X coordinate of the character.
 * @param rowIndex Index of the row containing the character.
 * @return The index of the character at the specified coordinate.
 */

int CMultiLineTextBox::getCharIndexAtCoordinate(nxgl_coord_t x, int rowIndex) const
{
  // Locate the character within the row

  int startIndex = m_text->getLineStartIndex(rowIndex);
  int stopIndex  = m_text->getLineLength(rowIndex);
  int width      = getRowX(rowIndex);
  int index      = -1;

  CStringIterator *iterator = m_text->newStringIterator();
  iterator->moveTo(startIndex);

  width += m_text->getFont()->getCharWidth(iterator->getChar());

  for (int i = 0; i < stopIndex; ++i)
    {
      if (width > x)
        {
          if (i == 0)
            {
              // If the coordinate is on the left of the text, we add nothing
              // to the index

              index = startIndex;
            }
          else
            {
              // Character within the row.
              // This is the character that contains the coordinate.

              index = startIndex + i;
            }
          break;
        }

      iterator->moveToNext();
      width += m_text->getFont()->getCharWidth(iterator->getChar());
    }

  delete iterator;

  // If the coordinate is past the last character, index will still be -1.
  // We need to set it to the last character

  if (index == -1)
    {
      if (rowIndex == m_text->getLineCount() - 1)
        {
          // Index past the end point of the text, so return an index
          // just past the text

          index = startIndex + stopIndex;
        }
      else
        {
          // Index at the end of a row, so return the last index of the
          // row

          index = startIndex + stopIndex - 1;
        }
    }

  return index;
}

/**
 * Get the index of the character at the specified coordinates.
 *
 * @param x X coordinate of the character.
 * @param y Y coordinate of the character.
 * @return The index of the character at the specified coordinates.
 */

unsigned int
CMultiLineTextBox::getCharIndexAtCoordinates(nxgl_coord_t x, nxgl_coord_t y) const
{
  int rowIndex = getRowContainingCoordinate(y);
  return getCharIndexAtCoordinate(x, rowIndex);
}

/**
 * Get the row containing the specified Y coordinate.
 *
 * @param y Y coordinate to locate.
 * @return The index of the row containing the specified Y coordinate.
 */

int CMultiLineTextBox::getRowContainingCoordinate(nxgl_coord_t y) const
{
  int row = -1;

  // Locate the row containing the character

  for (int i = 0; i < m_text->getLineCount(); ++i)
    {
      // Abort search if we've found the row below the y coordinate

      if (getRowY(i) > y)
        {
          if (i == 0)
            {
              // If the coordinate is above the text, we return the top
              // row

              row = 0;
            }
          else
            {
              // Row within the text, so return the previous row - this is
              // the row that contains the coordinate.

              row = i - 1;
            }

          break;
        }
    }

  // If the coordinate is below the text, row will still be -1.
  // We need to set it to the last row

  if (row == -1)
    {
      row = m_text->getLineCount() - 1;
    }

  return row;
}

/**
 * Draw the area of this widget that falls within the clipping region.
 * Called by the redraw() function to draw all visible regions.
 *
 * @param port The CGraphicsPort to draw to.
 * @see redraw()
 */

void CMultiLineTextBox::drawContents(CGraphicsPort *port)
{
  drawText(port);

  // Draw the cursor

  drawCursor(port);
}

/**
 * Draw the area of this widget that falls within the clipping region.
 * Called by the redraw() function to draw all visible regions.
 *
 * @param port The CGraphicsPort to draw to.
 * @see redraw()
 */

void CMultiLineTextBox::drawBorder(CGraphicsPort *port)
{
  port->drawFilledRect(0, 0, getWidth(), getHeight(), getBackgroundColor());

  // Stop drawing if the widget indicates it should not have an outline

  if (!isBorderless())
    {
      port->drawBevelledRect(0, 0, getWidth(), getHeight(),
                             getShadowEdgeColor(), getShineEdgeColor());
    }
}

/**
 * Move cursor one character to the left.
 */

void CMultiLineTextBox::moveCursorLeft(void)
{
  if (m_cursorPos > 0)
    {
      moveCursorToPosition(m_cursorPos - 1);
    }

  jumpToCursor();
}

/**
 * Move cursor one character to the right.
 */

void CMultiLineTextBox::moveCursorRight(void)
{
  if (m_cursorPos < (int)m_text->getLength())
    {
      moveCursorToPosition(m_cursorPos + 1);
    }

  jumpToCursor();
}

/**
 * Move cursor one row upwards.
 */

void CMultiLineTextBox::moveCursorUp(void)
{
  nxgl_coord_t cursorX = 0;
  nxgl_coord_t cursorY = 0;

  getCursorCoordinates(cursorX, cursorY);

  // Get the midpoint of the cursor.  We use the midpoint to ensure that
  // the cursor does not drift off to the left as it moves up the text, which
  // is a problem when we use the left edge as the reference point when the
  // font is proportional

  cursorX += m_text->getFont()->getCharWidth(m_text->getCharAt(m_cursorPos)) >> 1;

  // Locate the character above the midpoint

  int index = getCharIndexAtCoordinates(cursorX, cursorY + m_text->getLineHeight());
  moveCursorToPosition(index);
  jumpToCursor();
}

/**
 * Move cursor one row downwards.
 */

void CMultiLineTextBox::moveCursorDown(void)
{
  nxgl_coord_t cursorX = 0;
  nxgl_coord_t cursorY = 0;

  getCursorCoordinates(cursorX, cursorY);

  // Get the midpoint of the cursor.  We use the midpoint to ensure that
  // the cursor does not drift off to the left as it moves up the text, which
  // is a problem when we use the left edge as the reference point when the
  // font is proportional

  cursorX += m_text->getFont()->getCharWidth(m_text->getCharAt(m_cursorPos)) >> 1;

  // Locate the character above the midpoint

  int index = getCharIndexAtCoordinates(cursorX, cursorY - m_text->getLineHeight());
  moveCursorToPosition(index);
  jumpToCursor();
}

/**
 * Ensures that the textbox only contains the maximum allowed
 * number of rows by culling any excess rows from the top of
 * the text.
 *
 * @return True if lines were removed from the text; false if not.
 */

bool CMultiLineTextBox::cullTopLines(void)
{
  // Ensure that we have the correct number of rows

  if (m_text->getLineCount() > m_maxRows)
    {
      m_text->stripTopLines(m_text->getLineCount() - m_maxRows);
      return true;
    }

  return false;
}

/**
 * Ensures that the canvas height is the height of the widget,
 * if the widget exceeds the size of the text, or the height of
 * the text if the text exceeds the size of the widget.
 */

void CMultiLineTextBox::limitCanvasHeight(void)
{
  m_canvasHeight = m_text->getPixelHeight();

  CRect rect;
  getClientRect(rect);
  if (m_canvasHeight < rect.getHeight())
    {
      m_canvasHeight = rect.getHeight();
    }
}

/**
 * Ensures that the canvas cannot scroll beyond its height.
 */

void CMultiLineTextBox::limitCanvasY(void)
{
  CRect rect;
  getClientRect(rect);

  // Ensure that the visible portion of the canvas is not less than the
  // height of the viewer window

  if (m_canvasY + m_canvasHeight < rect.getHeight())
    {
      jumpToTextBottom();
    }
}

/**
 * Jumps to the cursor coordinates of the text.
 */

void CMultiLineTextBox::jumpToCursor(void)
{
  // Get the co-odinates of the cursor

  nxgl_coord_t cursorX;
  nxgl_coord_t cursorY;

  getCursorCoordinates(cursorX, cursorY);

  // Work out which row the cursor falls within

  int cursorRow = m_text->getLineContainingCharIndex(m_cursorPos);
  nxgl_coord_t rowY = getRowY(cursorRow);

  // If the cursor is outside the visible portion of the canvas, jump to it

  CRect rect;
  getClientRect(rect);

  if (rowY + m_text->getLineHeight() + m_canvasY > rect.getHeight())
    {
      // Cursor is below the visible portion of the canvas, so
      // jump down so that the cursor's row is the bottom row of
      // text

      jump(0, -(rowY + m_text->getLineHeight() - rect.getHeight()));
    }
  else if (rowY + m_canvasY < 0)
    {
      // Cursor is above the visible portion of the canvas, so
      // jump up so that the cursor's row is the top row of text

      jump(0, -cursorY);
    }
}

/**
 * Jumps to the bottom of the text.
 */

void CMultiLineTextBox::jumpToTextBottom(void)
{
  CRect rect;
  getClientRect(rect);
  jump(0, -(m_canvasHeight - rect.getHeight()));
}

/**
 * Resize the textbox to the new dimensions.
 *
 * @param width The new width.
 * @param height The new height.
 */

void CMultiLineTextBox::onResize(nxgl_coord_t width, nxgl_coord_t height)
{
  // Ensure the base class resize method is called

  CScrollingPanel::onResize(width, height);

  // Resize the canvas' width

  CRect rect;
  getClientRect(rect);

  m_canvasWidth  = rect.getWidth();
  m_canvasHeight = rect.getHeight();
  m_canvasX      = 0;
  m_canvasY      = 0;

  calculateVisibleRows();

  // Re-wrap the text

  m_text->setWidth(getWidth());
  m_text->wrap();

  bool raiseEvent = cullTopLines();
  limitCanvasHeight();
  limitCanvasY();

  if (raiseEvent)
    {
      m_widgetEventHandlers->raiseValueChangeEvent();
    }
}

/**
 * Starts the dragging system.
 *
 * @param x The x coordinate of the click.
 * @param y The y coordinate of the click.
 */

void CMultiLineTextBox::onClick(nxgl_coord_t x, nxgl_coord_t y)
{
  startDragging(x, y);

  // Move cursor to clicked coordinates

  CRect rect;
  getClientRect(rect);

  // Adjust x and y from screen coordinates to canvas coordinates

  nxgl_coord_t canvasRelativeX = x - getX() - rect.getX() - m_canvasX;
  nxgl_coord_t canvasRelativeY = y - getY() - rect.getY() - m_canvasY;

  moveCursorToPosition(getCharIndexAtCoordinates(canvasRelativeX, canvasRelativeY));
}

/**
 * Opens the keyboard on the bottom display.
 *
 * @param x The x coordinates of the click.
 * @param y The y coordinates of the click.
 */

void CMultiLineTextBox::onDoubleClick(nxgl_coord_t x, nxgl_coord_t y)
{
}

/**
 * Handles cursor control events.  Moves the cursor
 * in the direction selected.
 *
 * @param key The key that was pressed.
 */

void CMultiLineTextBox::handleCursorControlEvent(const CWidgetEventArgs &e)
{
  ECursorControl control = e.getCursorControl();

  switch (control)
    {
      case CURSOR_LEFT:
        moveCursorLeft();
        break;

      case CURSOR_RIGHT:
        moveCursorRight();
        break;

      case CURSOR_UP:
        moveCursorDown();
        break;

      case CURSOR_DOWN:
        moveCursorUp();
        break;

      default:
        // Not interested in other controls

        break;
    }
}

/**
 * Handle a keyboard press event.
 *
 * @param e The event data.
 */

    void handleKeyPressEvent(const CWidgetEventArgs &e);

/**
 * Gets the x position of a row of text based on the width of the row and the
 * type of horizontal alignment currently set.
 *
 * @param row The index of the row.
 * @return The x coordinate of the row.
 */

nxgl_coord_t CMultiLineTextBox::getRowX(int row) const
{
  CRect rect;
  getClientRect(rect);

  uint8_t rowLength = m_text->getLineTrimmedLength(row);
  uint8_t rowPixelWidth = m_text->getFont()->getStringWidth(*m_text, m_text->getLineStartIndex(row), rowLength);

  // Calculate horizontal position

  switch (m_hAlignment)
    {
      case TEXT_ALIGNMENT_HORIZ_CENTER:
        return (rect.getWidth() - rowPixelWidth) >> 1;

      case TEXT_ALIGNMENT_HORIZ_LEFT:
        return 0;

      case TEXT_ALIGNMENT_HORIZ_RIGHT:
        return rect.getWidth() - rowPixelWidth;
    }

  // Will never be reached

  return 0;
}

/**
 * Gets the y position of the specified row of text based on the type of
 * vertical alignment currently set.
 *
 * @param row The row number to find the y coordinate of.
 * @return The y coordinate of the specified row of text.
 */

nxgl_coord_t CMultiLineTextBox::getRowY(int row) const
{
  // If the amount of text exceeds the size of the widget, force
  // the text to be top-aligned

  if (m_visibleRows <= m_text->getLineCount())
    {
      return row * m_text->getLineHeight();
    }

  // All text falls within the textbox, so obey the alignment
  // options

  nxgl_coord_t textY    = 0;
  nxgl_coord_t startPos = 0;

  int canvasRows        = 0;
  int textRows          = 0;

  CRect rect;
  getClientRect(rect);

  // Calculate vertical position

  switch (m_vAlignment)
    {
      case TEXT_ALIGNMENT_VERT_CENTER:

        // Calculate the maximum number of rows

        canvasRows = m_canvasHeight / m_text->getLineHeight();
        textY = row * m_text->getLineHeight();

        // Get the number of rows of text

        textRows = m_text->getLineCount();

        // Ensure there's always one row

        if (textRows == 0)
          {
            textRows = 1;
          }

        // Calculate the start position of the block of text

        startPos = ((canvasRows - textRows) * m_text->getLineHeight()) >> 1;

        // Calculate the row Y coordinate

        textY = startPos + textY;
        break;

      case TEXT_ALIGNMENT_VERT_TOP:
        textY = row * m_text->getLineHeight();
        break;

      case TEXT_ALIGNMENT_VERT_BOTTOM:
        textY = rect.getHeight() - (((m_text->getLineCount() - row) * m_text->getLineHeight()));
        break;
    }

  return textY;
}

/**
 * Return true if the cursor is visible
 */

bool CMultiLineTextBox::isCursorVisible(void) const
{
  return (m_showCursor == SHOW_CURSOR_ALWAYS ||
         (m_showCursor == SHOW_CURSOR_ONFOCUS && hasFocus()));
}

/**
 * Gets the character under the cursor.
 *
 * @return The character under the cursor.
 */

nxwidget_char_t CMultiLineTextBox::getCursorChar(void) const
{
  if (m_cursorPos < m_text->getLength())
    {
      return m_text->getCharAt(m_cursorPos);
    }
  else
    {
      return ' ';
    }
}

/**
 * Works out the number of visible rows within the textbox.
 */

void CMultiLineTextBox::calculateVisibleRows(void)
{
  CRect rect;
  getClientRect(rect);

  m_visibleRows = rect.getHeight() / m_text->getLineHeight();
}

/**
 * Draws text.
 *
 * @param port The CGraphicsPort to draw to.
 */

void CMultiLineTextBox::drawText(CGraphicsPort *port)
{
  // Early exit if there is no text to display

  if (m_text->getLineCount() == 0)
    {
      return;
    }

  // Determine the top and bottom rows within the graphicsport's clip rect.
  // We only draw these rows in order to increase the speed of the routine.

  int regionY   = -m_canvasY + m_rect.getY(); // Y coord of canvas visible region
  int topRow    = getRowContainingCoordinate(regionY);
  int bottomRow = getRowContainingCoordinate(regionY + m_rect.getHeight());

  // Early exit checks

  if ((topRow < 0) && (bottomRow < 0))
    {
      return;
    }

  if ((bottomRow >= m_text->getLineCount()) && (topRow >= m_text->getLineCount()))
    {
      return;
    }

  // Prevent overflows

  if (topRow < 0)
    {
      topRow = 0;
    }

  if (bottomRow >= m_text->getLineCount())
    {
      bottomRow = m_text->getLineCount() - 1;
    }

  // Draw lines of text

  int currentRow = topRow;

  // Draw all rows in this region

  while (currentRow <= bottomRow)
    {
      drawRow(port, currentRow);
      currentRow++;
    }
}

/**
 * Draws the cursor.
 *
 * @param port The CGraphicsPort to draw to.
 */

void CMultiLineTextBox::drawCursor(CGraphicsPort *port)
{
  // Get the cursor coordinates

  if (isCursorVisible())
    {
      nxgl_coord_t cursorX = 0;
      nxgl_coord_t cursorY = 0;

      getCursorCoordinates(cursorX, cursorY);

      // Adjust for canvas offsets

      cursorX += m_canvasX;
      cursorY += m_canvasY;

      // Draw cursor

      port->invert(cursorX, cursorY,
                   m_text->getFont()->getCharWidth( getCursorChar()),
                   m_text->getFont()->getHeight());
    }
}

/**
 * Draws a single line of text.
 *
 * @param port The CGraphicsPort to draw to.
 * @param row The index of the row to draw.
 */

void CMultiLineTextBox::drawRow(CGraphicsPort *port, int row)
{
  // Get the drawing region

  CRect rect;
  getRect(rect);

  uint8_t rowLength = m_text->getLineTrimmedLength(row);

  struct nxgl_point_s pos;
  pos.x = getRowX(row) + m_canvasX;
  pos.y = getRowY(row) + m_canvasY;

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

  port->drawText(&pos, &rect, m_text->getFont(), *m_text,
                 m_text->getLineStartIndex(row), rowLength, textColor);
}
