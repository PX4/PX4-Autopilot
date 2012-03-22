/****************************************************************************
 * NxWidgets/libnxwidgets/src/ctextbox.cxx
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
#include "ctextbox.hxx"
#include "cgraphicsport.hxx"
#include "cnxtimer.hxx"
#include "cstringiterator.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * CButton Method Implementations
 ****************************************************************************/

using namespace NXWidgets;

/**
 * Constructor for a textbox containing a string.
 *
 * @param pWidgetControl The controlling widget for the window
 * @param x The x coordinate of the text box, relative to its parent.
 * @param y The y coordinate of the text box, relative to its parent.
 * @param width The width of the textbox.
 * @param height The height of the textbox.
 * @param text Pointer to a string to display in the textbox.
 * @param style The style that the widget should use.  If this is not
 *   specified, the widget will use the values stored in the global
 *   defaultCWidgetStyle object.  The widget will copy the properties of
 *   the style into its own internal style object.
 */

CTextBox::CTextBox(CWidgetControl *pWidgetControl, nxgl_coord_t x, nxgl_coord_t y,
                   nxgl_coord_t width, nxgl_coord_t height,
                   const CNxString &text, CWidgetStyle *style)
                 : CLabel(pWidgetControl, x, y, width, height, text, style)
{
  // Initialize state

  m_cursorPos             = 0;
  m_showCursor            = SHOW_CURSOR_ONFOCUS;
  m_wrapCursor            = false;
  m_textChange            = false;
  m_flags.doubleClickable = true;

  // Set the border thickness (2 lines)

  m_borderSize.top        = 2;
  m_borderSize.right      = 2;
  m_borderSize.bottom     = 2;
  m_borderSize.left       = 2;

  // Move the cursor to the end of the string

  moveCursorToPosition(m_text.getLength());

  // Register to receive keypress events

  addWidgetEventHandler(static_cast<CWidgetEventHandler *>(this));
}

/**
 * Shows the cursor.
 *
 * @param cursorMode Determines cursor display mode
 */

void CTextBox::showCursor(EShowCursor cursorMode)
{
  if (m_showCursor != cursorMode)
    {
      m_showCursor = cursorMode;
      redraw();
    }
}

/**
 * Set the text displayed in the label.
 *
 * @param text String to display.
 */

void CTextBox::setText(const CNxString &text)
{
  m_text.setText(text);
  repositionCursor(m_text.getLength());
  onTextChange();
}

/**
 * Append new text to the end of the current text displayed in the
 * label.
 *
 * @param text String to append.
 */

void CTextBox::appendText(const CNxString &text)
{
  m_text.append(text);
  repositionCursor(m_text.getLength());
  onTextChange();
}

/**
 * Remove all characters from the string from the start index onwards.
 *
 * @param startIndex Index to remove from.
 */

void CTextBox::removeText(const unsigned int startIndex)
{
  m_text.remove(startIndex);
  repositionCursor(startIndex);
  onTextChange();
}

/**
 * Remove specified number of characters from the string from the
 * start index onwards.
 *
 * @param startIndex Index to remove from.
 * @param count Number of characters to remove.
 */

void CTextBox::removeText(const unsigned int startIndex, const unsigned int count)
{
  m_text.remove(startIndex, count);
  repositionCursor(startIndex);
  onTextChange();
}

/**
 * Insert text at the specified index.
 *
 * @param text The text to insert.
 * @param index Index at which to insert the text.
 */

void CTextBox::insertText(const CNxString &text, const unsigned int index)
{
  m_text.insert(text, index);
  repositionCursor(index + text.getLength());
  onTextChange();
}

/**
 * Insert text at the current cursor position.
 *
 * @param text The text to insert.
 */

void CTextBox::insertTextAtCursor(const CNxString &text)
{
  insertText(text, getCursorPosition());
}

/**
 * Move the cursor to the text position specified.  0 indicates the
 * start of the string.  If position is greater than the length of the
 * string, the cursor is moved to the end of the string.
 *
 * @param position The new cursor position.
 */

void CTextBox::moveCursorToPosition(const int position)
{
  if (repositionCursor(position))
    {
      calculateTextPositionHorizontal();
      redraw();
    }
}

/**
 * Handle a keyboard press event.
 *
 * @param e The event data.
 */

void CTextBox::handleKeyPressEvent(const CWidgetEventArgs &e)
{
  nxwidget_char_t key = e.getKey();

  if (key == KEY_CODE_BACKSPACE)
    {
      if (m_cursorPos == 0) return;

      // Delete the character in front of the cursor

      removeText(m_cursorPos - 1, 1);
    }
  else if (key == KEY_CODE_ENTER)
    {
      // Fire an action event

      m_widgetEventHandlers->raiseActionEvent();
    }
  else if (key != KEY_CODE_NONE)
    {
      // Not modifier; append value

      insertTextAtCursor(key);
    } 
}

/**
 * Handle a cursor control event.  Replaces CWidgetEventHandler method.
 *
 * @param e The event data.
 */

void CTextBox::handleCursorControlEvent(const CWidgetEventArgs &e)
{
  ECursorControl control = e.getCursorControl();

  if (control == CURSOR_LEFT)
    {
      if (m_cursorPos > 0)
        {
          moveCursorToPosition(m_cursorPos - 1);
        }
      else if (m_wrapCursor)
        {
          moveCursorToPosition(m_text.getLength() - 1);
        }
    }
  else if (control == CURSOR_RIGHT)
    {
      if (m_cursorPos < m_text.getLength())
        {
          moveCursorToPosition(m_cursorPos + 1);
        }
      else if (m_wrapCursor)
        {
          moveCursorToPosition(0);
        }
    }
}

/**
 * Redraws the widget
 */

void CTextBox::onBlur(void)
{
  redraw();
}

/**
 * Draw the area of this widget that falls within the clipping region.
 * Called by the redraw() function to draw all visible regions.
 *
 * @param port The CGraphicsPort to draw to.
 * @see redraw()
 */

void CTextBox::drawContents(CGraphicsPort *port)
{
  // Get the drawing area (excluding the border)

  CRect rect;
  getRect(rect);

  // Determine the background and text color

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

  // Get the X/Y position of the text within the textbox

  struct nxgl_point_s pos;
  pos.x = rect.getX() + m_align.x;
  pos.y = rect.getY() + m_align.y;

  // And draw the text

  CNxFont *font = getFont();
  port->drawText(&pos, &rect, font, m_text, 0, m_text.getLength(), textColor);

  // Draw cursor

  if (isCursorVisible())
    {
      // Calculate the (relative) cursor position
      // Warning: drawText modifies pos!

      pos.x = getCursorXPos() + m_align.x;
      pos.y = m_align.y;

      // Calculate the cursor width
 
      struct nxgl_size_s size;
      size.w = getCursorWidth();
      size.h = getFont()->getHeight();

      // Is the cursor wholly within the visible region?

      if (pos.x >= 0 && pos.x + size.w < rect.getWidth())
        {
          // Invert the colors in the cursor region.

          pos.x += rect.getX();
          pos.y += rect.getY();
 
          port->invert(pos.x, pos.y, size.w, size.h);
        }
    }
}

/**
 * Moves the cursor without redrawing.
 *
 * @param position New cursor position.
 */

bool CTextBox::repositionCursor(const int position)
{
  int len = m_text.getLength();

  // Set the cursor position if the new position is within the string.

  if (m_cursorPos != position && position <= len)
    {
      m_cursorPos = position;
      return true;
    }

  return false;
}

/**
 * Move the cursor to the specified coordinates.  The coordinates
 * are expected to be the result of a click, and therefore in
 * world-space rather than widget-space.
 */

void CTextBox::moveCursorToClickLocation(nxgl_coord_t x, nxgl_coord_t y)
{
  // Work out where in the string the click coordinates represent
  // and move the cursor to that location

  if (m_text.getLength() > 0)
    {
      // Transform click coordinates to widget-space coordinates

      nxgl_coord_t clickX = x - getX() - m_borderSize.left;
      nxgl_coord_t charX  = m_align.x;

      // Locate the first character that comes after the clicked character

      CStringIterator *iterator = m_text.newStringIterator();

      while (charX < clickX)
        {
          charX += getFont()->getCharWidth(iterator->getChar());
      
          if (!iterator->moveToNext())
            {
              break;
            }
        }
    
      int index = iterator->getIndex();

      // Move back to the clicked character if we've moved past it

      if (charX > clickX)
        {
          iterator->moveToPrevious();
          index = iterator->getIndex();
        }
      else if (charX < clickX)
        {
          // Move past end of string if click is after the text

          index++;
        }

      moveCursorToPosition(index);
      delete iterator;
    }
}

/**
 * Draw the area of this widget that falls within the clipping region.
 * Called by the redraw() function to draw all visible regions.
 *
 * @param port The CGraphicsPort to draw to.
 * @see redraw()
 */

void CTextBox::drawBorder(CGraphicsPort *port)
{
  // Check if the widget indicates it should have an outline: That
  // the outline is enabled and the this is not just a text-only
  // redraw

  if (!isBorderless() && !isTextChange())
    {
      port->drawBevelledRect(getX(), getY(), getWidth(), getHeight(),
                             getShineEdgeColor(), getShadowEdgeColor());
      port->drawBevelledRect(getX() + 1, getY() + 1, getWidth() - 2, getHeight() - 2,
                             getShadowEdgeColor(), getShineEdgeColor());
    }
}

/**
 * Moves the cursor to the clicked coordinates.
 *
 * @param x The x coordinates of the click.
 * @param y The y coordinates of the click.
 */

void CTextBox::onClick(nxgl_coord_t x, nxgl_coord_t y)
{
  moveCursorToClickLocation(x, y);
}

/**
 * Does nothing.
 *
 * @param x The x coordinates of the click.
 * @param y The y coordinates of the click.
 */

void CTextBox::onDoubleClick(nxgl_coord_t x, nxgl_coord_t y)
{
  // No double click action
}

/**
 * Return true if the cursor is visible
 */

bool CTextBox::isCursorVisible(void) const
{
  return (m_showCursor == SHOW_CURSOR_ALWAYS ||
         (m_showCursor == SHOW_CURSOR_ONFOCUS && hasFocus()));
}

/**
 * Get the x coordinate of the cursor in pixels relative
 * to the left-hand edge of the client rect.
 *
 * @return The x coordinate of the cursor in pixels.
 */

const nxgl_coord_t CTextBox::getCursorXPos(void) const
{
  // Calculate position of cursor

  nxgl_coord_t cursorX = 0;

  CStringIterator *iterator = m_text.newStringIterator();
  
  for (nxgl_coord_t i = 0; i < m_cursorPos; i++)
    {
      cursorX += getFont()->getCharWidth(iterator->getChar());
      iterator->moveToNext();
    }
  
  delete iterator;
  return cursorX;
}

/**
 * Get the width of the cursor in pixels.
 *
 * @return The width of the cursor in pixels.
 */

nxgl_coord_t CTextBox::getCursorWidth(void) const
{
  if (m_cursorPos < m_text.getLength())
    {
      // Cursor within the string - get the width of the character

      return getFont()->getCharWidth(m_text.getCharAt(m_cursorPos));
    }
  else
    {
      // Cursor past end of string - get the width of a space

      return getFont()->getCharWidth(' ');
    }
}

/**
 * Calculate the horizontal position of the string based on its length
 * and the alignment options.  Alignment options are overridden if the
 * width of the string exceeds the width of the textbox.
 */

void CTextBox::calculateTextPositionHorizontal(void)
{
  // Calculate the string width - if the width is longer than the box,
  // ignore alignment and align left

  nxgl_coord_t stringWidth = getFont()->getStringWidth(m_text);
  
  // Add the width of a blank space to the width to ensure that we can
  // see the cursor

  if (isCursorVisible())
    {
      stringWidth += getFont()->getCharWidth(' ');
    }
  
  CRect rect;
  getClientRect(rect);
  
  // Use alignment options if cursor is hidden or string is smaller
  // than textbox

  nxgl_coord_t width = rect.getWidth();
  if ((stringWidth < width) || !isCursorVisible())
    {
      // Text not wider than box, so apply alignment options

      switch (m_hAlignment)
        {
        case TEXT_ALIGNMENT_HORIZ_CENTER:
          m_align.x = (width - stringWidth) >> 1;
          break;

        case TEXT_ALIGNMENT_HORIZ_LEFT:
          m_align.x = rect.getX();
          break;

        case TEXT_ALIGNMENT_HORIZ_RIGHT:
          m_align.x = width - stringWidth;
          break;
        }

      return;
  }

  // Text is wider than box - view needs to follow the cursor
  // If cursor is at the end of the text, we can just right-align

  if (m_cursorPos == m_text.getLength())
    {
      m_align.x = width - stringWidth;
      return;
    }

  // Work out the coordinates of the left edge of the cursor

  int cursorX1 = getCursorXPos();

  // Work out the coordinates of the right edge of the cursor

  int cursorX2 = cursorX1 + getCursorWidth();

  // Ensure that the cursor is on-screen

  if (cursorX1 + m_align.x < 0)
    {
      // Cursor is off left side of screen, so adjust m_align.x

      m_align.x = 0 - cursorX1;
    }
  else if (cursorX2 + m_align.x > width)
    {
      // Cursor is off right side of screen, so adjust m_align.x

      m_align.x = width - cursorX2;
    }

  // We need to ensure that the text cannot be positioned in
  // such a way that there is a gap between the end of the
  // text and the right edge of the textbox

  if (stringWidth + m_align.x < width)
    {
      m_align.x = width - stringWidth;
    }
}
