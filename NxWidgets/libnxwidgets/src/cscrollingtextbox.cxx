/****************************************************************************
 * NxWidgets/libnxwidgets/src/cscrollingtextbox.cxx
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

#include "cscrollingtextbox.hxx"
#include "cscrollbarvertical.hxx"
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

CScrollingTextBox::CScrollingTextBox(CWidgetControl *pWidgetControl,
                                     nxgl_coord_t x, nxgl_coord_t y,
                                     nxgl_coord_t width, nxgl_coord_t height,
                                     const CNxString &text, uint32_t flags,
                                     nxgl_coord_t maxRows, CWidgetStyle *style)
: CNxWidget(pWidgetControl, x, y, width, height, flags, style)
{
  m_scrollbarWidth = 10;
  setBorderless(true);

  m_texbox = new CMultiLineTextBox(pWidgetControl,
                                   0, 0, width - m_scrollbarWidth, height,
                                   text, flags, maxRows, &m_style);
  m_texbox->addWidgetEventHandler(this);
  
  // Create scrollbar

  CRect rect;
  m_texbox->getClientRect(rect);
  m_scrollbar = new CScrollbarVertical(pWidgetControl,
                                       width - m_scrollbarWidth, 0,
                                       m_scrollbarWidth, height, &m_style);
  m_scrollbar->setMinimumValue(0);
  m_scrollbar->setMaximumValue(m_texbox->getCanvasHeight());
  m_scrollbar->setPageSize(rect.getHeight());
  m_scrollbar->setValue(0 - m_texbox->getCanvasY());
  m_scrollbar->addWidgetEventHandler(this);

  // Add children to child array

  addWidget(m_texbox);
  addWidget(m_scrollbar);
}

/**
 * Set the horizontal alignment of text within the textbox.
 *
 * @param alignment The horizontal position of the text.
 */

void CScrollingTextBox::setTextAlignmentHoriz(CMultiLineTextBox::TextAlignmentHoriz alignment)
{
  m_texbox->setTextAlignmentHoriz(alignment);
}

/**
 * Set the vertical alignment of text within the textbox.
 *
 * @param alignment The vertical position of the text.
 */

void CScrollingTextBox::setTextAlignmentVert(CMultiLineTextBox::TextAlignmentVert alignment)
{
  m_texbox->setTextAlignmentVert(alignment);
}

/**
 * Returns the number of "pages" that the text spans.  A page
 * is defined as the amount of text that can be displayed within
 * the textbox at one time.
 *
 * @return The page count.
 */

const uint16_t CScrollingTextBox::getPageCount(void) const
{
  return m_texbox->getPageCount();
}

/**
 * Returns the current page.
 *
 * @return The current page.
 * @see getPageCount().
 */

const uint16_t CScrollingTextBox::getCurrentPage(void) const
{
  return m_texbox->getCurrentPage();
}

/**
 * Returns a pointer to the Text object that contains the
 * wrapped text used in the textbox.  It is used as the
 * pre-processed data source for the textbox, and should
 * not be altered.
 *
 * @return Pointer to the Text object.
 */

const CText *CScrollingTextBox::getText(void) const
{
  return m_texbox->getText();
}

/**
 * Set the text displayed in the textbox.
 *
 * @param text String to display.
 */

void CScrollingTextBox::setText(const CNxString &text)
{
  m_texbox->setText(text);
  m_scrollbar->redraw();
}

/**
 * Append new text to the end of the current text
 * displayed in the textbox.
 *
 * @param text String to append.
 */

void CScrollingTextBox::appendText(const CNxString &text)
{
  m_texbox->appendText(text);
  m_scrollbar->redraw();
}

/**
 * Remove all characters from the string from the start index onwards.
 *
 * @param startIndex Index to remove from.
 */

void CScrollingTextBox::removeText(const unsigned int startIndex)
{
  m_texbox->removeText(startIndex);
  m_scrollbar->redraw();
}

/**
 * Remove specified number of characters from the string from the
 * start index onwards.
 *
 * @param startIndex Index to remove from.
 * @param count Number of characters to remove.
 */

void CScrollingTextBox::removeText(const unsigned int startIndex,
                                   const unsigned int count)
{
  m_texbox->removeText(startIndex, count);
  m_scrollbar->redraw();
}

/**
 * Set the font used in the textbox.
 *
 * @param font Pointer to the new font.
 */

void CScrollingTextBox::setFont(CNxFont *font)
{
  m_style.font = font;
  m_texbox->setFont(font);
  m_scrollbar->setFont(font);
}

/**
 * Get the length of the text string.
 *
 * @return The length of the text string.
 */

const unsigned int CScrollingTextBox::getTextLength(void) const
{
  return m_texbox->getTextLength();
}

/**
 * Sets the cursor display mode.
 *
 * @param cursorMode Determines cursor display mode
 */

void CScrollingTextBox::showCursor(EShowCursor cursorMode)
{
  m_texbox->showCursor(cursorMode);
}

/**
 * Enables/disables cursor wrapping
 *
 * @param wrap True enables cursor wrapping
 */

void CScrollingTextBox::wrapCursor(bool wrap)
{
  m_texbox->wrapCursor(wrap);
}

/**
 * Move the cursor to the text position specified.  0 indicates the start
 * of the string.  If position is greater than the length of the string,
 * the cursor is moved to the end of the string.
 *
 * @param position The new cursor position.
 */

void CScrollingTextBox::moveCursorToPosition(const int32_t position)
{
  m_texbox->moveCursorToPosition(position);
  m_scrollbar->redraw();
}

/**
 * Get the cursor position.  This is the index within the string that
 * the cursor is currently positioned over.
 *
 * @return position The cursor position.
 */

const int32_t CScrollingTextBox::getCursorPosition(void) const
{
  return m_texbox->getCursorPosition();
}

/**
 * Insert text at the specified index.
 *
 * @param text The text to insert.
 * @param index Index at which to insert the text.
 */

void CScrollingTextBox::insertText(const CNxString &text,
                                   const unsigned int index)
{
  m_texbox->insertText(text, index);
  m_scrollbar->redraw();
}

/**
 * Insert text at the current cursor position.
 *
 * @param text The text to insert.
 */

void CScrollingTextBox::insertTextAtCursor(const CNxString &text)
{
  m_texbox->insertTextAtCursor(text);
  m_scrollbar->redraw();
}

/**
 * Handles events raised by its sub-widgets.
 *
 * @param e Event arguments.
 */

void CScrollingTextBox::handleValueChangeEvent(const CWidgetEventArgs &e)
{
  if (e.getSource() != NULL)
    {
      if (e.getSource() == m_scrollbar)
        {
          if (m_texbox != NULL)
            {
              m_texbox->setRaisesEvents(false);
              m_texbox->jump(0, 0 - m_scrollbar->getValue());
              m_texbox->setRaisesEvents(true);
            }
        }
      else if (e.getSource() == m_texbox)
        {
          if (m_scrollbar != NULL)
            {
              m_scrollbar->setRaisesEvents(false);
              m_scrollbar->setMaximumValue(m_texbox->getCanvasHeight());
              m_scrollbar->setValue(0 - m_texbox->getCanvasY());
              m_scrollbar->setRaisesEvents(true);
            }
        }
    }
}

/**
 * Handles events raised by its sub-widgets.
 *
 * @param e Event arguments.
 */

void CScrollingTextBox::handleScrollEvent(const CWidgetEventArgs &e)
{
  if (e.getSource() != NULL)
    {
      if (e.getSource() == m_texbox)
        {
          if (m_scrollbar != NULL)
            {
              m_scrollbar->setRaisesEvents(false);
              m_scrollbar->setValue(0 - m_texbox->getCanvasY());
              m_scrollbar->setRaisesEvents(true);
            }
        }
    }
}

/**
 * Gets the x coordinate of the virtual canvas.
 *
 * @return The x coordinate of the virtual canvas.
 */

const int32_t CScrollingTextBox::getCanvasX(void) const
{
  return m_texbox->getCanvasX();
}

/**
 * Gets the y coordinate of the virtual canvas.
 *
 * @return The y coordinate of the virtual canvas.
 */

const int32_t CScrollingTextBox::getCanvasY(void) const
{
  return m_texbox->getCanvasY();
}

/**
 * Gets the width of the virtual canvas.
 *
 * @return The width of the virtual canvas.
 */

const int32_t CScrollingTextBox::getCanvasWidth(void) const
{
  return m_texbox->getCanvasWidth();
}

/**
 * Gets the height of the virtual canvas.
 *
 * @return The height of the virtual canvas.
 */

const int32_t CScrollingTextBox::getCanvasHeight(void) const
{
  return m_texbox->getCanvasHeight();
}

/**
 * Scroll the panel by the specified amounts.
 *
 * @param dx The horizontal distance to scroll.
 * @param dy The vertical distance to scroll.
 */

void CScrollingTextBox::scroll(int32_t dx, int32_t dy)
{
  m_texbox->scroll(dx, dy);
}

/**
 * Reposition the panel's scrolling region to the specified coordinates.
 *
 * @param x The new x coordinate of the scrolling region.
 * @param y The new y coordinate of the scrolling region.
 */

void CScrollingTextBox::jump(int32_t x, int32_t y)
{
  m_texbox->jump(x, y);
}

/**
 * Set whether or not horizontal scrolling is allowed.
 *
 * @param allow True to allow horizontal scrolling; false to deny it.
 */

void CScrollingTextBox::setAllowsVerticalScroll(bool allow)
{
  m_texbox->setAllowsVerticalScroll(allow);
}

/**
 * Set whether or not horizontal scrolling is allowed.
 *
 * @param allow True to allow horizontal scrolling; false to deny it.
 */

void CScrollingTextBox::setAllowsHorizontalScroll(bool allow)
{
  // NOP
}

/**
 * Sets the width of the virtual canvas.
 *
 * @param width The width of the virtual canvas.
 */

void CScrollingTextBox::setCanvasWidth(const int32_t width)
{
  // NOP
}

/**
 * Sets the height of the virtual canvas.
 *
 * @param height The height of the virtual canvas.
 */

void CScrollingTextBox::setCanvasHeight(const int32_t height)
{
  // NOP
}

/**
 * Returns true if vertical scrolling is allowed.
 *
 * @return True if vertical scrolling is allowed.
 */

bool CScrollingTextBox::allowsVerticalScroll(void) const
{
  return m_texbox->allowsVerticalScroll();
}

/**
 * Returns true if horizontal scrolling is allowed.
 *
 * @return True if horizontal scrolling is allowed.
 */

bool CScrollingTextBox::allowsHorizontalScroll(void) const
{
  return m_texbox->allowsHorizontalScroll();
}

/**
 * Draw the area of this widget that falls within the clipping region.
 * Called by the redraw() function to draw all visible regions.
 *
 * @param port The CGraphicsPort to draw to.
 * @see redraw()
 */

void CScrollingTextBox::drawContents(CGraphicsPort *port)
{
  port->drawFilledRect(0, 0, getWidth(), getHeight(), getBackgroundColor());
}

/**
 * Resize the textbox to the new dimensions.
 *
 * @param width The new width.
 * @param height The new height.
 */

void CScrollingTextBox::onResize(nxgl_coord_t width, nxgl_coord_t height)
{
  // Resize the children

  m_texbox->resize(width - m_scrollbarWidth, height);
  m_scrollbar->resize(m_scrollbarWidth, height);

  // Move the scrollbar

  m_scrollbar->moveTo(width - m_scrollbarWidth, 0);
}
