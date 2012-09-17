/****************************************************************************
 * NxWidgets/libnxwidgets/include/cscrollingtextbox.hxx
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

#ifndef __INCLUDE_CSCROLLINGTEXTBOX_HXX
#define __INCLUDE_CSCROLLINGTEXTBOX_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/nx/nxglib.h>

#include "cmultilinetextbox.hxx"
#include "cwidgeteventhandler.hxx"
#include "cwidgetstyle.hxx"
#include "cnxstring.hxx"
#include "iscrollable.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
 
/****************************************************************************
 * Implementation Classes
 ****************************************************************************/
 
#if defined(__cplusplus)

namespace NXWidgets
{
  class CScrollbarVertical;
  class CNxFont;

  /**
   * Widget containing a CMultiLineTextBox and a vertical scrollbar.  Exposed
   * methods are more or less identical to the methods exposed by the
   * CMultiLineTextBox to ensure that the two are interchangeable.
   */

  class CScrollingTextBox : public ITextBox, public CNxWidget,
                           public IScrollable, public CWidgetEventHandler
  {
  protected:
    CMultiLineTextBox  *m_texbox;         /**< Pointer to the textbox */
    CScrollbarVertical *m_scrollbar;      /**< Pointer to the scrollbar */
    uint8_t             m_scrollbarWidth; /**< Width of the scrollbar */

    /**
     * Draw the area of this widget that falls within the clipping region.
     * Called by the redraw() function to draw all visible regions.
     *
     * @param port The CGraphicsPort to draw to.
     * @see redraw()
     */

    virtual void drawContents(CGraphicsPort *port);

    /**
     * Resize the textbox to the new dimensions.
     *
     * @param width The new width.
     * @param height The new height.
     */

    virtual void onResize(nxgl_coord_t width, nxgl_coord_t height);
    
    /**
     * Destructor.
     */

    virtual inline ~CScrollingTextBox(void) { }

    /**
     * Copy constructor is protected to prevent usage.
     */

    inline CScrollingTextBox(const CScrollingTextBox& scrollingTextBox)
    : CNxWidget(scrollingTextBox) { }

  public:

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

    CScrollingTextBox(CWidgetControl *pWidgetControl,
                      nxgl_coord_t x, nxgl_coord_t y,
                      nxgl_coord_t width, nxgl_coord_t height,
                      const CNxString &text, uint32_t flags,
                      nxgl_coord_t maxRows = 0,
                      CWidgetStyle *style = (CWidgetStyle *)NULL);

    /**
     * Set the horizontal alignment of text within the textbox.
     *
     * @param alignment The horizontal position of the text.
     */

    virtual void setTextAlignmentHoriz(CMultiLineTextBox::TextAlignmentHoriz alignment);

    /**
     * Set the vertical alignment of text within the textbox.
     *
     * @param alignment The vertical position of the text.
     */

    virtual void setTextAlignmentVert(CMultiLineTextBox::TextAlignmentVert alignment);

    /**
     * Returns the number of "pages" that the text spans.  A page
     * is defined as the amount of text that can be displayed within
     * the textbox at one time.
     *
     * @return The page count.
     */

    virtual const uint16_t getPageCount(void) const;

    /**
     * Returns the current page.
     *
     * @return The current page.
     * @see getPageCount().
     */

    virtual const uint16_t getCurrentPage(void) const;

    /**
     * Returns a pointer to the Text object that contains the
     * wrapped text used in the textbox.  It is used as the
     * pre-processed data source for the textbox, and should
     * not be altered.
     *
     * @return Pointer to the Text object.
     */

    virtual const CText *getText(void) const;

    /**
     * Set the text displayed in the textbox.
     *
     * @param text String to display.
     */

    virtual void setText(const CNxString &text);

    /**
     * Append new text to the end of the current text
     * displayed in the textbox.
     *
     * @param text String to append.
     */

    virtual void appendText(const CNxString &text);
    
    /**
     * Remove all characters from the string from the start index onwards.
     *
     * @param startIndex Index to remove from.
     */

    virtual void removeText(const unsigned int startIndex);

    /**
     * Remove specified number of characters from the string from the
     * start index onwards.
     *
     * @param startIndex Index to remove from.
     * @param count Number of characters to remove.
     */

    virtual void removeText(const unsigned int startIndex, const unsigned int count);

    /**
     * Set the font used in the textbox.
     *
     * @param font Pointer to the new font.
     */

    virtual void setFont(CNxFont *font);

    /**
     * Get the length of the text string.
     *
     * @return The length of the text string.
     */

    virtual const unsigned int getTextLength(void) const;

    /**
     * Sets the cursor display mode.
     *
     * @param cursorMode Determines cursor display mode
     */

    virtual void showCursor(EShowCursor cursorMode);

    /**
     * Shows the cursor in default mode (only when the TextBox has focus).
     */

    inline void showCursor(void)
    {
      showCursor(SHOW_CURSOR_ONFOCUS);
    }

    /**
     * Hides the cursor.
     */

    inline void hideCursor(void)
    {
      showCursor(SHOW_CURSOR_NEVER);
    }

    /**
     * Enables/disables cursor wrapping
     *
     * @param wrap True enables cursor wrapping
     */

    virtual void wrapCursor(bool wrap);

    /**
     * Move the cursor to the text position specified.  0 indicates the start
     * of the string.  If position is greater than the length of the string,
     * the cursor is moved to the end of the string.
     *
     * @param position The new cursor position.
     */

    virtual void moveCursorToPosition(const int32_t position);

    /**
     * Get the cursor position.  This is the index within the string that
     * the cursor is currently positioned over.
     *
     * @return position The cursor position.
     */

    virtual const int32_t getCursorPosition(void) const;

    /**
     * Insert text at the specified index.
     *
     * @param text The text to insert.
     * @param index Index at which to insert the text.
     */

    virtual void insertText(const CNxString &text, const unsigned int index);
    
    /**
     * Insert text at the current cursor position.
     *
     * @param text The text to insert.
     */

    virtual void insertTextAtCursor(const CNxString &text);

    /**
     * Handles events raised by its sub-widgets.
     *
     * @param e Event arguments.
     */

    virtual void handleValueChangeEvent(const CWidgetEventArgs &e);

    /**
     * Handles events raised by its sub-widgets.
     *
     * @param e Event arguments.
     */

    virtual void handleScrollEvent(const CWidgetEventArgs &e);

    /**
     * Gets the x coordinate of the virtual canvas.
     *
     * @return The x coordinate of the virtual canvas.
     */

    virtual const int32_t getCanvasX(void) const;
    
    /**
     * Gets the y coordinate of the virtual canvas.
     *
     * @return The y coordinate of the virtual canvas.
     */

    virtual const int32_t getCanvasY(void) const;

    /**
     * Gets the width of the virtual canvas.
     *
     * @return The width of the virtual canvas.
     */

    virtual const int32_t getCanvasWidth(void) const;
    
    /**
     * Gets the height of the virtual canvas.
     *
     * @return The height of the virtual canvas.
     */

    virtual const int32_t getCanvasHeight(void) const;

    /**
     * Scroll the panel by the specified amounts.
     *
     * @param dx The horizontal distance to scroll.
     * @param dy The vertical distance to scroll.
     */

    virtual void scroll(int32_t dx, int32_t dy);
    
    /**
     * Reposition the panel's scrolling region to the specified coordinates.
     *
     * @param x The new x coordinate of the scrolling region.
     * @param y The new y coordinate of the scrolling region.
     */

    virtual void jump(int32_t x, int32_t y);

    /**
     * Set whether or not horizontal scrolling is allowed.
     *
     * @param allow True to allow horizontal scrolling; false to deny it.
     */

    virtual void setAllowsVerticalScroll(bool allow);

    /**
     * Set whether or not horizontal scrolling is allowed.
     *
     * @param allow True to allow horizontal scrolling; false to deny it.
     */

    virtual void setAllowsHorizontalScroll(bool allow);

    /**
     * Sets the width of the virtual canvas.
     *
     * @param width The width of the virtual canvas.
     */

    virtual void setCanvasWidth(const int32_t width);
    
    /**
     * Sets the height of the virtual canvas.
     *
     * @param height The height of the virtual canvas.
     */

    virtual void setCanvasHeight(const int32_t height);

    /**
     * Returns true if vertical scrolling is allowed.
     *
     * @return True if vertical scrolling is allowed.
     */

    virtual bool allowsVerticalScroll(void) const;

    /**
     * Returns true if horizontal scrolling is allowed.
     *
     * @return True if horizontal scrolling is allowed.
     */

    virtual bool allowsHorizontalScroll(void) const;
  };
}

#endif // __cplusplus

#endif // __INCLUDE_CSCROLLINGTEXTBOX_HXX

