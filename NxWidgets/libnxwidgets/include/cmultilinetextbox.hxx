/****************************************************************************
 * NxWidgets/libnxwidgets/include/cmultilinetextbox.hxx
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

#ifndef __INCLUDE_CMULTILINETEXTBOX_HXX
#define __INCLUDE_CMULTILINETEXTBOX_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nx.h>

#include "cscrollingpanel.hxx"
#include "cwidgetstyle.hxx"
#include "cnxstring.hxx"
#include "ctext.hxx"
#include "cwidgeteventhandler.hxx"
#include "itextbox.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
 
/****************************************************************************
 * Implementation Classes
 ****************************************************************************/

#if defined(__cplusplus)

namespace NXWidgets
{  
  class CNxTimer;

  /**
   * Textbox that offers multiple lines of text.  Has scrolling
   * capability and can be dragged using the mouse.  The text
   * it contains can be changed or added to.  It can remember more
   * rows of text than it can display, and these additional
   * rows can be scrolled through.
   */

  class CMultiLineTextBox : public ITextBox, public CScrollingPanel,
                            public CWidgetEventHandler 
  {
  public:

    /**
     * Enum of horizontal alignment options.
     */

    enum TextAlignmentHoriz
    {
      TEXT_ALIGNMENT_HORIZ_CENTER = 0,    /**< Centre the text */
      TEXT_ALIGNMENT_HORIZ_LEFT   = 1,    /**< Align left */
      TEXT_ALIGNMENT_HORIZ_RIGHT  = 2     /**< Align right */
    };

    /**
     * Enum of vertical alignment options.
     */

    enum TextAlignmentVert
    {
      TEXT_ALIGNMENT_VERT_CENTER  = 0,    /**< Align to centre of textbox */
      TEXT_ALIGNMENT_VERT_TOP     = 1,    /**< Align to top of textbox */
      TEXT_ALIGNMENT_VERT_BOTTOM  = 2     /**< Align to bottom of textbox */
    };

  protected:
    CText             *m_text;        /**< CText object that manipulates
                                           and wraps the raw text string. */
    uint8_t            m_visibleRows; /**< Total number of rows that the
                                           textbox can display at once. */
    nxgl_coord_t       m_maxRows;     /**< Maximum number of rows that the
                                           textbox should buffer. */
    int32_t            m_topRow;      /**< Index of the top row of text
                                           currently displayed. */
    TextAlignmentHoriz m_hAlignment;  /**< Horizontal alignment of the text. */
    TextAlignmentVert  m_vAlignment;  /**< Vertical alignment of the text. */
    int                m_cursorPos;   /**< Position of the cursor within
                                           the string. */
    uint8_t            m_showCursor;  /**< Cursor visibility. */
    bool               m_wrapCursor;  /**< True wrap cursor at the ends of the text */

    /**
     * Get the coordinates of the cursor relative to the text.
     *
     * @param x Will be populated with the x coordinate of the cursor.
     * @param y Will be populated with the y coordinate of the cursor.
     */

    virtual void getCursorCoordinates(nxgl_coord_t& x, nxgl_coord_t& y) const;

    /**
     * Gets the index of the character at the specified x coordinate in the
     * specified row.
     *
     * @param x X coordinate of the character.
     * @param rowIndex Index of the row containing the character.
     * @return The index of the character at the specified coordinate.
     */

    virtual int getCharIndexAtCoordinate(nxgl_coord_t x, int rowIndex) const;

    /**
     * Get the index of the character at the specified coordinates.
     *
     * @param x X coordinate of the character.
     * @param y Y coordinate of the character.
     * @return The index of the character at the specified coordinates.
     */

    virtual unsigned int getCharIndexAtCoordinates(nxgl_coord_t x, nxgl_coord_t y) const;

    /**
     * Get the row containing the specified Y coordinate.
     *
     * @param y Y coordinate to locate.
     * @return The index of the row containing the specified Y coordinate.
     */

    int getRowContainingCoordinate(nxgl_coord_t y) const;

    /**
     * Draw the area of this widget that falls within the clipping region.
     * Called by the redraw() function to draw all visible regions.
     *
     * @param port The CGraphicsPort to draw to.
     * @see redraw()
     */

    virtual void drawContents(CGraphicsPort *port);

    /**
     * Draw the area of this widget that falls within the clipping region.
     * Called by the redraw() function to draw all visible regions.
     *
     * @param port The CGraphicsPort to draw to.
     * @see redraw()
     */

    virtual void drawBorder(CGraphicsPort *port);

    /**
     * Move cursor one character to the left.
     */

    virtual void moveCursorLeft(void);

    /**
     * Move cursor one character to the right.
     */

    virtual void moveCursorRight(void);

    /**
     * Move cursor one row upwards.
     */

    virtual void moveCursorUp(void);

    /**
     * Move cursor one row downwards.
     */

    virtual void moveCursorDown(void);

    /**
     * Ensures that the textbox only contains the maximum allowed
     * number of rows by culling any excess rows from the top of
     * the text.
     *
     * @return True if lines were removed from the text; false if not.
     */

    virtual bool cullTopLines(void);

    /**
     * Ensures that the canvas height is the height of the widget,
     * if the widget exceeds the size of the text, or the height of
     * the text if the text exceeds the size of the widget.
     */

    virtual void limitCanvasHeight(void);

    /**
     * Ensures that the canvas cannot scroll beyond its height.
     */

    virtual void limitCanvasY(void);

    /**
     * Jumps to the cursor coordinates of the text.
     */

    void jumpToCursor(void);

    /**
     * Jumps to the bottom of the text.
     */

    void jumpToTextBottom(void);

    /**
     * Resize the textbox to the new dimensions.
     *
     * @param width The new width.
     * @param height The new height.
     */

    virtual void onResize(nxgl_coord_t width, nxgl_coord_t height);
    
    /**
     * Starts the dragging system.
     *
     * @param x The x coordinate of the click.
     * @param y The y coordinate of the click.
     */

    virtual void onClick(nxgl_coord_t x, nxgl_coord_t y);

    /**
     * Opens the keyboard on the bottom display.
     *
     * @param x The x coordinates of the click.
     * @param y The y coordinates of the click.
     */

    virtual void onDoubleClick(nxgl_coord_t x, nxgl_coord_t y);

    /**
     * Handles physical button presses.  Moves the cursor
     * in the direction pressed.
     *
     * @param key The key that was pressed.
     */

    void processPhysicalKey(nxwidget_char_t key);

    /**
     * Handle a keyboard press event.
     *
     * @param e The event data.
     */

    void handleKeyPressEvent(const CWidgetEventArgs &e);

    /**
     * Handle a cursor control event.  Replaces CWidgetEventHandler method.
     *
     * @param e The event data.
     */

    void handleCursorControlEvent(const CWidgetEventArgs &e);

    /**
     * Gets the x position of a row of text based on the width of the row and the
     * type of horizontal alignment currently set.
     *
     * @param row The index of the row.
     * @return The x coordinate of the row.
     */

    nxgl_coord_t getRowX(int row) const;

    /**
     * Gets the y position of the specified row of text based on the type of
     * vertical alignment currently set.
     *
     * @param row The row number to find the y coordinate of.
     * @return The y coordinate of the specified row of text.
     */

    nxgl_coord_t getRowY(int row) const;

    /**
     * Return true if the cursor is visible
     */

    virtual bool isCursorVisible(void) const;

    /**
     * Gets the character under the cursor.
     *
     * @return The character under the cursor.
     */

    nxwidget_char_t getCursorChar(void) const;

    /**
     * Works out the number of visible rows within the textbox.
     */

    void calculateVisibleRows(void);

    /**
     * Draws text.
     *
     * @param port The CGraphicsPort to draw to.
     */

    void drawText(CGraphicsPort *port);

    /**
     * Draws the cursor.
     *
     * @param port The CGraphicsPort to draw to.
     */

    void drawCursor(CGraphicsPort *port);

    /**
     * Draws a single line of text.
     *
     * @param port The CGraphicsPort to draw to.
     * @param row The index of the row to draw.
     */

    void drawRow(CGraphicsPort *port, int row);

    /**
     * Destructor.
     */

    inline virtual ~CMultiLineTextBox(void)
    {
      delete m_text;
      m_text = (CText *)NULL;
    }

    /**
     * Copy constructor is protected to prevent usage.
     */

    inline CMultiLineTextBox(const CMultiLineTextBox &multiLineTextBox)
    : CScrollingPanel(multiLineTextBox) { }

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

    CMultiLineTextBox(CWidgetControl *pWidgetControl,
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

    virtual void setTextAlignmentHoriz(TextAlignmentHoriz alignment);

    /**
     * Set the vertical alignment of text within the textbox.
     *
     * @param alignment The vertical position of the text.
     */

    virtual void setTextAlignmentVert(TextAlignmentVert alignment);

    /**
     * Returns the number of "pages" that the text spans.  A page
     * is defined as the amount of text that can be displayed within
     * the textbox at one time.
     *
     * @return The page count.
     */

    virtual const int getPageCount(void) const;

    /**
     * Returns the current page.
     *
     * @return The current page.
     * @see getPageCount().
     */

    virtual const int getCurrentPage(void) const;

    /**
     * Returns a pointer to the CText object that contains the
     * wrapped text used in the textbox.  It is used as the
     * pre-processed data source for the textbox, and should
     * not be altered.
     *
     * @return Pointer to the CText object.
     */

    virtual inline const CText *getText(void) const
    {
      return m_text;
    }

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

    virtual const int getTextLength(void) const;

    /**
     * Sets the cursor display mode.
     *
     * @param cursorMode Determines cursor display mode
     */

    virtual void showCursor(EShowCursor cursorMode);

    /**
     * Shows the cursor in default mode (only when the TextBox has focus).
     */

    virtual inline void showCursor(void)
    {
      showCursor(SHOW_CURSOR_ONFOCUS);
    }

    /**
     * Hides the cursor.
     */

    virtual inline void hideCursor(void)
    {
      showCursor(SHOW_CURSOR_NEVER);
    }

    /**
     * Enables/disables cursor wrapping
     *
     * @param wrap True enables cursor wrapping
     */

    virtual inline void wrapCursor(bool wrap)
    {
      m_wrapCursor = wrap;
    }

    /**
     * Move the cursor to the text position specified.  0 indicates the start
     * of the string.  If position is greater than the length of the string,
     * the cursor is moved to the end of the string.
     *
     * @param position The new cursor position.
     */

    virtual void moveCursorToPosition(const int position);

    /**
     * Get the cursor position.  This is the index within the string that
     * the cursor is currently positioned over.
     *
     * @return position The cursor position.
     */

    virtual inline const int getCursorPosition(void) const
    {
      return m_cursorPos;
    }

    /**
     * Insert text at the specified index.
     *
     * @param text The text to insert.
     * @param index Index at which to insert the text.
     */

    virtual void insertText(const CNxString &text,
                            const unsigned int index);

    /**
     * Insert text at the current cursor position.
     *
     * @param text The text to insert.
     */

    virtual void insertTextAtCursor(const CNxString & ext);
  };
}

#endif // __cplusplus

#endif // __INCLUDE_CMULTILINETEXTBOX_HXX
