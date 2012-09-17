/****************************************************************************
 * NxWidgets/libnxwidgets/include/ctextbox.hxx
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

#ifndef __INCLUDE_CTEXTBOX_HXX
#define __INCLUDE_CTEXTBOX_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/nx/nxglib.h>

#include "clabel.hxx"
#include "cnxstring.hxx"
#include "cwidgetstyle.hxx"
#include "cwidgeteventargs.hxx"
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
 /**
  * Forward references.
  */

  class CWidgetControl;
  class CNxTimer;
  class CNxString;

  /**
   * Single-line textbox widget.  Can align text both vertically and
   * horizontally in different ways.  The widget gains this functionality by
   * inheriting from the CLabel class.  However, if the amount of text exceeds
   * the dimensions of the widget, the widget will ignore its horizontal
   * alignment settings and switch to left-aligned instead.  This ensures that
   * moving the cursor over the text will scroll through it correctly.
   */

  class CTextBox : public ITextBox, public CLabel, public CWidgetEventHandler
  {
  protected:
    int      m_cursorPos;  /**< Position of the cursor within the string. */
    uint8_t  m_showCursor; /**< Controls cursor visibility. */
    bool     m_wrapCursor; /**< True wrap cursor at the ends of the text */

    /**
     * Redraws the widget
     */

    inline void onBlur(void);

    /**
     * Draw the area of this widget that falls within the clipping region.
     * Called by the redraw() function to draw all visible regions.
     *
     * @param port The CGraphicsPort to draw to.
     * @see redraw()
     */

    virtual void drawContents(CGraphicsPort *port);

    /**
     * Moves the cursor without redrawing.
     *
     * @param position New cursor position.
     * @return True if the cursor position changed
     */

    virtual bool repositionCursor(const int position);

    /**
     * Move the cursor to the specified coordinates.  The coordinates
     * are expected to be the result of a click, and therefore in
     * world-space rather than widget-space.
     */

    void moveCursorToClickLocation(nxgl_coord_t x, nxgl_coord_t y);

    /**
     * Draw the area of this widget that falls within the clipping region.
     * Called by the redraw() function to draw all visible regions.
     *
     * @param port The CGraphicsPort to draw to.
     * @see redraw()
     */

    virtual void drawBorder(CGraphicsPort *port);

    /**
     * Moves the cursor to the clicked coordinates.
     *
     * @param x The x coordinates of the click.
     * @param y The y coordinates of the click.
     */

    virtual void onClick(nxgl_coord_t x, nxgl_coord_t y);

    /**
     * Does nothing.
     *
     * @param x The x coordinates of the click.
     * @param y The y coordinates of the click.
     */

    virtual void onDoubleClick(nxgl_coord_t x, nxgl_coord_t y);

    /**
     * Return true if the cursor is visible
     */

    virtual bool isCursorVisible(void) const;

    /**
     * Get the x coordinate of the cursor in pixels relative
     * to the left-hand edge of the client rect.
     *
     * @return The x coordinate of the cursor in pixels.
     */

    virtual const nxgl_coord_t getCursorXPos(void) const;

    /**
     * Get the width of the cursor in pixels.
     *
     * @return The width of the cursor in pixels.
     */

    virtual nxgl_coord_t getCursorWidth(void) const;
    
    /**
     * Calculate the horizontal position of the string based on its length
     * and the alignment options.  Alignment options are overridden if the
     * width of the string exceeds the width of the textbox.
     */

    virtual void calculateTextPositionHorizontal(void);

    /**
     * Copy constructor is protected to prevent usage.
     */

    inline CTextBox(const CTextBox& textbox) : CLabel(textbox) { };

  public:

    /**
     * Constructor for a textbox containing a string.
     *
     * @param pWidgetControl The controlling widget for the window
     * @param x The x coordinate of the text box, relative to its parent.
     * @param y The y coordinate of the text box, relative to its parent.
     * @param width The width of the textbox.
     * @param height The height of the textbox.
     * @param text Pointer to a string to display in the textbox.
     * @param style The style that the button should use.  If this is not
     *        specified, the button will use the global default widget
     *        style.
     */

    CTextBox(CWidgetControl *pWidgetControl, nxgl_coord_t x, nxgl_coord_t y,
             nxgl_coord_t width, nxgl_coord_t height, const CNxString &text,
             CWidgetStyle *style = (CWidgetStyle *)NULL);

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
     * Set the text displayed in the label.
     *
     * @param text String to display.
     */

    virtual void setText(const CNxString &text);

    /**
     * Append new text to the end of the current text displayed in the
     * label.
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
     * Move the cursor to the text position specified.  0 indicates the
     * start of the string.  If position is greater than the length of the
     * string, the cursor is moved to the end of the string.
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
     * Handle a keyboard press event.  Replaces CWidgetEventHandler method.
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
  };
}

#endif // __cplusplus

#endif // __INCLUDE_CTEXTBOX_HXX
