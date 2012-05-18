/****************************************************************************
 * NxWidgets/libnxwidgets/include/itextbox.hxx
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

#ifndef __INCLUDE_ITEXTBOX_HXX
#define __INCLUDE_ITEXTBOX_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/nx/nxglib.h>

#include "cnxstring.hxx"
#include "cwidgetstyle.hxx"
#include "cwidgeteventargs.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
 
/****************************************************************************
 * Abstract Base Classes
 ****************************************************************************/
 
#if defined(__cplusplus)

namespace NXWidgets
{
  /**
   *  Cursor display options.
   */

  typedef enum
  {
    SHOW_CURSOR_ONFOCUS = 0, /**< Show the cursor only if the widget has focus */
    SHOW_CURSOR_NEVER,       /**< The cursor is never displayed */
    SHOW_CURSOR_ALWAYS       /**< Always show the cursor */
  } EShowCursor;

  /**
   * Defines the interface that textbox-like classes should implement.
   */

  class ITextBox
  {
  public:
    /**
     * A virtual destructor is required in order to override the ITextBox
     * destructor.  We do this because if we delete ITextBox, we want the
     * destructor of the class that inherits from ITextBox to run, not this
     * one.
     */

    virtual ~ITextBox(void) { }

    /**
     * Sets the cursor display mode.
     *
     *@param cursorMode Determines cursor display mode
     */

    virtual void showCursor(EShowCursor cursorMode) = 0;

    /**
     * Enables/disables cursor wrapping
     *
     * @param wrap True enables cursor wrapping
     */

    virtual void wrapCursor(bool wrap) = 0;

    /**
     * Set the text displayed in the label.
     *
     * @param text String to display.
     */

    virtual void setText(const CNxString &text) = 0;
    
    /**
     * Append new text to the end of the current text displayed in the
     * label.
     *
     * @param text String to append.
     */

    virtual void appendText(const CNxString &text) = 0;

    /**
     * Remove all characters from the string from the start index onwards.
     *
     * @param startIndex Index to remove from.
     */

    virtual void removeText(const unsigned int startIndex) = 0;

    /**
     * Remove specified number of characters from the string from the
     * start index onwards.
     *
     * @param startIndex Index to remove from.
     * @param count Number of characters to remove.
     */

    virtual void removeText(const unsigned int startIndex, const unsigned int count) = 0;

    /**
     * Insert text at the specified index.
     *
     * @param text The text to insert.
     * @param index Index at which to insert the text.
     */

    virtual void insertText(const CNxString &text, const unsigned int index) = 0;

    /**
     * Insert text at the current cursor position.
     *
     * @param text The text to insert.
     */

    virtual void insertTextAtCursor(const CNxString &text) = 0;

    /**
     * Move the cursor to the text position specified.  0 indicates the
     * start of the string.  If position is greater than the length of the
     * string, the cursor is moved to the end of the string.
     *
     * @param position The new cursor position.
     */

    virtual void moveCursorToPosition(const int position) = 0;

    /**
     * Get the cursor position.  This is the index within the string that
     * the cursor is currently positioned over.
     * @return position The cursor position.
     */

    virtual const int getCursorPosition(void) const = 0;
  };
}

#endif // __cplusplus

#endif // __INCLUDE_ITEXTBOX_HXX
