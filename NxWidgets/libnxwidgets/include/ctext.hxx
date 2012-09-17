/****************************************************************************
 * NxWidgets/libnxwidgets/include/ctext.hxx
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

#ifndef __INCLUDE_CTEXT_HXX
#define __INCLUDE_CTEXT_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nx.h>

#include "nxconfig.hxx"
#include "cnxfont.hxx"
#include "tnxarray.hxx"
#include "cnxstring.hxx"

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
   * This class functions as a wrapper around a char array offering
   * more advanced functionality - it can wrap text, calculate its
   * height in pixels, calculate the width of a row, etc.
   */

  class CText : public CNxString 
  {
  private:

    /**
     * Struct defining the position and length of a longest line within
     * the m_linePositions array.
     */

    typedef struct
    {
      int     index;
      uint8_t width;
    } LongestLine;
    
    CNxFont              *m_font;            /**< Font to be used for output */
    TNxArray<int>         m_linePositions;   /**< Array containing start indexes
                                                  of each wrapped line */
    TNxArray<LongestLine> m_longestLines;    /**< Array containing data describing
                                                  successively longer wrapped
                                                  lines */
    uint8_t               m_lineSpacing;     /**< Spacing between lines of text */
    int32_t               m_textPixelHeight; /**< Total height of the wrapped
                                                  text in pixels */
    uint8_t               m_textPixelWidth;  /**< Total width of the wrapped text
                                                  in pixels */
    nxgl_coord_t          m_width;           /**< Width in pixels available t
                                                  the text */

  public:

    /**
     * Constructor.
     *
     * @param font The font to use for this text object.
     * @param text A string that this text object should wrap around.
     * @param width The pixel width at which the text should wrap.
     */

    CText(CNxFont *font, const CNxString &text, nxgl_coord_t width);

    /**
     * Set the text in the string.
     *
     * @param text Char array to use as the new data for this string.
     */

    virtual void setText(const CNxString &text);

    /**
     * Set the text in the string.
     *
     * @param text Char array to use as the new data for this string.
     */

    virtual void setText(FAR const char *text);

    /**
     * Set the text in the string.
     *
     * @param text Character to to use as the new data for this string.
     */

    virtual void setText(const nxwidget_char_t text);

    /**
     * Append text to the end of the string.
     *
     * @param text String to append.
     */

    virtual void append(const CNxString &text);

    /**
     * Insert text at the specified character index.
     *
     * @param text The text to insert.
     * @param index The char index to insert at.
     */

    virtual void insert(const CNxString &text, const int index);

    /**
     * Remove all characters from the string from the start index onwards.
     *
     * @param startIndex The char index to start removing from.
     */

    virtual void remove(const int startIndex);

    /**
     * Remove all characters from the string from the start index onwards.
     *
     * @param startIndex The char index to start removing from.
     * @param count The number of chars to remove.
     */

    virtual void remove(const int startIndex, const int count);
    
    /**
     * Set the vertical spacing between rows of text.
     *
     * @param lineSpacing The line spacing.
     */

    void setLineSpacing(uint8_t lineSpacing);

    /**
     * Sets the pixel width of the text; text wider than 
     * this will automatically wrap.
     *
     * @param width Maximum pixel width of the text.
     */

    void setWidth(nxgl_coord_t width);

    /**
     * Set the font to use.
     *
     * @param font Pointer to the new font.
     */

    void setFont(CNxFont *font);

    /**
     * Get the number of characters in the specified line number.
     *
     * @param lineNumber The line number to check.
     * @return The number of characters in the line.
     */

    const int getLineLength(const int lineNumber) const;

    /**
     * Get the number of characters in the specified line number,
     * ignoring any trailing blank characters.
     *
     * @param lineNumber The line number to check.
     * @return The number of characters in the line.
     */

    const int getLineTrimmedLength(const int lineNumber) const;

    /**
     * Get the width in pixels of the specified line number.
     *
     * @param lineNumber The line number to check.
     * @return The pixel width of the line.
     */

    const nxgl_coord_t getLinePixelLength(const int lineNumber) const;

    /**
     * Get the width in pixels of the specified line number,
     * ignoring any trailing blank characters.
     *
     * @param lineNumber The line number to check.
     * @return The pixel width of the line.
     */

    const nxgl_coord_t getLineTrimmedPixelLength(const int lineNumber) const;

    /**
     * Get the total height of the text in pixels.
     *
     * @return The total height of the text.
     */

    inline const int32_t getPixelHeight(void) const
    {
      return m_textPixelHeight;
    }

    /**
     * Get the width of the longest line in pixels.
     *
     * @return The width of the longest line.
     */

    inline const uint8_t getPixelWidth(void) const
    {
      return m_textPixelWidth;
    }

    /**
     * Get the pixel spacing between each line of text.
     *
     * @return The line spacing.
     */

    inline const uint8_t getLineSpacing(void) const
    {
      return m_lineSpacing;
    }

    /**
     * Get the height in pixels of a line, given as the
     * height of the font plus the line spacing.
     *
     * @return The height of a line.
     */

    inline const uint8_t getLineHeight(void) const
    {
      return m_font->getHeight() + m_lineSpacing;
    }

    /**
     * Get the total number of lines in the text.
     *
     * @return The line count.
     */

    inline const int getLineCount(void) const
    {
      return m_linePositions.size() - 1;
    }

    /**
     * Get a pointer to the CText object's font.
     *
     * @return Pointer to the font.
     */

    CNxFont *getFont(void) const;

    /**
     * Removes lines of text from the start of the text buffer.
     *
     * @param lines Number of lines to remove
     */

    void stripTopLines(const int lines);
    
    /**
     * Wrap all of the text.
     */

    void wrap(void);

    /**
     * Wrap the text from the line containing the specified char index onwards.
     *
     * @param charIndex The index of the char to start wrapping from; note
     * that the wrapping function will re-wrap that entire line of text.
     */

    void wrap(int charIndex);

    /**
     * Get the index of the line of text that contains the specified index
     * within the raw char array.
     *
     * @param index The index to locate within the wrapped lines of text.
     * @return The number of the line of wrapped text that contains the
     * specified index.
     */

    const int getLineContainingCharIndex(const int index) const;

    /**
     * Gets the index within the char array that represents the start of the line of
     * text indicated by the line parameter.
     *
     * @param line The line number to locate within the char array.
     * @return The index within the char array of the start of the supplied line.
     */

    const int getLineStartIndex(const int line) const
    {
      return m_linePositions[line];
    }
  };
}

#endif // __cplusplus

#endif // __INCLUDE_CTEXT_HXX

