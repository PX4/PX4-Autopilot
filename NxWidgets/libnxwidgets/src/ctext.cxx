/****************************************************************************
 * NxWidgets/libnxwidgets/src/ctext.cxx
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

#include "ctext.hxx"
#include "cstringiterator.hxx"

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
 * @param font The font to use for this text object.
 * @param text A string that this text object should wrap around.
 * @param width The pixel width at which the text should wrap.
 */

CText::CText(CNxFont *font, const CNxString &text, nxgl_coord_t width)
: CNxString(text)
{
  m_font        = font;
  m_width       = width;
  m_lineSpacing = 1;
  wrap();
}

/**
 * Set the text in the string.
 *
 * @param text Char array to use as the new data for this string.
 */

void CText::setText(const CNxString &text)
{
  CNxString::setText(text);
  wrap();
}

/**
 * Set the text in the string.
 *
 * @param text Char array to use as the new data for this string.
 */

void CText::setText(FAR const char *text)
{
  CNxString::setText(text);
  wrap();
}

/**
 * Set the text in the string.
 *
 * @param text Character to to use as the new data for this string.
 */

void CText::setText(const nxwidget_char_t text)
{
  CNxString::setText(text);
  wrap();
}

/**
 * Append text to the end of the string.
 *
 * @param text String to append.
 */

void CText::append(const CNxString &text)
{
  CNxString::append(text);
  wrap(getLength() - 1);
}

/**
 * Insert text at the specified character index.
 *
 * @param text The text to insert.
 * @param index The char index to insert at.
 */

void CText::insert(const CNxString &text, const int index)
{
  CNxString::insert(text, index);
  wrap(index);
}

/**
 * Remove all characters from the string from the start index onwards.
 *
 * @param startIndex The char index to start removing from.
 */

void CText::remove(const int startIndex)
{
  CNxString::remove(startIndex);
  wrap(startIndex);
}

/**
 * Remove all characters from the string from the start index onwards.
 *
 * @param startIndex The char index to start removing from.
 * @param count The number of chars to remove.
 */

void CText::remove(const int startIndex, const int count)
{
  CNxString::remove(startIndex, count);
  wrap(startIndex);
}


/**
 * Set the vertical spacing between rows of text.
 *
 * @param lineSpacing The line spacing.
 */

void CText::setLineSpacing(uint8_t lineSpacing)
{
  m_lineSpacing = lineSpacing;
  wrap();
}

/**
 * Sets the pixel width of the text; text wider than 
 * this will automatically wrap.
 *
 * @param width Maximum pixel width of the text.
 */

void CText::setWidth(nxgl_coord_t width)
{
  m_width = width;
  wrap();
}

/**
 * Set the font to use.
 *
 * @param font Pointer to the new font.
 */

void CText::setFont(CNxFont* font)
{
  m_font = font;
  wrap();
}

/**
 * Get the number of characters in the specified line number.
 *
 * @param lineNumber The line number to check.
 * @return The number of characters in the line.
 */

const int CText::getLineLength(const int lineNumber) const
{
  if (lineNumber < getLineCount() - 1)
    {
      return m_linePositions[lineNumber + 1] - m_linePositions[lineNumber];
    }
  
  return getLength() - m_linePositions[lineNumber];
}

/**
 * Get the number of characters in the specified line number,
 * ignoring any trailing blank characters.
 *
 * @param lineNumber The line number to check.
 * @return The number of characters in the line.
 */

const int CText::getLineTrimmedLength(const int lineNumber) const
{
  nxgl_coord_t length = getLineLength(lineNumber);
  
  // Loop through string until the end

  CStringIterator *iterator = newStringIterator();
  
  // Get char at the end of the line

  if (iterator->moveTo(m_linePositions[lineNumber] + length - 1))
    {
      do
        {
          if (!m_font->isCharBlank(iterator->getChar()))
            {
              break;
            }
          length--;
        }
      while (iterator->moveToPrevious() && (length > 0));

      delete iterator;
      return length;
    }
  
  // May occur if data has been horribly corrupted somewhere

  delete iterator;
  return 0;
}

/**
 * Get the width in pixels of the specified line number.
 *
 * @param lineNumber The line number to check.
 * @return The pixel width of the line.
 */

const nxgl_coord_t CText::getLinePixelLength(const int lineNumber) const
{
  return m_font->getStringWidth(*this, getLineStartIndex(lineNumber),
                                getLineLength(lineNumber));
}

/**
 * Get the width in pixels of the specified line number,
 * ignoring any trailing blank characters.
 *
 * @param lineNumber The line number to check.
 * @return The pixel width of the line.
 */

const nxgl_coord_t CText::getLineTrimmedPixelLength(const int lineNumber) const
{
  return m_font->getStringWidth(*this, getLineStartIndex(lineNumber),
                                getLineTrimmedLength(lineNumber));
}

/**
 * Get a pointer to the CText object's font.
 *
 * @return Pointer to the font.
 */

CNxFont *CText::getFont(void) const
{
  return m_font;
}

/**
 * Removes lines of text from the start of the text buffer.
 *
 * @param lines Number of lines to remove
 */

void CText::stripTopLines(const int lines)
{
  // Get the start point of the text we want to keep

  nxgl_coord_t textStart = 0;
  
  for (int i = 0; i < lines; i++)
    {
      textStart += getLineLength(i);
    }

  // Remove the characters from the start of the string to the found location

  remove(0, textStart);
  
  // Rewrap the text

  wrap();
}

/**
 * Wrap all of the text.
 */

void CText::wrap(void)
{
  wrap(0);
}

/**
 * Wrap the text from the line containing the specified char index onwards.
 *
 * @param charIndex The index of the char to start wrapping from; note
 * that the wrapping function will re-wrap that entire line of text.
 */

void CText::wrap(int charIndex)
{
  // Declare vars in advance of loop

  int pos = 0;
  int lineWidth;
  int breakIndex;
  bool endReached = false;
  
  if (m_linePositions.size() == 0)
    {
      charIndex = 0;
    }
  
  // If we're wrapping from an offset in the text, ensure that any existing data
  // after the offset gets removed

  if (charIndex > 0)
    {
      // Remove wrapping data past this point

      // Get the index of the line in which the char index appears

      int lineIndex = getLineContainingCharIndex(charIndex);

      // Remove any longest line records that occur from the line index onwards

      while ((m_longestLines.size() > 0) &&
             (m_longestLines[m_longestLines.size() - 1].index >= lineIndex))
        {
          m_longestLines.pop_back();
        }
    
      // If there are any longest line records remaining, update the text pixel width
      // The last longest line record will always be the last valid longest line as
      // the vector is sorted by length

      if (m_longestLines.size() > 0)
        {
          m_textPixelWidth = m_longestLines[m_longestLines.size() - 1].width;
        }
      else
        {
          m_textPixelWidth = 0;
        }

      // Remove any wrapping data from after this line index onwards

      while ((m_linePositions.size() > 0) &&
             (m_linePositions.size() - 1 > (int)lineIndex))
       {
          m_linePositions.pop_back();
        }

      // Adjust start position of wrapping loop so that it starts with
      // the current line index

      if (m_linePositions.size() > 0)
        {
          pos = m_linePositions[m_linePositions.size() - 1];
        }
    }
  else
    {
      // Remove all wrapping data
    
      // Wipe the width variable

      m_textPixelWidth = 0;
    
      // Empty existing longest lines

      m_longestLines.clear();
    
      // Empty existing line positions

      m_linePositions.clear();
    
      // Push first line start into vector

      m_linePositions.push_back(0);
    }
  
  // Loop through string until the end

  CStringIterator *iterator = newStringIterator();
  
  while (!endReached)
    {
      breakIndex = 0;
      lineWidth = 0;
    
      if (iterator->moveTo(pos))
        {
          // Search for line breaks and valid breakpoints until we
          // exceed the width of the text field or we run out of
          // string to process

          while (lineWidth + m_font->getCharWidth(iterator->getChar()) <= m_width)
            {
              lineWidth += m_font->getCharWidth(iterator->getChar());

              // Check for line return

              if (iterator->getChar() == '\n')
                {
                  // Remember this breakpoint

                  breakIndex = iterator->getIndex();
                  break;
                }
              else if ((iterator->getChar() == ' ') ||
                       (iterator->getChar() == ',') ||
                       (iterator->getChar() == '.') ||
                       (iterator->getChar() == '-') ||
                       (iterator->getChar() == ':') ||
                       (iterator->getChar() == ';') ||
                       (iterator->getChar() == '?') ||
                       (iterator->getChar() == '!') ||
                       (iterator->getChar() == '+') ||
                       (iterator->getChar() == '=') ||
                       (iterator->getChar() == '/') ||
                       (iterator->getChar() == '\0'))
                {
                  // Remember the most recent breakpoint

                  breakIndex = iterator->getIndex();
                }

              // Move to the next character

              if (!iterator->moveToNext())
                {
                  // No more text; abort loop

                  endReached = true;
                  break;
                }
            }
        }
      else
        {
          endReached = true;
        }

      if ((!endReached) && (iterator->getIndex() > pos))
        {
          // Process any found data

          // If we didn't find a breakpoint split at the current position

          if (breakIndex == 0)
            {
              breakIndex = iterator->getIndex() - 1;
            }

          // Trim blank space from the start of the next line

          CStringIterator *breakIterator = newStringIterator();

          if (breakIterator->moveTo(breakIndex + 1))
            {
              while (breakIterator->getChar() == ' ')
                {
                  if (breakIterator->moveToNext())
                    {
                      breakIndex++;
                    }
                  else
                    {
                      break;
                    }
                }
            }
      
          delete breakIterator;
      
          // Add the start of the next line to the vector

          pos = breakIndex + 1;
          m_linePositions.push_back(pos);

          // Is this the longest line observed so far?

          if (lineWidth > m_textPixelWidth)
            {
              m_textPixelWidth = lineWidth;

              // Push the description of the line into the longest lines
              // vector (note that we store the index in m_linePositions that
              // refers to the start of the line, *not* the position of the
              // line in the char array)

              LongestLine line;
              line.index = m_linePositions.size() - 2;
              line.width = lineWidth;
              m_longestLines.push_back(line);
            }
        }
      else if (!endReached)
        {
          // Add a blank row if we're not at the end of the string

          pos++;
          m_linePositions.push_back(pos);
        }
    }
  
  // Add marker indicating end of text
  // If we reached the end of the text, append the stopping point

  if (m_linePositions[m_linePositions.size() - 1] != getLength() + 1)
    {
      m_linePositions.push_back(getLength());
    }

  delete iterator;
  
  // Calculate the total height of the text

  m_textPixelHeight = getLineCount() * (m_font->getHeight() + m_lineSpacing);
  
  // Ensure height is always at least one row

  if (m_textPixelHeight == 0)
    {
      m_textPixelHeight = m_font->getHeight() + m_lineSpacing;
    }
}

/**
 * Get the index of the line of text that contains the specified index
 * within the raw char array.
 *
 * @param index The index to locate within the wrapped lines of text.
 * @return The number of the line of wrapped text that contains the
 * specified index.
 */

const int CText::getLineContainingCharIndex(const int index) const
{
  // Early exit if there is no existing line data

  if (m_linePositions.size() == 0)
    {
      return 0;
    }

  // Early exit if the character is in the last row

  if (index >= m_linePositions[m_linePositions.size() - 2])
    {
      return m_linePositions.size() - 2;
    }

  // Binary search the line vector for the line containing the supplied index

  int bottom = 0;
  int top = m_linePositions.size() - 1;
  int mid;
  
  while (bottom <= top)
    {
      // Standard binary search

      mid = (bottom + top) >> 1;
    
      if (index < m_linePositions[mid])
        {
          // Index is somewhere in the lower search space

          top = mid - 1;
        }
      else if (index > m_linePositions[mid])
        {
          // Index is somewhere in the upper search space

          bottom = mid + 1;
        }
      else if (index == m_linePositions[mid])
        {
          // Located the index

          return mid;
        }
    
      // Check to see if we've moved past the line that contains the index
      // We have to do this because the index we're looking for can be within
      // a line; it isn't necessarily the start of a line (which is what is
      // stored in the m_linePositions vector)

      if (index > m_linePositions[top])
        {
          // Search index falls within the line represented by the top position

          return top;
        }
      else if (index < m_linePositions[bottom])
        {
          // Search index falls within the line represented by the bottom position

          return bottom - 1;
        }
    }

  // Line cannot be found

  return 0;
}
