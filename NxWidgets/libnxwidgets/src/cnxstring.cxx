/****************************************************************************
 * NxWidgets/libnxwidgets/src/cnxstring.cxx
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
#include <string.h>

#include "cnxstring.hxx"
#include "cstringiterator.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * CNxString Method Implementations
 ****************************************************************************/

using namespace NXWidgets;

/**
 * Constructor to create an empty string object.
 */

CNxString::CNxString()
{
  m_text          = (FAR nxwidget_char_t *)NULL;
  m_stringLength  = 0;
  m_allocatedSize = 0;
  m_growAmount    = 16;
}

/**
 * Constructor to create a string from a C character array.
 *
 * @param text Pointer to a char array to use as the basis of the
 * string.
 */

CNxString::CNxString(FAR const char *text)
{
  m_text          = (FAR nxwidget_char_t *)NULL;
  m_stringLength  = 0;
  m_allocatedSize = 0;
  m_growAmount    = 16;

  setText(text);
}

/**
 * Constructor to create a string from a single character.
 * @param letter Single character to use as the basis of the string.
 */

CNxString::CNxString(const nxwidget_char_t text)
{
  m_text          = (FAR nxwidget_char_t *)NULL;
  m_stringLength  = 0;
  m_allocatedSize = 0;
  m_growAmount    = 16;

  setText(text);
}

CNxString::CNxString(const CNxString &string)
{
  m_text          = (FAR nxwidget_char_t *)NULL;
  m_stringLength  = 0;
  m_allocatedSize = 0;
  m_growAmount    = 16;

  setText(string);
}

/**
 * Creates and returns a new CCStringIterator object that will iterate
 * over this string.  The object must be manually deleted once it is
 * no longer needed.
 *
 * @return A new CCStringIterator object.
 */

CStringIterator *CNxString::newStringIterator() const
{
  return new CStringIterator(this);
}

/**
 * Copy the internal array to the supplied buffer.  The buffer must be
 * large enough to contain the full text in the string.  The
 * getByteCount() method can be used to obtain the length of the string.
 * Unlike the CNxString class, the char array is null-terminated.
 * The buffer must be (getByteCount() + 2) bytes long, in order to
 * accommodate the terminator.
 *
 * @param buffer Buffer to copy the internal char array to.
 */

void CNxString::copyToCharArray(FAR nxwidget_char_t *buffer) const
{
  unsigned int dataLength = sizeof(nxwidget_char_t) * m_stringLength;
  memcpy(buffer, m_text, dataLength);
  buffer[dataLength] = '\0';
}

/**
 * Set the text in the string.
 *
 * @param text CNxString containing the new data for this string.
 */

void CNxString::setText(const CNxString &text)
{
  // Ensure we've got enough memory available

  allocateMemory(text.getLength(), false);

  // Copy the text into the internal array

  memcpy(m_text, text.getCharArray(), sizeof(nxwidget_char_t) * text.getLength());

  // Save size (in characters) of the string

  m_stringLength = text.getLength();
}

/**
 * Set the 8-bit C-string text in the string.
 *
 * @param text Char array to use as the new data for this string.
 */

void CNxString::setText(FAR const char *text)
{
  int length = strlen(text);

  // Ensure we've got enough memory available

  allocateMemory(length, false);

  // Copy characters into m_text, converting from 8- to 16-bit characters
  // (if necessary) and cache the length

  for (int i = 0; i < length; i++)
    {
      m_text[i] = (nxwidget_char_t)text[i];
    }

  m_stringLength = length;
}

/**
 * Set the text in the string.
 *
 * @param text Char array to use as the new data for this string.
 */

void CNxString::setText(FAR const nxwidget_char_t *text, int nchars)
{
  // Ensure we've got enough memory available

  allocateMemory(nchars, false);

  // Copy characters into m_text and cache the length

  m_stringLength = nchars;
  memcpy(m_text, text, sizeof(nxwidget_char_t) * nchars);
}

/**
 * Set the text in the string.
 *
 * @param text Character to to use as the new data for this string.
 */

void CNxString::setText(const nxwidget_char_t letter)
{
  // Ensure we've got enough memory available

  allocateMemory(1, false);

  // Copy the valid character into m_text and cache the length

  *m_text         = letter;
   m_stringLength = 1;
}

/**
 * Append text to the end of the string.
 *
 * @param text String to append.
 */

void CNxString::append(const CNxString &text)
{
  // Ensure we've got enough memory available

  allocateMemory(m_stringLength + text.getLength(), true);

  // Append the new string to the end of the array

  FAR nxwidget_char_t       *dest = &m_text[m_stringLength];
  FAR const nxwidget_char_t *src  = text.getCharArray();
  for (int i = 0; i <  text.getLength(); i++)
    {
      *dest++ = *src++;
    }

  // Update the size in characters and the size in bytes

  m_stringLength += text.getLength();
}

/**
 * Insert text at the specified character index.
 *
 * @param text The text to insert.
 * @param index The index at which to insert the text.
 */

void CNxString::insert(const CNxString &text, int index)
{
  // Early exit if the string is empty

  if (!hasData())
    {
      CNxString::setText(text);
      return;
    }

  // Early exit if we're just appending

  if (index >= m_stringLength)
    {
      CNxString::append(text);
      return;
    }

  // Get the total size of the string that we need

  int newLength = m_stringLength + text.getLength();
  int newSize   = newLength * sizeof(nxwidget_char_t);

  // Reallocate memory if the existing memory isn't large enough

  if (m_allocatedSize < newSize)
    {
      int allocLength = newLength + m_growAmount;

      // Allocate new string large enough to contain additional data

      FAR nxwidget_char_t *newText = new nxwidget_char_t[allocLength];

      // Copy the start of the existing text to the newly allocated string

      FAR nxwidget_char_t       *dest = newText;
      FAR const nxwidget_char_t *src  = m_text;
      for (int i = 0; i < index; i++)
        {
          *dest++ = *src++;
        }

      // Insert the additional text into the new string

      src = text.getCharArray();
      for (int i = 0; i < text.getLength(); i++)
        {
          *dest++ = *src++;
        }

      // Copy the end of the existing text the the newly allocated string

      src = &m_text[index];
      for (int i = index; i < m_stringLength; i++)
        {
          *dest++ = *src++;
        }

      m_allocatedSize = allocLength * sizeof(nxwidget_char_t);

      // Delete existing string

      delete[] m_text;

      // Swap pointers 

      m_stringLength = newLength;
      m_text         = newText;
    }
  else
    {
      // Existing size is large enough, so make space in string for insert

      FAR nxwidget_char_t       *dest = &m_text[newLength - 1];
      FAR const nxwidget_char_t *src  = &m_text[m_stringLength - 1];
      for (int i = 0; i < m_stringLength - index; i++)
        {
          *dest-- = *src--;
        }

      // Insert the additional text into the new string

      dest = &m_text[index];
      src  = text.getCharArray();      
      for (int i = 0; i < text.getLength(); i++)
        {
          *dest++ = *src++;
        }

      m_stringLength = newLength;
    }
}

/**
 * Remove all characters from the string from the start index onwards.
 *
 * @param startIndex Index to remove from.
 */

void CNxString::remove(const int startIndex)
{
  // Reject if requested operation makes no sense

  if (!hasData() || startIndex >= m_stringLength)
    {
      return;
    }

  // Removing characters from the end of the string is trivial - simply
  // decrease the length

  m_stringLength  = startIndex;
}

/**
 * Remove specified number of characters from the string from the
 * start index onwards.
 *
 * @param startIndex Index to remove from.
 * @param count Number of characters to remove.
 */

void CNxString::remove(const int startIndex, const int count)
{
  // Reject if requested operation makes no sense

  if (!hasData() || startIndex >= m_stringLength)
    {
      return;
    }

  // Don't remove in this way if the count includes the end of the string

  int endIndex = startIndex + count;
  if (endIndex > m_stringLength)
    {
      remove(startIndex);
      return;
    }

  // Copy characters from a point after the area to be deleted into the space created
  // by the deletion

  FAR nxwidget_char_t       *dest = &m_text[startIndex];
  FAR const nxwidget_char_t *src  = &m_text[endIndex];
  for (int i = m_stringLength - endIndex; i > 0; i--)
    {
      *dest++ = *src++;
    }

  // Decrease length

  m_stringLength -= count;

}

/**
 * Get the character at the specified index.  This function is useful
 * for finding the occasional character at an index, but for iterating
 * over strings it is exceptionally slow.  The newStringIterator()
 * method should be used to retrieve an iterator object that can iterate
 * over the string efficiently.
 *
 * @param index The index of the character to retrieve.
 * @return The character at the specified index.
 */

const nxwidget_char_t CNxString::getCharAt(int index) const
{
  return *getCharPointer(index);
}

/**
 * Returns the first index of the specified letter within the string.
 * Will return -1 if the letter is not found.
 *
 * @param letter Letter to find.
 * @return The index of the letter.
 */

const int CNxString::indexOf(nxwidget_char_t letter) const
{
  return indexOf(letter, 0, getLength());
}

/**
 * Returns the first index of the specified letter within the string.
 * Will return -1 if the letter is not found.  Scans through the string
 * from "startIndex" until it has examined all subsequent letters.
 *
 * @param letter Letter to find.
 * @param startIndex The index to start searching from.
 * @return The index of the letter.
 */

const int CNxString::indexOf(nxwidget_char_t letter, int startIndex) const
{
  return indexOf(letter, startIndex, getLength() - startIndex);
}

/**
 * Returns the first index of the specified letter within the string.
 * Will return -1 if the letter is not found.  Scans through the string
 * from "startIndex" until it has examined all letters within the
 * range "count".
 *
 * @param letter Letter to find.
 * @param startIndex The index to start searching from.
 * @param count The number of characters to examine.
 * @return The index of the letter.
 */

const int CNxString::indexOf(nxwidget_char_t letter, int startIndex, int count) const
{
  // Exit if no data available

  if (!hasData())
    {
      return -1;
    }

  int index = -1;
  int charsExamined = 0;

  CStringIterator *iterator = new CStringIterator(this);
  if (!iterator->moveTo(startIndex))
    {
      delete iterator;
      return -1;
    }

  do
    {
      if (iterator->getChar() == letter)
        {
          index = iterator->getIndex();
          break;
        }

      charsExamined++;
    }
  while (iterator->moveToNext() && (charsExamined < count));

  delete iterator;
  return index;
}

/**
 * Returns the last index of the specified letter within the string.
 * Will return -1 if the letter is not found.
 *
 * @param letter Letter to find.
 * @return The index of the letter.
 */

const int CNxString::lastIndexOf(nxwidget_char_t letter) const
{
  return lastIndexOf(letter, getLength() - 1, getLength());
}

/**
 * Returns the last index of the specified letter within the string.
 * Will return -1 if the letter is not found.  Scans through the string
 * backwards from "startIndex" until it has examined all preceding
 * letters within the string.
 *
 * @param letter Letter to find.
 * @param startIndex The index to start searching from.
 * @return The index of the letter.
 */

const int CNxString::lastIndexOf(nxwidget_char_t letter, int startIndex) const
{
  return lastIndexOf(letter, startIndex, getLength() - (getLength() - startIndex));
}

/**
 * Returns the last index of the specified letter within the string.
 * Will return -1 if the letter is not found.  Scans through the string
 * backwards from "startIndex" until it has examined all letters within
 * the range "count".
 * @param letter Letter to find.
 * @param startIndex The index to start searching from.
 * @param count The number of characters to examine.
 * @return The index of the letter.
 */

const int CNxString::lastIndexOf(nxwidget_char_t letter, int startIndex, int count) const
{
  // Exit if no data available

  if (!hasData())
    {
      return -1;
    }

  int index = -1;
  int charsExamined = 0;

  CStringIterator *iterator = new CStringIterator(this);
  if (!iterator->moveTo(startIndex))
    {
      delete iterator;
      return -1;
    }

  do
    {
      if (iterator->getChar() == letter)
        {
          index = iterator->getIndex();
          break;
        }

      charsExamined++;
    }
  while (iterator->moveToPrevious() && (charsExamined <= count));

  delete iterator;
  return index;
}

/**
 * Get a substring from this string.  It is the responsibility of the
 * caller to delete the substring when it is no longer required.
 *
 * @param startIndex The starting point of the substring.
 * @return A pointer to a new CNxString object containing the
 * substring.
 */

CNxString* CNxString::subString(int startIndex) const
{
  return subString(startIndex, getLength() - startIndex);
}

/**
 * Get a substring from this string.  It is the responsibility of the
 * caller to delete the substring when it is no longer required.
 *
 * @param startIndex The starting point of the substring.
 * @param length The length of the substring.
 * @return A pointer to a new CNxString object containing the
 * substring.
*/

CNxString *CNxString::subString(int startIndex, int length) const
{
  CNxString *newString = new CNxString();
  CStringIterator *iterator = new CStringIterator(this);

  if (!iterator->moveTo(startIndex))
    {
      delete iterator;
      return (CNxString *)0;
    }

  // Build up the string character by character.  We could do a memcpy
  // here and improve performance.

  int count = 0;
  while (count < length)
    {
      newString->append(iterator->getChar());
      iterator->moveToNext();
      count++;
    }

  delete iterator;
  return newString;
}

/**
 * Overloaded assignment operator.  Copies the data within the argument
 * string to this string.
 *
 * @param string The string to copy.
 * @return This string.
 */

CNxString& CNxString::operator=(const CNxString &string)
{
  if (&string != this)
    {
      setText(string);
    }
  return *this;
}

/**
 * Overloaded assignment operator.  Copies the data within the argument
 * char array to this string.
 *
 * @param string The string to copy.
 * @return This string.
 */

CNxString& CNxString::operator=(const char *string)
{
  setText(string);
  return *this;
}

/**
 * Overloaded assignment operator.  Copies the data from the argument
 * char to this string.
 *
 * @param letter The char to copy.
 * @return This string.
 */

CNxString& CNxString::operator=(const nxwidget_char_t letter)
{
  setText(letter);
  return *this;
}

/**
 * Compares this string to the argument.
 *
 * @param string String to compare to.
 * @return Zero if both strings are equal.  A value greater than zero
 * indicates that this string is greater than the argument string.  A
 * value less than zero indicates the opposite.  Note that the return
 * value indicates the *byte* that does not match, not the *character*.
 */

int CNxString::compareTo(const CNxString &string) const
{
  return memcmp((FAR const char*)m_text,
                (FAR const char*)string.getCharArray(),
                getLength());
}

/**
 * Allocate memory for the string.
 *
 * @param chars Number nxwidget_char_t size characters to allocate.
 * @param preserve If true, the data in the existing memory will be
 * preserved if new memory must be allocated
 */

void CNxString::allocateMemory(int nChars, bool preserve)
{
  // This is the size of the allocation that we need

  int nBytesNeeded = nChars * sizeof(nxwidget_char_t);

  // Do we already have enough memory allocated to contain this new size?
  // If so, we can avoid deallocating and allocating new memory by re-using the old
  
  if (nBytesNeeded > m_allocatedSize)
    {
      // Not enough space in existing memory; allocate new memory

      int allocChars = nChars + m_growAmount;
      nxwidget_char_t *newText = new nxwidget_char_t[allocChars];

      // Free old memory if necessary

      if (m_text != NULL)
        {
          // Preserve existing data if required

          if (preserve)
            {
              memcpy(newText, m_text, sizeof(nxwidget_char_t) * m_stringLength);
            }

          delete[] m_text;
        }

      // Set pointer to new memory

      m_text = newText;

      // Remember how much memory we've allocated.

      m_allocatedSize = allocChars * sizeof(nxwidget_char_t);
    }
}

/**
 * Return a pointer to the specified characters.
 *
 * @param index Index of the character to retrieve.
 */

FAR nxwidget_char_t *CNxString::getCharPointer(const int index) const
{
  // Early exit if the string is empty

  if (!hasData())
    {
      return (FAR nxwidget_char_t*)NULL;
    }

  // Early exit if the index is greater than the length of the string

  if (index >= m_stringLength)
    {
      return (FAR nxwidget_char_t*)NULL;
    }

  return &m_text[index];
}
