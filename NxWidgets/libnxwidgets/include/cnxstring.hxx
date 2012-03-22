/****************************************************************************
 * include/cnxtring.hxx
 * NxWidgets/libnxwidgets/
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

#ifndef __INCLUDE_CNXSTRING_HXX
#define __INCLUDE_CNXSTRING_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include "nxconfig.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Implementation Classes
 ****************************************************************************/

#if defined(__cplusplus)

namespace NXWidgets
{
  class CStringIterator;

  /**
   * Unicode string class.  Uses 16-bt wide-character encoding.  For optimal
   * performance, use the CStringIterator class to iterate over a CNxString
   * instance.
   *
   * Where possible, the string avoids allocating memory each time the
   * string grows or shrinks.  This means that the string may consume more
   * memory than the number of chars would seem to dictate if the object
   * previously contained a large string that has subsequently been truncated.
   * It also means that increasing the length of such a string is a cheaper
   * operation as memory does not need to allocated and copied.
   *
   * Additionally, the string increases its array size by m_growAmount every
   * time it needs to allocate extra memory, potentially reducing the number
   * of reallocs needed.
   *
   * The string is not null-terminated.  Instead, it uses a m_stringLength
   * member that stores the number of characters in the string.  This saves a
   * byte and makes calls to getLength() run in O(1) time instead of O(n).
   */

  class CNxString
  {
  private:
    friend class CStringIterator;
    
    int m_stringLength;  /**< Number of characters in the string */
    int m_allocatedSize; /**< Number of bytes allocated for this string */
    int m_growAmount;    /**< Number of chars that the string grows by
                              whenever it needs to get larger */


  protected:
    FAR nxwidget_char_t *m_text;  /**< Raw char array data */

    /**
     * Allocate memory for the string.
     *
     * @param chars Number of chars to allocate.
     * @param preserve If true, the data in the existing memory will be
     * preserved if new memory must be allocated
     */

    void allocateMemory(int chars, bool preserve);

    /**
     * Check if we've got any string data stored or not.
     *
     * @return True if the string contains any data; false if no data has
     * yet been supplied.
     */

    inline bool hasData(void) const
    {
      return m_stringLength > 0;
    }

    /**
     * Get the amount of allocated memory.
     *
     * @return The number of chars allocated in RAM.
     */

    inline int getAllocatedSize(void) const
    {
      return m_allocatedSize;
    }

    /**
     * Returns a pointer to the raw char array data.
     *
     * @return Pointer to the char array.
     */

    inline FAR const nxwidget_char_t *getCharArray(void) const
    {
      return m_text;
    }

    /**
     * Return a pointer to the specified character.
     *
     * @param index Index of the character to retrieve.
     */

    FAR nxwidget_char_t *getCharPointer(const int index) const;

  public:

    /**
     * Constructor to create an empty string object.
     */

    CNxString();

    /**
     * Constructor to create a string from a C character array.
     *
     * @param text Pointer to a char array to use as the basis of the
     * string.
     */

    CNxString(FAR const char *text);

    /**
     * Constructor to create a string from a single character.
     *
     * @param letter Single character to use as the basis of the string.
     */

    CNxString(const nxwidget_char_t letter);

    /**
     * Copy constructor.
     * @param string CNxString object to create a copy of.
     */

    CNxString(const CNxString &string);

    /**
     * Destructor.
     */

    inline ~CNxString()
    {
      delete[] m_text;
      m_text = (FAR nxwidget_char_t *)NULL;
    };

    /**
     * Creates and returns a new CStringIterator object that will iterate
     * over this string.  The object must be manually deleted once it is
     * no longer needed.
     *
     * @return A new CStringIterator object.
     */

    CStringIterator *newStringIterator(void) const;

    /**
     * Copy the internal array to the supplied buffer.  The buffer must be
     * large enough to contain the full text in the string.  The
     * getByteCount() method can be used to obtain the length of the string.
     * Unlike the CNxString class, the char array is null-terminated.
     * The buffer must be (getByteCount() + 1) bytes long, in order to
     * accommodate the terminator.
     *
     * @param buffer Buffer to copy the internal char array to.
     */

    void copyToCharArray(FAR nxwidget_char_t *buffer) const;

    /**
     * Set the text in the string.
     *
     * @param text CNxString containing the new data for this string.
     */

    void setText(const CNxString &text);

    /**
     * Set the text in the string.
     *
     * @param text Char array to use as the new data for this string.
     */

    void setText(FAR const char *text);

    /**
     * Set the nxwidget_char_t text in the string.
     *
     * @param text Char array to use as the new data for this string.
     */

    void setText(FAR const nxwidget_char_t *text, int nchars);

    /**
     * Set the 8-bit C-string text in the string.
     *
     * @param text Character to to use as the new data for this string.
     */

    void setText(const nxwidget_char_t text);

    /**
     * Append text to the end of the string.
     *
     * @param text String to append.
     */

    void append(const CNxString &text);

    /**
     * Insert text at the specified character index.
     *
     * @param text The text to insert.
     * @param index The index at which to insert the text.
     */

    void insert(const CNxString &text, const int index);

    /**
     * Remove all characters from the string from the start index onwards.
     *
     * @param startIndex Index to remove from.
     */

    void remove(const int startIndex);

    /**
     * Remove specified number of characters from the string from the
     * start index onwards.
     *
     * @param startIndex Index to remove from.
     * @param count Number of characters to remove.
     */

    void remove(const int startIndex, const int count);

    /**
     * Get the of number of letters (ie. the length) of the string.
     *
     * @return The length of the string.
     */

    inline const int getLength(void) const
    {
      return m_stringLength;
    };

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

    const nxwidget_char_t getCharAt(int index) const;

    /**
     * Returns the first index of the specified letter within the string.
     * Will return -1 if the letter is not found.
     *
     * @param letter Letter to find.
     * @return The index of the letter.
     */

    const int indexOf(nxwidget_char_t letter) const;

    /**
     * Returns the first index of the specified letter within the string.
     * Will return -1 if the letter is not found.  Scans through the string
     * from "startIndex" until it has examined all subsequent letters.
     * @param letter Letter to find.
     *
     * @param startIndex The index to start searching from.
     * @return The index of the letter.
     */

    const int indexOf(nxwidget_char_t letter, int startIndex) const;

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

    const int indexOf(nxwidget_char_t letter, int startIndex, int count) const;

    /**
     * Returns the last index of the specified letter within the string.
     * Will return -1 if the letter is not found.
     *
     * @param letter Letter to find.
     * @return The index of the letter.
     */

    const int lastIndexOf(nxwidget_char_t letter) const;

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

    const int lastIndexOf(nxwidget_char_t letter, int startIndex) const;

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

    const int lastIndexOf(nxwidget_char_t letter, int startIndex, int count) const;

    /**
     * Get a substring from this string.  It is the responsibility of the
     * caller to delete the substring when it is no longer required.
     *
     * @param startIndex The starting point of the substring.
     * @return A pointer to a new CNxString object containing the
     * substring.
     */

    CNxString *subString(int startIndex) const;

    /**
     * Get a substring from this string.  It is the responsibility of the
     * caller to delete the substring when it is no longer required.
     *
     * @param startIndex The starting point of the substring.
     * @param length The length of the substring.
     * @return A pointer to a new CNxString object containing the
     * substring.
     */

    CNxString *subString(int startIndex, int length) const;

    /**
     * Overloaded assignment operator.  Copies the data within the argument
     * string to this string.
     *
     * @param string The string to copy.
     * @return This string.
     */

    CNxString &operator=(const CNxString &string);

    /**
     * Overloaded assignment operator.  Copies the data within the argument
     * char array to this string.
     *
     * @param string The string to copy.
     * @return This string.
     */

    CNxString &operator=(const char *string);

    /**
     * Overloaded assignment operator.  Copies the data from the argument
     * char to this string.
     *
     * @param letter The char to copy.
     * @return This string.
     */

    CNxString& operator=(nxwidget_char_t letter);

    /**
     * Compares this string to the argument.
     *
     * @param string String to compare to.
     * @return Zero if both strings are equal.  A value greater than zero
     * indicates that this string is greater than the argument string.  A
     * value less than zero indicates the opposite.  Note that the return
     * value indicates the *byte* that does not match, not the *character*.
     */

    int compareTo(const CNxString &string) const;
  };
}

#endif // __cplusplus

#endif // __INCLUDE_CNXSTRING_HXX
