/****************************************************************************
 * NxWidgets/libnxwidgets/include/cstringiterator.hxx
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

#ifndef __INCLUDE_CSTRINGITERATOR_HXX
#define __INCLUDE_CSTRINGITERATOR_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Implementation Classes
 ****************************************************************************/

#if defined(__cplusplus)

namespace NXWidgets
{
  class CNxString;
  
  /**
   * Class used to efficiently iterate over the characters in a CNxString
   * object.  The CNxString is a Unicode string that uses the fixed-width
   * 16-bit encoding to represent its characters.
   */

  class CStringIterator
  {
  private:
    const CNxString       *m_pString;      /**< String being iterated over. */
    const nxwidget_char_t *m_pCurrentChar; /**< Pointer to the current position of the iterator. */
    int                    m_currentIndex; /**< Iterator's current index within the string. */

  public:

    /**
     * Constructor.  Moves the iterator to the first character in the string.
     *
     * @param string Pointer to the string that will be iterated over.
     */

    CStringIterator(FAR const CNxString* string);

    /**
     * Destructor.
     */

    inline ~CStringIterator(void) { }

    /**
     * Moves the iterator to the first character in the string.
     *
     * @param Returns false if the string is empty
     */

    bool moveToFirst(void);

    /**
     * Moves the iterator to the last character in the string.
     *
     * @param Returns false if the string is empty
     */

    bool moveToLast(void);

    /**
     * Move the iterator to the next character in the string.
     *
     * @return True if the iterator moved; false if not (indicates end of string).
     */

    bool moveToNext(void);

    /**
     * Move the iterator to the previous character in the string.
     *
     * @return True if the iterator moved; false if not (indicates start of string).
     */

    bool moveToPrevious(void);

    /**
     * Move the iterator to the specified index.
     *
     * @param index The index to move to.
     * @return True if the iterator moved; false if not (indicates end of string).
     */

    bool moveTo(int index);
    
    /**
     * Get the current position of the iterator within the string.
     *
     * @return The current character index of the iterator.
     */

    inline int getIndex(void) const
    {
      return m_currentIndex;
    }

    /**
     * Get the letter in the string at the iterator's current point.
     *
     * @return The current character in the string.
     */

    inline nxwidget_char_t getChar(void) const
    {
      return *m_pCurrentChar;
    }
  };
}

#endif // __cplusplus

#endif // __INCLUDE_CSTRINGITERATOR_HXX
