/****************************************************************************
 * NxWidgets/libnxwidgets/include/tnxarray.hxx
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

#ifndef __INCLUDE_TNXARRAY_HXX
#define __INCLUDE_TNXARRAY_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

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

/**
 * Class providing a dynamic array; that is, an array that will automatically
 * grow to accommodate new data.  It provides a fast way to randomly access
 * a list of data.  Essentially, it provides the most important functionality
 * of the STL vector class without any of the overhead of including an STL
 * class.
 *
 * If the data to be stored will store a lot of data that will predominantly
 * be read sequentially, consider using the LinkedList class instead.  Resizing
 * the list is an expensive operation that will occur frequently when filling
 * the array with large amounts of data.  Adding new data to the linked list is
 * very inexpensive.
 */
 
template <class T>
class TNxArray
{
private:
  T  *m_data;          /**< Internal array of data items */
  int m_size;          /**< Number of items in the array */
  int m_reservedSize;  /**< Total size of the array including unpopulated slots */

  /**
   * Re-allocate the array to this size;
   */

  void reallocate(const int newSize);

  /**
   * Resize the array if it is full.
   */

  void resize(void);

public:

  /**
   * Constructor.  Creates an un-allocated array.  The array will
   * be allocated when items are added to it or when preallocate()
   * is called.
   */

  inline TNxArray();

  /**
   * Constructor.  Creates an allocated array.
   *
   *@param initialSize The initial size of the array.
   */

  inline TNxArray(int initialSize);

  /**
   * Destructor.
   */

  inline ~TNxArray();

  /**
   * Set the initial size of the array.  Normally, the array is
   * unallocated until the first data is pushed into the array.
   * That works great for stacks and lists.  But if you want a
   * array of unitialized elements, then this method will
   * preallocate the array for you.
   *
   * @return The size of the array.
   */

  void preallocate(void);

  /**
   * Get the size of the array.
   *
   * @return The size of the array.
   */

  inline const int size(void) const;

  /**
   * Add a value to the end of the array.
   *
   * @param value The value to add to the array.
   */

  void push_back(const T &value);

  /**
   * Insert a value into the array.
   *
   * @param index The index to insert into.
   * @param value The value to insert.
   */

  void insert(const int index, const T &value);

  /**
   * Remove the last element from the array.
   */

  void pop_back(void);

  /**
   * Erase a single value at the specified index
   */

  void erase(const int index);

  /**
   * Get a value at the specified location.  Does not perform bounds checking.
   * @param index The index of the desired value.
   * @return The value at the specified index.
   */

  inline T &at(const int index) const;

  /**
   * Check if the array has any data.
   * @return True if the array is empty.
   */

  inline bool empty(void) const;

  /**
   * Remove all data.
   */

  void clear();

  /**
   * Overload the [] operator to allow array-style access.
   * @param index The index to retrieve.
   * @return The value at the specified index.
   */

  T& operator[](const int index) const;
};

template <class T>
TNxArray<T>::TNxArray()
{
  // Don't allocate anything until the first data is added to
  // the array

  m_size         = 0;       // Number of data items in use
  m_reservedSize = 0;       // Number of data items allocated
  m_data         = (T *)0;  // Allocated memory for data items
}

template <class T>
TNxArray<T>::TNxArray(int initialSize)
{
  m_size         = 0;       // Number of data items in use
  m_reservedSize = 0;       // Number of data items allocated
  m_data         = (T *)0;  // Allocated memory for data items
  preallocate(initialSize); // Allocate the initial array
}

template <class T>
TNxArray<T>::~TNxArray()
{
  if (m_data)
    {
      delete [] m_data;
    }
}

template <class T>
const int TNxArray<T>::size(void) const
{
  return m_size;
}

template <class T>
void TNxArray<T>::push_back(const T &value)
{
  // Ensure the array is large enough to hold one more data item

  resize();

  // Add data to array

  m_data[m_size] = value;

  // Remember we've filled a slot

  m_size++;
}

template <class T>
void TNxArray<T>::pop_back(void)
{
  if (m_size >= 1)
    {
      // We can just reduce the used size of the array, as the value
      // will get overwritten automatically
 
      m_size--;
    }
}

template <class T>
void TNxArray<T>::insert(const int index, const T &value)
{
  // Bounds check

  if ((index >= m_size) || (m_size == 0))
    {
      push_back(value);
      return;
    }

  // Ensure the array is large enough to hold one more data item

  resize();

  // Shift all of the data back one place to make a space for the new data

  for (int i = m_size; i > index; i--)
    {
      m_data[i] = m_data[i - 1];
    }

  // Add data to array

  m_data[index] = value;

  // Remember we've filled a slot

  m_size++;
}

template <class T>
void TNxArray<T>::erase(const int index)
{
  // Bounds check

  if (index >= m_size)
    {
      return;
    }

  // Shift all of the data back one place and overwrite the value

  for (int i = index; i < m_size - 1; i++)
    {
      m_data[i] = m_data[i + 1];
    }

  // Remember we've removed a slot

  m_size--;
}

template <class T>
void TNxArray<T>::reallocate(const int newSize)
{
  // Do we need to redim the array?

  if (m_reservedSize < newSize)
    {
      // Create the new array

      T *newData = new T[newSize];

      // Copy old array contents to new the new array

      for (int i = 0; i < m_reservedSize; i++)
        {
          newData[i] = m_data[i];
        }

      // Delete the old array (if there was one)

      if (m_data)
        {
          delete [] m_data;
        }

      // Update values

      m_data         = newData;
      m_reservedSize = newSize;
    }
}

template <class T>
void TNxArray<T>::resize(void)
{
  // Do we need to redim the array in order to add one more entry?

  if (m_reservedSize == m_size)
    {
      // We have filled the array, so resize it

      int newSize = m_reservedSize;
#if CONFIG_NXWIDGETS_TNXARRAY_INITIALSIZE != CONFIG_NXWIDGETS_TNXARRAY_SIZEINCREMENT
      newSize += m_reservedSize ?
                 CONFIG_NXWIDGETS_TNXARRAY_SIZEINCREMENT :
                 CONFIG_NXWIDGETS_TNXARRAY_INITIALSIZE;
#else
      newSize += CONFIG_NXWIDGETS_TNXARRAY_SIZEINCREMENT;
#endif

      // Re-allocate the array

      reallocate(newSize);
    }
}

template <class T>
T& TNxArray<T>::at(const int index) const
{
  // What if this is called with index > m_reservedSize?  What if
  // this is called before m_data is allocated?  Don't do that!

  return m_data[index];
}

template <class T>
bool TNxArray<T>::empty() const
{
  return (m_size == 0);
}

template <class T>
T& TNxArray<T>::operator[](const int index) const
{
  // What if this is called with index > m_reservedSize?  What if
  // this is called before m_data is allocated?  Don't do that!

  return m_data[index];
}

template <class T>
void TNxArray<T>::clear()
{
  // All we need to do is reset the size value

  m_size = 0;
}

#endif // __cplusplus

#endif // __INCLUDE_TNXARRAY_HXX
