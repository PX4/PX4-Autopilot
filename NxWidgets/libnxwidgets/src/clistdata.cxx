/****************************************************************************
 * NxWidgets/libnxwidgets/src/clistdata.cxx
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
#include <cstring>

#include "clistdata.hxx"
#include "ilistdataeventhandler.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Method Implementations
 ****************************************************************************/

using namespace NXWidgets;

/**
 * Constructor.
 */

CListData::CListData(void)
{
  m_allowMultipleSelections = true;
  m_sortInsertedItems      = false;
}

/**
 * Destructor.
 */

CListData::~CListData(void)
{
  removeAllItems();
}

/**
 * Add a new item.
 *
 * @param text Text to show in the option.
 * @param value The value of the option.
 */

void CListData::addItem(const CNxString &text, const uint32_t value)
{
  // Create new option

  addItem(new CListDataItem(text, value));
}

/**
 * Add an existing item.  CListData becomes the owner of the option and will delete it
 * when the list is deleted.
 *
 * @param item The item to add.
 */

void CListData::addItem(CListDataItem *item)
{
  // Determine insert type

  if (m_sortInsertedItems)
    {
      // Sorted insert

      m_items.insert(getSortedInsertionIndex(item), item);
    }
  else
    {
      // Append

      m_items.push_back(item);
    }

  raiseDataChangedEvent();
}

/**
 * Remove an item by its index.
 *
 * @param index The index of the option to remove.
 */

void CListData::removeItem(const int index)
{
  // Bounds check

  if (index < m_items.size())
    {
      // Delete the option

      delete m_items[index];

      // Erase the option from the list

      m_items.erase(index);
      raiseDataChangedEvent();
    }
}

/**
 * Select an item by its index.
 *
 * @param index The index of the item to select.
 */

void CListData::selectItem(const int index)
{
  setItemSelected(index, true);
}

/**
 * Deselect an item by its index.
 *
 * @param index The index of the item to select.
 */

void CListData::deselectItem(const int index)
{
  setItemSelected(index, false);
}

/**
 * Remove all items.
 */

void CListData::removeAllItems(void)
{
  // Delete all option data

  for (int i = 0; i < m_items.size(); i++)
    {
      delete m_items[i];
    }
  
  m_items.clear();
  raiseDataChangedEvent();
}

/**
 * Get the selected index.  Returns -1 if nothing is selected.  If more than one
 * item is selected, the index of the first selected item is returned.
 *
 * @return The selected index.
 */

const int CListData::getSelectedIndex(void) const
{
  // Get the first selected index

  for (int i = 0; i < m_items.size(); i++)
    {
      if (m_items[i]->isSelected())
        {
          return i;
        }
    }
  
  return -1;
}

/**
 * Sets the selected index.  Specify -1 to select nothing.  Resets any
 * other selected items to deselected.
 *
 * @param index The selected index.
 */

void CListData::setSelectedIndex(const int index)
{
  setItemSelected(index, true);
}

/**
 * Get the selected item.  Returns NULL if nothing is selected.
 *
 * @return The selected option.
 */

const CListDataItem *CListData::getSelectedItem(void) const
{
  // Get the first selected option

  int index = getSelectedIndex();
  if (index > -1)
    {
      return m_items[index];
    }
  return (CListDataItem *)NULL;
}

/**
 * Sort the items using their compareTo() methods.
 */

void CListData::sort(void)
{
  quickSort(0, m_items.size() - 1);
  raiseDataChangedEvent();
}

/**
 * Select all items.  Does nothing if the list does not allow
 * multiple selections.
 */

void CListData::selectAllItems(void)
{
  if (m_allowMultipleSelections)
    {
      for (int i = 0; i < m_items.size(); i++)
        {
          m_items[i]->setSelected(true);
        }

      raiseSelectionChangedEvent();
    }
}

/**
 * Deselect all items.
 */

void CListData::deselectAllItems(void)
{
  for (int i = 0; i < m_items.size(); i++)
    {
      m_items[i]->setSelected(false);
    }

  raiseSelectionChangedEvent();
}

/**
 * Select or deselect an item by its index.  Does not deselect any
 * other selected items. Set index to -1 to select nothing.
 *
 * @param index The index of the item to select.
 * @param selected True to select the item, false to deselect it.
 */

void CListData::setItemSelected(const int index, bool selected)
{
  // Deselect old options if we're making an option selected and
  // we're not a multiple list

  if (((!m_allowMultipleSelections) || (index == -1)) && (selected))
    {
      for (int i = 0; i < m_items.size(); i++)
        {
          m_items[i]->setSelected(false);
        }
    }

  // Select or deselect the new option

  if ((index > -1) && (index < m_items.size()))
    {
      m_items[index]->setSelected(selected);
    }

  raiseSelectionChangedEvent();
}

/**
 * Remove an event handler.
 *
 * @param eventHandler The event handler to remove.
 */

void CListData::removeListDataEventHandler(IListDataEventHandler *eventHandler)
{
  for (int i = 0; i < m_listDataEventhandlers.size(); ++i)
    {
      if (m_listDataEventhandlers.at(i) == eventHandler)
        {
          m_listDataEventhandlers.erase(i);
          return;
        }
    }
}

/**
 * Quick sort the items using their compareTo() methods.
 *
 * @param start The index to start sorting at.
 * @param end The index to stop sorting at.
 */

void CListData::quickSort(const int start, const int end)
{
  if (end > start)
    {
      int left = start;
      int right = end;

      CListDataItem *pivot = m_items[(start + end) >> 1];

      do
        {
          while ((pivot->compareTo(m_items[left]) > 0) && (left < end))
            {
              left++;
            }

          while ((pivot->compareTo(m_items[right]) < 0) && (right > start))
            {
              right--;
            }

          if (left > right)
            {
              break;
            }

         swapItems(left, right);
         left++;
         right--;
        }
      while (left <= right);

      quickSort(start, right);
      quickSort(left, end);
    }
}

/**
 * Swap the locations of two items in the array.
 *
 * @param index1 The index of the first item to swap.
 * @param index2 The index of the second item to swap.
 */

void CListData::swapItems(const int index1, const int index2)
{
  CListDataItem *tmp = m_items[index1];
  m_items[index1]    = m_items[index2];
  m_items[index2]    = tmp;
}

/**
 * Return the index that an item should be inserted at to maintain a sorted list of data.
 *
 * @param item The item to insert.
 * @return The index that the item should be imserted into at.
 */

const int CListData::getSortedInsertionIndex(const CListDataItem *item) const
{
  int i = 0;

  // Locate slot where new option should go

  while ((i < m_items.size()) && (item->compareTo(m_items[i]) > 0))
    {
      i++;
    }
  return i;
}

/**
 * Raise a data changed event.
 */

void CListData::raiseDataChangedEvent(void)
{
  CListDataEventArgs eventArgs(this);

  for (int i = 0; i < m_listDataEventhandlers.size(); ++i)
    {
      m_listDataEventhandlers.at(i)->handleListDataChangedEvent(eventArgs);
    }
}

/**
 * Raise a selection changed event.
 */

void CListData::raiseSelectionChangedEvent(void)
{
  CListDataEventArgs eventArgs(this);

  for (int i = 0; i < m_listDataEventhandlers.size(); ++i)
    {
      m_listDataEventhandlers.at(i)->handleListDataSelectionChangedEvent(eventArgs);
    }
}
