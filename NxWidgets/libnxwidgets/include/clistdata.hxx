/****************************************************************************
 * NxWidgets/libnxwidgets/include/clistdata.hxx
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

#ifndef __INCLUDE_CLISTDATA_HXX
#define __INCLUDE_CLISTDATA_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/nx/nxglib.h>

#include "tnxarray.hxx"
#include "ilistdataeventhandler.hxx"
#include "clistdataitem.hxx"
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
   * Class representing a list of items.  Designed to be used by the
   * CListBox class, etc, to store its data.  Fires events to notify
   * listeners when the list changes or a new selection is made.
   */

  class CListData
  {
  protected:
    TNxArray<CListDataItem*> m_items; /**< Collection of list data items. */
    TNxArray<IListDataEventHandler*> m_listDataEventhandlers;  /**< Collection of event handlers. */
    bool m_allowMultipleSelections;   /**< If true, multiple options can
                                           be selected. */
    bool m_sortInsertedItems;         /**< Automatically sorts items on
                                           insertion if true. */

    /**
     * Quick sort the items using their compareTo() methods.
     *
     * @param start The index to start sorting at.
     * @param end The index to stop sorting at.
     */

    virtual void quickSort(const int start, const int end);

    /**
     * Swap the locations of two items in the array.
     *
     * @param index1 The index of the first item to swap.
     * @param index2 The index of the second item to swap.
     */

    virtual void swapItems(const int index1, const int index2);

    /**
     * Return the index that an item should be inserted at to maintain a sorted list of data.
     *
     * @param item The item to insert.
     * @return The index that the item should be imserted into at.
     */

    const int getSortedInsertionIndex(const CListDataItem *item) const;

    /**
     * Raise a data changed event.
     */

    void raiseDataChangedEvent(void);

    /**
     * Raise a selection changed event.
     */

    void raiseSelectionChangedEvent(void);

  public:

    /**
     * Constructor.
     */
    CListData(void);

    /**
     * Destructor.
     */
    virtual ~CListData(void);

    /**
     * Add a new item.
     *
     * @param text Text to show in the option.
     * @param value The value of the option.
     */

    virtual void addItem(const CNxString &text, const uint32_t value);

    /**
     * Add an existing item.  CListData becomes the owner of the option and will delete it
     * when the list is deleted.
     *
     * @param item The item to add.
     */

    virtual void addItem(CListDataItem *item);

    /**
     * Remove an item by its index.
     *
     * @param index The index of the option to remove.
     */

    virtual void removeItem(const int index);

    /**
     * Select an item by its index.
     *
     * @param index The index of the item to select.
     */

    virtual void selectItem(const int index);

    /**
     * Deselect an item by its index.
     *
     * @param index The index of the item to select.
     */

    virtual void deselectItem(const int index);

    /**
     * Remove all items.
     */

    virtual void removeAllItems(void);

    /**
     * Get the selected index.  Returns -1 if nothing is selected.  If more than one
     * item is selected, the index of the first selected item is returned.
     *
     * @return The selected index.
     */

    virtual const int getSelectedIndex(void) const;

    /**
     * Sets the selected index.  Specify -1 to select nothing.  Resets any
     * other selected items to deselected.
     *
     * @param index The selected index.
     */

    virtual void setSelectedIndex(const int index);

    /**
     * Get the selected item.  Returns NULL if nothing is selected.
     *
     * @return The selected option.
     */

    virtual const CListDataItem *getSelectedItem(void) const;

    /**
     * Sets whether multiple selections are possible or not.
     *
     * @param allowMultipleSelections True to allow multiple selections.
     */

    virtual inline void
    setAllowMultipleSelections(const bool allowMultipleSelections)
    {
      m_allowMultipleSelections = allowMultipleSelections;
    }

    /**
     * Get the specified item.
     *
     * @return The specified item.
     */

    virtual inline const CListDataItem *getItem(const int index) const
    {
      return m_items[index];
    }

    /**
     * Sort the items using their compareTo() methods.
     */

    virtual void sort(void);

    /**
     * Get the total number of items.
     *
     * @return The number of items.
     */

    virtual inline const int getItemCount(void) const
    {
      return m_items.size();
    }

    /**
     * Select all items.  Does nothing if the list does not allow
     * multiple selections.
     */

    virtual void selectAllItems(void);

    /**
     * Deselect all items.
     */

    virtual void deselectAllItems(void);

    /**
     * Select or deselect an item by its index.  Does not deselect any
     * other selected items. Set index to -1 to select nothing.
     *
     * @param index The index of the item to select.
     * @param selected True to select the item, false to deselect it.
     */

    virtual void setItemSelected(const int index, const bool selected);

    /**
     * Returns whether multiple selections are possible or not.
     *
     * @return True if multiple selections are allowed.
     */

    virtual inline const bool allowsMultipleSelections(void) const
    {
      return m_allowMultipleSelections;
    }

    /**
     * Sets whether or not items added to the list are automatically
     * sorted on insert or not.
     *
     * @param sortInsertedItems True to enable sort on insertion.
     */

    virtual inline void setSortInsertedItems(const bool sortInsertedItems)
    {
      m_sortInsertedItems = sortInsertedItems;
    }

    /**
     * Add an event handler.
     *
     * @param eventHandler The event handler to add.
     */

    inline void addListDataEventHandler(IListDataEventHandler *eventHandler)
    {
      m_listDataEventhandlers.push_back(eventHandler);
    }

    /**
     * Remove an event handler.
     *
     * @param eventHandler The event handler to remove.
     */

    void removeListDataEventHandler(IListDataEventHandler *eventHandler);
  };
}

#endif // __cplusplus

#endif // __INCLUDE_CLISTDATA_HXX
