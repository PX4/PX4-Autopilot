/****************************************************************************
 * NxWidgets/libnxwidgets/include/ilistbox.hxx
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

#ifndef __INCLUDE_ILISTBOX_HXX
#define __INCLUDE_ILISTBOX_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/nx/nxglib.h>

#include "clistboxdataitem.hxx"

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
   * Defines the interface for ListBox classes.
   */

  class IListBox
  {
  public:
    /**
     * A virtual destructor is required in order to override the IListBox
     * destructor.  We do this because if we delete IListBox, we want the
     * destructor of the class that inherits from IListBox to run, not this
     * one.
     */

    virtual ~IListBox(void) { }

    /**
     * Add a new option to the widget using default colors.
     *
     * @param text Text to show in the option.
     * @param value The value of the option.
     */

    virtual void addOption(const CNxString &text, const uint32_t value) = 0;

    /**
     * Add an option to the widget.
     *
     * @param option The option to add.
     */

    virtual void addOption(CListBoxDataItem *option) = 0;

    /**
     * Remove an option from the widget by its index.
     *
     * @param index The index of the option to remove.
     */

    virtual void removeOption(const int index) = 0;

    /**
     * Remove all options from the widget.
     */

    virtual void removeAllOptions(void) = 0;

    /**
     * Add a new option to the widget.
     *
     * @param text Text to show in the option.
     * @param value The value of the option.
     * @param normalTextColor Color to draw the text with when not selected.
     * @param normalBackColor Color to draw the background with when not selected.
     * @param selectedTextColor Color to draw the text with when selected.
     * @param selectedBackColor Color to draw the background with when selected.
     */

    virtual void addOption(const CNxString &text, const uint32_t value,
                           const nxwidget_pixel_t normalTextColor,
                           const nxwidget_pixel_t normalBackColor,
                           const nxwidget_pixel_t selectedTextColor,
                           const nxwidget_pixel_t selectedBackColor) = 0;

    /**
     * Select an option by its index.
     * Redraws the widget and raises a value changed event.
     *
     * @param index The index of the option to select.
     */

    virtual void selectOption(const int index) = 0;

    /**
     * Select an option by its index.
     * Redraws the widget and raises a value changed event.
     *
     * @param index The index of the option to select.
     */

    virtual void deselectOption(const int index) = 0;

    /**
     * Select all options.  Does nothing if the listbox does not allow
     * multiple selections. Redraws the widget and raises a value
     * changed event.
     */

    virtual void selectAllOptions(void) = 0;

    /**
     * Deselect all options.
     * Redraws the widget and raises a value changed event.
     */

    virtual void deselectAllOptions(void) = 0;

    /**
     * Get the selected index.  Returns -1 if nothing is selected.  If more than one
     * option is selected, the index of the first selected option is returned.
     *
     * @return The selected index.
     */

    virtual const int getSelectedIndex(void) const = 0;

    /**
     * Sets the selected index.  Specify -1 to select nothing.  Resets any
     * other selected options to deselected.
     * Redraws the widget and raises a value changed event.
     *
     * @param index The selected index.
     */

    virtual void setSelectedIndex(const int index) = 0;

    /**
     * Get the selected option.  Returns NULL if nothing is selected.
     *
     * @return The selected option.
     */

    virtual const CListBoxDataItem *getSelectedOption(void) const = 0;
    
    /**
     * Sets whether multiple selections are possible or not.
     *
     * @param allowMultipleSelections True to allow multiple selections.
     */

    virtual void setAllowMultipleSelections(const bool allowMultipleSelections) = 0;

    /**
     * Sets whether multiple selections are possible or not.
     *
     * @return True if multiple selections are allowed.
     */

    virtual const bool allowsMultipleSelections(void) const = 0;

    /**
     * Get the specified option.
     *
     * @return The specified option.
     */

    virtual const CListBoxDataItem *getOption(const int index) = 0;

    /**
     * Sort the options alphabetically by the text of the options.
     */

    virtual void sort(void) = 0;

    /**
     * Get the total number of options.
     *
     * @return The number of options.
     */

    virtual const int getOptionCount(void) const = 0;

    /**
     * Get the height of a single option.
     *
     * @return The height of an option.
     */

    virtual const nxgl_coord_t getOptionHeight(void) const = 0;

    /**
     * Sets whether or not items added to the list are automatically sorted on insert or not.
     *
     * @param sortInsertedItems True to enable sort on insertion.
     */

    virtual void setSortInsertedItems(const bool sortInsertedItems) = 0;
  };
}

#endif // __cplusplus

#endif // __INCLUDE_ILISTBOX_HXX

