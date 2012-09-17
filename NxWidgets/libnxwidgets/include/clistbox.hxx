/****************************************************************************
 * NxWidgets/libnxwidgets/include/clistbox.hxx
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
 * in all NxWidget files.  Thanks Antony!
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

#ifndef __INCLUDE_CLISTBOX_HXX
#define __INCLUDE_CLISTBOX_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/nx/nxglib.h>

#include "nxconfig.hxx"
#include "cscrollingpanel.hxx"
#include "ilistdataeventhandler.hxx"
#include "clistdata.hxx"
#include "clistboxdataitem.hxx"
#include "cwidgetstyle.hxx"
#include "ilistbox.hxx"

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
   * Class providing a scrollable list of options.  The CListBox can be set
   * up to only allow one selection or multiple selections.  Processes
   * double-clicks and raises double-click events, so that a double-click
   * on an option can be made to automatically select and close a window/etc.
   * The options themselves have user-definable text and background colors
   * for their selected and unselected states.
   */

  class CListBox : public IListBox, public CScrollingPanel,
                   public IListDataEventHandler
  {
  protected:
    CListData m_options;           /**< Option storage. */
    uint8_t   m_optionPadding;     /**< Padding between options. */
    int       m_lastSelectedIndex; /**< Index of the last option selected. */

    /**
     * Draw the area of this widget that falls within the clipping region.
     * Called by the redraw() function to draw all visible regions.
     *
     * @param port The CGraphicsPort to draw to.
     * @see redraw()
     */

    virtual void drawContents(CGraphicsPort *port);

    /**
     * Draw the area of this widget that falls within the clipping region.
     * Called by the redraw() function to draw all visible regions.
     *
     * @param port The CGraphicsPort to draw to.
     * @see redraw()
     */

    virtual void drawBorder(CGraphicsPort *port);

    /**
     * Determines which item was clicked and selects or deselects it as
     * appropriate.  Also starts the dragging system.
     *
     * @param x The x coordinate of the click.
     * @param y The y coordinate of the click.
     */

    virtual void onClick(nxgl_coord_t x, nxgl_coord_t y);
    
    /**
     * Selects the clicked item and deselects all others.
     *
     * @param x The x coordinate of the click.
     * @param y The y coordinate of the click.
     */

    virtual void onDoubleClick(nxgl_coord_t x, nxgl_coord_t y);

    /**
     * Select or deselect an option by its index.  Does not deselect any other
     * selected options.  Set index to -1 to select nothing. Redraws the widget
     * and raises a value changed event.
     *
     * @param index The index of the option to select.
     * @param selected True to select the option, false to deselect it.
     */

    virtual void setOptionSelected(const int index, const bool selected);

    /**
     * Copy constructor is protected to prevent usage.
     */

    inline CListBox(const CListBox &listBox) : CScrollingPanel(listBox) { }

  public:
    
    /**
     * Constructor.
     *
     * @param pWidgetControl The controlling widget for the display.
     * @param x The x coordinate of the widget.
     * @param y The y coordinate of the widget.
     * @param width The width of the widget.
     * @param height The height of the widget.
     * @param style The style that the widget should use.  If this is not
     *   specified, the widget will use the values stored in the global
     *   g_defaultWidgetStyle object.  The widget will copy the properties of
     *   the style into its own internal style object.
     */

    CListBox(CWidgetControl *pWidgetControl,
             nxgl_coord_t x, nxgl_coord_t y,
             nxgl_coord_t width, nxgl_coord_t height,
             CWidgetStyle *style = (CWidgetStyle *)NULL);

    /**
     * Destructor.
     */

    virtual ~CListBox(void);

    /**
     * Add a new option to the widget using default colors.
     *
     * @param text Text to show in the option.
     * @param value The value of the option.
     */

    virtual void addOption(const CNxString &text, const uint32_t value);

    /**
     * Add an option to the widget.
     *
     * @param option The option to add.
     */

    virtual void addOption(CListBoxDataItem *option);

    /**
     * Remove an option from the widget by its index.
     *
     * @param index The index of the option to remove.
     */

    virtual void removeOption(const int index);

    /**
     * Remove all options from the widget.
     */

    virtual void removeAllOptions(void);

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
                           const nxwidget_pixel_t selectedBackColor);

    /**
     * Select an option by its index.
     * Redraws the widget and raises a value changed event.
     *
     * @param index The index of the option to select.
     */

    virtual void selectOption(const int index);

    /**
     * Select an option by its index.
     * Redraws the widget and raises a value changed event.
     *
     * @param index The index of the option to select.
     */

    virtual void deselectOption(const int index);

    /**
     * Select all options.  Does nothing if the listbox does not allow
     * multiple selections. Redraws the widget and raises a value changed
     * event.
     */

    virtual void selectAllOptions(void);

    /**
     * Deselect all options.
     * Redraws the widget and raises a value changed event.
     */

    virtual void deselectAllOptions(void);

    /**
     * Get the selected index.  Returns -1 if nothing is selected.  If
     * more than one option is selected, the index of the first selected
     * option is returned.
     *
     * @return The selected index.
     */

    virtual const int getSelectedIndex(void) const;

    /**
     * Sets the selected index.  Specify -1 to select nothing.  Resets any
     * other selected options to deselected. Redraws the widget and raises
     * a value changed event.
     *
     * @param index The selected index.
     */

    virtual void setSelectedIndex(const int index);

    /**
     * Get the selected option.  Returns NULL if nothing is selected.
     *
     * @return The selected option.
     */

    virtual const CListBoxDataItem *getSelectedOption(void) const;
    
    /**
     * Sets whether multiple selections are possible or not.
     *
     * @param allowMultipleSelections True to allow multiple selections.
     */

    virtual inline void
    setAllowMultipleSelections(const bool allowMultipleSelections)
    {
      m_options.setAllowMultipleSelections(allowMultipleSelections);
    }

    /**
     * Sets whether multiple selections are possible or not.
     *
     * @return True if multiple selections are allowed.
     */

    virtual inline const bool allowsMultipleSelections(void) const
    {
      return m_options.allowsMultipleSelections();
    }

    /**
     * Resize the scrolling canvas to encompass all options.
     */

    virtual void resizeCanvas(void);

    /**
     * Get the specified option.
     *
     * @return The specified option.
     */

    virtual inline const CListBoxDataItem *getOption(const int index)
    {
      return (const CListBoxDataItem *)m_options.getItem(index);
    }

    /**
     * Sort the options alphabetically by the text of the options.
     */

    virtual void sort(void);

    /**
     * Get the total number of options.
     *
     * @return The number of options.
     */

    virtual inline const int getOptionCount(void) const
    {
      return m_options.getItemCount();
    }

    /**
     * Get the height of a single option.
     *
     * @return The height of an option.
     */

    virtual const nxgl_coord_t getOptionHeight(void) const;

    /**
     * Sets whether or not items added to the list are automatically sorted on insert or not.
     *
     * @param sortInsertedItems True to enable sort on insertion.
     */

    virtual inline void setSortInsertedItems(const bool sortInsertedItems)
    {
      m_options.setSortInsertedItems(sortInsertedItems);
    }

    /**
     * Handles list data changed events.
     *
     * @param e Event arguments.
     */

    virtual void handleListDataChangedEvent(const CListDataEventArgs &e);

    /**
     * Handles list selection changed events.
     *
     * @param e Event arguments.
     */

    virtual void handleListDataSelectionChangedEvent(const CListDataEventArgs &e);

    /**
     * Insert the dimensions that this widget wants to have into the rect
     * passed in as a parameter.  All coordinates are relative to the widget's
     * parent.  Value is based on the length of the largest string in the
     * set of options.
     *
     * @param rect Reference to a rect to populate with data.
     */

    virtual void getPreferredDimensions(CRect &rect) const;

    /**
     * Check if the click is a double-click.
     *
     * @param x X coordinate of the click.
     * @param y Y coordinate of the click.
     * @return True if the click is a double-click.
     */

    virtual bool isDoubleClick(nxgl_coord_t x, nxgl_coord_t y);
  };
}

#endif // __cplusplus

#endif // __INCLUDE_CLISTBOX_HXX

