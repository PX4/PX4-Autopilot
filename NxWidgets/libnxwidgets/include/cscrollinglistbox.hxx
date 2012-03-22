/****************************************************************************
 * NxWidgets/libnxwidgets/include/cscrollinglistbox.hxx
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

#ifndef __INCLUDE_CSCROLLINGLISTBOX_HXX
#define __INCLUDE_CSCROLLINGLISTBOX_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/nx/nxglib.h>

#include "cnxwidget.hxx"
#include "clistbox.hxx"
#include "cwidgeteventhandler.hxx"
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
  class CScrollbarVertical;
  class CWidgetControl;

  /**
   * Widget containing a CListBox and a vertical scrollbar.  Exposed
   * methods are more or less identical to the methods exposed by the CListBox
   * to ensure that the two are interchangeable.
   */

  class CScrollingListBox : public IListBox, public CNxWidget,
                            public CWidgetEventHandler
  {
  protected:
    CListBox           *m_listbox;        /**< Pointer to the list box. */
    CScrollbarVertical *m_scrollbar;      /**< Pointer to the scrollbar. */
    uint8_t             m_scrollbarWidth; /**< Width of the scrollbar. */

    /**
     * Draw the area of this widget that falls within the clipping region.
     * Called by the redraw() function to draw all visible regions.
     *
     * @param port The CGraphicsPort to draw to.
     * @see redraw()
     */

    virtual void drawContents(CGraphicsPort *port);

    /**
     * Resize the listbox to the new dimensions.
     *
     * @param width The new width.
     * @param height The new height.
     */

    virtual void onResize(nxgl_coord_t width, nxgl_coord_t height);
    
    /**
     * Destructor.
     */

    virtual inline ~CScrollingListBox(void) { }

    /**
     * Copy constructor is protected to prevent usage.
     */

    inline CScrollingListBox(const CScrollingListBox &scrollingListBox)
    : CNxWidget(scrollingListBox) { }

  public:

    /**
     * Constructor.
     *
     * @param pWidgetControl The widget control for the display.
     * @param x The x coordinate of the widget.
     * @param y The y coordinate of the widget.
     * @param width The width of the widget.
     * @param height The height of the widget.
     * @param style The style that the widget should use.  If this is not
     *   specified, the widget will use the values stored in the global
     *   g_defaultWidgetStyle object.  The widget will copy the properties of
     *   the style into its own internal style object.
     */

    CScrollingListBox(CWidgetControl *pWidgetControl,
                      nxgl_coord_t x, nxgl_coord_t y,
                      nxgl_coord_t width, nxgl_coord_t height,
                      CWidgetStyle *style = (CWidgetStyle *)NULL);

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
     * Select an option by its index.  Does not deselect any other selected options.
     * Redraws the widget and raises a value changed event.
     *
     * @param index The index of the option to select.
     */

    virtual inline void selectOption(const int index)
    {
      m_listbox->selectOption(index);
    }

    /**
     * Select an option by its index.  Does not deselect any other selected options.
     * Redraws the widget and raises a value changed event.
     *
     * @param index The index of the option to select.
     */

    virtual inline void deselectOption(const int index)
    {
      m_listbox->deselectOption(index);
    }

    /**
     * Select all options.  Does nothing if the listbox does not allow multiple selections.
     * Redraws the widget and raises a value changed event.
     */

    virtual inline void selectAllOptions(void)
    {
      m_listbox->selectAllOptions();
    }

    /**
     * Deselect all options.
     * Redraws the widget and raises a value changed event.
     */

    virtual inline void deselectAllOptions(void)
    {
      m_listbox->deselectAllOptions();
    }

    /**
     * Get the selected index.  Returns -1 if nothing is selected.  If more than one
     * option is selected, the index of the first selected option is returned.
     *
     * @return The selected index.
     */

    virtual inline const int getSelectedIndex(void) const
    {
      return m_listbox->getSelectedIndex();
    }

    /**
     * Sets the selected index.  Specify -1 to select nothing.  Resets any
     * other selected items to deselected.
     * Redraws the widget and raises a value changed event.
     *
     * @param index The selected index.
     */

    virtual inline void setSelectedIndex(const int index)
    {
      m_listbox->setSelectedIndex(index);
    }

    /**
     * Get the selected option.  Returns NULL if nothing is selected.
     *
     * @return The selected option.
     */

    virtual inline const CListBoxDataItem *getSelectedOption(void) const
    {
      return m_listbox->getSelectedOption();
    }
    
    /**
     * Sets whether multiple selections are possible or not.
     *
     * @param allowMultipleSelections True to allow multiple selections.
     */

    virtual inline
    void setAllowMultipleSelections(const bool allowMultipleSelections)
    {
      m_listbox->setAllowMultipleSelections(allowMultipleSelections);
    }

    /**
     * Sets whether multiple selections are possible or not.
     *
     * @return True if multiple selections are allowed.
     */

    virtual inline const bool allowsMultipleSelections(void) const
    {
      return m_listbox->allowsMultipleSelections();
    }

    /**
     * Resize the scrolling canvas to encompass all options.
     */

    virtual inline void resizeCanvas(void)
    {
      m_listbox->resizeCanvas();
    }

    /**
     * Get the specified option.
     *
     * @return The specified option.
     */

    virtual inline const CListBoxDataItem *getOption(const int index)
    {
      return m_listbox->getOption(index);
    }

    /**
     * Get the selected index.  Returns -1 if nothing is selected.
     *
     * @return The selected index.
     */

    virtual inline const CListBoxDataItem *getOption(const int index) const
    {
      return m_listbox->getOption(index);
    }

    /**
     * Sort the options alphabetically by the text of the options.
     */

    virtual inline void sort(void)
    {
      m_listbox->sort();
    }

    /**
     * Get the total number of options.
     *
     * @return The number of options.
     */

    virtual inline const int getOptionCount(void) const
    {
      return m_listbox->getOptionCount();
    }

    /**
     * Get the height of a single option.
     *
     * @return The height of an option.
     */

    virtual inline const nxgl_coord_t getOptionHeight(void) const
    {
      return m_listbox->getOptionHeight();
    }

    /**
     * Handles events raised by its sub-widgets.
     *
     * @param e Event arguments.
     */

    virtual void handleValueChangeEvent(const CWidgetEventArgs &e);

    /**
     * Handle a widget action event.
     *
     * @param e The event data.
     */

    virtual void handleActionEvent(const CWidgetEventArgs &e);

    /**
     * Handles events raised by its sub-widgets.
     *
     * @param e Event arguments.
     */

    virtual void handleScrollEvent(const CWidgetEventArgs &e);

    /**
     * Handle a mouse button click event.
     *
     * @param e The event data.
     */

    virtual void handleClickEvent(const CWidgetEventArgs &e);

    /**
     * Handles events raised by its sub-widgets.
     *
     * @param e Event arguments.
     */

    virtual void handleDoubleClickEvent(const CWidgetEventArgs &e);

    /**
     * Handle a mouse button release event that occurred within the bounds of
     * the source widget.
     *
     * @param e The event data.
     */

    virtual void handleReleaseEvent(const CWidgetEventArgs &e);

    /**
     * Handle a mouse button release event that occurred outside the bounds of
     * the source widget.
     *
     * @param e The event data.
     */

    virtual void handleReleaseOutsideEvent(const CWidgetEventArgs &e);

    /**
     * Set the font used in the textbox.
     *
     * @param font Pointer to the new font.
     */

    virtual void setFont(CNxFont *font);

    /**
     * Sets whether or not items added to the list are automatically
     * sorted on insert or not.
     *
     * @param sortInsertedItems True to enable sort on insertion.
     */

    virtual inline void setSortInsertedItems(const bool sortInsertedItems)
    {
      m_listbox->setSortInsertedItems(sortInsertedItems);
    }

    /**
     * Insert the dimensions that this widget wants to have into the rect
     * passed in as a parameter.  All coordinates are relative to the widget's
     * parent.  Value is based on the length of the largest string in the
     * set of options.
     *
     * @param rect Reference to a rect to populate with data.
     */

    virtual void getPreferredDimensions(CRect &rect) const;
  };
}

#endif // __cplusplus

#endif // __INCLUDE_CSCROLLINGLISTBOX_HXX
