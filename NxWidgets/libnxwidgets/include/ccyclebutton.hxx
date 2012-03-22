/****************************************************************************
 * NxWidgets/libnxwidgets/include/ccyclebutton.hxx
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

#ifndef __INCLUDE_CCYLEBUTTON_HXX
#define __INCLUDE_CCYLEBUTTON_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/nx/nxglib.h>

#include "cbutton.hxx"
#include "ilistdataeventhandler.hxx"
#include "clistdata.hxx"
#include "clistdataitem.hxx"
#include "cwidgetstyle.hxx"
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
   * Forward references
   */

  class CWidgetControl;

  /**
   * Cycle button widget.  Displays text within the button.  Clicking it cycles
   * through its available options.
   */

  class CCycleButton : public CButton, public IListDataEventHandler 
  {
  protected:

    CListData m_options;  /**< Option storage. */

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
     * Draws the outline of the button.
     *
     * @param port Graphics port to draw to.
     */

    virtual void drawOutline(CGraphicsPort *port);

    /**
     * Selects the next option in the list and redraws the button.
     *
     * @param x The x coordinate of the mouse.
     * @param y The y coordinate of the mouse.
     */

    virtual void onRelease(nxgl_coord_t x, nxgl_coord_t y);
    
    /**
     * Redraws the button.
     *
     * @param x The x coordinate of the mouse.
     * @param y The y coordinate of the mouse.
     */

    virtual void onReleaseOutside(nxgl_coord_t x, nxgl_coord_t y);

    /**
     * Prevents the CButton onResize() method from recalculating the text
     * positions by overriding it.
     *
     * @param width The new width.
     * @param height The new height.
     */

    virtual inline void onResize(nxgl_coord_t width, nxgl_coord_t height) { }

    /**
     * Override method in Label class to prevent recalculation of text positions.
     */

    virtual inline void calculateTextPosition(void) { }

    /**
     * Destructor.
     */

    virtual ~CCycleButton(void) { }

    /**
     * Copy constructor is protected to prevent usage.
     */

    inline CCycleButton(const CCycleButton &cycleButton) : CButton(cycleButton) { }

  public:

    /**
     * Constructor for cycle buttons.
     *
     * @param pWidgetControl The widget control for the display.
     * @param x The x coordinate of the button, relative to its parent.
     * @param y The y coordinate of the button, relative to its parent.
     * @param width The width of the button.
     * @param height The height of the button.
     * @param style The style that the button should use.  If this is not
     *   specified, the button will use the values stored in the global
     *   g_defaultWidgetStyle object.  The button will copy the properties of
     *   the style into its own internal style object.
     */

    CCycleButton(CWidgetControl *pWidgetControl,
                 nxgl_coord_t x, nxgl_coord_t y,
                 nxgl_coord_t width, nxgl_coord_t height,
                 CWidgetStyle *style = (CWidgetStyle *)NULL);

    /**
     * Add a new option to the widget.
     *
     * @param text The text of the option.
     * @param value The value of the option.
     */

    void addOption(const CNxString &text, const uint32_t value);

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
     * Select an option by its index.
     * Redraws the widget and raises a value changed event.
     *
     * @param index The index of the option to select.
     */

    virtual void selectOption(const int index);

    /**
     * Get the selected index.  Returns -1 if nothing is selected.  If more than one
     * option is selected, the index of the first selected option is returned.
     *
     * @return The selected index.
     */

    virtual const int getSelectedIndex(void) const;

    /**
     * Sets the selected index.  Specify -1 to select nothing.  Resets any
     * other selected options to deselected.
     * Redraws the widget and raises a value changed event.
     *
     * @param index The selected index.
     */

    virtual void setSelectedIndex(const int index);

    /**
     * Get the selected option.  Returns NULL if nothing is selected.
     *
     * @return The selected option.
     */

    virtual const CListDataItem *getSelectedOption(void) const;

    /**
     * Get the value of the current option.
     *
     * @return Value of the current option.
     */

    inline const uint32_t getValue(void) const
    {
      return getSelectedOption()->getValue();
    }

    /**
     * Get the specified option.
     *
     * @return The specified option.
     */

    virtual inline const CListDataItem *getOption(const int index)
    {
      return m_options.getItem(index);
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
     * Sets whether or not items added to the list are automatically sorted
     * on insert or not.
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
  };
}

#endif // __cplusplus

#endif // __INCLUDE_CCYLEBUTTON_HXX
