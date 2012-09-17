/****************************************************************************
 * NxWidgets/libnxwidgets/include/cradiobuttongroup.hxx
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

#ifndef __INCLUDE_CRADIOBUTTONGROUP_HXX
#define __INCLUDE_CRADIOBUTTONGROUP_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nx.h>

#include "cnxwidget.hxx"
#include "cbutton.hxx"
#include "cwidgetstyle.hxx"
#include "cwidgeteventhandler.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
 
/****************************************************************************
 * Implementation Classes
 ****************************************************************************/

#if defined(__cplusplus)

namespace NXWidgets
{
  class CWidgetControl;
  class CRadioButton;

  /**
   * Container class that holds radio button widgets and tracks their status.
   * The group provides an easy way to determine which radio button is
   * selected.  Note that, in order to set the "mu" state for a radio button,
   * it is necessary to set the state via the radio button, not the group.
   */

  class CRadioButtonGroup : public CNxWidget, public CWidgetEventHandler 
  {
  protected:
    CWidgetControl *m_pWidgetControl; /**< The controlling widget */
    CRadioButton   *m_selectedWidget; /**< Pointer to the currently selected radio button */

    /**
     * Draw the area of this widget that falls within the clipping region.
     * Called by the redraw() function to draw all visible regions.
     * @param port The CGraphicsPort to draw to.
     * @see redraw()
     */

    virtual void drawContents(CGraphicsPort* port);

    /**
     * Copy constructor is protected to prevent usage.
     */

    inline CRadioButtonGroup(const CRadioButtonGroup &radioButtonGroup) : CNxWidget(radioButtonGroup) { }

  public:

    /**
     * Constructor.  Note that the group determines its width and height
     * from the position and dimensions of its children.
     *
     * @param pWidgetControl The controlling widget for the display.
     * @param x The x coordinate of the group.
     * @param y The y coordinate of the group.
     * @param style The style that the button should use.  If this is not
     *        specified, the button will use the global default widget
     *        style.
     */

    CRadioButtonGroup(CWidgetControl *pWidgetControl,
                      nxgl_coord_t x, nxgl_coord_t y,
                      CWidgetStyle *style = (CWidgetStyle *)NULL);

    /**
     * Destructor.
     */

    ~CRadioButtonGroup(void) { }

    /**
     * Simple method for adding a new radio button to the group.
     * This should be used in preference to the usual addWidget() method,
     * as this method automatically resizes the group.
     *
     * @param x The x coordinate of the new button, relative to this
     * widget.
     * @param y The y coordinate of the new button, relative to this
     * widget.
     * @param width The width of the new button.
     * @param height The height of the new button.
     */

    CRadioButton *newRadioButton(nxgl_coord_t x, nxgl_coord_t y,
                                 nxgl_coord_t width, nxgl_coord_t height);

    /**
     * Gets a pointer to the selected widget.
     *
     * @return Pointer to the selected widget.
     */

    virtual const CRadioButton *getSelectedWidget(void) const;

    /**
     * Gets the index of the selected widget.
     *
     * @return The index of the selected widget.
     */

    virtual const int getSelectedIndex(void) const;

    /**
     * Sets the selected radio button to the supplied widget.
     *
     * @param widget The radio button to select.
     */

    virtual void setSelectedWidget(CRadioButton *widget);

    /**
     * Selects the widget at the specified index.
     *
     * @param index The index of the widget to select.
     */

    virtual void setSelectedIndex(int index);

    /**
     * Insert the dimensions that this widget wants to have into the rect
     * passed in as a parameter.  All coordinates are relative to the
     * widget's parent.  Value is based on the length of the largest string
     * in the set of options.
     *
     * @param rect Reference to a rect to populate with data.
     */

    virtual void getPreferredDimensions(CRect &rect) const;

    /**
     * Handle a mouse click event.
     *
     * @param e The event data.
     */

    virtual void handleClickEvent(const CWidgetEventArgs &e);

    /**
     * Handle a mouse double-click event.
     *
     * @param e The event data.
     */

    virtual void handleDoubleClickEvent(const CWidgetEventArgs &e);

    /**
     * Handle a mouse button release event that occurred within the bounds of
     * the source widget.
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
  };
}

#endif // __cplusplus

#endif // __INCLUDE_CRADIOBUTTONGROUP_HXX
