/****************************************************************************
 * NxWidgets/libnxwidgets/include/cradiobutton.hxx
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

#ifndef __INCLUDE_CRADIOBUTTON_HXX
#define __INCLUDE_CRADIOBUTTON_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nx.h>

#include "cbutton.hxx"
#include "cwidgetstyle.hxx"

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
  class CRadioButtonGroup;

  /**
   * Represents a radio button.  Radio buttons can only exist as part of a
   * CRadioButtonGroup class, and should not be instantiated individually.
   * Radio buttons are tri-state - off, on and "mu".
   * The mu state cannot be enabled by a user - it can only be set by the
   * developer.
   */
 
  class CRadioButton : public CButton
  {
  public:

    /**
     * Enum listing all possible radio button states.
     */

    enum RadioButtonState
    {
      RADIO_BUTTON_STATE_OFF = 0,     /**< Radio button is off */
      RADIO_BUTTON_STATE_ON  = 1,     /**< Radio button is on */
      RADIO_BUTTON_STATE_MU  = 2      /**< Radio button is in the third state */
    };

  protected:

    RadioButtonState m_state;    /**< The state of the radio button */

    /**
     * Draw the area of this widget that falls within the clipping region.
     * Called by the redraw() function to draw all visible regions.
     *
     * @param port The CGraphicsPort to draw to.
     * @see redraw()
     */

    virtual void drawContents(CGraphicsPort *port);

    /**
     * Sets the radiobutton's state to "on".
     *
     * @param x The x coordinate of the click.
     * @param y The y coordinate of the click.
     */

    virtual void onClick(nxgl_coord_t x, nxgl_coord_t y);

    /**
     * Destructor.
     */
 
    virtual inline ~CRadioButton() { }

    /**
     * Copy constructor is protected to prevent usage.
     */

    inline CRadioButton(const CRadioButton &radioButton) : CButton(radioButton) { }

  public:

    /**
     * Constructor.
     *
     * @param pWidgetControl The controlling widget for the display.
     * @param x The x coordinate of the radio button, relative to its
     * parent.
     * @param y The y coordinate of the radio button, relative to its
     * parent.
     * @param width The width of the radio button.
     * @param height The height of the radio button.
     * @param style The style that the widget should use.  If this is not
     * specified, the widget will use the values stored in the global
     * defaultCWidgetStyle object.  The widget will copy the properties of
     * the style into its own internal style object.
     */

    CRadioButton(CWidgetControl *pWidgetControl,
                 nxgl_coord_t x, nxgl_coord_t y,
                 nxgl_coord_t width, nxgl_coord_t height,
                 CWidgetStyle *style = (CWidgetStyle *)NULL);

    /**
     * Get the current state of the radio button.
     *
     * @return The state of the radio button.
     */

    virtual inline RadioButtonState getState(void)
    {
      return m_state;
    }

    /**
     * Set the state of the radio button.
     *
     * @param state The new radio button state.
     */

    virtual void setState(RadioButtonState state);
  };
}

#endif // __cplusplus

#endif // __INCLUDE_CRADIOBUTTON_HXX
