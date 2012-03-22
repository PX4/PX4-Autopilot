/****************************************************************************
 * NxWidgets/libnxwidgets/include/ckeypad.hxx
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
 ****************************************************************************/

#ifndef __INCLUDE_CKEYPAD_HXX
#define __INCLUDE_CKEYPAD_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/nx/nxglib.h>

#include "cbuttonarray.hxx"

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
  class CWidgetStyle;

  /**
   * Extends the CButtonArray class to support a alphanumeric keypad.
   */

  class CKeypad : public CButtonArray
  {
  protected:
    NXHANDLE m_hNxServer;  /**< NX server handle */
    bool     m_numeric;    /**< True: Numeric keypad, False: Alpha */

    /**
     * Configure the keypad for the currenly selected display mode.
     */

    void configureKeypadMode(void);
 
    /**
     * Copy constructor is protected to prevent usage.
     */

    inline CKeypad(const CKeypad &keypad) : CButtonArray(keypad) { }

  public:

    /**
     * Constructor for buttons that display a string.
     *
     * @param pWidgetControl The widget control for the display.
     * @param hNxServer The NX server that will receive the keyboard input
     * @param x The x coordinate of the keypad, relative to its parent.
     * @param y The y coordinate of the keypad, relative to its parent.
     * @param width The width of the keypad
     * @param height The height of the keypad
     * @param style The style that the button should use.  If this is not
     *        specified, the button will use the global default widget
     *        style.
     */

    CKeypad(CWidgetControl *pWidgetControl, NXHANDLE hNxServer,
              nxgl_coord_t x, nxgl_coord_t y,
              nxgl_coord_t width, nxgl_coord_t height,
              CWidgetStyle *style = (CWidgetStyle *)NULL);

    /**
     * CKeypad Destructor.
     */

    inline ~CKeypad(void) {}

    /**
     * Returns the current keypad display mode
     *
     * @return True: keypad is in numeric mode.  False: alphanumeric.
     */

    inline const bool isNumericKeypad(void) const
    {
      return m_numeric;
    }

    /**
     * Returns the current keypad display mode
     *
     * @param mode True: put keypad in numeric mode.  False: in alphanumeric.
     */

    inline void setKeypadMode(bool numeric)
    {
      m_numeric = numeric;
      configureKeypadMode();
    }

    /**
     * Catch button clicks.
     *
     * @param x The x coordinate of the click.
     * @param y The y coordinate of the click.
     */

    virtual void onClick(nxgl_coord_t x, nxgl_coord_t y);
  };
}

#endif // __cplusplus

#endif // __INCLUDE_CKEYPAD_HXX
