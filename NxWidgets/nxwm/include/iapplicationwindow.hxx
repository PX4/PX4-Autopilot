/****************************************************************************
 * NxWidgets/nxwm/include/iapplicationwindow.hxx
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

#ifndef __INCLUDE_IAPPLICATIONWINDOW_NXX
#define __INCLUDE_IAPPLICATIONWINDOW_NXX

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include "cnxtkwindow.hxx"
#include "cnxtoolbar.hxx"
#include "cwidgeteventargs.hxx"
#include "cwidgeteventhandler.hxx"
#include "cimage.hxx"
#include "clabel.hxx"
#include "crlepalettebitmap.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Abstract Base Class
 ****************************************************************************/

#if defined(__cplusplus)

namespace NxWM
{
  /**
   * This callback class is used by the application to get notification of toolbar
   * related events.
   */

  class IApplicationCallback
  {
  public:
    /**
     * Called when the window minimize button is pressed.
     */

    virtual void minimize(void) = 0;

    /**
     * Called when the window minimize close is pressed.
     */

    virtual void close(void) = 0;
  };

  /**
   * This class represents the general application window.  The actual window
   * may be a contained, framed window or and unframed, fullscreen window.
   */

  class IApplicationWindow 
  {
  public:
    /**
     * Initialize window.  Window initialization is separate from
     * object instantiation so that failures can be reported.
     *
     * @return True if the window was successfully initialized.
     */

    virtual bool open(void) = 0;

    /**
     * Re-draw the application window
     */

    virtual void redraw(void) = 0;

    /**
     * The application window is hidden (either it is minimized or it is
     * maximized, but not at the top of the hierarchy)
     */

    virtual void hide(void) = 0;

    /**
     * Recover the contained window instance
     *
     * @return.  The window used by this application
     */

    virtual NXWidgets::INxWindow *getWindow(void) const = 0;

    /**
     * Set the window label
     *
     * @param appname.  The name of the application to place on the window
     */

    virtual void setWindowLabel(NXWidgets::CNxString &appname) = 0;

    /**
     * Report of this is a "normal" window or a full screen window.  The
     * primary purpose of this method is so that window manager will know
     * whether or not it show draw the task bar.
     *
     * @return True if this is a full screen window.
     */

    virtual bool isFullScreen(void) const = 0;

    /**
     * Register to receive callbacks when toolbar icons are selected
     */

    virtual void registerCallbacks(IApplicationCallback *callback) = 0;

    /**
     * Simulate a mouse click on the minimize icon.  This inline method is only
     * used during automated testing of NxWM.
     */

#if defined(CONFIG_NXWM_UNITTEST) && !defined(CONFIG_NXWM_TOUCHSCREEN)
    virtual void clickMinimizeIcon(int index) = 0;
#endif

    /**
     * Simulate a mouse click on the stop applicaiton icon.  This inline method is only
     * used during automated testing of NxWM.
     */

#if defined(CONFIG_NXWM_UNITTEST) && !defined(CONFIG_NXWM_TOUCHSCREEN)
    virtual void clickStopIcon(int index) = 0;
#endif
  };
}

#endif // __cplusplus

#endif // __INCLUDE_IAPPLICATIONWINDOW_NXX
