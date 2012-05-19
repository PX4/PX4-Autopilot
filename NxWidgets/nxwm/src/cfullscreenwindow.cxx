/********************************************************************************************
 * NxWidgets/nxwm/src/cfullscreenwindow.cxx
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
 ********************************************************************************************/

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include <nuttx/nx/nxglib.h>

#include "nxconfig.hxx"
#include "cwidgetcontrol.hxx"
#include "cgraphicsport.hxx"

#include "nxwmconfig.hxx"
#include "nxwmglyphs.hxx"
#include "cfullscreenwindow.hxx"

/********************************************************************************************
 * Pre-Processor Definitions
 ********************************************************************************************/

/********************************************************************************************
 * CFullScreenWindow Method Implementations
 ********************************************************************************************/

using namespace NxWM;

/**
 * CFullScreenWindow Constructor
 *
 * @param window.  The raw window to be used by this application.
 */

CFullScreenWindow::CFullScreenWindow(NXWidgets::CNxWindow *window)
{
  // Save the window for later use

  m_window = window;
}

/**
 * CFullScreenWindow Destructor
 */

CFullScreenWindow::~CFullScreenWindow(void)
{
  // We didn't create the window.  That was done by the task bar,
  // But we will handle destruction of with window as a courtesy.

  if (m_window)
    {
      delete m_window;
    }
}

/**
 * Initialize window.  Window initialization is separate from
 * object instantiation so that failures can be reported.
 *
 * @return True if the window was successfully initialized.
 */

bool CFullScreenWindow::open(void)
{
  return true;
}

/**
 * Re-draw the application window
 */

void CFullScreenWindow::redraw(void)
{
}

/**
 * The application window is hidden (either it is minimized or it is
 * maximized, but not at the top of the hierarchy)
 */

void CFullScreenWindow::hide(void)
{
}

/**
 * Recover the contained raw window instance
 *
 * @return.  The window used by this application
 */

NXWidgets::INxWindow *CFullScreenWindow::getWindow(void) const
{
  return static_cast<NXWidgets::INxWindow*>(m_window);
}

/**
 * Recover the contained widget control
 *
 * @return.  The widget control used by this application
 */

NXWidgets::CWidgetControl *CFullScreenWindow::getWidgetControl(void) const
{
  return m_window->getWidgetControl();
}

/**
 * Block further activity on this window in preparation for window
 * shutdown.
 *
 * @param app. The application to be blocked
 */

void CFullScreenWindow::block(IApplication *app)
{
  // Get the widget control from the NXWidgets::CNxWindow instance

  NXWidgets::CWidgetControl *control = m_window->getWidgetControl();

  // And then block further reporting activity on the underlying
  // NX raw window

  nx_block(control->getWindowHandle(), (FAR void *)app);
}

/**
 * Set the window label
 *
 * @param appname.  The name of the application to place on the window
 */

void CFullScreenWindow::setWindowLabel(NXWidgets::CNxString &appname)
{
}

/**
 * Report of this is a "normal" window or a full screen window.  The
 * primary purpose of this method is so that window manager will know
 * whether or not it show draw the task bar.
 *
 * @return True if this is a full screen window.
 */

bool CFullScreenWindow::isFullScreen(void) const
{
  return true;
}

/**
 * Register to receive callbacks when toolbar icons are selected
 */

void CFullScreenWindow::registerCallbacks(IApplicationCallback *callback)
{
}

/**
 * Simulate a mouse click or release on the minimize icon.  This method
 * is only available for automated testing of NxWM.
 *
 * @param click.  True to click; false to release;
 */

#if defined(CONFIG_NXWM_UNITTEST) && !defined(CONFIG_NXWM_TOUCHSCREEN)
void CFullScreenWindow::clickMinimizePosition(bool click)
{
}
#endif

/**
 * Simulate a mouse click or release on the stop icon.  This method
 * is only available for automated testing of NxWM.
 *
 * @param click.  True to click; false to release;
 */

#if defined(CONFIG_NXWM_UNITTEST) && !defined(CONFIG_NXWM_TOUCHSCREEN)
void CFullScreenWindow::clickStopIcon(bool click)
{
}
#endif


