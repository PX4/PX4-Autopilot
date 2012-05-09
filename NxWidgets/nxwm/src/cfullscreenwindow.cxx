/********************************************************************************************
 * NxWidgets/nxwm/src/cfullscreen.cxx
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
  // Get the window control

  NXWidgets::CWidgetControl *control = m_window->getWidgetControl();

  // Register to receive callbacks on a few select window events

  control->addWindowEventHandler(this);
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
 * Set the window label
 *
 * @param appname.  The name of the application to place on the window
 */

void CFullScreenWindow::setWindowLabel(NXWidgets::CNxString &appname)
{
}

/**
 * Register to receive callbacks when toolbar icons are selected
 */

void CFullScreenWindow::registerCallbacks(IApplicationCallback *callback)
{
}

/**
 * Simulate a mouse click on the minimize icon.  This method is only
 * used during automated testing of NxWM.
 */

#if defined(CONFIG_NXWM_UNITTEST) && !defined(CONFIG_NXWM_TOUCHSCREEN)
void CFullScreenWindow::clickMinimizeIcon(int index)
{
}
#endif

/**
 * Simulate a mouse click on the stop applicaiton icon.  This method is only
 * used during automated testing of NxWM.
 */

#if defined(CONFIG_NXWM_UNITTEST) && !defined(CONFIG_NXWM_TOUCHSCREEN)
void CFullScreenWindow::clickStopIcon(int index)
{
}
#endif

/**
 * Handle an NX window mouse input event.
 *
 * @param e The event data.
 */

#ifdef CONFIG_NX_MOUSE
void CFullScreenWindow::handleMouseEvent(void)
{
  // The logic path here is tortuous but flexible:
  //
  // 1. A listener thread receives mouse input and injects that into NX
  // 2. In the multi-user mode, this will send a message to the NX server
  // 3. The NX server will determine which window gets the mouse input
  //    and send a message to the listener.
  // 4. The listener will call the NX message dispatcher will will do the
  //    message callback.
  // 5. The callback goes into an instance of NXWidgets::CCallback that is
  //    part of the CWidget control.
  // 6. That callback will update mouse information then raise the
  //    mouse event,
  // 7. Which will finally call this function -- still running deep on the
  //    stack in the listener thread.
  // 8. This function will then call back into the wiget control to process
  //    the mouse input.

  // Get the CWidgetControl associated with the window

  NXWidgets::CWidgetControl *control = m_window->getWidgetControl();

  // And perform a poll

  control->pollEvents();
}
#endif

/**
 * Handle a NX window keyboard input event.
 */

#ifdef CONFIG_NX_KBD
void CFullScreenWindow::handleKeyboardEvent(void)
{
  // The logic path here is tortuous but flexible:
  //
  // 1. A listener thread receives keyboard input and injects that into NX
  // 2. In the multi-user mode, this will send a message to the NX server
  // 3. The NX server will determine which window gets the keyboard input
  //    and send a message to the listener.
  // 4. The listener will call the NX message dispatcher will will do the
  //    message callback.
  // 5. The callback goes into an instance of NXWidgets::CCallback that is
  //    part of the CWidget control.
  // 6. That callback will update keyboard information then raise the
  //    keyboard event,
  // 7. Which will finally call this function -- still running deep on the
  //    stack in the listener thread.
  // 8. This function will then call back into the wiget control to process
  //    the keyboard input.

  // Get the CWidgetControl associated with the window

  NXWidgets::CWidgetControl *control = m_window->getWidgetControl();

  // And perform a poll

  control->pollEvents();
}
#endif


