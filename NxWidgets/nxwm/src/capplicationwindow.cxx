/********************************************************************************************
 * NxWidgets/nxwm/src/cnxtaskbar.cxx
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

#include "nxwmconfig.hxx"
#include "cappliationwinow.hxx"

/********************************************************************************************
 * Pre-Processor Definitions
 ********************************************************************************************/

/********************************************************************************************
 * CNxApplicationWindow Method Implementations
 ********************************************************************************************/

 using namespace NxWM;

/**
 * CNxApplicationWindow Constructor
 *
 * @param taskbar.  A pointer to the parent task bar instance
 * @param window.  The window to be used by this application.
 */

CNxApplicationWindow::CNxApplicationWindow(NxWidgets::CNxTkWindow *window);
{
  m_window     = (NxWidgets::CNxTkWindow *)0;
  m_toolbar    = (NxWidgets::CNxToolbar  *)0;
  m_minimize   = (NxWidgets::CImage      *)0;
  m_close      = (NxWidgets::CImage      *)0;
  m_callback   = (IApplicationCallback   *)0;
}

/**
 * CNxApplicationWindow Destructor
 */

CNxApplicationWindow::~CNxApplicationWindow(void)
{
  // Free the resources that we are responsible for

  if (m_toolbar)
    {
      delete m_toolbar;
    }

  if (m_minimize)
    {
      delete m_minimize;
    }

  if (m_close)
    {
      delete m_close;
    }

  // We didn't create the window.  That was done by the task bar,
  // Be we will handle destruction of with window as a courtesy.

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

bool CNxApplicationWindow::open(void)
{

}


