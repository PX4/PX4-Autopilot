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
#include "cnxconsole.hxx"

/********************************************************************************************
 * Pre-Processor Definitions
 ********************************************************************************************/

/********************************************************************************************
 * CNxConsole Method Implementations
 ********************************************************************************************/

/**
 * CNxTaskBar Constructor
 *
 * @param hWnd - NX server handle
 */

CNxTaskBar::CNxTaskBar(void)
{
  m_toolbar       = (INxWindow *)NULL;
  m_background    = (INxWindow *)NULL;
  m_start         = (INxWindow *)NULL;
}

/**
 * CNxTaskBar Destructor
 */

CNxTaskBar::~CNxTaskBar(void)
{
  disconnect();
}
/**
 * Add the application to the start window.  The window manager start-up
 * sequence is:
 *
 * 1. Create the CNxTaskBar instance,
 * 2. Call addApplication repeatedly to add applications to the start window
 * 3. Call startWindowManager to start the display
 *
 * @param application.  The new application to add to the start window
 * @return true on success
 */

bool CNxTaskBar::addApplication(INxApplication *application)
{
}

/**
 * Start the window manager and present the initial displays.  The window
 * manager start-up sequence is:
 *
 * 1. Create the CNxTaskBar instance,
 * 2. Call addApplication repeatedly to add applications to the start window
 * 3. Call startWindowManager to start the display
 *
 * startWindowManager will present the taskar and the background image.  The
 * initial taskbar will contain only the start window icon.
 *
 * @param application.  The new application to add to the start window
 * @return true on success
 */

bool CNxTaskBar::startWindowManager(start);

/**
 * Start an application and add its icon to the taskbar
 *
 * @param application.  The new application to add to the task bar
 * @return true on success
 */

bool CNxTaskBar::startApplication(INxApplication *application)
{
}

/**
 * Hide an application by moving its window to the bottom.
 *
 * @param application.  The new application to add to the task bar
 * @return true on success
 */

bool CNxTaskBar::hideApplication(INxApplication *application)
{
}

/**
 * Destroy an application.  Move its window to the bottom and remove its
 * icon from the task bar.
 *
 * @param application.  The new application to remove from the task bar
 * @return true on success
 */

bool CNxTaskBar::stopApplication(INxApplication *application)
{
}

/**
 * Connect to the server
 */
 
bool CNxTaskBar::connect(void)
{
  // Connect to the server

  bool nxConnected = CNxServer::connect();
  if (nxConnected)
    {
      // Set the background color

      if (!setBackgroundColor(CONFIG_CNXWM_BGCOLOR))
        {
          message("CNxwm::connect: setBackgroundColor failed\n");
        }
    }

  return nxConnected;
}

/**
 * Disconnect from the server
 */

void CNxTaskBar::disconnect(void)
{
  // Delete all applications
#warning "Missing logic

  // Close the windows

  if (m_toolbar)
    {
      delete m_toolbar;
    }

  if (m_background)
    {
      delete m_background;
    }

  if (m_start)
    {
      delete m_start;
    }

  // And disconnect from the server

  CNxServer::disconnect();
}

/**
 * Create a raw window. 
 *
 * 1) Create a dumb CWigetControl instance
 * 2) Pass the dumb CWidgetControl instance to the window constructor
 *    that inherits from INxWindow.  This will "smarten" the CWidgetControl
 *    instance with some window knowlede
 * 3) Call the open() method on the window to display the window.
 * 4) After that, the fully smartened CWidgetControl instance can
 *    be used to generate additional widgets by passing it to the
 *    widget constructor
 */

NXWidgets::INxWindow *CNxTaskBar::openRawWindow(void)
{
  // Initialize the widget control using the default style

  NxWidgets::CWidgetControl *widgetControl = new CWidgetControl((CWidgetStyle *)NULL);

  // Get an (uninitialized) instance of the background window as a class
  // that derives from INxWindow.

  NxWidgets::CNxWindow window = createRawWindow(widgetControl);
  if (!window)
    {
      message("CNxwm::createGraphics: Failed to create background window\n");
      delete widgetControl;
      return false;
    }

  // Open (and initialize) the window

  bool success = window->open();
  if (!success)
    {
      message("CNxwm::createGraphics: Failed to open background window\n");
      delete window;
      window = (NXWidgets::INxWindow *)0;
      return false;
    }

  return static_cast<INxWindow*>(window);
}

/**
 * Create a framed application window
 *
 * This may be used to provide the window parater to the INxApplication constructor
 *
 * @return A partially initialized application window instance.
 */
 
NXWidgets::INxWindow *CNxTaskBar::openFramedWindow(void)
{
  // Initialize the widget control using the default style

  NxWidgets::CWidgetControl *widgetControl = new CWidgetControl((CWidgetStyle *)NULL);

  // Get an (uninitialized) instance of the background window as a class
  // that derives from INxWindow.

  NxWidgets:CFramedWindow window = createRawWindow(widgetControl);
  if (!window)
    {
      message("CNxwm::createGraphics: Failed to create background window\n");
      delete widgetControl;
      return false;
    }

  // Open (and initialize) the window

  bool success = window->open();
  if (!success)
    {
      message("CNxwm::createGraphics: Failed to open background window\n");
      delete window;
      window = (NXWidgets::INxWindow *)0;
      return false;
    }

  return static_cast<INxWindow*>(window);
}

/**
 * Create the toolbar window. 
 *
 * @return true on success
 */

bool CNxTaskBar::createToolbarWindow(void)
{
  // Create a raw window to present the toolbar

  m_toolbar = openRawWindow();
  if (!m_toolbar)
    {
      message("CNxwm::createGraphics: Failed to create CBgWindow instance\n");
      return false;
    }

  return true;
}

/**
 * Create the background window. 
 *
 * @return true on success
 */

bool CNxTaskBar::createBackgroundWindow(void)
{
  // Create a raw window to present the background image

  m_background = openRawWindow();
  if (!m_background)
    {
      message("CNxwm::createGraphics: Failed to create background window\n");
      return false;
    }

  return true;
}

/**
 * Create the background window. 
 *
 * @return true on success
 */

bool CNxTaskBar::createStartWindow(void)
{
  // Create a raw window to present the background image

  INxWindow *window = openFramedWindow();
  if (!window)
    {
      message("CNxwm::createGraphics: Failed to create start window\n");
      return false;
    }

  // Now create the start up application
#warning "Missing logic"

  // m_start
  return true;
}
