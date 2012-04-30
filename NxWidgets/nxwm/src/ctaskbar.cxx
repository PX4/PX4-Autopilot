/********************************************************************************************
 * NxWidgets/nxwm/src/ctaskbar.cxx
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

#include "crect.hxx"
#include "cwidgetcontrol.hxx"
#include "cnxtkwindow.hxx"

#include "nxwmconfig.hxx"
#include "ctaskbar.hxx"

/********************************************************************************************
 * Pre-Processor Definitions
 ********************************************************************************************/

/********************************************************************************************
 * CNxConsole Method Implementations
 ********************************************************************************************/

using namespace NxWM;

/**
 * CTaskbar Constructor
 *
 * @param hWnd - NX server handle
 */

CTaskbar::CTaskbar(void)
{
  m_taskbar       = (NXWidgets::INxWindow *)0;
  m_background    = (NXWidgets::INxWindow *)0;
}

/**
 * CTaskbar Destructor
 */

CTaskbar::~CTaskbar(void)
{
  disconnect();
}

/**
 * Start the window manager and present the initial displays.  The window
 * manager start-up sequence is:
 *
 * 1. Create the CTaskbar instance,
 * 2. Call startApplication repeatedly to add applications to the task bar
 * 3. Call startWindowManager to start the display with applications in place
 *
 * startWindowManager will present the taskar and the background image.  The
 * initial taskbar will contain only the start window icon.
 *
 * @param application.  The new application to add to the start window
 * @return true on success
 */

bool CTaskbar::startWindowManager(void)
{
#warning "Missing logic"
  return false;
}

/**
 * Create an application window.  Creating a new application in the start
 * window requires three steps:
 *
 * 1. Call openApplicationWindow to create a window for the application,
 * 2. Instantiate the application, providing the window to the application's
 *    constructor,
 * 3. Then call addApplication to add the application to the start window.
 *
 * When the application is selected from the start window:
 *
 * 4. Call startApplication start the application and bring its window to
 *    the top.
 */

CApplicationWindow *CTaskbar::openApplicationWindow(void)
{
#warning "Missing logic"
  return (CApplicationWindow *)0;
}

/**
 * Start an application and add its icon to the taskbar.  The applications's
 * window is brought to the top.  Creating a new applicatino in the start
 * window requires three steps:
 *
 * 1. Create the CTaskbar instance,
 * 2. Call startApplication repeatedly to add applications to the task bar
 * 3. Call startWindowManager to start the display with applications in place
 *
 * When the application is selected from the start window:
 *
 * 4. Call startApplication start the application and bring its window to
 *    the top.
 *
 * @param app.  The new application to add to the task bar
 * @param minimized.  The new application starts in the minimized state
 * @return true on success
 */

bool CTaskbar::startApplication(IApplication *app, bool minimized)
{
#warning "Missing logic"
  return false;
}

/**
 * Hide an application by moving its window to the bottom.
 *
 * @param application.  The new application to add to the task bar
 * @return true on success
 */

bool CTaskbar::hideApplication(IApplication *application)
{
  // Every application provides a method to obtain its applicatin window

  CApplicationWindow *appWindow = application->getWindow();

  // Each application window provides a method to get the underlying NX window

  NXWidgets::CNxTkWindow *window = appWindow->getWindow();

  // Lower the window

  window->lower();

  // Grey out the image in task bar
#warning "Missing logic"
  return true;
}

/**
 * Destroy an application.  Move its window to the bottom and remove its
 * icon from the task bar.
 *
 * @param application.  The new application to remove from the task bar
 * @return true on success
 */

bool CTaskbar::stopApplication(IApplication *application)
{
#warning "Missing logic"
  return false;
}

/**
 * Connect to the server
 */
 
bool CTaskbar::connect(void)
{
  // Connect to the server

  bool nxConnected = CNxServer::connect();
  if (nxConnected)
    {
      // Set the background color

      if (!setBackgroundColor(CONFIG_NXWM_DEFAULT_BACKGROUNDCOLOR))
        {
          // Failed
        }
    }

  return nxConnected;
}

/**
 * Disconnect from the server
 */

void CTaskbar::disconnect(void)
{
  // Stop all applications and remove them from the task bar.  Clearly, there 
  // are some ordering issues here... On an orderly system shutdown, disconnection
  // should really occur priority to deleting instances

  while (!m_applications.empty())
    {
       IApplication *app = m_applications.at(0);
       stopApplication(app);
    }

  // Close the windows

  NXWidgets::CWidgetControl *control;
  if (m_taskbar)
    {
      // Delete the contained widget control.  We are responsible for it
      // because we created it

      control = m_taskbar->getWidgetControl();
      if (control)
        {
          delete control;
        }

      // Then delete the toolbar

      delete m_taskbar;
    }

  if (m_background)
    {
      // Delete the contained widget control.  We are responsible for it
      // because we created it

      control = m_background->getWidgetControl();
      if (control)
        {
          delete control;
        }

      // Then delete the background

      delete m_background;
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

NXWidgets::CNxWindow *CTaskbar::openRawWindow(void)
{
  // Initialize the widget control using the default style

  NXWidgets::CWidgetControl *widgetControl = new NXWidgets::CWidgetControl((NXWidgets::CWidgetStyle *)NULL);

  // Get an (uninitialized) instance of the background window as a class
  // that derives from INxWindow.

  NXWidgets::CNxWindow *window = createRawWindow(widgetControl);
  if (!window)
    {
      delete widgetControl;
      return false;
    }

  // Open (and initialize) the window

  bool success = window->open();
  if (!success)
    {
      delete window;
      window = (NXWidgets::CNxWindow *)0;
      return false;
    }

  return window;
}

/**
 * Create a framed application window
 *
 * This may be used to provide the window parater to the IApplication constructor
 *
 * @return A partially initialized application window instance.
 */
 
NXWidgets::CNxTkWindow *CTaskbar::openFramedWindow(void)
{
  // Initialize the widget control using the default style

  NXWidgets::CWidgetControl *widgetControl = new NXWidgets::CWidgetControl((NXWidgets::CWidgetStyle *)NULL);

  // Get an (uninitialized) instance of the background window as a class
  // that derives from INxWindow.

  NXWidgets::CNxTkWindow *window = createFramedWindow(widgetControl);
  if (!window)
    {
      delete widgetControl;
      return false;
    }

  // Open (and initialize) the window

  bool success = window->open();
  if (!success)
    {
      delete window;
      window = (NXWidgets::CNxTkWindow *)0;
      return false;
    }

  return window;
}

/**
 * Set size and position of a window in the application area.
 *
 * @param window.   The window to be resized and repositioned
 *
 * @return true on success
 */

void CTaskbar::setApplicationGeometry(NXWidgets::INxWindow *window)
{
  // Get the widget control from the toolbar window.  The physical window geometry
  // should be the same for all windows.

  NXWidgets::CWidgetControl *control = m_taskbar->getWidgetControl();

  // Get the size of the window from the widget control

  NXWidgets::CRect rect = control->getWindowBoundingBox();

  // Now position and size the application.  This will depend on the position and
  // orientation of the toolbar.

  struct nxgl_point_s pos;
  struct nxgl_size_s  size;

#if defined(CONFIG_NXWM_TASKBAR_TOP)
  pos.x = 0;
  pos.y = CONFIG_NXWM_TASKBAR_WIDTH;

  size.w = rect.getWidth();
  size.h = rect.getWidth() - CONFIG_NXWM_TASKBAR_WIDTH;
#elif defined(CONFIG_NXWM_TASKBAR_BOTTOM)
  pos.x = 0;
  pos.y = 0;

  size.w = rect.getWidth();
  size.h = rect.getWidth() - CONFIG_NXWM_TASKBAR_WIDTH;
#elif defined(CONFIG_NXWM_TASKBAR_LEFT)
  pos.x = CONFIG_NXWM_TASKBAR_WIDTH;
  pos.y = 0;

  size.w = rect.getWidth() - CONFIG_NXWM_TASKBAR_WIDTH;
  size.h = rect.getHeight();
#else
  pos.x = 0;
  pos.y = 0;

  size.w = rect.getWidth() - CONFIG_NXWM_TASKBAR_WIDTH;
  size.h = rect.getHeight();
#endif

  /* Set the size and position the window.
   *
   * @param pPos The new position of the window.
   * @return True on success, false on failure.
   */
     
  window->setPosition(&pos);
  window->setSize(&size);
}

/**
 * Create the toolbar window. 
 *
 * @return true on success
 */

bool CTaskbar::createToolbarWindow(void)
{
  // Create a raw window to present the toolbar

  m_taskbar = openRawWindow();
  if (!m_taskbar)
    {
      return false;
    }

  // Get the contained widget control

  NXWidgets::CWidgetControl *control = m_taskbar->getWidgetControl();

  // Get the size of the window from the widget control

  NXWidgets::CRect rect = control->getWindowBoundingBox();

  // Now position and size the toolbar.  This will depend on the position and
  // orientation of the toolbar.

  struct nxgl_point_s pos;
  struct nxgl_size_s  size;

#if defined(CONFIG_NXWM_TASKBAR_TOP)
  pos.x = 0;
  pos.y = 0;

  size.w = rect.getWidth();
  size.h = CONFIG_NXWM_TASKBAR_WIDTH;
#elif defined(CONFIG_NXWM_TASKBAR_BOTTOM)
  pos.x = 0;
  pos.y = rect.getHeight() - CONFIG_NXWM_TASKBAR_WIDTH;

  size.w = rect.getWidth();
  size.h = CONFIG_NXWM_TASKBAR_WIDTH;
#elif defined(CONFIG_NXWM_TASKBAR_LEFT)
  pos.x = 0;
  pos.y = 0;

  size.w = CONFIG_NXWM_TASKBAR_WIDTH;
  size.h = rect.getHeight();
#else
  pos.x = rect.getWidgth() - CONFIG_NXWM_TASKBAR_WIDTH;
  pos.y = 0;

  size.w = CONFIG_NXWM_TASKBAR_WIDTH;
  size.h = rect.getHeight();
#endif

  /* Set the size and position the window.
   *
   * @param pPos The new position of the window.
   * @return True on success, false on failure.
   */
     
  m_taskbar->setPosition(&pos);
  m_taskbar->setSize(&size);

  /* And raise the window to the top of the display */

  m_taskbar->raise();

  // Add the start menu's icon to the toolbar
#warning "Missing logic"
  return true;
}

/**
 * Create the background window. 
 *
 * @return true on success
 */

bool CTaskbar::createBackgroundWindow(void)
{
  // Create a raw window to present the background image

  m_background = openRawWindow();
  if (!m_background)
    {
      return false;
    }

  // Set the geometry to fit in the application window space

  setApplicationGeometry(static_cast<NXWidgets::INxWindow*>(m_background));

  /* And lower the background window to the bottom of the display */

  m_background->lower();

  return true;
}

/**
 * Handle a mouse button click event.
 *
 * @param e The event data.
 */

void CTaskbar::handleClickEvent(const NXWidgets::CWidgetEventArgs &e)
{
#warning "Missing logic"
}


