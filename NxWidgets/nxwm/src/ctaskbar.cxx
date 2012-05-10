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
  m_taskbar       = (NXWidgets::CNxWindow *)0;
  m_background    = (NXWidgets::CNxWindow *)0;
  m_backImage     = (NXWidgets::CImage    *)0;
  m_topApp        = (IApplication         *)0;
  m_started       = false;
}

/**
 * CTaskbar Destructor
 */

CTaskbar::~CTaskbar(void)
{
  disconnect();
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

  while (!m_slots.empty())
    {
       IApplication *app = m_slots.at(0).app;
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

      // Then delete the task bar window

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
 * Initialize task bar.  Task bar initialization is separate from
 * object instantiation so that failures can be reported.  The window
 * manager start-up sequence is:
 *
 * 1. Create the CTaskbar instance,
 * 2. Call the CTaskbar::connect() method to connect to the NX server (CTaskbar
 *    inherits the connect method from CNxServer),
 * 3. Call the CTaskbar::initWindowManager() method to initialize the task bar.
 * 4. Call CTaskBar::startApplication repeatedly to add applications to the task bar
 * 5. Call CTaskBar::startWindowManager to start the display with applications in place
 *
 * CTaskbar::initWindowManager() prepares the task bar to receive applications.
 * CTaskBar::startWindowManager() brings the window manager up with those applications
 * in place.
 *
 * @return True if the window was successfully initialized.
 */

bool CTaskbar::initWindowManager(void)
{
  // Create the taskbar window

  if (!createTaskbarWindow())
    {
      return false;
    }

  // Create the background window

  if (!createBackgroundWindow())
    {
      return false;
    }

  // Create the background image

  if (!createBackgroundImage())
    {
      return false;
    }

  return true;
}

/**
 * Start the window manager and present the initial displays.  The window
 * manager start-up sequence is:
 *
 * 1. Create the CTaskbar instance,
 * 2. Call startApplication repeatedly to add applications to the task bar
 * 3. Call startWindowManager to start the display with applications in place
 *
 * CTaskbar::initWindowManager() prepares the task bar to receive applications.
 * CTaskBar::startWindowManager() brings the window manager up with those applications
 * in place.
 *
 * startWindowManager will present the task bar and the background image.  The
 * initial taskbar will contain only the start window icon.
 *
 * @return true on success
 */

bool CTaskbar::startWindowManager(void)
{
  // Have we already been started

  if (!m_started)
    {
      // We are now started

      m_started = true;

      // Draw the taskbar

     if (!redrawTaskbarWindow())
       {
         return false;
       }

      // Draw the top application window

      return redrawTopWindow();
    }

  return false;
}

/**
 * Create an normal application window.  Creating a normal application in the
 * start window requires three steps:
 *
 * 1. Call CTaskBar::openApplicationWindow to create a window for the application,
 * 2. Instantiate the application, providing the window to the application's
 *    constructor,
 * 3. Then call CStartWindow::addApplication to add the application to the
 *    start window.
 *
 * When the application is selected from the start window:
 *
 * 4. Call CTaskBar::startApplication start the application and bring its window to
 *    the top.
 */

CApplicationWindow *CTaskbar::openApplicationWindow(void)
{
  // Get a framed window for the application

  NXWidgets::CNxTkWindow *window = openFramedWindow();
  if (!window)
    {
      return (CApplicationWindow *)0;
    }

  // Set size and position of a window in the application area.

  setApplicationGeometry(window, false);

  // Use this window to instantiate the application window

  CApplicationWindow *appWindow = new CApplicationWindow(window);
  if (!appWindow)
    {
      delete window;
    }

  return appWindow;
}

/**
 * Create a full screen application window.  Creating a new full screen application
 * requires three steps:
 *
 * 1. Call CTaskBar::FullScreenWindow to create a window for the application,
 * 2. Instantiate the application, providing the window to the application's
 *    constructor,
 * 3. Then call CStartWindow::addApplication to add the application to the
 *    start window.
 *
 * When the application is selected from the start window:
 *
 * 4. Call CTaskBar::startApplication start the application and bring its window to
 *    the top.
 */

CFullScreenWindow *CTaskbar::openFullScreenWindow(void)
{
  // Get a raw window for the application

  NXWidgets::CNxWindow *window = openRawWindow();
  if (!window)
    {
      return (CFullScreenWindow *)0;
    }

  // Set size and position of a window in the application area.

  setApplicationGeometry(window, true);

  // Use this window to instantiate the generia application window

  CFullScreenWindow *appWindow = new CFullScreenWindow(window);
  if (!appWindow)
    {
      delete window;
    }

  return appWindow;
}

/**
 * Start an application and add its icon to the taskbar.  The applications's
 * window is brought to the top.  Creating a new application in the start
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
  // Get the widget control associated with the task bar window

  NXWidgets::CWidgetControl *control =  m_taskbar->getWidgetControl();

  // Get the bitmap icon that goes with this application

  NXWidgets::IBitmap *bitmap = app->getIcon();

  // Create a CImage instance to manage the applications icon

  NXWidgets::CImage *image =
    new NXWidgets::CImage(control, 0, 0, bitmap->getWidth(),
                          bitmap->getHeight(), bitmap, 0);
  if (!image)
    {
      return false;
    }

  // Configure the image, disabling drawing for now

  image->setBorderless(true);
  image->disableDrawing();
  image->setRaisesEvents(false);

  // Register to get events from the mouse clicks on the image

  image->addWidgetEventHandler(this);

  // Add the application to end of the list of applications managed by
  // the task bar

  struct STaskbarSlot slot;
  slot.app   = app;
  slot.image = image;
  m_slots.push_back(slot);

  // Start the application (whatever that means).

  if (!app->run())
    {
      stopApplication(app);
      image->disable();
      return false;
    }

  // Has the window manager been started?  Or were we ask to start
  // the application minimized?

  if (minimized || !m_started)
    {
      // Bring the application up in the minimized state

      hideApplicationWindow(app);
    }
  else
    {
      // Bring up the application as the new top application

      app->setTopApplication(false);
      app->setMinimized(false);
      topApplication(app);
    }

  // Redraw the task bar with the new icon (if we have been started)

  if (m_started)
    {
      redrawTaskbarWindow();
    }

  return true;
}

/**
 * Move window to the top of the hierarchy and re-draw it.  This method
 * does nothing if the application is minimized.
 *
 * @param app.  The new application to show
 * @return true on success
 */

bool CTaskbar::topApplication(IApplication *app)
{
  // Verify that the application is not minimized and is not already the top application

  if (!app->isMinimized() && !app->isTopApplication())
    {
      // It is not minimized.  We are going to bring it to the top of the display.
      // Is there already a top application?

      if (m_topApp)
        {
          // Yes.. then disable it

          hideApplicationWindow(m_topApp);
        }

      // Make the application the top application and redraw it

      return redrawApplicationWindow(app);
    }

  return false;
}

/**
 * Maximize an application by moving its window to the top of the hierarchy
 * and re-drawing it.  If the application was already maximized, then this
 * method is equivalent to topApplication().
 *
 * @param app.  The new application to add to the task bar
 * @return true on success
 */

bool CTaskbar::maximizeApplication(IApplication *app)
{
  // Mark that the application is no longer minimized

  app->setMinimized(false);

  // Then bring the application to the top of the hierarchy

  return topApplication(app);
}

/**
 * Minimize an application by moving its window to the bottom of the and
 * redrawing the next visible appliation.
 *
 * @param app.  The new application to add to the task bar
 * @return true on success
 */

bool CTaskbar::minimizeApplication(IApplication *app)
{
  // Verify that the application is not already minimized

  if (!app->isMinimized())
    {
      // No, then we are going to minimize it but disabling its components,
      // marking it as minized, then raising a new window to the top window.

      hideApplicationWindow(app);

      // Re-draw the new top, non-minimized application

      return redrawTopWindow();
    }

  return false;
}

/**
 * Destroy an application.  Move its window to the bottom and remove its
 * icon from the task bar.
 *
 * @param app.  The new application to remove from the task bar
 * @return true on success
 */

bool CTaskbar::stopApplication(IApplication *app)
{
  // First, minimize the application.  That will move the application
  // to the bottom of the hiearachy and redraw the next application
  // (If the application is already minimized, it does nothing)

  minimizeApplication(app);

  // Stop the application

  app->stop();

  // Find the application in the list of applications

  for (int i = 0; i < m_slots.size(); i++)
    {
      // Is this it?

      IApplication *candidate = m_slots.at(i).app;
      if (candidate == app)
        {
          // Yes.. found it.  Delete the icon image and remove it
          // from the list of applications

          delete m_slots.at(i).image;
          m_slots.erase(i);
          break;
        }
    }

  // And redraw the task bar (without the icon for this task)

  return redrawTaskbarWindow();
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

  // Get an (uninitialized) instance of the framed window as a class
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
 * @param fullscreen.  True: Use full screen
 *
 * @return true on success
 */

void CTaskbar::setApplicationGeometry(NXWidgets::INxWindow *window, bool fullscreen)
{
  // Get the widget control from the task bar window.  The physical window geometry
  // should be the same for all windows.

  NXWidgets::CWidgetControl *control = m_taskbar->getWidgetControl();

  // Get the size of the window from the widget control

  NXWidgets::CRect rect = control->getWindowBoundingBox();

  // Now position and size the application.  This will depend on the position and
  // orientation of the task bar.

  struct nxgl_point_s pos;
  struct nxgl_size_s  size;

  // In fullscreen mode, the application window gets everything

  if (fullscreen)
    {
      pos.x = 0;
      pos.y = 0;

      size.w = rect.getWidth();
      size.h = rect.getHeight();
    }
  else
    {
#if defined(CONFIG_NXWM_TASKBAR_TOP)
      pos.x = 0;
      pos.y = CONFIG_NXWM_TASKBAR_WIDTH;

      size.w = rect.getWidth();
      size.h = rect.getHeight() - CONFIG_NXWM_TASKBAR_WIDTH;
#elif defined(CONFIG_NXWM_TASKBAR_BOTTOM)
      pos.x = 0;
      pos.y = 0;

      size.w = rect.getWidth();
      size.h = rect.getHeight() - CONFIG_NXWM_TASKBAR_WIDTH;
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
    }

  /* Set the size and position the window.
   *
   * @param pPos The new position of the window.
   * @return True on success, false on failure.
   */
     
  window->setPosition(&pos);
  window->setSize(&size);
}

/**
 * Create the task bar window. 
 *
 * @return true on success
 */

bool CTaskbar::createTaskbarWindow(void)
{
  // Create a raw window to present the task bar

  m_taskbar = openRawWindow();
  if (!m_taskbar)
    {
      return false;
    }

  // Get the contained widget control

  NXWidgets::CWidgetControl *control = m_taskbar->getWidgetControl();

  // Get the size of the window from the widget control

  NXWidgets::CRect rect = control->getWindowBoundingBox();

  // Now position and size the task bar.  This will depend on the position and
  // orientation of the task bar.

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

  setApplicationGeometry(static_cast<NXWidgets::INxWindow*>(m_background), false);
  return true;
}

/**
 * Create the background image. 
 *
 * @return true on success
 */

bool CTaskbar::createBackgroundImage(void)
{
 // Get the size of the display

  struct nxgl_size_s windowSize;
  if (!m_background->getSize(&windowSize))
    {
      return false;
    }

  // Get the widget control from the background window

  NXWidgets::CWidgetControl *control = m_background->getWidgetControl();

  // Create the bitmap object

  NXWidgets::CRlePaletteBitmap *bitmap =
    new NXWidgets::CRlePaletteBitmap(&CONFIG_NXWM_BACKGROUND_IMAGE);

  if (!bitmap)
    {
      return false;
    }

  // Get the size of the bitmap image

  struct nxgl_size_s imageSize;
  imageSize.w  = bitmap->getWidth();
  imageSize.h = (nxgl_coord_t)bitmap->getHeight();

  // Pick an X/Y position such that the image will be centered in the display

  struct nxgl_point_s imagePos;
  if (imageSize.w >= windowSize.w)
    {
      imagePos.x = 0;
    }
  else
    {
      imagePos.x = (windowSize.w - imageSize.w) >> 1;
    }

  if (imageSize.h >= windowSize.h)
    {
      imagePos.y = 0;
    }
  else
    {
      imagePos.y = (windowSize.h - imageSize.h) >> 1;
    }

  // Now we have enough information to create the image

  m_backImage = new NXWidgets::CImage(control, imagePos.x, imagePos.y,
                                      imageSize.w, imageSize.h, bitmap);
  if (!m_backImage)
    {
      delete bitmap;
      return false;
    }

  // Configure the background image

  m_backImage->setBorderless(true);
  m_backImage->setRaisesEvents(false);
  return true;
}

/**
 * (Re-)draw the task bar window.
 *
 * @return true on success
 */

bool CTaskbar::redrawTaskbarWindow(void)
{
  // Get the widget control from the task bar

  NXWidgets::CWidgetControl *control = m_taskbar->getWidgetControl();

  // Get the graphics port for drawing on the background window

  NXWidgets::CGraphicsPort *port = control->getGraphicsPort();

  // Get the size of the window

  struct nxgl_size_s windowSize;
  if (!m_taskbar->getSize(&windowSize))
    {
      return false;
    }

  // Raise the task bar to the top of the display.  This is only necessary
  // after stopping a full screen application.  Other applications do not
  // overlap the task bar and, hence, do not interfere.

  m_taskbar->raise();

  // Fill the entire window with the background color

  port->drawFilledRect(0, 0, windowSize.w, windowSize.h,
                       CONFIG_NXWM_DEFAULT_BACKGROUNDCOLOR);

  // Add a border to the task bar to delineate it from the background window

  port->drawBevelledRect(0, 0,  windowSize.w, windowSize.h,
                         CONFIG_NXWM_DEFAULT_SHINEEDGECOLOR,
                         CONFIG_NXWM_DEFAULT_SHADOWEDGECOLOR);

  // Begin adding icons in the upper left corner

  struct nxgl_point_s taskbarPos;
#if defined(CONFIG_NXWM_TASKBAR_TOP) || defined(CONFIG_NXWM_TASKBAR_BOTTOM)
  taskbarPos.x = CONFIG_NXWM_TASKBAR_HSPACING;
  taskbarPos.y = 0;
#else
  taskbarPos.x = 0;
  taskbarPos.y = CONFIG_NXWM_TASKBAR_VSPACING;
#endif

  // Add each icon in the list of applications

  for (int i = 0; i < m_slots.size(); i++)
    {
      // Get the icon associated with this application

      NXWidgets::CImage *image = m_slots.at(i).image;

      // Disable drawing of the icon image; disable events from the icon

      image->disableDrawing();
      image->setRaisesEvents(false);

      // Get the size of the icon image

      NXWidgets::CRect rect;
      image->getPreferredDimensions(rect);

      // Position the icon

      struct nxgl_point_s iconPos;

#if defined(CONFIG_NXWM_TASKBAR_TOP) || defined(CONFIG_NXWM_TASKBAR_BOTTOM)
      // For horizontal task bars, the icons will be aligned at the top of
      // the task bar

      iconPos.x = taskbarPos.x;
      iconPos.y = taskbarPos.y + CONFIG_NXWM_TASKBAR_VSPACING;
#else
      // For vertical task bars, the icons will be centered horizontally

      iconPos.x = (windowSize.w - rect.getWidth()) >> 1;
      iconPos.y = taskbarPos.y;
#endif

      // Set the position of the icon bitmap

     (void)image->moveTo(iconPos.x, iconPos.y);

     // Then re-draw the icon at the new position

     image->enableDrawing();
     image->redraw();
     image->setRaisesEvents(true);

      // Do we add icons left-to-right?  Or top-to-bottom?

#if defined(CONFIG_NXWM_TASKBAR_TOP) || defined(CONFIG_NXWM_TASKBAR_BOTTOM)
      // left-to-right ... increment the X display position

      taskbarPos.x += rect.getWidth() + CONFIG_NXWM_TASKBAR_HSPACING;
      if (taskbarPos.x > windowSize.w)
        {
          break;
        }
#else
      // top-to-bottom ... increment the Y display position

      taskbarPos.y += rect.getHeight() + CONFIG_NXWM_TASKBAR_VSPACING;
      if (taskbarPos.y > windowSize.h)
        {
          break;
        }
#endif
    }

  return true;
}

/**
 * Redraw the window at the top of the heirarchy.
 *
 * @return true on success
 */

bool CTaskbar::redrawTopWindow(void)
{
  // Check if there is already a top application

  IApplication *app = m_topApp;
  if (!app)
    {
      // No.. Search for that last, non-minimized application

      for (int i = m_slots.size() - 1; i >= 0; i--)
        {
          IApplication *candidate = m_slots.at(i).app;
          if (!candidate->isMinimized())
            {
              app = candidate;
              break;
            }
        }
    }

  // Did we find one?

  if (app)
    {
      // Yes.. make it the top application window and redraw it

      return redrawApplicationWindow(app);
      return true;
    }
  else
    {
      // Otherwise, there is no top application.  Re-draw the background image.

      m_topApp = (IApplication *)0;
      return redrawBackgroundWindow();
    }
}

/**
 * (Re-)draw the background window.
 *
 * @return true on success
 */

bool CTaskbar::redrawBackgroundWindow(void)
{
  // Get the widget control from the background window

  NXWidgets::CWidgetControl *control = m_background->getWidgetControl();

  // Get the graphics port for drawing on the background window

  NXWidgets::CGraphicsPort *port = control->getGraphicsPort();

  // Get the size of the window

  struct nxgl_size_s windowSize;
  if (!m_background->getSize(&windowSize))
    {
      return false;
    }

  // Raise the background window to the top of the hierarchy

  m_background->raise();
 
  // Fill the entire window with the background color

  port->drawFilledRect(0, 0, windowSize.w, windowSize.h,
                       CONFIG_NXWM_DEFAULT_BACKGROUNDCOLOR);

  // Add a border to the task bar to delineate it from the task bar

  port->drawBevelledRect(0, 0,  windowSize.w, windowSize.h,
                         CONFIG_NXWM_DEFAULT_SHINEEDGECOLOR,
                         CONFIG_NXWM_DEFAULT_SHADOWEDGECOLOR);

  // Then re-draw the background image on the window

  m_backImage->enableDrawing();
  m_backImage->redraw();
  return true;
}

/**
 * Redraw the last application in the list of application maintained by
 * the task bar.
 *
 * @param app. The new top application to draw
 * @return true on success
 */

bool CTaskbar::redrawApplicationWindow(IApplication *app)
{
  // Mark the window as the top application

  m_topApp = app;
  app->setTopApplication(true);

  // Disable drawing of the background image.

  m_backImage->disableDrawing();

  // Every application provides a method to obtain its application window

  IApplicationWindow *appWindow = app->getWindow();

  // Each application window provides a method to get the underlying NX window

  NXWidgets::INxWindow *window = appWindow->getWindow();

  // Raise the application window to the top of the hierarchy

  window->raise();

  // Re-draw the application window toolbar

  appWindow->redraw();

  // And re-draw the application window itself

  app->redraw();
  return true;
}

/**
 * The application window is hidden (either it is minimized or it is
 * maximized, but not at the top of the hierarchy)
 *
 * @param app. The application to hide
 */

void CTaskbar::hideApplicationWindow(IApplication *app)
{
  // The hidden window is certainly not the top application any longer
  // If it was before then redrawTopWindow() will pick a new one (rather
  // arbitrarily).

  if (app->isTopApplication())
    {
      m_topApp = (IApplication *)0;
      app->setTopApplication(false);
    }

  // Make sure that the application is marked as minimized.

   app->setMinimized(true);

  // We do not need to lower the application to the back.. the new top
  // window will be raised instead.
  //
  // So all that we really have to do is to make sure that all of the
  // components of the hidden window are inactive.

  // Every application provides a method to obtain its application window

  IApplicationWindow *appWindow = app->getWindow();

  // Hide the application window toolbar

  appWindow->hide();

  // The hide the application window itself

  app->hide();
}

/**
 * Handle a mouse button click event.
 *
 * @param e The event data.
 */

void CTaskbar::handleClickEvent(const NXWidgets::CWidgetEventArgs &e)
{
  // icon was clicked?

  for (int i = 0; i < m_slots.size(); i++)
    {
      // Is this it?

      NXWidgets::CImage *image = m_slots.at(i).image;
      if (image->isClicked())
        {
          // Was the icon minimized

          IApplication *app = m_slots.at(i).app;
          if (app->isMinimized())
            {
              // Maximize the application by moving its window to the top of
              // the hierarchy and re-drawing it.

              (void)maximizeApplication(app);
            }

          // No, it is not minimized.  Is it already the top application?

          else if (!app->isTopApplication())
            {
              /* Move window to the top of the hierarchy and re-draw it. */

              (void)topApplication(app);
            }

          // Then break out of the loop

          break;
        }
    }
}
