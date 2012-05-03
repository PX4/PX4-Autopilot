/****************************************************************************
 * NxWidgets/nxwm/include/cnxtaskbar.hxx
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

#ifndef __NXWM_INCLUDE_CTASKBAR_HXX
#define __NXWM_INCLUDE_CTASKBAR_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "nxconfig.hxx"
#include "tnxarray.hxx"
#include "cnxwindow.hxx"
#include "cnxserver.hxx"
#include "cwidgeteventhandler.hxx"
#include "cwidgeteventargs.hxx"

#include "iapplication.hxx"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Implementation Class Definition
 ****************************************************************************/

#if defined(__cplusplus)

namespace NxWM
{
  /**
   * This class describes the NX window manager's task bar.  That task bar is,
   * of course, used to dock active applications.  But in NxWM, it is also
   * the heart of the window manager:  It inherits for CNxServer and, hence,
   * represents the NX server itself.  It also then serves as the NxWM
   * window factory.
   *
   * Why do it this way?  The only reason is so that if you have an instance
   * of CTaskbar, you have everything you need to manage new applications.
   * It might have been a better decision to separate the window factory into
   * a separate class instead of making the task bar of such central importance
   * (and I may still do that someday)
   */

  class CTaskbar : public  NXWidgets::CNxServer, private NXWidgets::CWidgetEventHandler
  {
  private:
    /**
     * This structure represents an application and its associated icon image
     */
 
    struct STaskbarSlot
    {
      IApplication      *app;   /**< A reference to the icon */
      NXWidgets::CImage *image; /**< The icon image that goes with the application */
    };

    /**
     * Taskbar state
     */

    NXWidgets::CNxWindow         *m_taskbar;    /**< The task bar window */
    NXWidgets::CNxWindow         *m_background; /**< The background window */
    NXWidgets::CImage            *m_backImage;  /**< The background image */
    IApplication                 *m_topApp;     /**< The top application in the hierarchy */
    TNxArray<struct STaskbarSlot> m_slots;      /**< List of application slots in the task bar */
    bool                          m_started;    /**< True if window manager has been started */

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

    NXWidgets::CNxWindow *openRawWindow(void);

    /**
     * Create a framed application window
     *
     * This may be used to provide the window parater to the IApplication constructor
     *
     * @return A partially initialized application window instance.
     */
 
    NXWidgets::CNxTkWindow *openFramedWindow(void);

    /**
     * Set size and position of a window in the application area.
     *
     * @param window.   The window to be resized and repositioned
     */

    void setApplicationGeometry(NXWidgets::INxWindow *window);

    /**
     * Create the task bar window.
     *
     * @return true on success
     */

    bool createTaskbarWindow(void);

    /**
     * Create the background window. 
     *
     * @return true on success
     */

    bool createBackgroundWindow(void);

    /**
     * Create the background image. 
     *
     * @return true on success
     */

    bool createBackgroundImage(void);

    /**
     * (Re-)draw the task bar window.
     *
     * @return true on success
     */

    bool redrawTaskbarWindow(void);

    /**
     * Redraw the window at the top of the heirarchy.
     *
     * @return true on success
     */

    bool redrawTopWindow(void);

    /**
     * (Re-)draw the background window.
     *
     * @return true on success
     */

    bool redrawBackgroundWindow(void);

    /**
     * Redraw the last application in the list of application maintained by
     * the task bar.
     *
     * @param app. The new top application to draw
     * @return true on success
     */

    bool redrawApplicationWindow(IApplication *app);

    /**
     * The application window is hidden (either it is minimized or it is
     * maximized, but not at the top of the hierarchy)
     *
     * @param app. The application to hide
     */

    void hideApplicationWindow(IApplication *app);

    /**
     * Handle a mouse button click event.
     *
     * @param e The event data.
     */

    void handleClickEvent(const NXWidgets::CWidgetEventArgs &e);

  public:
    /**
     * CTaskbar Constructor
     *
     * @param hWnd - NX server handle
     */

    CTaskbar(void);

    /**
     * CTaskbar Destructor
     */

    ~CTaskbar(void);

    /**
     * Connect to the server
     */
 
    bool connect(void);

    /**
     * Disconnect from the server
     */

    void disconnect(void);

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
     * 5. Call CTaskBar::startWindowManager() to start the display with applications in place
     *
     * CTaskbar::initWindowManager() prepares the task bar to receive applications.
     * CTaskBar::startWindowManager() brings the window manager up with those applications
     * in place.
     *
     * @return True if the window was successfully initialized.
     */

    bool initWindowManager(void);

    /**
     * Start the window manager and present the initial displays.  The window
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
     * CTaskBar::startWindowManager() will present the task bar and the background image.
     * The The initial taskbar will contain only the start window icon.
     *
     * @return true on success
     */

    bool startWindowManager(void);

    /**
     * Create an application window.  Creating a new application in the start
     * window requires three steps:
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

    CApplicationWindow *openApplicationWindow(void);

    /**
     * Start an application and add its icon to the taskbar.  The applications's
     * window is brought to the top.  Creating a new application in the start
     * window requires three steps:
     *
     * 1. Call CTaskBar::openApplicationWindow to create a window for the application,
     * 2. Instantiate the application, providing the window to the application's
     *    constructor,
     * 3. Then call CStartWindow::addApplication to add the application to the start window.
     *
     * When the application is selected from the start window:
     *
     * 4. Call CTaskBar::startApplication start the application and bring its window to
     *    the top.
     *
     * @param app.  The new application to add to the task bar
     * @param minimized.  The new application starts in the minimized state
     * @return true on success
     */

    bool startApplication(IApplication *app, bool minimized);

    /**
     * Move window to the top of the hierarchy and re-draw it.  This method
     * does nothing if the application is minimized.
     *
     * @param app.  The new application to show
     * @return true on success
     */

    bool topApplication(IApplication *app);

    /**
     * Maximize an application by moving its window to the top of the hierarchy
     * and re-drawing it.  If the application was already maximized, then this
     * method is equivalent to topApplication().
     *
     * @param app.  The new application to add to the task bar
     * @return true on success
     */

    bool maximizeApplication(IApplication *app);

    /**
     * Minimize an application by moving its window to the bottom of the
     * and redrawing the next visible appliation.
     *
     * @param app.  The new application to add to the task bar
     * @return true on success
     */

    bool minimizeApplication(IApplication *app);

    /**
     * Destroy an application.  Move its window to the bottom and remove its
     * icon from the task bar.
     *
     * @param app.  The new application to remove from the task bar
     * @return true on success
     */

    bool stopApplication(IApplication *app);

    /**
     * Simulate a mouse click on the icon at index.  This inline method is only
     * used during automated testing of NxWM.
     */

    inline void clickIcon(int index)
    {
      if (index < m_slots.size())
       {
         // Get the image widget at this index

         NXWidgets::CImage *image = m_slots.at(index).image;

         // Get the size and position of the widget

         struct nxgl_size_s imageSize;
         image->getSize(imageSize);

         struct nxgl_point_s imagePos;
         image->getPos(imagePos);

         // And click the image at its center

         image->click(imagePos.x + (imageSize.w >> 1), imagePos.y + (imageSize.h >> 1));
       }
    }
  };
}

#endif // __cplusplus
#endif // __NXWM_INCLUDE_CTASKBAR_HXX
