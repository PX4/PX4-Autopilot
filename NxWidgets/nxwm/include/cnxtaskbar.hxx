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

#ifndef __NXWM_INCLUDE_CNXTASKBAR_HXX
#define __NXWM_INCLUDE_CNXTASKBAR_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "nxconfig.hxx"

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
   * This class describes the NX window manager's task bar
   */

  class CNxTaskBar : public  NXWidgets::CNxServer, private NXWidgets::CWidgetEventHandler
  {
  private:
    NXWidgets:INxWindow       *m_taskbar;        /**< The toolbar window */
    NXWidgets:INxWindow       *m_background;     /**< The background window */
    INxApplication            *m_start;          /**< The start window */

    /**
     * Connect to the server
     */
 
    bool connect(void);

    /**
     * Disconnect from the server
     */

    void disconnect(void);

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
     * This may be used to provide the window parater to the INxApplication constructor
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
     * Create the toolbar window.
     *
     * @return true on success
     */

    bool createToolbarWindow(void);

    /**
     * Create the background window. 
     *
     * @return true on success
     */

    bool createBackgroundWindow(void);


    /**
     * Handle a mouse button click event.
     *
     * @param e The event data.
     */

    void handleClickEvent(const CWidgetEventArgs &e);

    /**
     * CNxTaskBar Destructor
     */

    ~CNxTaskBar(void);

  public:
    /**
     * CNxTaskBar Constructor
     *
     * @param hWnd - NX server handle
     */

    CNxTaskBar(void);

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

    bool addApplication(INxApplication *application);

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

    bool startWindowManager(start);

    /**
     * Create an application window.  Creating a new applicatino in the start
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

    CApplicationWindow *openApplicationWindow(void);

    /**
     * Start an application and add its icon to the taskbar.  The applications's
     * window is brought to the top.  Creating a new applicatino in the start
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
     *
     * @param application.  The new application to add to the task bar
     * @return true on success
     */

    bool startApplication(INxApplication *application);

    /**
     * Hide an application by moving its window to the bottom.
     *
     * @param application.  The new application to add to the task bar
     * @return true on success
     */

    bool hideApplication(INxApplication *application);

    /**
     * Destroy an application.  Move its window to the bottom and remove its
     * icon from the task bar.
     *
     * @param application.  The new application to remove from the task bar
     * @return true on success
     */

    bool stopApplication(INxApplication *application);
  };
}

#endif // __cplusplus
#endif // __NXWM_INCLUDE_CNXTASKBAR_HXX
