/****************************************************************************
 * NxWidgets/nxwm/include/cnxstart.hxx
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

#ifndef __INCLUDE_CSTARTWINDOW_NXX
#define __INCLUDE_CSTARTWINDOW_NXX

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include "tnxarray.hxx"

#include "iapplication.hxx"
#include "capplicationwindow.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Abstract Base Classes
 ****************************************************************************/

#if defined(__cplusplus)

namespace NxWM
{
  class CTaskbar;

  class CStartWindow : public IApplication, private IApplicationCallback,
                       private NXWidgets::CWidgetEventHandler
  {
  protected:
    /**
     * This structure represents an application and its associated icon image
     */
 
    struct SStartWindowSlot
    {
      IApplication                   *app;      /**< A reference to the icon */
      NXWidgets::CImage              *image;    /**< The icon image that goes with the application */
    };

    /**
     * CStartWindow state data
     */

    CTaskbar                         *m_taskbar;  /**< Reference to the "parent" taskbar */
    CApplicationWindow               *m_window;   /**< Reference to the application window */
    TNxArray<struct SStartWindowSlot> m_slots;    /**< List of apps in the start window */
    struct nxgl_size_s                m_iconSize; /**< A box big enough to hold the largest icon */

    /**
     * Called when the window minimize button is pressed.
     */

    void minimize(void);

    /**
     * Called when the window minimize close is pressed.
     */

    void close(void);

    /**
     * Calculate the icon bounding box
     */

     void getIconBounds(void);

    /**
     * Stop all applications
     */

    void stopAllApplications(void);

    /**
     * Handle a mouse button click event.
     *
     * @param e The event data.
     */

    void handleClickEvent(const NXWidgets::CWidgetEventArgs &e);

  public:

    /**
     * CStartWindow Constructor
     *
     * @param taskbar.  A pointer to the parent task bar instance
     * @param window.  The window to be used by this application.
     */

    CStartWindow(CTaskbar *taskbar, CApplicationWindow *window);

    /**
     * CStartWindow Constructor
     */

    ~CStartWindow(void);

    /**
     * Each implementation of IApplication must provide a method to recover
     * the contained CApplicationWindow instance.
     */

    CApplicationWindow *getWindow(void) const;

    /**
     * Get the icon associated with the application
     *
     * @return An instance if IBitmap that may be used to rend the
     *   application's icon.  This is an new IBitmap instance that must
     *   be deleted by the caller when it is no long needed.
     */

    NXWidgets::IBitmap *getIcon(void);

    /**
     * Get the name string associated with the application
     *
     * @return A copy if CNxString that contains the name of the application.
     */

    NXWidgets::CNxString getName(void);

    /**
     * Start the application.
     *
     * @return True if the application was successfully started.
     */

    bool run(void);

    /**
     * Stop the application.
     */

    void stop(void);

    /**
     * The application window is hidden (either it is minimized or it is
     * maximized, but not at the top of the hierarchy)
     */

    void hide(void);

    /**
     * Redraw the entire window.  The application has been maximized or
     * otherwise moved to the top of the hiearchy.  This method is call from
     * CTaskbar when the application window must be displayed
     */

    void redraw(void);

    /**
     * Add the application to the start window.  The general sequence for
     * setting up the start window is:
     *
     * 1. Call CTaskBar::openApplicationWindow to create a window for the start window,
     * 2. Use the window to instantiate CStartWindow
     * 3. Call CStartWindow::addApplication numerous times to install applications
     *    in the start window.
     * 4. Call CTaskBar::startApplication (initially minimized) to start the start
     *    window application.
     *
     * @param app.  The new application to add to the start window
     * @return true on success
     */

    bool addApplication(IApplication *app);

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

#endif // __INCLUDE_CSTARTWINDOW_NXX
