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

#include <debug.h>

#include "tnxarray.hxx"

#include "iapplication.hxx"
#include "capplicationwindow.hxx"
#include "cwindowmessenger.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Abstract Base Classes
 ****************************************************************************/

#if defined(__cplusplus)

namespace NxWM
{
  /**
   * Forward references
   */

  class CTaskbar;

  /**
   * Start window message opcodes and format
   */

  enum EStartWindowMessageOpcodes
  {
    MSGID_POSITIONAL_CHANGE = 1, /**< Change in window positional data (not used) */
    MSGID_REDRAW_REQUEST,        /**< Request to redraw a portion of the window (not used) */
    MSGID_MOUSE_INPUT,           /**< New mouse input is available */
    MSGID_KEYBOARD_INPUT,        /**< New keyboard input is available */
    MSGID_DESTROY_APP            /**< Destroy the application */
  };

  struct SStartWindowMessage
  {
    enum EStartWindowMessageOpcodes msgId;    /**< The message opcode */
    FAR void                       *instance; /**< Object instance. */
  };

  /**
   * The well-known name for the Start Window's message queue.
   */

  extern FAR const char *g_startWindowMqName;
  
  /**
   * This class is the the start window application.
   */

  class CStartWindow : public IApplication,
                       private IApplicationCallback,
                       private NXWidgets::CWidgetEventHandler
  {
  protected:
    /**
     * This structure represents an application and its associated icon image
     */
 
    struct SStartWindowSlot
    {
      IApplicationFactory            *app;      /**< A reference to the icon */
      NXWidgets::CImage              *image;    /**< The icon image that goes with the application */
    };

    /**
     * CStartWindow state data
     */

    CWindowMessenger                  m_messenger; /**< Window event handler/messenger */
    CTaskbar                         *m_taskbar;   /**< Reference to the "parent" taskbar */
    CApplicationWindow               *m_window;    /**< Reference to the application window */
    TNxArray<struct SStartWindowSlot> m_slots;     /**< List of apps in the start window */
    struct nxgl_size_s                m_iconSize;  /**< A box big enough to hold the largest icon */
    pid_t                             m_taskId;    /**< ID of the start window task */

    /**
     * This is the start window task.  This function receives window events from
     * the NX listener threads indirectly through this sequence:
     *
     * 1. The NX listener thread receives a windows event.  The NX listener thread
     *    which is part of CTaskBar and was created when NX server connection was
     *    established).  This event may be a positional change notification, a
     *    redraw request, or mouse or keyboard input.
     * 2. The NX listener thread handles the message by calling nx_eventhandler().
     *    nx_eventhandler() dispatches the message by calling a method in the
     *    NXWidgets::CCallback instance associated with the window.
     *    NXWidgets::CCallback is a part of the CWidgetControl.
     * 3. NXWidgets::CCallback calls into NXWidgets::CWidgetControl to process
     *    the event.
     * 4. NXWidgets::CWidgetControl records the new state data and raises a
     *    window event.
     * 5. NXWidgets::CWindowEventHandlerList will give the event to
     *    NxWM::CWindowMessenger.
     * 6. NxWM::CWindowMessenger will send the a message on a well-known message
     *    queue.
     * 7. This CStartWindow::startWindow task will receive and process that
     *    message.
     */

    static int startWindow(int argc, char *argv[]);

    /**
     * Called when the window minimize button is pressed.
     */

    void minimize(void);

    /**
     * Called when the window close button is pressed.
     */

    void close(void);

    /**
     * Calculate the icon bounding box
     */

     void getIconBounds(void);

    /**
     * Stop all applications
     */

    void removeAllApplications(void);

    /**
     * Handle a widget action event.  For CImage, this is a mouse button pre-release event.
     *
     * @param e The event data.
     */

    void handleActionEvent(const NXWidgets::CWidgetEventArgs &e);

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

    IApplicationWindow *getWindow(void) const;

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
     * Destroy the application and free all of its resources.  This method
     * will initiate blocking of messages from the NX server.  The server
     * will flush the window message queue and reply with the blocked
     * message.  When the block message is received by CWindowMessenger,
     * it will send the destroy message to the start window task which
     * will, finally, safely delete the application.
     */

    void destroy(void);

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
     * Report of this is a "normal" window or a full screen window.  The
     * primary purpose of this method is so that window manager will know
     * whether or not it show draw the task bar.
     *
     * @return True if this is a full screen window.
     */

    bool isFullScreen(void) const;

    /**
     * Add the application to the start window.  The general sequence for
     * setting up the start window is:
     *
     * 1. Call IAppicationFactory::create to a new instance of the application
     * 2. Call CStartWindow::addApplication to add the application to the
     *    start window.
     *
     * @param app.  The new application to add to the start window
     * @return true on success
     */

    bool addApplication(IApplicationFactory *app);

    /**
     * Simulate a mouse click or release on the icon at index.  This method
     * is only available during automated testing of NxWM.
     *
     * @param index.  Selects the icon in the start window
     * @param click.  True to click and false to release
     */

#if defined(CONFIG_NXWM_UNITTEST) && !defined(CONFIG_NXWM_TOUCHSCREEN)
    void clickIcon(int index, bool click);
#endif
  };
}

#endif // __cplusplus

#endif // __INCLUDE_CSTARTWINDOW_NXX
