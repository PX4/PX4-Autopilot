/****************************************************************************
 * NxWidgets/nxwm/include/ccalibration.hxx
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
 
#ifndef __INCLUDE_CCALIBRATION_HXX
#define __INCLUDE_CCALIBRATION_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/nx/nxglib.h>

#include <pthread.h>
#include <fixedmath.h>

#include "cnxstring.hxx"
#include "cwidgeteventhandler.hxx"
#include "cwidgetcontrol.hxx"

#include "ctaskbar.hxx"
#include "iapplication.hxx"
#include "cfullscreenwindow.hxx"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/**
 * Calibration indices
 */

#define CALIB_UPPER_LEFT_INDEX  0
#define CALIB_UPPER_RIGHT_INDEX 1
#define CALIB_LOWER_RIGHT_INDEX 2
#define CALIB_LOWER_LEFT_INDEX  3

#define CALIB_DATA_POINTS       4

/****************************************************************************
 * Implementation Classes
 ****************************************************************************/

namespace NxWM
{
  /**
   * Forward references
   */

  struct CTouchscreen;

  /**
   * Touchscreen calibration data
   */

  struct SCalibrationData
  {
    b16_t xSlope;   // X conversion: xSlope*(x) + xOffset
    b16_t xOffset;
    b16_t ySlope;   // Y conversion: ySlope*(y) + yOffset
    b16_t yOffset;
  };

  /**
   * The CCalibration class provides the the calibration window and obtains
   * callibration data.
   */

  class CCalibration : public IApplication
  {
  private:
    /**
     * The state of the calibration thread.
     */

    enum ECalThreadState
    {
      CALTHREAD_NOTRUNNING = 0,                  /**< The calibration thread has not yet been started */
      CALTHREAD_STARTED,                         /**< The calibration thread has been started, but is not yet running */
      CALTHREAD_RUNNING,                         /**< The calibration thread is running normally */
      CALTHREAD_STOPREQUESTED,                   /**< The calibration thread has been requested to stop */
      CALTHREAD_HIDE,                            /**< The hide() called by calibration thread running */
      CALTHREAD_SHOW,                            /**< The redraw() called by calibration thread running */
      CALTHREAD_TERMINATED                       /**< The calibration thread terminated normally */
    };

    /**
     * Identifies the current display state
     */

    enum ECalibrationPhase
    {
      CALPHASE_NOT_STARTED = 0,                  /**< Constructed, but not yet started */
      CALPHASE_UPPER_LEFT,                       /**< Touch point is in the upper left corner */
      CALPHASE_UPPER_RIGHT,                      /**< Touch point is in the upper right corner */
      CALPHASE_LOWER_RIGHT,                      /**< Touch point is in the lower left corner */
      CALPHASE_LOWER_LEFT,                       /**< Touch point is in the lower right corner */
      CALPHASE_COMPLETE                          /**< Calibration is complete */
    };

    /**
     * Characterizes one calibration screen
     */

    struct SCalibScreenInfo
    {
      struct nxgl_point_s      pos;               /**< The position of the touch point */
      nxgl_mxpixel_t           lineColor;         /**< The color of the cross-hair lines */
      nxgl_mxpixel_t           circleFillColor;   /**< The color of the circle */
    };

    /**
     * CCalibration state data
     */

    CTaskbar                  *m_taskbar;         /**< The taskbar (used to terminate calibration) */
    CFullScreenWindow         *m_window;          /**< The window for the calibration display */
    CTouchscreen              *m_touchscreen;     /**< The touchscreen device */
    pthread_t                  m_thread;          /**< The calibration thread ID */
    struct SCalibScreenInfo    m_screenInfo;      /**< Describes the current calibration display */
    struct nxgl_point_s        m_touchPos;        /**< This is the last touch position */
    volatile uint8_t           m_calthread;       /**< Current calibration display state (See ECalibThreadState)*/
    uint8_t                    m_calphase;        /**< Current calibration display state (See ECalibrationPhase)*/
    bool                       m_stop;            /**< True: We have been asked to stop the calibration */
    bool                       m_touched;         /**< True: The screen is touched */
    uint8_t                    m_touchId;         /**< The ID of the touch */
    struct nxgl_point_s        m_calibData[CALIB_DATA_POINTS];

    /**
     * Accept raw touchscreen input.
     *
     * @param sample Touchscreen input sample
     */

    void touchscreenInput(struct touch_sample_s &sample);

    /**
     * Start the calibration thread.
     *
     * @param initialState.  The initial state of the calibration thread
     * @return True if the thread was successfully started.
     */

    bool startCalibration(enum ECalThreadState initialState);

    /**
     *  Return true if the calibration thread is running normally.  There are
     *  lots of potential race conditions.  Let's hope that things are running
     *  orderly and we that we do not have to concern ourself with them
     *
     * @return True if the calibration thread is runnning normally.
     */

    inline bool isRunning(void) const
      {
        // What if the boundary states CALTHREAD_STARTED and CALTHREAD_STOPREQUESTED?
 
        return (m_calthread ==  CALTHREAD_RUNNING ||
                m_calthread ==  CALTHREAD_HIDE ||
                m_calthread ==  CALTHREAD_SHOW);
      }

    /**
     * The calibration thread.  This is the entry point of a thread that provides the
     * calibration displays, waits for input, and collects calibration data.
     *
     * @param arg.  The CCalibration 'this' pointer cast to a void*.
     * @return This function always returns NULL when the thread exits
     */

    static FAR void *calibration(FAR void *arg);

    /**
     * This is the calibration state machine.  It is called initially and then
     * as new touchscreen data is received.
     */

     void stateMachine(void);

    /**
     * Presents the next calibration screen
     */

    void showCalibration(void);

    /**
     * Finish calibration steps and provide the calibration data to the
     * touchscreen driver.
     */

    void finishCalibration(void);

    /**
     * Given the raw touch data collected by the calibration thread, create the
     * massaged calibration data needed by CTouchscreen.
     *
     * @param data. A reference to the location to save the calibration data
     * @return True if the calibration data was successfully created.
     */

    bool createCalibrationData(struct SCalibrationData &data);

  public:

    /**
     * CCalibration Constructor
     *
     * @param taskbar.  The taskbar instance used to terminate calibration
     * @param window.  The window to use for the calibration display
     * @param touchscreen. An instance of the class that wraps the
     *   touchscreen device.
     */

    CCalibration(CTaskbar *taskbar, CFullScreenWindow *window,
                 CTouchscreen *touchscreen);

    /**
     * CCalibration Destructor
     */

    ~CCalibration(void);

    /**
     * Each implementation of IApplication must provide a method to recover
     * the contained IApplicationWindow instance.
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
     * Start the application (perhaps in the minimized state).
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
     * maximized, but not at the top of the hierarchy
     */

    void hide(void);

    /**
     * Redraw the entire window.  The application has been maximized or
     * otherwise moved to the top of the hierarchy.  This method is called from
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
  };

  class CCalibrationFactory : public IApplicationFactory
  {
  private:
    CTaskbar      *m_taskbar;      /**< The taskbar */
    CTouchscreen  *m_touchscreen;  /**< The touchscreen device */

  public:
    /**
     * CCalibrationFactory Constructor
     *
     * @param taskbar.  The taskbar instance used to terminate calibration
     * @param touchscreen. An instance of the class that wraps the
     *   touchscreen device.
     */

    CCalibrationFactory(CTaskbar *taskbar, CTouchscreen *touchscreen);

    /**
     * CCalibrationFactory Destructor
     */

    inline ~CCalibrationFactory(void) { }

    /**
     * Create a new instance of an CCalibration (as IApplication).
     */

    IApplication *create(void);

    /**
     * Get the icon associated with the application
     *
     * @return An instance if IBitmap that may be used to rend the
     *   application's icon.  This is an new IBitmap instance that must
     *   be deleted by the caller when it is no long needed.
     */

    NXWidgets::IBitmap *getIcon(void);
  };
}

#endif // __INCLUDE_CCALIBRATION_HXX
