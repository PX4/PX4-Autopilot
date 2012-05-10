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

#include <fixedmath.h>

#include "cnxstring.hxx"
#include "cwidgeteventhandler.hxx"
#include "cwidgetcontrol.hxx"

#include "iapplication.hxx"
#include "cfullscreenwindow.hxx"
#include "ctouchscreen.hxx"

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
     * Identifies the current display state
     */

    enum ECalibState
    {
      CALIB_NOT_STARTED = 0,                      /**< Constructed, but not yet started */
      CALIB_UPPER_LEFT,                           /**< Touch point is in the upper left corner */
      CALIB_UPPER_RIGHT,                          /**< Touch point is in the upper right corner */
      CALIB_LOWER_RIGHT,                          /**< Touch point is in the lower left corner */
      CALIB_LOWER_LEFT,                           /**< Touch point is in the lower right corner */
      CALIB_COMPLETE                              /**< Calibration is complete */
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

    CFullScreenWindow         *m_window;          /**< The window for the calibration display */
    CTouchscreen              *m_touchscreen;     /**< The touchscreen device */
    enum ECalibState           m_state;           /**< Current calibration display state */
    struct SCalibScreenInfo    m_screenInfo;      /**< Describes the current calibration display */
    struct nxgl_point_s        m_touchPos;        /**< This is the last touch position */
    bool                       m_stop;            /**< True: We have been asked to stop the calibration */
    bool                       m_touched;         /**< True: The screen is touched */
    sem_t                      m_waitSem;         /**< Supports wait for calibration data */
    struct nxgl_point_s        m_calibData[CALIB_DATA_POINTS];

    /**
     * Accept raw touchscreen input.
     *
     * @param sample Touchscreen input sample
     */

    void touchscreenInput(struct touch_sample_s &sample);

    /**
     * This is the calibration state machine.  It is called initially and then
     * as new touchscreen data is received.
     */

     void stateMachine(void);

    /**
     * Presents the next calibration screen
     */

    void showCalibration(void);

  public:

    /**
     * CCalibration Constructor
     *
     * @param window.  The window to use for the calibration display
     * @param touchscreen. An instance of the class that wraps the touchscreen device.
     */

    CCalibration(CFullScreenWindow *window, CTouchscreen *touchscreen);

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
     * Wait for calibration data to be received.
     *
     * @return True if the calibration data was successfully obtained.
     */

    bool waitCalibrationData(struct SCalibrationData &data);
  };
}

#endif // __INCLUDE_CCALIBRATION_HXX
