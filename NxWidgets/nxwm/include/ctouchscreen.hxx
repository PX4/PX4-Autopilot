/****************************************************************************
 * NxWidgets/nxwm/include/ctouchscreen.hxx
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
 
#ifndef __INCLUDE_CTOUCHSCREEN_HXX
#define __INCLUDE_CTOUCHSCREEN_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/nx/nxglib.h>

#include <semaphore.h>
#include <pthread.h>

#include <nuttx/input/touchscreen.h>

#include "cnxserver.hxx"
#include "ccalibration.hxx"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Implementation Classes
 ****************************************************************************/

namespace NxWM
{
  /**
   * The CTouchscreen class provides the the calibration window and obtains
   * callibration data.
   */

  class CTouchscreen
  {
  private:
    /**
     * The state of the listener thread.
     */

    enum EListenerState
    {
      LISTENER_NOTRUNNING = 0,  /**< The listener thread has not yet been started */
      LISTENER_STARTED,         /**< The listener thread has been started, but is not yet running */
      LISTENER_RUNNING,         /**< The listener thread is running normally */
      LISTENER_STOPREQUESTED,   /**< The listener thread has been requested to stop */
      LISTENER_TERMINATED,      /**< The listener thread terminated normally */
      LISTENER_FAILED           /**< The listener thread terminated abnormally */
    };

    /**
     * CTouchscreen state data
     */

    NXWidgets::CNxServer        *m_server;     /**< The current NX server */
    int                          m_touchFd;    /**< File descriptor of the opened touchscreen device */
    sem_t                        m_waitSem;    /**< Semaphore the supports waits for touchscreen data */
    pthread_t                    m_thread;     /**< The listener thread ID */
    volatile enum EListenerState m_state;      /**< The state of the listener thread */
    volatile bool                m_enabled;    /**< True: Normal touchscreen processing */
    volatile bool                m_capture;    /**< True: There is a thread waiting for raw touch data */
    volatile bool                m_calibrated; /**< True: If have calibration data */
    struct nxgl_size_s           m_windowSize; /**< The size of the physical display */
    struct SCalibrationData      m_calibData;  /**< Calibration data */
    struct touch_sample_s        m_sample;     /**< In normal mode, touch data is collected here */
    struct touch_sample_s       *m_touch;      /**< Points to the current touch data buffer */

    /**
     * The touchscreen listener thread.  This is the entry point of a thread that
     * listeners for and dispatches touchscreens events to the NX server.
     *
     * @param arg.  The CTouchscreen 'this' pointer cast to a void*.
     * @return This function normally does not return but may return NULL on
     *   error conditions.
     */

    static FAR void *listener(FAR void *arg);

    /**
     *  Inject touchscreen data into NX as mouse intput
     *
     * @param sample.  The buffer where data was collected.
     */

    void handleMouseInput(struct touch_sample_s *sample);

  public:

    /**
     * CTouchscreen Constructor
     *
     * @param server. An instance of the NX server.  This will be needed for
     *   injecting mouse data.
     * @param windowSize.  The size of the physical window in pixels.  This
     *   is needed for touchscreen scaling.
     */

    CTouchscreen(NXWidgets::CNxServer *server, struct nxgl_size_s *windowSize);

    /**
     * CTouchscreen Destructor
     */

    ~CTouchscreen(void);

    /**
     * Start the touchscreen listener thread.
     *
     * @return True if the touchscreen listener thread was correctly started.
     */

    bool start(void);

    /**
     * Enable/disable touchscreen data processing.  When enabled, touchscreen events
     * are calibrated and forwarded to the NX layer which dispatches the touchscreen
     * events in window-relative positions to the correct NX window.
     *
     * When disabled, touchscreen data is not forwarded to NX, but is instead captured
     * and made available for touchscreen calibration.  The touchscreen driver is
     * initially disabled and must be specifically enabled be begin normal processing.
     * Normal processing also requires calibration data (see method setCalibrationData)
     *
     * @param capture.  True enables capture mode; false disables.
     */

    inline void setEnabled(bool enable)
    {
      // Set the capture flag.  m_calibrated must also be set to get to normal
      // mode where touchscreen data is forwarded to NX.
 
      m_enabled = enable;
    }

    /**
     * Is the touchscreen calibrated?
     *
     * @return True if the touchscreen has been calibrated.
     */

    inline bool isCalibrated(void) const
    {
      return m_calibrated;
    }

    /**
     * Provide touchscreen calibration data.  If calibration data is received (and
     * the touchscreen is enabled), then received touchscreen data will be scaled
     * using the calibration data and forward to the NX layer which dispatches the
     * touchscreen events in window-relative positions to the correct NX window.
     *
     * @param data.  A reference to the touchscreen data.
     */

    void setCalibrationData(const struct SCalibrationData &caldata);

    /**
     * Recover the calibration data so that it can be saved to non-volatile storage.
     *
     * @param data.  A reference to the touchscreen data.
     * @return True if calibration data was successfully returned.
     */

    inline bool getCalibrationData(struct SCalibrationData &caldata) const
    {
      if (m_calibrated)
        {
          caldata = m_calibData;
        }
      return m_calibrated;
    }

    /**
     * Capture raw driver data.  This method will capture mode one raw touchscreen
     * input.  The normal use of this method is for touchscreen calibration.
     *
     * This function is not re-entrant:  There may be only one thread waiting for
     * raw touchscreen data.
     *
     * @return True if the raw touchscreen data was sucessfully obtained
     */

    bool waitRawTouchData(struct touch_sample_s *touch);
  };
}

#endif // __INCLUDE_CTOUCHSCREEN_HXX
