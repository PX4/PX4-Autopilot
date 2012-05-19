/********************************************************************************************
 * NxWidgets/nxwm/src/ctouchscreen.cxx
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

#include <cunistd>
#include <cerrno>
#include <cfcntl>

#include <sched.h>
#include <pthread.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/nx/nxglib.h>

#include "nxconfig.hxx"
#include "cwidgetcontrol.hxx"
#include "cgraphicsport.hxx"

#include "nxwmconfig.hxx"
#include "nxwmglyphs.hxx"
#include "ctouchscreen.hxx"

/********************************************************************************************
 * Pre-Processor Definitions
 ********************************************************************************************/
/* We want debug output from this file if either input/touchscreen or graphics debug is
 * enabled.
 */

#if !defined(CONFIG_DEBUG_INPUT) && !defined(CONFIG_DEBUG_GRAPHICS)
#  undef dbg
#  undef vdbg
#  ifdef CONFIG_CPP_HAVE_VARARGS
#    define dbg(x...)
#    define vdbg(x...)
#  else
#    define dbg  (void)
#    define vdbg (void)
#  endif
#endif

/********************************************************************************************
 * CTouchscreen Method Implementations
 ********************************************************************************************/

using namespace NxWM;

/**
 * CTouchscreen Constructor
 *
 * @param server. An instance of the NX server.  This will be needed for
 *   injecting mouse data.
 * @param windowSize.  The size of the physical window in pixels.  This
 *   is needed for touchscreen scaling.
 */

CTouchscreen::CTouchscreen(NXWidgets::CNxServer *server, struct nxgl_size_s *windowSize)
{
  m_server      = server;              // Save the NX server
  m_touchFd     = -1;                  // Device driver is not opened
  m_state       = LISTENER_NOTRUNNING; // The listener thread is not running yet
  m_enabled     = false;               // Normal forwarding is not enabled
  m_capture     = false;               // There is no thread waiting for touchscreen data
  m_calibrated  = false;               // We have no calibration data

  // Save the window size

  m_windowSize = *windowSize;

  // Use the default touch data buffer

  m_touch       = &m_sample;
  
  // Initialize the m_waitSem semaphore so that any waits for data will block

  sem_init(&m_waitSem, 0, 0);
}

/**
 * CTouchscreen Destructor
 */

CTouchscreen::~CTouchscreen(void)
{
  // Stop the listener thread

  m_state = LISTENER_STOPREQUESTED;

  // Wake up the listener thread so that it will use our buffer
  // to receive data
  // REVISIT:  Need wait here for the listener thread to terminate

  (void)pthread_kill(m_thread, CONFIG_NXWM_TOUCHSCREEN_SIGNO);

  // Close the touchscreen device (or should these be done when the thread exits?)

  if (m_touchFd >= 0)
    {
      std::close(m_touchFd);
    }

   // Destroy the semaphores that we created.
 
   sem_destroy(&m_waitSem);
}

/**
 * Start the touchscreen listener thread.
 *
 * @return True if the touchscreen listener thread was correctly started.
 */

bool CTouchscreen::start(void)
{
  pthread_attr_t attr;

  vdbg("Starting listener\n");

  // Start a separate thread to listen for touchscreen events

  (void)pthread_attr_init(&attr);

  struct sched_param param;
  param.sched_priority = CONFIG_NXWM_TOUCHSCREEN_LISTENERPRIO;
  (void)pthread_attr_setschedparam(&attr, &param);

  (void)pthread_attr_setstacksize(&attr, CONFIG_NXWM_TOUCHSCREEN_LISTENERSTACK);

  m_state  = LISTENER_STARTED; // The listener thread has been started, but is not yet running

  int ret = pthread_create(&m_thread, &attr, listener, (FAR void *)this);
  if (ret != 0)
    {
      dbg("CTouchscreen::start: pthread_create failed: %d\n", ret);
      return false;
    }

  // Detach from the thread

  (void)pthread_detach(m_thread);

  // Don't return until we are sure that the listener thread is running
  // (or until it reports an error).

  while (m_state == LISTENER_STARTED)
    {
      // Wait for the listener thread to wake us up when we really
      // are connected.

      (void)sem_wait(&m_waitSem);
    }

  // Then return true only if the listener thread reported successful
  // initialization.

  vdbg("Listener m_state=%d\n", (int)m_state);
  return m_state == LISTENER_RUNNING;
}

/**
 * Provide touchscreen calibration data.  If calibration data is received (and
 * the touchscreen is enabled), then received touchscreen data will be scaled
 * using the calibration data and forward to the NX layer which dispatches the
 * touchscreen events in window-relative positions to the correct NX window.
 *
 * @param data.  A reference to the touchscreen data.
 */

void CTouchscreen::setCalibrationData(const struct SCalibrationData &caldata)
{
  // Save a copy of the calibration data

  m_calibData = caldata;
 
  // Note that we have calibration data.  Data will now be scaled and forwarded
  // to NX (unless we are still in cpature mode)
 
   m_calibrated = true;

  // Wake up the listener thread so that it will use our buffer
  // to receive data

  (void)pthread_kill(m_thread, CONFIG_NXWM_TOUCHSCREEN_SIGNO);
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

bool CTouchscreen::waitRawTouchData(struct touch_sample_s *touch)
{
  vdbg("Capturing touch input\n");

  // Setup to cpature raw data into the user provided buffer

  sched_lock();
  m_touch   = touch;
  m_capture = true;

  // Wake up the listener thread so that it will use our buffer
  // to receive data

  (void)pthread_kill(m_thread, CONFIG_NXWM_TOUCHSCREEN_SIGNO);

  // And wait for touch data

  int ret = OK;
  while (m_capture)
    {
      ret = sem_wait(&m_waitSem);
      DEBUGASSERT(ret == 0 || errno == EINTR);
    }
  sched_unlock();

  // And return success.  The listener thread will have (1) reset both
  // m_touch and m_capture and (2) posted m_waitSem

  vdbg("Returning touch input: %d\n", ret);
  return ret == OK;
}

/**
 * The touchscreen listener thread.  This is the entry point of a thread that
 * listeners for and dispatches touchscreens events to the NX server.
 *
 * @param arg.  The CTouchscreen 'this' pointer cast to a void*.
 * @return This function normally does not return but may return NULL on
 *   error conditions.
 */

FAR void *CTouchscreen::listener(FAR void *arg)
{
  CTouchscreen *This = (CTouchscreen *)arg;

  vdbg("Listener started\n");

  // Initialize the touchscreen device

  int ret = arch_tcinitialize(CONFIG_NXWM_TOUCHSCREEN_DEVNO);
  if (ret < 0)
    {
      dbg("ERROR Failed initialize the touchscreen device: %d\n", ret);
      This->m_state = LISTENER_FAILED;
      sem_post(&This->m_waitSem);
      return (FAR void *)0;
    }

  // Open the touchscreen device that we just created.

  This->m_touchFd = std::open(CONFIG_NXWM_TOUCHSCREEN_DEVPATH, O_RDONLY);
  if (This->m_touchFd < 0)
    {
      dbg("ERROR Failed to open %s for reading: %d\n",
           CONFIG_NXWM_TOUCHSCREEN_DEVPATH, errno);
      This->m_state = LISTENER_FAILED;
      sem_post(&This->m_waitSem);
      return (FAR void *)0;
    }

  // Indicate that we have successfully initialized

  This->m_state = LISTENER_RUNNING;
  sem_post(&This->m_waitSem);

  // Now loop, reading and dispatching touchscreen data

  while (This->m_state == LISTENER_RUNNING)
    {
      // We may be running in one of three states
      //
      // 1. Disabled or no calibration data:  In this case, just wait for a signal
      //    indicating that the state has changed.
      // 2. Performing calibration and reporting raw touchscreen data
      // 3. Normal operation, reading touchscreen data and forwarding it to NX

      // Check if we need to collect touchscreen data.  That is, that we are enabled,
      // AND have calibratation data OR if we need to collect data for the calbration
      // process.

      while ((!This->m_enabled || !This->m_calibrated) && !This->m_capture)
        {
          // No.. just sleep.  This sleep will be awakened by a signal if there
          // is anything for this thread to do

          sleep(1);

          // We woke up here either because the one second elapsed or because we
          // were signalled.  In either case we need to check the conditions and
          // determine what to do next.
        }

      // We are going to collect a sample..
      //
      // The sample pointer can change dynamically let's sample it once
      // and stick with that pointer.

      struct touch_sample_s *sample = This->m_touch;

      // Read one touchscreen sample

      vdbg("Listening for sample %p\n", sample);
      DEBUGASSERT(sample);
      ssize_t nbytes = read(This->m_touchFd, sample,
                            sizeof(struct touch_sample_s));

      // Check for errors

      if (nbytes < 0)
        {
          // The only expect error is to be interrupt by a signal
#ifdef CONFIG_DEBUG
          int errval = errno;

          dbg("read %s failed: %d\n",
              CONFIG_NXWM_TOUCHSCREEN_DEVPATH, errval);
          DEBUGASSERT(errval == EINTR);
#endif
        }

      // On a truly success read, the size of the returned data will
      // be exactly the size of one touchscreen sample

      else if (nbytes == sizeof(struct touch_sample_s))
        {
          // Looks like good touchscreen input... process it

          This->handleMouseInput(sample);
        }
      else
        {
          dbg("ERROR Unexpected read size=%d, expected=%d\n",
                nbytes, sizeof(struct touch_sample_s));
        }
    }

  // We should get here only if we were asked to terminate via
  // m_state = LISTENER_STOPREQUESTED

  vdbg("Listener exiting\n");
  This->m_state = LISTENER_TERMINATED;
  return (FAR void *)0;
}

/**
 *  Inject touchscreen data into NX as mouse intput
 */

void CTouchscreen::handleMouseInput(struct touch_sample_s *sample)
{
  vdbg("Touch id: %d flags: %02x x: %d y: %d h: %d w: %d pressure: %d\n",
       sample->point[0].id, sample->point[0].flags, sample->point[0].x,
       sample->point[0].y,  sample->point[0].h,     sample->point[0].w,
       sample->point[0].pressure);

  // Verify the touchscreen data

  if (sample->npoints < 1 ||
      ((sample->point[0].flags & TOUCH_POS_VALID) == 0 &&
       (sample->point[0].flags & TOUCH_UP) == 0))
    {
      // The pen is (probably) down, but we have do not have valid
      // X/Y position data to report.  This should not happen.

      return;
    }

  // Was this data captured by some external logic? (probably the
  // touchscreen calibration logic)

  if (m_capture && sample != &m_sample)
    {
      // Yes.. let waitRawTouchData know that the data is available
      // and restore normal buffering

      m_touch   = &m_sample;
      m_capture = false;
      sem_post(&m_waitSem);
      return;
    }

  // Sanity checks.  Re-directed touch data should never reach this point.
  // After posting m_waitSem, m_touch might change asynchronously.

  DEBUGASSERT(sample == &m_sample);

  // Check if normal processing of touchscreen data is enaable.  Check if
  // we have been given calibration data.

  if (!m_enabled || !m_calibrated)
    {
      // No.. we are not yet ready to process touchscreen data (We don't
      // really every get to this condition.

      return;
    }

  // Now we will inject the touchscreen into NX as mouse input.  First
  // massage the data a litle so that it behaves a little more like a
  // mouse with only a left button
  //
  // Was the button up or down?

  uint8_t buttons;
  if ((sample->point[0].flags & (TOUCH_DOWN|TOUCH_MOVE)) != 0)
    {
      buttons = NX_MOUSE_LEFTBUTTON;
    }
  else if ((sample->point[0].flags & TOUCH_UP) != 0)
    {
      buttons = NX_MOUSE_NOBUTTONS;
    }
  else
    {
      // The pen is neither up nor down. This should not happen

      return;
    }

  // Get the "raw" touch coordinates (if they are valid)

  nxgl_coord_t x;
  nxgl_coord_t y;

  if ((sample->point[0].flags & TOUCH_POS_VALID) == 0)
    {
       x = 0;
       y = 0;
    }
  else
    {
      // We have valid coordinates.  Get the raw touch
      // position from the sample

      uint32_t rawX = (uint32_t)sample->point[0].x;
      uint32_t rawY = (uint32_t)sample->point[0].y;

      // Get the fixed precision, scaled X and Y values

      b16_t scaledX = rawX * m_calibData.xSlope + m_calibData.xOffset;
      b16_t scaledY = rawY * m_calibData.ySlope + m_calibData.yOffset;

      // Get integer scaled X and Y positions and clip
      // to fix in the window

      int32_t bigX = b16toi(scaledX + b16HALF);
      int32_t bigY = b16toi(scaledY + b16HALF);

      // Clip to the display

      if (bigX < 0)
        {
          x = 0;
        }
      else if (bigX >= m_windowSize.w)
        {
          x = m_windowSize.w - 1;
        }
      else
        {
          x = (nxgl_coord_t)bigX;
        }

      if (bigY < 0)
        {
          y = 0;
        }
      else if (bigY >= m_windowSize.h)
        {
          y = m_windowSize.h - 1;
        }
      else
        {
          y = (nxgl_coord_t)bigY;
        }

      vdbg("raw: (%d, %d) scaled: (%d, %d)\n", rawX, rawY, x, y);
    }

  // Get the server handle and "inject the mouse data

  NXHANDLE handle = m_server->getServer();
  (void)nx_mousein(handle, x, y, buttons);
}



