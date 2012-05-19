/****************************************************************************
 * NxWidgets/nxwm/src/ccalibration.cxx
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <cunistd>
#include <cerrno>

#include <sched.h>
#include <assert.h>
#include <debug.h>

#include "nxwmconfig.hxx"
#include "nxwmglyphs.hxx"
#include "ctouchscreen.hxx"
#include "ccalibration.hxx"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/****************************************************************************
 * Configuration
 */

#ifndef CONFIG_NX
#  error "NX is not enabled (CONFIG_NX)"
#endif

/**
 * Positional/size data for the calibration lines and circles
 */

#define CALIBRATION_LEFTX              40
#define CALIBRATION_RIGHTX             (windowSize.w - 41)
#define CALIBRATION_TOPY               40
#define CALIBRATION_BOTTOMY            (windowSize.h - 41)

#define CALIBRATION_CIRCLE_RADIUS      16
#define CALIBRATION_LINE_THICKNESS     2

/****************************************************************************
 * CCalibration Implementation Classes
 ****************************************************************************/

using namespace NxWM;

/**
 * CCalibration Constructor
 *
 * @param taskbar.  The taskbar instance used to terminate calibration
 * @param window.  The window to use for the calibration display
 * @param touchscreen. An instance of the class that wraps the touchscreen
 *   device.
 */

CCalibration::CCalibration(CTaskbar *taskbar, CFullScreenWindow *window,
                           CTouchscreen *touchscreen)
{
  // Initialize state data

  m_taskbar       = taskbar;
  m_window        = window;
  m_touchscreen   = touchscreen;
  m_thread        = 0;
  m_calthread     = CALTHREAD_NOTRUNNING;
  m_calphase      = CALPHASE_NOT_STARTED;
  m_touched       = false;
}

/**
 * CCalibration Destructor
 */

CCalibration::~CCalibration(void)
{
  // Make sure that the application is not running (it should already
  // have been stopped)

  stop();

  // Although we did not create the window, the rule is that I have to dispose
  // of it

  delete m_window;
}

/**
 * Each implementation of IApplication must provide a method to recover
 * the contained IApplicationWindow instance.
 */

IApplicationWindow *CCalibration::getWindow(void) const
{
  return static_cast<IApplicationWindow*>(m_window);
}

/**
 * Get the icon associated with the application
 *
 * @return An instance if IBitmap that may be used to rend the
 *   application's icon.  This is an new IBitmap instance that must
 *   be deleted by the caller when it is no long needed.
 */

NXWidgets::IBitmap *CCalibration::getIcon(void)
{
  NXWidgets::CRlePaletteBitmap *bitmap =
    new NXWidgets::CRlePaletteBitmap(&CONFIG_NXWM_CALIBRATION_ICON);

  return bitmap;
}

/**
 * Get the name string associated with the application
 *
 * @return A copy if CNxString that contains the name of the application.
 */

NXWidgets::CNxString CCalibration::getName(void)
{
  return NXWidgets::CNxString("Touchscreen Calibration");
}

/**
 * Start the application (perhaps in the minimized state).
 *
 * @return True if the application was successfully started.
 */

bool CCalibration::run(void)
{
  gvdbg("Starting calibration: m_calthread=%d\n", (int)m_calthread);

  return startCalibration(CALTHREAD_STARTED);
}

/**
 * Stop the application.
 */

void CCalibration::stop(void)
{
  gvdbg("Stopping calibration: m_calthread=%d\n", (int)m_calthread);

  // Was the calibration thread created?

  if (m_thread != 0)
    {
      // Is the calibration thread running? 

      if (isRunning())
        {
          // The main thread is stuck waiting for the next touchscreen input...
          // We can signal that we would like the thread to stop, but we will be
          // stuck here until the next touch

          m_calthread = CALTHREAD_STOPREQUESTED;

          // Try to wake up the calibration thread so that it will see our
          // terminatin request

         gvdbg("Stopping calibration: m_calthread=%d\n", (int)m_calthread);
          (void)pthread_kill(m_thread, CONFIG_NXWM_CALIBRATION_SIGNO);
        }
    }
}

/**
 * Destroy the application and free all of its resources.  This method
 * will initiate blocking of messages from the NX server.  The server
 * will flush the window message queue and reply with the blocked
 * message.  When the block message is received by CWindowMessenger,
 * it will send the destroy message to the start window task which
 * will, finally, safely delete the application.
 */

void CCalibration::destroy(void)
{
  // Make sure that the application is stopped (should already be stopped)

  stop();

  // Block any further window messages

  m_window->block(this);
}

/**
 * The application window is hidden (either it is minimized or it is
 * maximized, but it is not at the top of the hierarchy)
 */

void CCalibration::hide(void)
{
  gvdbg("Entry\n");

  // Is the calibration thread running?

  if (m_calthread == CALTHREAD_RUNNING || m_calthread == CALTHREAD_SHOW)
    {
      // Ask the calibration thread to hide the display

      m_calthread = CALTHREAD_HIDE;
      (void)pthread_kill(m_thread, CONFIG_NXWM_CALIBRATION_SIGNO);
    }
}

/**
 * Redraw the entire window.  The application has been maximized or
 * otherwise moved to the top of the hierarchy.  This method is called from
 * CTaskbar when the application window must be displayed
 */

void CCalibration::redraw(void)
{
  gvdbg("Entry\n");

  // Is the calibration thread running?  We might have to restart it if
  // we have completed the calibration early but are being brought to
  // top of the display again

  // Is the calibration thread running?

  if (!isRunning())
    {
      gvdbg("Starting calibration: m_calthread=%d\n", (int)m_calthread);
      (void)startCalibration(CALTHREAD_SHOW);
    }

  // The calibration thread is running.  Make sure that is is not
  // already processing a redraw

  else if (m_calthread != CALTHREAD_SHOW)
    {
      // Ask the calibration thread to restart the calibration and redraw the display

      m_calthread = CALTHREAD_SHOW;
      (void)pthread_kill(m_thread, CONFIG_NXWM_CALIBRATION_SIGNO);
    }
}

/**
 * Report of this is a "normal" window or a full screen window.  The
 * primary purpose of this method is so that window manager will know
 * whether or not it show draw the task bar.
 *
 * @return True if this is a full screen window.
 */

bool CCalibration::isFullScreen(void) const
{
  return m_window->isFullScreen();
}

/**
 * Accept raw touchscreen input.
 *
 * @param sample Touchscreen input sample
 */

void CCalibration::touchscreenInput(struct touch_sample_s &sample)
{
  // Is this a new touch event?  Or is it a drag event?

  if ((sample.point[0].flags & (TOUCH_DOWN|TOUCH_MOVE)) != 0)
    {
      // Yes.. but ignore drag events if we did not see the matching
      // touch down event

      if ((sample.point[0].flags & TOUCH_DOWN) != 0 ||
          (m_touched && sample.point[0].id == m_touchId))
        {
          // Yes.. save the touch position and wait for the TOUCH_UP report

          m_touchPos.x = sample.point[0].x;
          m_touchPos.y = sample.point[0].y;

          gvdbg("Touch id: %d flags: %02x x: %d y: %d h: %d w: %d pressure: %d\n",
                sample.point[0].id, sample.point[0].flags, sample.point[0].x,
                sample.point[0].y,  sample.point[0].h,     sample.point[0].w,
                sample.point[0].pressure);

          // Show calibration screen again, changing the color of the circle to
          // make it clear that the touch has been noticed.

          if (!m_touched)
            {
              m_screenInfo.circleFillColor = CONFIG_NXWM_CALIBRATION_TOUCHEDCOLOR;
              showCalibration();
              m_touched = true;
            }

          // Remember the ID of the touch down event

          m_touchId = sample.point[0].id;
        }
    }

  // Was the touch released?

  else if ((sample.point[0].flags & TOUCH_UP) != 0)
    {
      // Yes.. did we see the pen down event?

      if (m_touched)
        {
          // Yes.. For the matching touch ID?

          if (sample.point[0].id == m_touchId)
            {
              // Yes.. invoke the state machine.

              gvdbg("State: %d Screen x: %d y: %d  Touch x: %d y: %d\n",
                    m_calphase, m_screenInfo.pos.x, m_screenInfo.pos.y,
                    m_touchPos.x, m_touchPos.y);

              stateMachine();
            }
          else
            {
              // No... restore the un-highlighted circle
 
              m_screenInfo.circleFillColor = CONFIG_NXWM_CALIBRATION_CIRCLECOLOR;
              showCalibration();
            }
        }

      // In any event, the screen is not touched

      m_touched = false;
    }
}

/**
 * Start the calibration thread.
 *
 * @param initialState.  The initial state of the calibration thread
 * @return True if the thread was successfully started.
 */

bool CCalibration::startCalibration(enum ECalThreadState initialState)
{

  // Verify that the thread is not already running

  if (isRunning())
    {
      gdbg("The calibration thread is already running\n");
      return false;
    }

  // Configure the calibration thread

  pthread_attr_t attr;
  (void)pthread_attr_init(&attr);

  struct sched_param param;
  param.sched_priority = CONFIG_NXWM_CALIBRATION_LISTENERPRIO;
  (void)pthread_attr_setschedparam(&attr, &param);

  (void)pthread_attr_setstacksize(&attr, CONFIG_NXWM_CALIBRATION_LISTENERSTACK);

  // Set the initial state of the thread

  m_calthread = initialState;

  // Start the thread that will perform the calibration process

  int ret = pthread_create(&m_thread, &attr, calibration, (FAR void *)this);
  if (ret != 0)
    {
      gdbg("pthread_create failed: %d\n", ret);
      return false;
    }

  // Detach from the pthread so that we do not have any memory leaks

  (void)pthread_detach(m_thread);

  gvdbg("Calibration thread m_calthread=%d\n", (int)m_calthread);
  return true;
}

/**
 * The calibration thread.  This is the entry point of a thread that provides the
 * calibration displays, waits for input, and collects calibration data.
 *
 * @param arg.  The CCalibration 'this' pointer cast to a void*.
 * @return This function always returns NULL when the thread exits
 */

FAR void *CCalibration::calibration(FAR void *arg)
{
  CCalibration *This = (CCalibration *)arg;
  bool stalled = true;

  // The calibration thread is now running

  This->m_calthread = CALTHREAD_RUNNING;
  This->m_calphase  = CALPHASE_NOT_STARTED;
  gvdbg("Started: m_calthread=%d\n", (int)This->m_calthread);
  
  // Loop until calibration completes or we have been requested to terminate

  while (This->m_calthread != CALTHREAD_STOPREQUESTED &&
         This->m_calphase != CALPHASE_COMPLETE)
    {
      // Check for state changes due to display order changes

      if (This->m_calthread == CALTHREAD_HIDE)
        {
          // This state is set by hide() when our display is no longer visible

          This->m_calthread = CALTHREAD_RUNNING;
          This->m_calphase  = CALPHASE_NOT_STARTED;
          stalled           = true;
        }
      else if (This->m_calthread == CALTHREAD_SHOW)
        {
          // This state is set by redraw() when our display has become visible

          This->m_calthread = CALTHREAD_RUNNING;
          This->m_calphase  = CALPHASE_NOT_STARTED;
          stalled           = false;
          This->stateMachine();
        }

      // The calibration thread will stall if has been asked to hide the
      // display.  While stalled, we will just sleep for a bit abd test
      // the state again.  If we are re-awakened by a redraw(), then we
      // will be given a signal which will wake us up immediately.
      //
      // Note that stalled is also initially true so we have to receive
      // redraw() before we attempt to draw anything

      if (stalled)
        {
          // Sleep for a while (or until we receive a signal)

          std::usleep(500*1000);
        }
      else
        {
          // Wait for the next raw touchscreen input (or possibly a signal)

          struct touch_sample_s sample;
          while (!This->m_touchscreen->waitRawTouchData(&sample) &&
                  This->m_calthread == CALTHREAD_RUNNING);

          // Then process the raw touchscreen input

          if (This->m_calthread == CALTHREAD_RUNNING)
            {
              This->touchscreenInput(sample);
            } 
        }
    }

  // Perform the final steps of calibration

  This->finishCalibration();

  gvdbg("Terminated: m_calthread=%d\n", (int)This->m_calthread);
  return (FAR void *)0;
}

/**
 * This is the calibration state machine.  It is called initially and then
 * as new touchscreen data is received.
 */

void CCalibration::stateMachine(void)
{
  gvdbg("Old m_calphase=%d\n", m_calphase);

  // Recover the window instance contained in the full screen window

  NXWidgets::INxWindow *window = m_window->getWindow();

  // Get the size of the fullscreen window

  struct nxgl_size_s windowSize;
  if (!window->getSize(&windowSize))
    {
      return;
    }

  switch (m_calphase)
    {
      default:
      case CALPHASE_NOT_STARTED:
        {
          // Clear the entire screen
          // Get the widget control associated with the full screen window

          NXWidgets::CWidgetControl *control =  window->getWidgetControl();

          // Get the CCGraphicsPort instance for this window

          NXWidgets::CGraphicsPort *port = control->getGraphicsPort();

          // Fill the entire window with the background color

          port->drawFilledRect(0, 0, windowSize.w, windowSize.h,
                               CONFIG_NXWM_DEFAULT_BACKGROUNDCOLOR);

          // Then draw the first calibration screen

          m_screenInfo.pos.x           = CALIBRATION_LEFTX;
          m_screenInfo.pos.y           = CALIBRATION_TOPY;
          m_screenInfo.lineColor       = CONFIG_NXWM_CALIBRATION_LINECOLOR;
          m_screenInfo.circleFillColor = CONFIG_NXWM_CALIBRATION_CIRCLECOLOR;
          showCalibration();

          // Then set up the current state

          m_calphase = CALPHASE_UPPER_LEFT;
        }
        break;

      case CALPHASE_UPPER_LEFT:
        {
          // A touch has been received while in the CALPHASE_UPPER_LEFT state.
          // Save the touch data and set up the next calibration display

          m_calibData[CALIB_UPPER_LEFT_INDEX].x = m_touchPos.x;
          m_calibData[CALIB_UPPER_LEFT_INDEX].y = m_touchPos.y;

          // Clear the previous screen by re-drawing it using the backgro9und
          // color.  That is much faster than clearing the whole display

          m_screenInfo.lineColor       = CONFIG_NXWM_CALIBRATION_BACKGROUNDCOLOR;
          m_screenInfo.circleFillColor = CONFIG_NXWM_CALIBRATION_BACKGROUNDCOLOR;
          showCalibration();

          // Then draw the next calibration screen

          m_screenInfo.pos.x           = CALIBRATION_RIGHTX;
          m_screenInfo.pos.y           = CALIBRATION_TOPY;
          m_screenInfo.lineColor       = CONFIG_NXWM_CALIBRATION_LINECOLOR;
          m_screenInfo.circleFillColor = CONFIG_NXWM_CALIBRATION_CIRCLECOLOR;
          showCalibration();

          // Then set up the current state

          m_calphase = CALPHASE_UPPER_RIGHT;
        }
        break;

      case CALPHASE_UPPER_RIGHT:
        {
          // A touch has been received while in the CALPHASE_UPPER_RIGHT state.
          // Save the touch data and set up the next calibration display

          m_calibData[CALIB_UPPER_RIGHT_INDEX].x = m_touchPos.x;
          m_calibData[CALIB_UPPER_RIGHT_INDEX].y = m_touchPos.y;

          // Clear the previous screen by re-drawing it using the backgro9und
          // color.  That is much faster than clearing the whole display

          m_screenInfo.lineColor       = CONFIG_NXWM_CALIBRATION_BACKGROUNDCOLOR;
          m_screenInfo.circleFillColor = CONFIG_NXWM_CALIBRATION_BACKGROUNDCOLOR;
          showCalibration();

          // Then draw the next calibration screen

          m_screenInfo.pos.x           = CALIBRATION_RIGHTX;
          m_screenInfo.pos.y           = CALIBRATION_BOTTOMY;
          m_screenInfo.lineColor       = CONFIG_NXWM_CALIBRATION_LINECOLOR;
          m_screenInfo.circleFillColor = CONFIG_NXWM_CALIBRATION_CIRCLECOLOR;
          showCalibration();

          // Then set up the current state

          m_calphase = CALPHASE_LOWER_RIGHT;
        }
        break;

      case CALPHASE_LOWER_RIGHT:
        {
          // A touch has been received while in the CALPHASE_LOWER_RIGHT state.
          // Save the touch data and set up the next calibration display

          m_calibData[CALIB_LOWER_RIGHT_INDEX].x = m_touchPos.x;
          m_calibData[CALIB_LOWER_RIGHT_INDEX].y = m_touchPos.y;

          // Clear the previous screen by re-drawing it using the backgro9und
          // color.  That is much faster than clearing the whole display

          m_screenInfo.lineColor       = CONFIG_NXWM_CALIBRATION_BACKGROUNDCOLOR;
          m_screenInfo.circleFillColor = CONFIG_NXWM_CALIBRATION_BACKGROUNDCOLOR;
          showCalibration();

          // Then draw the next calibration screen

          m_screenInfo.pos.x           = CALIBRATION_LEFTX;
          m_screenInfo.pos.y           = CALIBRATION_BOTTOMY;
          m_screenInfo.lineColor       = CONFIG_NXWM_CALIBRATION_LINECOLOR;
          m_screenInfo.circleFillColor = CONFIG_NXWM_CALIBRATION_CIRCLECOLOR;
          showCalibration();

          // Then set up the current state

          m_calphase = CALPHASE_LOWER_LEFT;
        }
        break;

      case CALPHASE_LOWER_LEFT:
        {
          // A touch has been received while in the CALPHASE_LOWER_LEFT state.
          // Save the touch data and set up the next calibration display

          m_calibData[CALIB_LOWER_LEFT_INDEX].x = m_touchPos.x;
          m_calibData[CALIB_LOWER_LEFT_INDEX].y = m_touchPos.y;

          // Clear the previous screen by re-drawing it using the backgro9und
          // color.  That is much faster than clearing the whole display

          m_screenInfo.lineColor       = CONFIG_NXWM_CALIBRATION_BACKGROUNDCOLOR;
          m_screenInfo.circleFillColor = CONFIG_NXWM_CALIBRATION_BACKGROUNDCOLOR;
          showCalibration();

          // Inform any waiter that calibration is complete

          m_calphase = CALPHASE_COMPLETE;
        }
        break;

      case CALPHASE_COMPLETE:
        // Might happen... do nothing if it does
        break;
    }

  gvdbg("New m_calphase=%d Screen x: %d y: %d\n",
        m_calphase, m_screenInfo.pos.x, m_screenInfo.pos.y);
}

/**
 * Presents the next calibration screen
 *
 * @param screenInfo Describes the next calibration screen
 */

void CCalibration::showCalibration(void)
{
  // Recover the window instance contained in the full screen window

  NXWidgets::INxWindow *window = m_window->getWindow();

  // Get the widget control associated with the full screen window

  NXWidgets::CWidgetControl *control =  window->getWidgetControl();

  // Get the CCGraphicsPort instance for this window

  NXWidgets::CGraphicsPort *port = control->getGraphicsPort();

  // Get the size of the fullscreen window

  struct nxgl_size_s windowSize;
  if (!window->getSize(&windowSize))
    {
      return;
    }

  // Draw the circle at the center of the touch position

  port->drawFilledCircle(&m_screenInfo.pos, CALIBRATION_CIRCLE_RADIUS,
                          m_screenInfo.circleFillColor);

  /* Draw horizontal line */
  
  port->drawFilledRect(0, m_screenInfo.pos.y, windowSize.w, CALIBRATION_LINE_THICKNESS,
                       m_screenInfo.lineColor);

  /* Draw vertical line */
  
  port->drawFilledRect(m_screenInfo.pos.x, 0, CALIBRATION_LINE_THICKNESS, windowSize.h,
                       m_screenInfo.lineColor);
}

/**
 * Finish calibration steps and provide the calibration data to the
 * touchscreen driver.
 */

void CCalibration::finishCalibration(void)
{
  // Did we finish calibration successfully?

  if (m_calphase == CALPHASE_COMPLETE)
    {
      // Yes... Get the final Calibration data
 
      struct SCalibrationData caldata;
      if (createCalibrationData(caldata))
        {
          // And provide this to the touchscreen, enabling touchscreen processing

          m_touchscreen->setEnabled(false);
          m_touchscreen->setCalibrationData(caldata);
          m_touchscreen->setEnabled(true);
        }
    }

  // Remove the touchscreen application from the taskbar

  m_taskbar->stopApplication(this);

  // And set the terminated stated

  m_calthread = CALTHREAD_TERMINATED;
}

/**
 * Given the raw touch data collected by the calibration thread, create the
 * massaged calibration data needed by CTouchscreen.
 *
 * @param data. A reference to the location to save the calibration data
 * @return True if the calibration data was successfully created.
 */

bool CCalibration::createCalibrationData(struct SCalibrationData &data)
{
  // Recover the window instance contained in the full screen window

  NXWidgets::INxWindow *window = m_window->getWindow();

  // Get the size of the fullscreen window

  struct nxgl_size_s windowSize;
  if (!window->getSize(&windowSize))
    {
      gdbg("NXWidgets::INxWindow::getSize failed\n"); 
      return false;
    }

  // Calculate the calibration parameters
  //
  // (scaledX - LEFTX) / (rawX - leftX) = (RIGHTX - LEFTX) / (rightX - leftX)
  // scaledX = (rawX - leftX) * (RIGHTX - LEFTX) / (rightX - leftX) + LEFTX
  //         = rawX * xSlope + (LEFTX - leftX * xSlope)
  //         = rawX * xSlope + xOffset
  //
  // where:
  // xSlope  = (RIGHTX - LEFTX) / (rightX - leftX)
  // xOffset = (LEFTX - leftX * xSlope)

  b16_t leftX  = (m_calibData[CALIB_UPPER_LEFT_INDEX].x +
                  m_calibData[CALIB_LOWER_LEFT_INDEX].x) << 15;
  b16_t rightX = (m_calibData[CALIB_UPPER_RIGHT_INDEX].x +
                  m_calibData[CALIB_LOWER_RIGHT_INDEX].x) << 15;

  data.xSlope  = b16divb16(itob16(CALIBRATION_RIGHTX - CALIBRATION_LEFTX), (rightX - leftX));
  data.xOffset = itob16(CALIBRATION_LEFTX) - b16mulb16(leftX, data.xSlope);

  gdbg("New xSlope: %08x xOffset: %08x\n", data.xSlope, data.xOffset);

  // Similarly for Y
  //
  // (scaledY - TOPY) / (rawY - topY) = (BOTTOMY - TOPY) / (bottomY - topY)
  // scaledY = (rawY - topY) * (BOTTOMY - TOPY) / (bottomY - topY) + TOPY
  //         = rawY * ySlope + (TOPY - topY * ySlope)
  //         = rawY * ySlope + yOffset
  //
  // where:
  // ySlope  = (BOTTOMY - TOPY) / (bottomY - topY)
  // yOffset = (TOPY - topY * ySlope)

  b16_t topY    = (m_calibData[CALIB_UPPER_LEFT_INDEX].y +
                   m_calibData[CALIB_UPPER_RIGHT_INDEX].y) << 15;
  b16_t bottomY = (m_calibData[CALIB_LOWER_LEFT_INDEX].y +
                   m_calibData[CALIB_LOWER_RIGHT_INDEX].y) << 15;

  data.ySlope  = b16divb16(itob16(CALIBRATION_BOTTOMY - CALIBRATION_TOPY), (bottomY - topY));
  data.yOffset = itob16(CALIBRATION_TOPY) - b16mulb16(topY, data.ySlope);

  gdbg("New ySlope: %08x yOffset: %08x\n", data.ySlope, data.yOffset);
  return true;
}

/**
 * CCalibrationFactory Constructor
 *
 * @param taskbar.  The taskbar instance used to terminate calibration
 * @param touchscreen. An instance of the class that wraps the
 *   touchscreen device.
 */

CCalibrationFactory::CCalibrationFactory(CTaskbar *taskbar, CTouchscreen *touchscreen)
{
  m_taskbar     = taskbar;
  m_touchscreen = touchscreen;
}

/**
 * Create a new instance of an CCalibration (as IApplication).
 */

IApplication *CCalibrationFactory::create(void)
{
  // Call CTaskBar::openFullScreenWindow to create a full screen window for
  // the calibation application

  CFullScreenWindow *window = m_taskbar->openFullScreenWindow();
  if (!window)
    {
      gdbg("ERROR: Failed to create CFullScreenWindow\n");
      return (IApplication *)0;
    }

  // Open the window (it is hot in here)

  if (!window->open())
    {
      gdbg("ERROR: Failed to open CFullScreenWindow \n");
      delete window;
      return (IApplication *)0;
    }

  // Instantiate the application, providing the window to the application's
  // constructor

  CCalibration *calibration = new CCalibration(m_taskbar, window, m_touchscreen);
  if (!calibration)
    {
      gdbg("ERROR: Failed to instantiate CCalibration\n");
      delete window;
      return (IApplication *)0;
    }

  return static_cast<IApplication*>(calibration);
}

/**
 * Get the icon associated with the application
 *
 * @return An instance if IBitmap that may be used to rend the
 *   application's icon.  This is an new IBitmap instance that must
 *   be deleted by the caller when it is no long needed.
 */

NXWidgets::IBitmap *CCalibrationFactory::getIcon(void)
{
  NXWidgets::CRlePaletteBitmap *bitmap =
    new NXWidgets::CRlePaletteBitmap(&CONFIG_NXWM_CALIBRATION_ICON);

  return bitmap;
}
