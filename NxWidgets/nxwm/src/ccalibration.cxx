/****************************************************************************
 * NxWidgets/nxwm/src/capplicationwindow.cxx
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

#include <assert.h>
#include <errno.h>
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
 * @param window.  The window to use for the calibration display
 * @param touchscreen. An instance of the class that wraps the touchscreen device.
 */

CCalibration::CCalibration(CFullScreenWindow *window, CTouchscreen *touchscreen)
{
  // Initialize state data

  m_window        = window;
  m_touchscreen   = touchscreen;
  m_state         = CALIB_NOT_STARTED;
  m_stop          = false;
  m_touched       = false;
}

/**
 * CCalibration Destructor
 */

CCalibration::~CCalibration(void)
{
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
  // Provide the initial display

  m_state = CALIB_NOT_STARTED;
  stateMachine();

  // Loop until calibration completes

  while (!m_stop && m_state != CALIB_COMPLETE)
    {
      // Wait for the next raw touchscreen input

      struct touch_sample_s sample;
      while (!m_touchscreen->waitRawTouchData(&sample));

      // Then process the raw touchscreen input

      touchscreenInput(sample);
    }
 
   return !m_stop;
}

/**
 * Stop the application.
 */

void CCalibration::stop(void)
{
   // The main thread is stuck waiting for the next touchscreen input...
   // So this is probably just a waste of good FLASH space.

   m_stop = true;
}

/**
 * The application window is hidden (either it is minimized or it is
 * maximized, but it is not at the top of the hierarchy)
 */

void CCalibration::hide(void)
{
  // REVISIT
}

/**
 * Redraw the entire window.  The application has been maximized or
 * otherwise moved to the top of the hierarchy.  This method is called from
 * CTaskbar when the application window must be displayed
 */

void CCalibration::redraw(void)
{
  // Reset the state machine and start over

  if (m_state != CALIB_COMPLETE)
    {
      m_state = CALIB_NOT_STARTED;
      stateMachine();
    }
}

/**
 * Wait for calibration data to be received.
 *
 * @return True if the calibration data was successfully obtained.
 */

bool CCalibration::waitCalibrationData(struct SCalibrationData &data)
{
  // Wait until calibration is finished

  while (m_state != CALIB_COMPLETE)
    {
      int ret = sem_wait(&m_waitSem);
      DEBUGASSERT(ret == 0 || errno == EINTR);
    }

  // Recover the window instance contained in the full screen window

  NXWidgets::INxWindow *window = m_window->getWindow();

  // Get the size of the fullscreen window

  struct nxgl_size_s windowSize;
  if (!window->getSize(&windowSize))
    {
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

      if ((sample.point[0].flags & TOUCH_DOWN) != 0 || m_touched)
        {
          // Yes.. save the touch position and wait for the TOUCH_UP report

          m_touchPos.x = sample.point[0].x;
          m_touchPos.y = sample.point[0].y;

          gvdbg("Touch id: %d flags: %02x x: %d y: %d h: %d w: %d pressure: %d\n",
                sample.point[0].id, sample.point[0].flags, sample.point[0].x,
                sample.point[0].y,  sample.point[0].h,     sample.point[0].w,
                sample.point[0].pressure);

          // Remember that we saw the touch down event

          m_touched    = true;
        }
    }

  // Was the touch released?

  else if ((sample.point[0].flags & TOUCH_UP) != 0)
    {
      // Yes.. did we see the matching pen down event?

      if (m_touched)
        {
          // Yes.. invoke the state machine.

          gvdbg("State: %d Screen x: %d y: %d  Touch x: %d y: %d\n",
                m_state, m_screenInfo.pos.x, m_screenInfo.pos.y,
                m_touchPos.x, m_touchPos.y);

          stateMachine();
        }

      // In any event, the touch is not down

      m_touched = false;
    }
}

/**
 * This is the calibration state machine.  It is called initially and then
 * as new touchscreen data is received.
 */

void CCalibration::stateMachine(void)
{
  // Recover the window instance contained in the full screen window

  NXWidgets::INxWindow *window = m_window->getWindow();

  // Get the size of the fullscreen window

  struct nxgl_size_s windowSize;
  if (!window->getSize(&windowSize))
    {
      return;
    }

  switch (m_state)
    {
      default:
      case CALIB_NOT_STARTED:
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

          m_state = CALIB_UPPER_LEFT;
        }
        break;

      case CALIB_UPPER_LEFT:
        {
          // A touch has been received while in the CALIB_UPPER_LEFT state.
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

          m_state = CALIB_UPPER_RIGHT;
        }
        break;

      case CALIB_UPPER_RIGHT:
        {
          // A touch has been received while in the CALIB_UPPER_RIGHT state.
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

          m_state = CALIB_LOWER_RIGHT;
        }
        break;

      case CALIB_LOWER_RIGHT:
        {
          // A touch has been received while in the CALIB_LOWER_RIGHT state.
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

          m_state = CALIB_LOWER_LEFT;
        }
        break;

      case CALIB_LOWER_LEFT:
        {
          // A touch has been received while in the CALIB_LOWER_LEFT state.
          // Save the touch data and set up the next calibration display

          m_calibData[CALIB_LOWER_LEFT_INDEX].x = m_touchPos.x;
          m_calibData[CALIB_LOWER_LEFT_INDEX].y = m_touchPos.y;

          // Inform any waiter that calibration is complete

          m_state = CALIB_COMPLETE;
          sem_post(&m_waitSem);
        }
        break;

      case CALIB_COMPLETE:
        // Might happen... do nothing if it does
        break;
    }

  gvdbg("State: %d Screen x: %d y: %d\n",
        m_state, m_screenInfo.pos.x, m_screenInfo.pos.y);
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
