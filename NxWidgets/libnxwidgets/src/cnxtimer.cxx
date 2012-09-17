/****************************************************************************
 * NxWidgets/libnxwidgets/src/cnxtimer.cxx
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
 ****************************************************************************
 *
 * Portions of this package derive from Woopsi (http://woopsi.org/) and
 * portions are original efforts.  It is difficult to determine at this
 * point what parts are original efforts and which parts derive from Woopsi.
 * However, in any event, the work of  Antony Dzeryn will be acknowledged
 * in most NxWidget files.  Thanks Antony!
 *
 *   Copyright (c) 2007-2011, Antony Dzeryn
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * * Neither the names "Woopsi", "Simian Zombie" nor the
 *   names of its contributors may be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY Antony Dzeryn ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Antony Dzeryn BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <cstring>
#include <ctime>
#include <csignal>
#include <debug.h>

#include "cnxtimer.hxx"
#include "singletons.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Static Data Members
 ****************************************************************************/

using namespace NXWidgets;

/****************************************************************************
 * Implementation Classes
 ****************************************************************************/

/**
 * Constructor.
 *
 * @param pWidgetControl The controlling widget for the display.
 * @param timeout Time, in milliseconds, before the timer fires an
 *    EVENT_ACTION event.
 * @param repeat If true, the timer will fire multiple events.  If false,
 *   the timer will fire just once and stop.
 */

CNxTimer::CNxTimer(CWidgetControl *pWidgetControl, uint32_t timeout, bool repeat)
: CNxWidget(pWidgetControl, 0, 0, 0, 0, 0, 0)
{
  // Remember the timer configuration

  m_timeout    = timeout;
  m_isRepeater = repeat;
  m_isRunning  = false;

  // Create a POSIX timer (We can't do anything about failures here)

  int ret = timer_create(CLOCK_REALTIME, (FAR struct sigevent *)NULL, &m_timerid);
  if (ret < 0)
    {
      gdbg("timer_create() failed\n");
      return;
    }

  // If we are the first timer created in the whole system, then create
  // the timer list and attach the SIGALRM timer handler.

  if (!g_nxTimers)
    {
      if (!g_nxTimers)
        {
          gdbg("Failed to create the timer list\n");
          return;
        }
 
      // Attach the SIGALM signal handler (no harm if this is done multiple times)

      struct sigaction sigact;
      sigact.sa_handler = signalHandler;
      sigact.sa_flags   = 0;
      sigemptyset(&sigact.sa_mask);

      ret = sigaction(SIGALRM, &sigact, (FAR struct sigaction *)NULL);
      if (ret < 0)
        {
          gdbg("sigaction() failed\n");
          return;
        }
    }

  // Add ourself onto the array of timers
#warning "Need to disable SIGALRM here"
  g_nxTimers->push_back(this);
}

/**
 * Destructor.
 */

CNxTimer::~CNxTimer(void)
{
  // Locate ourself in the list of timers and remove ourselves

#warning "Need to disable SIGALRM here"
  for (int i = 0; i < g_nxTimers->size(); i++)
    {
      CNxTimer *timer = g_nxTimers->at(i);
      if (timer == this)
        {
          g_nxTimers->erase(i);
          break;
        }
    }

  // Destroy the timer

  (void)timer_delete(m_timerid);
}

/**
 * Return the timeout of this timer.
 *
 * @return The number of milliseconds that this timer will run before firing
 *   an event.
 */

const uint32_t CNxTimer::getTimeout(void)
{
  // If the timer is not running, then just return the timeout value

  if (!m_isRunning)
    {
      return m_timeout;
    }
  else
    {
      // Get the time remaining on the POSIX timer.  Of course, there
      // are race conditions here.. the timer could expire at anytime

      struct itimerspec remaining;
      int ret = timer_gettime(m_timerid, &remaining);
      if (ret < 0)
        {
          gdbg("timer_gettime() failed\n");
          return 0;
        }

      return timespecToMilliseconds(&remaining.it_value);
    }
}

/**
 * Resets the millisecond timer.
 */

void CNxTimer::reset(void)
{
  // It does not make sense to reset the timer if the timer is not running

  if (m_isRunning)
    {
      // If the specified timer was already armed when timer_settime() is
      // called, this call will reset the time until next expiration to the
      // value specified. 

      m_isRunning = false;
      start();
    }
}

/**
 * Starts the timer.
 */

void CNxTimer::start(void)
{
  // If the timer is running, reset should be used to restart it

  if (!m_isRunning)
    {
      // If the specified timer was already armed when timer_settime() is
      // called, this call will reset the time until next expiration to the
      // value specified. 
 
      struct itimerspec timerspec;
      millisecondsToTimespec(m_timeout, &timerspec.it_value);
      timerspec.it_interval.tv_sec  = 0;
      timerspec.it_interval.tv_nsec = 0;

      int ret = timer_settime(m_timerid, 0, &timerspec,
                             (FAR struct itimerspec *)NULL);
      if (ret < 0)
        {
          gdbg("timer_settime() failed\n");
        }

      // The timer is now running

      m_isRunning = true;
    }
}

/**
 * Stops the timer
 */

void CNxTimer::stop(void)
{
  if (m_isRunning)
    {
      // If the it_value member of value is zero, the timer will be disarmed.
      // The effect of disarming or resetting a timer with pending expiration
      // notifications is unspecified.

      struct itimerspec nullTime;
      memset(&nullTime, 0, sizeof(struct itimerspec));

      int ret = timer_settime(m_timerid, 0, &nullTime,
                              (FAR struct itimerspec *)NULL);
      if (ret < 0)
        {
          gdbg("timer_settime failed\n");
        }

      // The time is no longer running

      m_isRunning = false;
    }
}

/**
 * The SIGALM signal handler that will be called when the timer goes off
 *
 * @param signo The signal number call caused the handler to run (SIGALM)
 */

void CNxTimer::signalHandler(int signo)
{
  // Call handlerTimerExpiration on every timer instance

  for (int i = 0; i < g_nxTimers->size(); i++)
    {
      CNxTimer *timer = g_nxTimers->at(i);
      timer->handleTimerExpiration();
    }
}

/**
 * Handle an expired timer
 */

void CNxTimer::handleTimerExpiration(void)
{
  // Do we think our timer is running?

  if (m_isRunning)
    {
      // Is it running?  It the timer is not running, it will return an
      // it_value of zero.

      struct itimerspec status;
      int ret = timer_gettime(m_timerid, &status);
      if (ret < 0)
        {
          gdbg("timer_gettime() failed\n");
          return;
        }

      // it_value == 0 means that timer is not running

      if (status.it_value.tv_sec == 0 && status.it_value.tv_nsec == 0)
        {
          // It has expired

          m_isRunning = false;

          // Raise the action event.  Hmmm.. are there any issues with
          // doing this from a signal handler?  We'll find out

          m_widgetEventHandlers->raiseActionEvent();

          // Restart the timer if this is a repeating timer

          if (m_isRepeater)
            {
              start();
            }
        }
    }
}
 
/**
 * Convert a timespec to milliseconds
 *
 * @param tp The pointer to the timespec to convert
 * @return The corresponding time in milliseconds
 */

uint32_t CNxTimer::timespecToMilliseconds(FAR const struct timespec *tp)
{
  return (uint32_t)tp->tv_sec * 1000 + (uint32_t)tp->tv_nsec / 10000000;
}

/**
 * Convert milliseconds to a timespec
 *
 * @param milliseconds The milliseconds to be converted
 * @param tp The pointer to the location to store the converted timespec
 */

 void CNxTimer::millisecondsToTimespec(uint32_t milliseconds,
                                       FAR struct timespec *tp)
 {
   tp->tv_sec         = milliseconds / 1000;
   uint32_t remainder = milliseconds - (uint32_t)tp->tv_sec * 1000;
   tp->tv_nsec        = remainder * 1000000;
 }


