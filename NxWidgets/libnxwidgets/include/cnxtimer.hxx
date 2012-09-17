/****************************************************************************
 * NxWidgets/libnxwidgets/include/cnxtimer.hxx
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

#ifndef __INCLUDE_CNXTIMER_HXX
#define __INCLUDE_CNXTIMER_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <ctime>

#include "cnxwidget.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
 
/****************************************************************************
 * Implementation Classes
 ****************************************************************************/
 
#if defined(__cplusplus)

namespace NXWidgets
{
  class CWidgetcontrol;

  /**
   * Timer widget.  It can drive time-based events, animations, etc.
   *
   * Using the timer is simple:
   *  - Create an instance of the CNxTimer and add it as a child to a widget. 
   *  - Call the instance's "start()" method.
   *  - Catch the timer's action event and call any code that should run.
   */

  class CNxTimer : public CNxWidget
  {
  protected:
    FAR timer_t       m_timerid;    /**< POSIX timer */
    uint32_t          m_timeout;    /**< The timeout value in milliseconds */
    bool              m_isRunning;  /**< Indicates whether or not the timer is running */
    bool              m_isRepeater; /**< Indicates whether or not the timer repeats */

    /**
     * The SIGALM signal handler that will be called when the timer goes off
     *
     * @param signo The signal number call caused the handler to run (SIGALM)
     */

    static void signalHandler(int signo);

    /**
     * Handle an expired timer
     */

    void handleTimerExpiration(void);
 
    /**
     * Convert a timespec to milliseconds
     *
     * @param tp The pointer to the timespec to convert
     * @return The corresponding time in milliseconds
     */

    uint32_t timespecToMilliseconds(FAR const struct timespec *tp);

   /**
    * Convert milliseconds to a timespec
    *
    * @param milliseconds The milliseconds to be converted
    * @param tp The pointer to the location to store the converted timespec
    */

    void millisecondsToTimespec(uint32_t milliseconds, FAR struct timespec *tp);

    /**
     * Copy constructor is protected to prevent usage.
     */

    inline CNxTimer(const CNxTimer &timer) : CNxWidget(timer) { }

  public:

    /**
     * Constructor.
     *
     * @param pWidgetControl The controlling widget for the display.
     * @param timeout Time, in milliseconds, before the timer fires an
     *   EVENT_ACTION event.
     * @param repeat If true, the timer will fire multiple events.  If false,
     *   the timer will fire just once and stop.
     */

    CNxTimer(CWidgetControl *pWidgetControl, uint32_t timeout, bool repeat);

    /**
     * Destructor.
     */

    ~CNxTimer(void);

    /**
     * Return the time remaining on this timer.
     *
     * @return The number of milliseconds that this timer runs before
     *   firing an event.  Zero is returned if the timer is not running.
     */

    const uint32_t getTimeout(void);

    /**
     * Resets the (running) timer to its initial timeout value.  This
     * call does nothing if the timer is not running.
     */

    void reset(void);

    /**
     * Starts the timer.  This call does nothing if the timer is already
     * running.
     */

    void start(void);

    /**
     * Stops the timer.  Does nothing if the timer is not running.
     */

    void stop(void);

    /**
     * Set the timeout of this timer.  This timeout value will not
     * take effect until start() or reset() is called.
     *
     * @param timeout The number of milliseconds that this timer will run
     *   before firing an event.
     */

    inline void setTimeout(uint32_t timeout)
    {
      m_timeout = timeout;
    }
  };
}

#endif // __cplusplus

#endif // __INCLUDE_CNXTIMER_HXX
