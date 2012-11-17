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
#include <debug.h>
#include <errno.h>

#include <nuttx/clock.h>
#include <nuttx/wqueue.h>

#include "cnxtimer.hxx"

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

  // Reset the work structure

  memset(&m_work, 0, sizeof(m_work));
}

/**
 * Destructor.
 */

CNxTimer::~CNxTimer(void)
{
}

/**
 * Resets the millisecond timer.
 */

void CNxTimer::reset(void)
{
  // It does not make sense to reset the timer if the timer is not running

  if (m_isRunning)
    {
      stop();
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
      uint32_t ticks = m_timeout / MSEC_PER_TICK;
      int ret = work_queue(USRWORK, &m_work, workQueueCallback, this, ticks);

      if (ret < 0)
        {
          gdbg("work_queue failed: %d\n", ret);
        }

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
      int ret = work_cancel(USRWORK, &m_work);

      if (ret < 0)
        {
          gdbg("work_cancel failed: %d\n", ret);
        }

      m_isRunning = false;
    }
}

void CNxTimer::workQueueCallback(FAR void *arg)
{
  CNxTimer* This = (CNxTimer*)arg;

  This->m_isRunning = false;

  // Raise the action event.

  This->m_widgetEventHandlers->raiseActionEvent();

  // Restart the timer if this is a repeating timer

  if (This->m_isRepeater)
    {
      This->start();
    }
}


