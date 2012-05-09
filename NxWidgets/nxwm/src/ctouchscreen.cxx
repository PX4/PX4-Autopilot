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

/********************************************************************************************
 * CTouchscreen Method Implementations
 ********************************************************************************************/

using namespace NxWM;

/**
 * CTouchscreen Constructor
 */

CTouchscreen::CTouchscreen(void)
{
  m_touchFd  = -1;
  sem_init(&m_waitSem, 0, 0);
}

/**
 * CTouchscreen Destructor
 */

CTouchscreen::~CTouchscreen(void)
{
  if (m_touchFd >= 0)
    {
      std::close(m_touchFd);
    }

   sem_destroy(&m_waitSem);
}

/**
 * Initialize the touchscreen device.  Initialization is separate from
 * object instantiation so that failures can be reported.
 *
 * @return True if the touchscreen device was correctly initialized
 */

bool CTouchscreen::open(void)
{
  // Open the touchscreen device

  m_touchFd = std::open(CONFIG_NXWM_TOUCHSCREEN_DEVPATH, O_RDONLY);
  if (m_touchFd < 0)
    {
      gdbg("ERROR Failed to open %s for reading: %d\n",
           CONFIG_NXWM_TOUCHSCREEN_DEVPATH, errno);
      return false;
    }

  return true;
}

/**
 * Capture raw driver data.
 *
 *
 * @return True if the raw touchscreen data was sucessfully obtained
 */

bool CTouchscreen::waitRawTouchData(struct touch_sample_s &touch)
{
#warning "Missing logic"
  return true;
}
