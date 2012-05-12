/********************************************************************************************
 * NxWidgets/nxwm/src/cwindowcontrol.cxx
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

#include "cwindowcontrol.hxx"

/********************************************************************************************
 * Pre-Processor Definitions
 ********************************************************************************************/

/********************************************************************************************
 * CWindowControl Method Implementations
 ********************************************************************************************/

using namespace NxWM;

/**
 * Constructor
 *
 * @param style The default style that all widgets on this display
 *   should use.  If this is not specified, the widget will use the
 *   values stored in the defaultCWidgetStyle object.
 */

CWindowControl::CWindowControl(FAR const NXWidgets::CWidgetStyle *style)
: NXWidgets::CWidgetControl(style)
{
  // Add ourself as the window callback

  addWindowEventHandler(this);
}

/**
 * Destructor.
 */

CWindowControl::~CWindowControl(void)
{
  // Remove ourself from the window callback

  removeWindowEventHandler(this);
}

/**
 * Handle an NX window mouse input event.
 *
 * @param e The event data.
 */

#ifdef CONFIG_NX_MOUSE
void CWindowControl::handleMouseEvent(void)
{
  // The logic path here is tortuous but flexible:
  //
  // 1. A listener thread receives mouse input and injects that into NX
  // 2. In the multi-user mode, this will send a message to the NX server
  // 3. The NX server will determine which window gets the mouse input
  //    and send a message to the listener.
  // 4. The listener will call the NX message dispatcher will will do the
  //    message callback.
  // 5. The callback goes into an instance of NXWidgets::CCallback that is
  //    part of the CWidget control.
  // 6. That callback will update mouse information then raise the
  //    mouse event,
  // 7. Which will finally call this function -- still running deep on the
  //    stack in the listener thread.
  // 8. This function will then call back into the widget control to process
  //    the mouse input.

  // Perform the poll

  pollEvents();
}
#endif

/**
 * Handle a NX window keyboard input event.
 */

#ifdef CONFIG_NX_KBD
void CWindowControl::handleKeyboardEvent(void)
{
  // The logic path here is tortuous but flexible:
  //
  // 1. A listener thread receives keyboard input and injects that into NX
  // 2. In the multi-user mode, this will send a message to the NX server
  // 3. The NX server will determine which window gets the keyboard input
  //    and send a message to the listener.
  // 4. The listener will call the NX message dispatcher will will do the
  //    message callback.
  // 5. The callback goes into an instance of NXWidgets::CCallback that is
  //    part of the CWidget control.
  // 6. That callback will update keyboard information then raise the
  //    keyboard event,
  // 7. Which will finally call this function -- still running deep on the
  //    stack in the listener thread.
  // 8. This function will then call back into the widget control to process
  //    the keyboard input.

  // Perform the poll

  pollEvents();
}
#endif


