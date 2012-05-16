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

#include <cfcntl>
#include <cerrno>

#include <debug.h>

#include "nxwmconfig.hxx"
#include "cstartwindow.hxx"
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
  // Open a message queue to communicate with the start window task.  We need to create
  // the message queue if it does not exist.

  struct mq_attr attr;
  attr.mq_maxmsg  = CONFIG_NXWM_STARTWINDOW_MXMSGS;
  attr.mq_msgsize = sizeof(struct SStartWindowMessage);
  attr.mq_flags   = 0;

  m_mqd = mq_open(CONFIG_NXWM_STARTWINDOW_MQNAME, O_WRONLY|O_CREAT, 0666, &attr);
  if (m_mqd == (mqd_t)-1)
    {
      gdbg("ERROR: mq_open(%s) failed: %d\n", CONFIG_NXWM_STARTWINDOW_MQNAME, errno);
    }

  // Add ourself as the window callback

  addWindowEventHandler(this);
}

/**
 * Destructor.
 */

CWindowControl::~CWindowControl(void)
{
  // Close the message queue

  (void)mq_close(m_mqd);
 
  // Remove ourself from the window callback

  removeWindowEventHandler(this);
}

/**
 * Destroy the application window and everything in it.  This is
 * handled by CWindowControl (vs just calling the destructors) because
 * in the case where an application destroys itself (because of pressing
 * the stop button), then we need to unwind and get out of the application
 * logic before destroying all of its objects.
 */

void CWindowControl::destroy(IApplication *app)
{
  // Send a message to destroy the window isntance at a later time

  struct SStartWindowMessage outmsg;
  outmsg.msgId    = MSGID_DESTROY_APP;
  outmsg.instance = (FAR void *)app;

  gdbg("Sending MSGID_DESTROY_APP with instance=%p\n", app);
  int ret = mq_send(m_mqd, &outmsg, sizeof(struct SStartWindowMessage),
                    CONFIG_NXWM_STARTWINDOW_MXMPRIO);
  if (ret < 0)
    {
      gdbg("ERROR: mq_send failed: %d\n", errno);
    }

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
  //  1. A listener thread receives mouse or touchscreen input and injects
  //     that into NX via nx_mousein
  //  2. In the multi-user mode, this will send a message to the NX server
  //  3. The NX server will determine which window gets the mouse input
  //     and send a window event message to the NX listener thread.
  //  4. The NX listener thread receives a windows event.  The NX listener thread
  //     which is part of CTaskBar and was created when NX server connection was
  //     established).  This event may be a positional change notification, a
  //     redraw request, or mouse or keyboard input.  In this case, mouse input.
  //  5. The NX listener thread handles the message by calling nx_eventhandler().
  //     nx_eventhandler() dispatches the message by calling a method in the
  //     NXWidgets::CCallback instance associated with the window.
  //     NXWidgets::CCallback is a part of the CWidgetControl.
  //  6. NXWidgets::CCallback calls into NXWidgets::CWidgetControl to process
  //     the event.
  //  7. NXWidgets::CWidgetControl records the new state data and raises a
  //     window event.
  //  8. NXWidgets::CWindowEventHandlerList will give the event to this method
  //     NxWM::CWindowControl.
  //  9. This NxWM::CWindowControl method will send the a message on a well-
  //     known message queue.
  // 10. This CStartWindow::startWindow task will receive and process that
  //     message by calling CWidgetControl::pollEvents()

  struct SStartWindowMessage outmsg;
  outmsg.msgId    = MSGID_MOUSE_INPUT;
  outmsg.instance = (FAR void *)static_cast<CWidgetControl*>(this);

  int ret = mq_send(m_mqd, &outmsg, sizeof(struct SStartWindowMessage),
                    CONFIG_NXWM_STARTWINDOW_MXMPRIO);
  if (ret < 0)
    {
      gdbg("ERROR: mq_send failed: %d\n", errno);
    }
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
  //  1. A listener thread receives keyboard input and injects  that into NX
  //     via nx_kbdin.
  //  2. In the multi-user mode, this will send a message to the NX server
  //  3. The NX server will determine which window gets the keyboard input
  //     and send a window event message to the NX listener thread.
  //  4. The NX listener thread receives a windows event.  The NX listener thread
  //     which is part of CTaskBar and was created when NX server connection was
  //     established).  This event may be a positional change notification, a
  //     redraw request, or mouse or keyboard input.  In this case, keyboard input.
  //  5. The NX listener thread handles the message by calling nx_eventhandler().
  //     nx_eventhandler() dispatches the message by calling a method in the
  //     NXWidgets::CCallback instance associated with the window.
  //     NXWidgets::CCallback is a part of the CWidgetControl.
  //  6. NXWidgets::CCallback calls into NXWidgets::CWidgetControl to process
  //     the event.
  //  7. NXWidgets::CWidgetControl records the new state data and raises a
  //     window event.
  //  8. NXWidgets::CWindowEventHandlerList will give the event to this method
  //     NxWM::CWindowControl.
  //  9. This NxWM::CWindowControl method will send the a message on a well-
  //     known message queue.
  // 10. This CStartWindow::startWindow task will receive and process that
  //     message by calling CWidgetControl::pollEvents()

  struct SStartWindowMessage outmsg;
  outmsg.msgId    = MSGID_KEYBOARD_INPUT;
  outmsg.instance = (FAR void *)static_cast<CWidgetControl*>(this);

  int ret = mq_send(m_mqd, &outmsg, sizeof(struct SStartWindowMessage),
                    CONFIG_NXWM_STARTWINDOW_MXMPRIO);
  if (ret < 0)
    {
      gdbg("ERROR: mq_send failed: %d\n", errno);
    }
}
#endif
