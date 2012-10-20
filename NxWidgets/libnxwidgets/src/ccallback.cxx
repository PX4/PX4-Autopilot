/****************************************************************************
 * NxWidgets/libnxwidgets/src/ccallback.cxx
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

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#ifdef CONFIG_NXCONSOLE_NXKBDIN
#  include <nuttx/nx/nxconsole.h>
#endif

#include "cwidgetcontrol.hxx"
#include "ccallback.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Method Implementations
 ****************************************************************************/

using namespace NXWidgets;

 /**
  * Constructor.
  *
  * @param widgetControl Control object associated with this window
  */

CCallback::CCallback(CWidgetControl *widgetControl)
{
  // Save the widgetControl

  m_widgetControl      = widgetControl;

  // Initialize the callback vtable

  m_callbacks.redraw   = redraw;
  m_callbacks.position = position;
#ifdef CONFIG_NX_MOUSE
  m_callbacks.mousein  = newMouseEvent;
#endif
#ifdef CONFIG_NX_KBD
  m_callbacks.kbdin    = newKeyboardEvent;
#endif
  m_callbacks.blocked  = windowBlocked;

  // Keyboard input is initially direct to the widgets within the window

#ifdef CONFIG_NXCONSOLE_NXKBDIN
  m_nxconsole          = (NXCONSOLE)0;
#endif
}

 /**
  * ReDraw Callback.  The redraw action is handled by CWidgetControl:redrawEvent.
  *
  * @param hwnd Handle to a specific NX window.
  * @param rect The rectangle that needs to be re-drawn (in window
  * relative coordinates).
  * @param bMore true: More re-draw requests will follow.
  * @param arg User provided argument (see nx_openwindow, nx_requestbg,
  * nxtk_openwindow, or nxtk_opentoolbar).
  */
     
void CCallback::redraw(NXHANDLE hwnd,
                       FAR const struct nxgl_rect_s *rect,
                       bool bMore, FAR void *arg)
{
  gvdbg("hwnd=%p rect={(%d,%d),(%d,%d)} bMore=%s\n",
         hwnd,
         rect->pt1.x, rect->pt1.y, rect->pt2.x, rect->pt2.y,
         bMore ? "true" : "false");

  // The argument must be the CCallback instance

  CCallback *This = (CCallback *)arg;

  // Just forward the callback to the CWidgetControl::redrawEvent method

  This->m_widgetControl->redrawEvent(rect, bMore);
}

 /**
  * Position Callback. The new positional data is handled by
  * CWidgetControl::geometryEvent.
  *
  * @param hwnd Handle to a specific NX window.
  * @param size The size of the window.
  * @param pos The position of the upper left hand corner of the window on
  * the overall display.
  * @param bounds The bounding rectangle that describes the entire display.
  * @param arg User provided argument (see nx_openwindow, nx_requestbg,
  * nxtk_openwindow, or nxtk_opentoolbar).
  */

void CCallback::position(NXHANDLE hwnd,
                         FAR const struct nxgl_size_s *size,
                         FAR const struct nxgl_point_s *pos,
                         FAR const struct nxgl_rect_s *bounds,
                         FAR void *arg)
{
  gvdbg("hwnd=%p size=(%d,%d) pos=(%d,%d) bounds={(%d,%d),(%d,%d)} arg=%p\n",
        hwnd, size->w, size->h, pos->x, pos->y,
        bounds->pt1.x, bounds->pt1.y, bounds->pt2.x, bounds->pt2.y,
        arg);


  // The argument must be the CCallback instance

  CCallback *This = (CCallback *)arg;

  // Just forward the callback to the CWidgetControl::geometry method

  This->m_widgetControl->geometryEvent(hwnd, size, pos, bounds);
}

 /**
  * New mouse data is available for the window.  The new mouse data is
  * handled by CWidgetControl::newMouseEvent.
  *
  * @param hwnd Handle to a specific NX window.
  * @param pos The (x,y) position of the mouse.
  * @param buttons See NX_MOUSE_* definitions.
  * @param arg User provided argument (see nx_openwindow, nx_requestbg,
  * nxtk_openwindow, or nxtk_opentoolbar).
  */

#ifdef CONFIG_NX_MOUSE
void CCallback::newMouseEvent(NXHANDLE hwnd,
                              FAR const struct nxgl_point_s *pos,
                              uint8_t buttons, FAR void *arg)
{
  gvdbg("hwnd=%p pos=(%d,%d) buttons=%02x arg=%p\n", 
        hwnd, pos->x, pos->y, buttons, arg);

  // The argument must be the CCallback instance

  CCallback *This = (CCallback *)arg;

  // Just forward the callback to the CWidgetControl::newMouseEvent method

  This->m_widgetControl->newMouseEvent(pos, buttons);
}
#endif /* CONFIG_NX_MOUSE */

/**
 * New keyboard/keypad data is available for the window.  The new keyboard
 * data is handled by CWidgetControl::newKeyboardEvent.
 *
 * @param hwnd Handle to a specific NX window.
 * @param nCh The number of characters that are available in str[].
 * @param str The array of characters.
 * @param arg User provided argument (see nx_openwindow, nx_requestbg,
 * nxtk_openwindow, or nxtk_opentoolbar).
 */

#ifdef CONFIG_NX_KBD
void CCallback::newKeyboardEvent(NXHANDLE hwnd, uint8_t nCh,
                                 FAR const uint8_t *str,
                                 FAR void *arg)
{
  gvdbg("hwnd=%p nCh=%d arg=%p\n", hwnd, nCh, arg);

  // The argument must be the CCallback instance

  CCallback *This = (CCallback *)arg;

  // Is NX keyboard input being directed to the widgets within the window
  // (default) OR is NX keyboard input being re-directed to an NxConsole
  // driver?

#ifdef CONFIG_NXCONSOLE_NXKBDIN
  if (This->m_nxconsole)
    {
      // Keyboard input is going to an NxConsole

      nxcon_kbdin(This->m_nxconsole, str, nCh);
    }
  else
#endif
    {
      // Just forward the callback to the CWidgetControl::newKeyboardEvent method

      This->m_widgetControl->newKeyboardEvent(nCh, str);
    }
}

#endif // CONFIG_NX_KBD

/**
 * This callback is the response from nx_block (or nxtk_block). Those
 * blocking interfaces are used to assure that no further messages are
 * directed to the window. Receipt of the blocked callback signifies
 * that (1) there are no further pending callbacks and (2) that the
 * window is now 'defunct' and will receive no further callbacks.
 *
 * This callback supports coordinated destruction of a window in multi-
 * user mode.  In multi-use more, the client window logic must stay
 * intact until all of the queued callbacks are processed.  Then the
 * window may be safely closed.  Closing the window prior with pending
 * callbacks can lead to bad behavior when the callback is executed.
 *
 * @param hwnd. Window handle of the blocked window
 * @param arg1. User provided argument (see nx_openwindow, nx_requestbkgd,
 *   nxtk_openwindow, or nxtk_opentoolbar)
 * @param arg2 - User provided argument (see nx_block or nxtk_block)
 */

#ifdef CONFIG_NX_MULTIUSER
void CCallback::windowBlocked(NXWINDOW hwnd, FAR void *arg1, FAR void *arg2)
{
  gvdbg("hwnd=%p arg1=%p arg2=%p\n", hwnd, arg1, arg2);

  // The first argument must be the CCallback instance

  CCallback *This = (CCallback *)arg1;

  // Just forward the callback to the CWidgetControl::windowBlocked method

  This->m_widgetControl->windowBlocked(arg2);
}
#endif

