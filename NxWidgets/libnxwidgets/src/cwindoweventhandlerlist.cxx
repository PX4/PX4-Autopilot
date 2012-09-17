/****************************************************************************
 * NxWidgets/libnxwidgets/src/cwindoweventhandlerlist.cxx
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

#include "nxconfig.hxx"

#include "cwindoweventhandler.hxx"
#include "cwindoweventhandlerlist.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Method Implementations
 ****************************************************************************/

using namespace NXWidgets;

/**
 * Adds a window event handler.  The event handler will receive
 * all events raised by this object.
 * @param eventHandler A pointer to the event handler.
 */
 
void CWindowEventHandlerList::addWindowEventHandler(CWindowEventHandler *eventHandler)
{
  // Make sure that the event handler does not already exist

  int index;
  if (!findWindowEventHandler(eventHandler, index))
    {
      // Add the new handler

      m_eventHandlers.push_back(eventHandler);
    }
}

/**
 * Remove a window event handler.
 *
 * @param eventHandler A pointer to the event handler to remove.
 */

void CWindowEventHandlerList::removeWindowEventHandler(CWindowEventHandler *eventHandler)
{
  // Find the event handler to be removed

  int index;
  if (findWindowEventHandler(eventHandler, index))
    {
      // and remove it

      m_eventHandlers.erase(index);
    }
}

/**
 * Return the index to the window event handler.
 */
 
bool CWindowEventHandlerList::findWindowEventHandler(CWindowEventHandler *eventHandler, int &index)
{
  for (int i = 0; i < m_eventHandlers.size(); ++i)
    {
      if (m_eventHandlers.at(i) == eventHandler)
        {
          index = i;
          return true;
        }
    }

  return false;
}

/**
 * Raise the NX window redraw event.
 */

void CWindowEventHandlerList::raiseRedrawEvent(void)
{
  for (int i = 0; i < m_eventHandlers.size(); ++i)
    {
      m_eventHandlers.at(i)->handleRedrawEvent();
    }
}

/**
 * Raise an NX window position/size change event.
 */

void CWindowEventHandlerList::raiseGeometryEvent(void)
{
  for (int i = 0; i < m_eventHandlers.size(); ++i)
    {
      m_eventHandlers.at(i)->handleGeometryEvent();
    }
}

/**
 * Raise an NX mouse window input event.
 */

#ifdef CONFIG_NX_MOUSE
void CWindowEventHandlerList::raiseMouseEvent(void)
{
  for (int i = 0; i < m_eventHandlers.size(); ++i)
    {
      m_eventHandlers.at(i)->handleMouseEvent();
    }
}
#endif

/**
 * Raise an NX keybord input event
 */

#ifdef CONFIG_NX_KBD
void CWindowEventHandlerList::raiseKeyboardEvent(void)
{
  for (int i = 0; i < m_eventHandlers.size(); ++i)
    {
      m_eventHandlers.at(i)->handleKeyboardEvent();
    }
#endif
}

/**
 * Raise an NX window blocked event.
 *
 * @param arg - User provided argument (see nx_block or nxtk_block)
 */

void CWindowEventHandlerList::raiseBlockedEvent(FAR void *arg)
{
  for (int i = 0; i < m_eventHandlers.size(); ++i)
    {
      m_eventHandlers.at(i)->handleBlockedEvent(arg);
    }
}
