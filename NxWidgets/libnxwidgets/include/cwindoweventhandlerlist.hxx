/****************************************************************************
 * NxWidgets/libnxwidgets/include/cwindoweventhandlerlist.hxx
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

#ifndef __INCLUDE_CWINDOWEVENTHANDLERLIST_HXX
#define __INCLUDE_CWINDOWEVENTHANDLERLIST_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/nx/nxglib.h>

#include "cwindoweventhandler.hxx"
#include "tnxarray.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
 
/****************************************************************************
 * Implementation Classes
 ****************************************************************************/
 
#if defined(__cplusplus)

namespace NXWidgets
{
  class CListDataItem;

  /**
   * List of window event handlers.
   */
  class CWindowEventHandlerList
  {
  protected:
    TNxArray<CWindowEventHandler*> m_eventHandlers; /**< List of event handlers */

    /**
     * Return the index to the window event handler.
     */
 
    bool findWindowEventHandler(CWindowEventHandler *eventHandler, int &index);

  public:

    /**
     * Constructor.
     *
     * @param widget The owning widget.
     */
 
    CWindowEventHandlerList(void) { }

    /**
     * Destructor.
     */

    inline ~CWindowEventHandlerList(void) { }
    
    /**
     * Get the event handler at the specified index.
     *
     * @param index The index of the event handler.
     * @return The event handler at the specified index.
     */

    inline CWindowEventHandler *at(const int index) const
    {
      return m_eventHandlers.at(index);
    }

    /**
     * Get the size of the array.
     *
     * @return The size of the array.
     */

    inline const nxgl_coord_t size(void) const
    {
      return m_eventHandlers.size();
    }

    /**
     * Adds a window event handler.  The event handler will receive
     * all events raised by this object.
     * @param eventHandler A pointer to the event handler.
     */
 
    void addWindowEventHandler(CWindowEventHandler *eventHandler);

    /**
     * Remove a window event handler.
     *
     * @param eventHandler A pointer to the event handler to remove.
     */

    void removeWindowEventHandler(CWindowEventHandler *eventHandler);

    /**
     * Raise the NX window redraw event.
     */

    void raiseRedrawEvent(void);

    /**
     * Raise an NX window position/size change event.
     */

    void raiseGeometryEvent(void);

    /**
     * Raise an NX mouse window input event.
     */

#ifdef CONFIG_NX_MOUSE
    void raiseMouseEvent(void);
#endif

    /**
     * Raise an NX keybord input event
     */

#ifdef CONFIG_NX_KBD
    void raiseKeyboardEvent(void);
#endif

    /**
     * Raise an NX window blocked event.
     *
     * @param arg - User provided argument (see nx_block or nxtk_block)
     */

    void raiseBlockedEvent(FAR void *arg);
  };
}

#endif // __cplusplus

#endif // __INCLUDE_CWINDOWEVENTHANDLERLIST_HXX
