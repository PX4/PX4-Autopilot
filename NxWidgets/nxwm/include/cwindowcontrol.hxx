/****************************************************************************
 * NxWidgets/nxwm/include/cwindowcontrol.hxx
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

#ifndef __INCLUDE_CWINDOWCONTROL_HXX
#define __INCLUDE_CWINDOWCONTROL_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <sys/types.h>
#include <mqueue.h>

#include <nuttx/nx/nxtk.h>
#include <nuttx/nx/nxconsole.h>

#include "cwindoweventhandler.hxx"
#include "cwidgetcontrol.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Implementation Classes
 ****************************************************************************/

#if defined(__cplusplus)

namespace NxWM
{
  /**
   * Forward references.
   */

  class IApplication;

  /**
   * The class CWindowControl integrates the widget control with some special
   * handling of mouse and keyboard inputs neesed by NxWM
   */

  class CWindowControl : public NXWidgets::CWidgetControl,
                         private NXWidgets::CWindowEventHandler
  {
  private:
    mqd_t m_mqd; /**< Message queue descriptor used to commincate with the
                  **  start window thread. */
 
    /**
     * Handle an NX window mouse input event.
     *
     * @param e The event data.
     */

#ifdef CONFIG_NX_MOUSE
    void handleMouseEvent(void);
#endif

    /**
     * Handle a NX window keyboard input event.
     */

#ifdef CONFIG_NX_KBD
    void handleKeyboardEvent(void);
#endif

  public:

    /**
     * Constructor
     *
     * @param style The default style that all widgets on this display
     *   should use.  If this is not specified, the widget will use the
     *   values stored in the defaultCWidgetStyle object.
     */

     CWindowControl(FAR const NXWidgets::CWidgetStyle *style = (const NXWidgets::CWidgetStyle *)NULL);

    /**
     * Destructor.
     */

    ~CWindowControl(void);

    /**
     * Destroy the application window and everything in it.  This is
     * handled by CWindowControl (vs just calling the destructors) because
     * in the case where an application destroys itself (because of pressing
     * the stop button), then we need to unwind and get out of the application
     * logic before destroying all of its objects.
     */

    void destroy(IApplication *app);
  };
}
#endif // __cplusplus

#endif // __INCLUDE_CWINDOWCONTROL_HXX
