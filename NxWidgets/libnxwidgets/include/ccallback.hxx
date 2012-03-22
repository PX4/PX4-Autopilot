/****************************************************************************
 * NxWidgets/libnxwidgets/include/ccallback.hxx
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

#ifndef __INCLUDE_CCALLBACK_HXX
#define __INCLUDE_CCALLBACK_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxtk.h>

#include "crect.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
 
/****************************************************************************
 * Implementation Classes
 ****************************************************************************/
 
#if defined(__cplusplus)

namespace NXWidgets
{
  class CWidgetControl;

  /**
   * Callback function proxies.  This class receives and dispatches callbacks
   * from the NX server.  This calls also manages a few lower-level details
   * such as keeping track of the reported window handles and window positions
   * and sizes.
   *
   * There are three instances that represent an NX window from the
   * perspective of NXWidgets.
   *
   * - There is one widget control instance per NX window,
   * - One CCallback instance per window,
   * - One window instance.
   *
   * There a various kinds of of window instances, but each inherits
   * (1) CCallback and dispatches the Windows callbacks and (2) INxWindow
   * that describes the common window behavior.

   */

  class CCallback
  {
  private:
    struct nx_callback_s m_callbacks;  /**< C-callable vtable of callback function pointers */

    // Methods in the callback vtable
 
    /**
     * Re-Draw Callback.  The redraw event is handled by CWidgetControl::redrawEvent.
     *
     * NOTE:  This method runs in the context of the NX callback which may
     * either be the context of the owning thread or, in the case of multi-
     * user NX, the context of the NX event listener thread.
     *
     * @param hWindow Handle to a specific NX window.
     * @param pRect The rectangle that needs to be re-drawn (in window
     * relative coordinates).
     * @param bMore true: More re-draw requests will follow.
     * @param pvArg User provided argument (see nx_openwindow, nx_requestbg,
     * nxtk_openwindow, or nxtk_opentoolbar).
     */
     
    static void redraw(NXHANDLE hWindow, FAR const struct nxgl_rect_s *pRect,
                       bool bMore, FAR void *pvArg);

    /**
     * Position Callback.  The new positional data is handled by
     * CWidgetControl::geometryEvent.
     *
     * NOTE:  This method runs in the context of the NX callback which may
     * either be the context of the owning thread or, in the case of multi-
     * user NX, the context of the NX event listener thread.
     *
     * @param hWindow Handle to a specific NX window.
     * @param pSize The size of the window.
     * @param pPos The position of the upper left hand corner of the window on
     * the overall display.
     * @param pBounds The bounding rectangle that describes the entire display.
     * @param pvArg User provided argument (see nx_openwindow, nx_requestbg,
     * nxtk_openwindow, or nxtk_opentoolbar).
     */

    static void position(NXHANDLE hWindow, FAR const struct nxgl_size_s *pSize,
                         FAR const struct nxgl_point_s *pPos,
                         FAR const struct nxgl_rect_s *pBounds,
                         FAR void *pvArg);

    /**
     * New mouse data is available for the window.  The new mouse
     * data is handled by CWidgetControl::newMouseEvent.
     *
     * NOTE:  This method runs in the context of the NX callback which may
     * either be the context of the NX event listener thread (if multi-
     * user NX), or possibly in the connects of device driver or even a
     * device driver interrupt.
     *
     * The GUI thread is probably sleeping a semaphore, waiting to be
     * awakened by a mouse or keyboard event.
     *
     * @param hWindow Handle to a specific NX window.
     * @param pPos The (x,y) position of the mouse.
     * @param buttons See NX_MOUSE_* definitions.
     * @param pvArg User provided argument (see nx_openwindow, nx_requestbg,
     * nxtk_openwindow, or nxtk_opentoolbar).
     */
     
#ifdef CONFIG_NX_MOUSE
    static void newMouseEvent(NXHANDLE hWindow, FAR const struct nxgl_point_s *pPos,
                              uint8_t buttons, FAR void *pvArg);
#endif /* CONFIG_NX_MOUSE */

    /**
     * New keyboard/keypad data is available for the window.  The new
     * keyboard data is handled by CWidgetControl::newKeyboardEvent.
     *
     * NOTE:  This method runs in the context of the NX callback which may
     * either be the context of the NX event listener thread (if multi-
     * user NX), or possibly in the connects of device driver or even a
     * device driver interrupt.
     *
     * The GUI thread is probably sleeping a semaphore, waiting to be
     * awakened by a mouse or keyboard event.
     *
     * @param hWindow Handle to a specific NX window.
     * @param nCh The number of characters that are available in pStr[].
     * @param pStr The array of characters.
     * @param pvArg User provided argument (see nx_openwindow, nx_requestbg,
     * nxtk_openwindow, or nxtk_opentoolbar).
     */

#ifdef CONFIG_NX_KBD
    static void newKeyboardEvent(NXHANDLE hWindow, uint8_t nCh, FAR const uint8_t *pStr,
                                 FAR void *pvArg);
#endif // CONFIG_NX_KBD

  public:

    /**
     * Constructor.
     *
     * @param pWidgetControl Control object associated with this window
     */

    CCallback(CWidgetControl *pWidgetControl);

    /**
     * Destructor.
     */

    inline ~CCallback(void) {}

    /**
     * Get the callback vtable.  This is neeed only by the window
     * instance that inherits this class.  The window instance needs the
     * C-callable vtable in order to create the NX window.  Once the
     * window is created, this class will begin to receive callbacks via
     * the C-callable vtable methods.
     *
     * @return This method returns the C-callable vtable needed for
     *         NX window creation.
     */

    inline FAR struct nx_callback_s *getCallbackVTable(void)
    {
      return &m_callbacks;
    }
  };
}

#endif // __cplusplus

#endif // __INCLUDE_CCALLBACK_HXX

