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
  * @param pWidgetControl Control object associated with this window
  */

CCallback::CCallback(CWidgetControl *pWidgetControl)
{
  // Initialize the callback vtable

  m_callbacks.redraw   = redraw;
  m_callbacks.position = position;
  m_callbacks.mousein  = newMouseEvent;
  m_callbacks.kbdin    = newKeyboardEvent;
}

 /**
  * ReDraw Callback.  The redraw action is handled by CWidgetControl:redrawEvent.
  *
  * @param hWindow Handle to a specific NX window.
  * @param pRect The rectangle that needs to be re-drawn (in window
  * relative coordinates).
  * @param bMore true: More re-draw requests will follow.
  * @param pvArg User provided argument (see nx_openwindow, nx_requestbg,
  * nxtk_openwindow, or nxtk_opentoolbar).
  */
     
void CCallback::redraw(NXHANDLE hWindow,
                       FAR const struct nxgl_rect_s *pRect,
                       bool bMore, FAR void *pvArg)
{
  gvdbg("hWindow=%p pRect={(%d,%d),(%d,%d)} bMore=%s\n",
         hWindow,
         pRect->pt1.x, pRect->pt1.y, pRect->pt2.x, pRect->pt2.y,
         bMore ? "true" : "false");

  // The argument must be the CWidgetControl instance

  CWidgetControl *This = (CWidgetControl *)pvArg;

  // Just forward the callback to the CWidgetControl::redrawEvent method

  This->redrawEvent(pRect, bMore);
}

 /**
  * Position Callback. The new positional data is handled by
  * CWidgetControl::geometryEvent.
  *
  * @param hWindow Handle to a specific NX window.
  * @param pSize The size of the window.
  * @param pPos The position of the upper left hand corner of the window on
  * the overall display.
  * @param pBounds The bounding rectangle that describes the entire display.
  * @param pvArg User provided argument (see nx_openwindow, nx_requestbg,
  * nxtk_openwindow, or nxtk_opentoolbar).
  */

void CCallback::position(NXHANDLE hWindow,
                         FAR const struct nxgl_size_s *pSize,
                         FAR const struct nxgl_point_s *pPos,
                         FAR const struct nxgl_rect_s *pBounds,
                         FAR void *pvArg)
{
  gvdbg("hWindow=%p pSize=(%d,%d) pPos=(%d,%d) pBounds={(%d,%d),(%d,%d)} pvArg=%p\n",
        hWindow, pSize->w, pSize->h, pPos->x, pPos->y,
        pBounds->pt1.x, pBounds->pt1.y, pBounds->pt2.x, pBounds->pt2.y,
        pvArg);


  // The argument must be the CWidgetControl instance

  CWidgetControl *This = (CWidgetControl *)pvArg;

  // Just forward the callback to the CWidgetControl::geometry method

  This->geometryEvent(hWindow, pSize, pPos, pBounds);
}

 /**
  * New mouse data is available for the window.  The new mouse data is
  * handled by CWidgetControl::newMouseEvent.
  *
  * @param hWindow Handle to a specific NX window.
  * @param pPos The (x,y) position of the mouse.
  * @param buttons See NX_MOUSE_* definitions.
  * @param pvArg User provided argument (see nx_openwindow, nx_requestbg,
  * nxtk_openwindow, or nxtk_opentoolbar).
  */
     
#ifdef CONFIG_NX_MOUSE
void CCallback::newMouseEvent(NXHANDLE hWindow,
                              FAR const struct nxgl_point_s *pPos,
                              uint8_t buttons, FAR void *pvArg)
{
  gvdbg("hWindow=%p pPos=(%d,%d) buttons=%02x pvArg=%p\n", 
        hWindow, pPos->x, pPos->y, buttons, pvArg);

  // The argument must be the CWidgetControl instance

  CWidgetControl *This = (CWidgetControl *)pvArg;

  // Just forward the callback to the CWidgetControl::newMouseEvent method

  This->newMouseEvent(pPos, buttons);
}
#endif /* CONFIG_NX_MOUSE */

/**
 * New keyboard/keypad data is available for the window.  The new keyboard
 * data is handled by CWidgetControl::newKeyboardEvent.
 *
 * @param hWindow Handle to a specific NX window.
 * @param nCh The number of characters that are available in pStr[].
 * @param pStr The array of characters.
 * @param pvArg User provided argument (see nx_openwindow, nx_requestbg,
 * nxtk_openwindow, or nxtk_opentoolbar).
 */

#ifdef CONFIG_NX_KBD
void CCallback::newKeyboardEvent(NXHANDLE hWindow, uint8_t nCh,
                                 FAR const uint8_t *pStr,
                                 FAR void *pvArg)
{
  gvdbg("hWindow=%p nCh=%d pvArg=%p\n",
        hWindow, nCh, pvArg);

  // The argument must be the CWidgetControl instance

  CWidgetControl *This = (CWidgetControl *)pvArg;

  // Just forward the callback to the CWidgetControl::newKeyboardEvent method

  This->newKeyboardEvent(nCh, pStr);
}
#endif // CONFIG_NX_KBD
