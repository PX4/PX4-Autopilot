/****************************************************************************
 * NxWidgets/libnxwidgets/src/clatchbuttonarray.cxx
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

#include <nuttx/nx/nxglib.h>

#include "cstickybuttonarray.hxx"
#include "clatchbuttonarray.hxx"
#include "cgraphicsport.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * CButton Method Implementations
 ****************************************************************************/

using namespace NXWidgets;

/**
 * Constructor for an array of latch buttons.
 *
 * @param pWidgetControl The widget control for the display.
 * @param x The x coordinate of the button array, relative to its parent.
 * @param y The y coordinate of the button array, relative to its parent.
 * @param buttonColumns The number of buttons in one row of the button array
 * @param buttonRows The number of buttons in one column of the button array
 * @param buttonWidth The width of one button
 * @param buttonHeight The height of one button
 * @param style The style that the button should use.  If this is not
 *        specified, the button will use the global default widget
 *        style.
 */

CLatchButtonArray::CLatchButtonArray(CWidgetControl *pWidgetControl,
                                     nxgl_coord_t x, nxgl_coord_t y,
                                     uint8_t buttonColumns, uint8_t buttonRows,
                                     nxgl_coord_t buttonWidth, nxgl_coord_t buttonHeight,
                                     CWidgetStyle *style)
: CStickyButtonArray(pWidgetControl, x, y, buttonColumns, buttonRows, buttonWidth, buttonHeight, style)
{
}

/**
 * Handles button click events
 *
 * @param x The x coordinate of the click.
 * @param y The y coordinate of the click.
 */

void CLatchButtonArray::onClick(nxgl_coord_t x, nxgl_coord_t y)
{
  // We need to save the button position so that when CButtonArray
  // gets the release, it will have the correct position.  This would
  // look nicer if we just called CButtonArray::onClock() but that
  // would cause duplicate redraw()

  m_clickX = x;
  m_clickY = y;

  // Convert the new key press to row/columin indices

  int column;
  int row;

  if (posToButton(x, y, column, row))
    {
      // Change the stuck down button on each in-range button press
      // NOTE that normally button click events are not processed (but
      // button release events will be processed).

      stickDown(column, row);
    }
}
