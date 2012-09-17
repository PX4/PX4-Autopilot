/****************************************************************************
 * NxWidgets/libnxwidgets/include/cstickybuttonarray.cxx
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

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/nx/nxglib.h>

#include "cstickybuttonarray.hxx"
#include "cgraphicsport.hxx"
#include "cwidgetstyle.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
 
/****************************************************************************
 * CStickyButtonArray Method Implementations
 ****************************************************************************/

using namespace NXWidgets;

/**
 * Constructor for an array of sticky buttons.
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

CStickyButtonArray::CStickyButtonArray(CWidgetControl *pWidgetControl,
                                       nxgl_coord_t x, nxgl_coord_t y,
                                       uint8_t buttonColumns, uint8_t buttonRows,
                                       nxgl_coord_t buttonWidth, nxgl_coord_t buttonHeight,
                                       CWidgetStyle *style)
: CButtonArray(pWidgetControl, x, y, buttonColumns, buttonRows, buttonWidth, buttonHeight, style)
{
  // Initialize state

  m_isStuckDown = false;
  m_stickDown   = false;
}

/**
 * Check if any button in the array is stuck down and, if so,
 * return the indices of the stuck down button.
 *
 * @param column The location to return the column index of the button
 *    of interest
 * @param row The location to return the row index of the button of
 *    interest
 * @return True if a button in the array is clicked
 */

bool CStickyButtonArray::isAnyButtonStuckDown(int &column, int &row) const
{
  // Return the indices of the last stuck button

  column = m_stickyColumn;
  row    = m_stickyRow;

  // Return true if the button is currently stuck

  return m_isStuckDown;
}

/**
 * Check if this specific button in the array is stuck down
 *
 * @param column The column of the button to check.
 * @param row The row of the button to check.
 * @return True if this button is stuck down
 */

bool CStickyButtonArray::isThisButtonStuckDown(int column, int row) const
{
  // Does this match the index of the last stuck button?

  if (column == m_stickyColumn && row == m_stickyRow)
    {
      // Yes.. return true if it is still stuck

      return m_isStuckDown;
    }
  else
    {
       // This button is definitely not stuck down

       return false;
     }
}

/**
 * Force the button at this position into the stuck down state
 *
 * @param column The column containing the button to stick down
 * @param row The rowtcontaining the button to stick down
 * @return False(0) is returned if the indices are out of range.
 */

bool CStickyButtonArray::stickDown(int column, int row)
{
  // Verify that the cursor position is within range

  if ((unsigned)column < m_buttonColumns && (unsigned)row < m_buttonRows)
    {
      // First, unstick any currently stuck buttons

      unstick();

      // Then stick this button down

      m_stickyColumn = column;
      m_stickyRow    = row;
      m_isStuckDown  = true;

      // We only want to update the stuck down buttons

      m_stickDown    = true;
      redraw();
      m_stickDown    = false;
      return true;
    }

  return false;
}

/**
 * Unstick all buttons.
 */

void CStickyButtonArray::unstick(void)
{
  // Is any button stuck down?

  if (m_isStuckDown)
    {
      // Yes.. unstick it and update the button display

      m_isStuckDown = false;

      // We only want to update the stuck down buttons

      m_stickDown   = true;
      redraw();
      m_stickDown   = false;
   }
}

/**
 * Draw the area of this widget that falls within the clipping region.
 * Called by the redraw() function to draw all visible regions.
 *
 * @param port The CGraphicsPort to draw to.
 * @see redraw()
 */

void CStickyButtonArray::drawContents(CGraphicsPort *port)
{
  // Are we just updating the stuck down button?

  if (m_stickDown)
    {
      // There are two cases:  (1) A sticky button was unstuck, or (2) a new
      // button is stuck down.  m_isStuckDown will distinguish these
      //
      // Draw the button using the clicked style if either (1) it is
      // stuck down OR (2) it is currently clicked.  Draw the button in
      // the normal style if is is un-stuck and not clicked.
      //
      // NOTE: If usedClicked is false, the button may revert to either
      // (1) the "normal" button state, or (2) the "highlighted" button
      // stated.  drawButton() will handle that.

      bool useClicked = m_isStuckDown || isThisButtonClicked(m_stickyColumn, m_stickyRow);
      drawButton(port, m_stickyColumn, m_stickyRow, useClicked);
    }

  // Do we draw just one button (due to click or release)? 

  else if (m_redrawButton)
    {
      // Just one.  Get the row/column indices from the last click

      int column;
      int row;
      bool useClicked = isButtonClicked(column, row);

      // If this is a press or release on the stuck down button, then don't
      // redraw the button.  Drawing of stuck down and unstuck buttons is
      // controlled entirely by the m_stickDown case.

      if (!isThisButtonStuckDown(column, row))
        {
          // Not a stuck down button. Re-draw that button

          drawButton(port, column, row, useClicked);
        }
    }

  // Do we just draw the hightlighted button?

  else if (m_cursorChange)
    {
      // Do nothing if the highlighted button is also the stuck down button
      // or the clicked button

      if (!isThisButtonStuckDown(m_cursorColumn, m_cursorRow) &&
          !isThisButtonClicked(m_cursorColumn, m_cursorRow))
        {
          drawButton(port, m_cursorColumn, m_cursorRow, false);
        }
    }

  // Draw all buttons

  else
    {
      // Visit each column

      for (int column = 0; column < m_buttonColumns; column++)
        {
          // Visit each row

          for (int row = 0; row < m_buttonRows; row++)
            {
              // Is the button clicked?

              bool useClicked = isThisButtonClicked(column, row);

              // If the button is not clicked but it is the stuck button,
              // then draw it as clicked anyway.

              if (!useClicked)
                {
                  useClicked =  isThisButtonStuckDown(column, row);
                }

               // Draw each button.  If useClicked is false, then drawButton
               // will do the right thing if the button is highlighted.

               drawButton(port, column, row, useClicked);
            }
        }
    }
}

