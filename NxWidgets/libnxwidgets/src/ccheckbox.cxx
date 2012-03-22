/****************************************************************************
 * NxWidgets/libnxwidgets/src/ccheckbox.cxx
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
 ****************************************************************************
 *
 * Portions of this package derive from Woopsi (http://woopsi.org/) and
 * portions are original efforts.  It is difficult to determine at this
 * point what parts are original efforts and which parts derive from Woopsi.
 * However, in any event, the work of  Antony Dzeryn will be acknowledged
 * in most NxWidget files.  Thanks Antony!
 *
 *   Copyright (c) 2007-2011, Antony Dzeryn
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * * Neither the names "Woopsi", "Simian Zombie" nor the
 *   names of its contributors may be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY Antony Dzeryn ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Antony Dzeryn BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "nxconfig.hxx"
#include "cwidgetcontrol.hxx"
#include "ccheckbox.hxx"
#include "cgraphicsport.hxx"
#include "cbitmap.hxx"
#include "singletons.hxx"
#include "glyphs.hxx"

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
 * @param pWidgetControl The widget control for the display.
 * @param x The x coordinate of the checkbox, relative to its parent.
 * @param y The y coordinate of the checkbox, relative to its parent.
 * @param width The width of the checkbox.
 * @param height The height of the checkbox.
 * @param style The style that the widget should use.  If this is not
 *   specified, the widget will use the values stored in the global
 *   g_defaultWidgetStyle object.  The widget will copy the properties of
 * the style into its own internal style object.
 */

CCheckBox::CCheckBox(CWidgetControl *pWidgetControl,
                     nxgl_coord_t x, nxgl_coord_t y,
                     nxgl_coord_t width, nxgl_coord_t height,
                     CWidgetStyle *style)
: CButton(pWidgetControl, x, y, width, height, *NXWidgets::g_nullString, style)
{
  m_state             = CHECK_BOX_STATE_OFF;
  m_flags.borderless  = false;

  // Border width is one line

  m_borderSize.top    = 1;
  m_borderSize.right  = 1;
  m_borderSize.bottom = 1;
  m_borderSize.left   = 1;
}

/**
 * Set the state of the checkbox.
 *
 * @param state The new checkbox state.
 */

void CCheckBox::setState(CCheckBox::CheckBoxState state)
{
  if (m_state != state)
    {
      m_state = state;
      m_widgetEventHandlers->raiseValueChangeEvent();
      redraw();
    }
}

/**
 * Draw the area of this widget that falls within the clipping region.
 * Called by the redraw() function to draw all visible regions.
 *
 * @param port The CGraphicsPort to draw to.
 * @see redraw()
 */

void CCheckBox::drawContents(CGraphicsPort *port)
{
  // Get the X/Y position of the drawable region within the CheckBox

  nxgl_coord_t x = getX();
  nxgl_coord_t y = getY();

  nxgl_coord_t width  = getWidth();
  nxgl_coord_t height = getHeight();

  // Include and offset for the border

  if (!m_flags.borderless)
    {
      x      += m_borderSize.left;
      y      += m_borderSize.top;

      width  -= (m_borderSize.left + m_borderSize.right);
      height -= (m_borderSize.top + m_borderSize.bottom);
    }

  // Decide which glyph to draw

  const struct SBitmap *glyph;

  switch (m_state)
    {
    default:
    case CHECK_BOX_STATE_ON:
      glyph = &g_checkBoxOn;
      break;

    case CHECK_BOX_STATE_OFF:
      glyph = &g_checkBoxOff;
      break;

    case CHECK_BOX_STATE_MU:
      glyph = &g_checkBoxMu;
      break;
  }

  // Don't exceed the size of the glyph

  if (width > glyph->width)
    {
      width = glyph->width;
    }

  if (height > glyph->height)
    {
      height = glyph->height;
    }

  // Draw the checkbox

  port->drawBitmap(x, y, width, height, glyph, 0, 0,
                   CONFIG_NXWIDGETS_TRANSPARENT_COLOR);
}

/**
 * Draw the area of this widget that falls within the clipping region.
 * Called by the redraw() function to draw all visible regions.
 *
 * @param port The CGraphicsPort to draw to.
 * @see redraw()
 */

void CCheckBox::drawBorder(CGraphicsPort *port)
{
  // Determine the background color

  nxgl_mxpixel_t backColor;

  if (m_highlighted)
    {
      backColor = getSelectedBackgroundColor();
    }
  else
    {
      backColor = getBackgroundColor();
    }

  // Draw the background (excluding the border)

  port->drawFilledRect(getX(), getY(), getWidth(), getHeight(),
                       backColor);

  // Stop drawing if the widget indicates it should not have an outline

  if (!isBorderless())
    {
      port->drawBevelledRect(getX(), getY(), getWidth(), getHeight(),
                             getShineEdgeColor(), getShadowEdgeColor());
    }
}

/**
 * Toggles the state of the checkbox.
 *
 * @param x The x coordinate of the click.
 * @param y The y coordinate of the click.
 */

void CCheckBox::onClick(nxgl_coord_t x, nxgl_coord_t y)
{
  if (m_state == CHECK_BOX_STATE_ON)
    {
      setState(CHECK_BOX_STATE_OFF);
    }
  else
    {
      setState(CHECK_BOX_STATE_ON);
    }
}
