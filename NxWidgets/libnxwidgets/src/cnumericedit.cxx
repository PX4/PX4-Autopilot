/****************************************************************************
 * NxWidgets/libnxwidgets/include/cnumericedit.cxx
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Petteri Aimonen <jpa@kapsi.fi>
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
 * in all NxWidget files.  Thanks Antony!
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
#include <stdio.h>
#include <limits.h>

#include <nuttx/nx/nxglib.h>
#include <debug.h>

#include "cnumericedit.hxx"
#include "cbutton.hxx"
#include "clabel.hxx"
#include "cnxtimer.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * CNumericEdit Method Implementations
 ****************************************************************************/

using namespace NXWidgets;

// Label that passes drag events

class CDraggableLabel: public CLabel
{
public:
  CDraggableLabel(CWidgetControl *pWidgetControl, nxgl_coord_t x, nxgl_coord_t y,
           nxgl_coord_t width, nxgl_coord_t height, const CNxString &text,
           CWidgetStyle *style = (CWidgetStyle *)NULL):
    CLabel(pWidgetControl, x, y, width, height, text, style)
  {
    setDraggable(true);
  }

  virtual void onClick(nxgl_coord_t x, nxgl_coord_t y)
  {
    startDragging(x, y);
  }
};

/**
  * Constructor for a numeric edit control.
  *
  * @param pWidgetControl The controlling widget for the display
  * @param x The x coordinate of the text box, relative to its parent.
  * @param y The y coordinate of the text box, relative to its parent.
  * @param width The width of the textbox.
  * @param height The height of the textbox.
  * @param style The style that the button should use.  If this is not
  *        specified, the button will use the global default widget
  *        style.
  */

CNumericEdit::CNumericEdit(CWidgetControl *pWidgetControl, nxgl_coord_t x, nxgl_coord_t y,
        nxgl_coord_t width, nxgl_coord_t height,
        CWidgetStyle *style)
: CNxWidget(pWidgetControl, x, y, width, height, 0, style)
{
  m_label = new CDraggableLabel(pWidgetControl, height, 0, width - 2 * height, height, CNxString("0"), style);
  m_label->addWidgetEventHandler(this);
  addWidget(m_label);

  m_button_minus = new CButton(pWidgetControl, 0, 0, height, height, CNxString("-"));
  m_button_minus->addWidgetEventHandler(this);
  addWidget(m_button_minus);

  m_button_plus = new CButton(pWidgetControl, width - height, 0, height, height, CNxString("+"));
  m_button_plus->addWidgetEventHandler(this);
  addWidget(m_button_plus);

  m_timer = new CNxTimer(pWidgetControl, 100, true);
  m_timer->addWidgetEventHandler(this);
  addWidget(m_timer);

  m_minimum = INT_MIN;
  m_maximum = INT_MAX;
  m_increment = 1;
  setValue(0);
}

CNumericEdit::~CNumericEdit()
{
  // CNxWidget destroys all children

  m_label = 0;
  m_button_minus = 0;
  m_button_plus = 0;
}

void CNumericEdit::getPreferredDimensions(CRect &rect) const
{
}

void CNumericEdit::setFont(CNxFont *font)
{
  m_label->setFont(font);
}

void CNumericEdit::onResize(nxgl_coord_t width, nxgl_coord_t height)
{
}

void CNumericEdit::handleClickEvent(const CWidgetEventArgs &e)
{
  if (e.getSource() == m_button_plus)
    {
      setValue(m_value + m_increment);
      m_timercount = 0;
      m_timer->start();
    }
  else if (e.getSource() == m_button_minus)
    {
      setValue(m_value - m_increment);
      m_timercount = 0;
      m_timer->start();
    }
}

void CNumericEdit::handleReleaseEvent(const CWidgetEventArgs &e)
{
  if (e.getSource() == m_button_plus || e.getSource() == m_button_minus)
    {
      m_timer->stop();
    }
}

void CNumericEdit::handleReleaseOutsideEvent(const CWidgetEventArgs &e)
{
  if (e.getSource() == m_button_plus || e.getSource() == m_button_minus)
    {
      m_timer->stop();
    }
}

void CNumericEdit::handleActionEvent(const CWidgetEventArgs &e)
{
  if (e.getSource() == m_timer)
    {
      m_timercount++;

      int increment = m_increment;
      if (m_timercount > 50)
        {
          increment = m_increment * 100;
        }
      else if (m_timercount > 10)
        {
          increment = m_increment * 10;
        }

      if (m_button_minus->isClicked())
        {
          setValue(m_value - increment);
        }
      else if (m_button_plus->isClicked())
        {
          setValue(m_value + increment);
        }
    }
}

void CNumericEdit::handleDragEvent(const CWidgetEventArgs &e)
{
  int x = e.getX() - m_label->getX();
  int width = m_label->getWidth();
  int value = m_minimum + (m_maximum - m_minimum) * x / width;
  setValue(value / m_increment * m_increment);
}

void CNumericEdit::setValue(int value)
{
  if (value < m_minimum) value = m_minimum;
  if (value > m_maximum) value = m_maximum;

  m_value = value;

  char buf[10];
  snprintf(buf, sizeof(buf), "%d", m_value);
  CNxString text(buf);

  m_label->setText(text);

  m_widgetEventHandlers->raiseValueChangeEvent();

  redraw();
}
