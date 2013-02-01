/****************************************************************************
 * NxWidgets/libnxwidgets/src/ctabpanel.hxx
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Petteri Aimonen <jpa@kapsi.fi>
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

#include "ctabpanel.hxx"
#include "cgraphicsport.hxx"
#include "cwidgetstyle.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
 
/****************************************************************************
 * CTabPanel Method Implementations
 ****************************************************************************/

using namespace NXWidgets;

CTabPanel::CTabPanel(CWidgetControl *pWidgetControl, uint8_t numPages,
                     nxgl_coord_t x, nxgl_coord_t y,
                     nxgl_coord_t width, nxgl_coord_t height,
                     nxgl_coord_t buttonHeight,
                     FAR const CWidgetStyle *style
                    ):
  CNxWidget(pWidgetControl, x, y, width, height, 0, style)
{
  m_buttonbar = new CLatchButtonArray(pWidgetControl, x, y,
                                      numPages, 1,
                                      width / numPages,
                                      buttonHeight,
                                      0);
  m_buttonbar->addWidgetEventHandler(this);
  this->addWidget(m_buttonbar);
  
  for (int i = 0; i < numPages; i++)
    {
      CNxWidget *tabpage = new CNxWidget(pWidgetControl, x, y + buttonHeight,
                                         width, height - buttonHeight, 0);
      tabpage->setBackgroundColor(getBackgroundColor());
      tabpage->setBorderless(true);
      m_tabpages.push_back(tabpage);
      this->addWidget(tabpage);
    }
  
  // Activate the first page

  showPage(0);
}

void CTabPanel::setPageName(uint8_t index, const CNxString &name)
{
  m_buttonbar->setText(index, 0, name);
}

void CTabPanel::showPage(uint8_t index)
{
  if (!m_buttonbar->isThisButtonStuckDown(index, 0))
    {
      m_buttonbar->stickDown(index, 0);
    }
  
  for (int i = 0; i < m_tabpages.size(); i++)
    {
      if (i == index)
        {
          m_tabpages.at(i)->enable();
          m_tabpages.at(i)->show();
          m_tabpages.at(i)->redraw();
        }
      else
        {
          m_tabpages.at(i)->hide();
          m_tabpages.at(i)->disable();
        }
    }
}

void CTabPanel::handleActionEvent(const CWidgetEventArgs &e)
{
  if (e.getSource() == m_buttonbar)
    {
      int x = 0;
      int y = 0;

      m_buttonbar->isAnyButtonStuckDown(x, y);
      showPage(x);
    }
}



