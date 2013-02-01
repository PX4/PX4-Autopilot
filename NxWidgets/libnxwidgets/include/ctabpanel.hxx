/****************************************************************************
 * NxWidgets/libnxwidgets/include/ctabpanel.hxx
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

#ifndef __INCLUDE_CTABPANEL_HXX
#define __INCLUDE_CTABPANEL_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/nx/nxglib.h>

#include "cnxwidget.hxx"
#include "cwidgetstyle.hxx"
#include "cnxstring.hxx"
#include "tnxarray.hxx"
#include "clatchbuttonarray.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
 
/****************************************************************************
 * Implementation Classes
 ****************************************************************************/
 
#if defined(__cplusplus)

namespace NXWidgets
{
  /**
   * Forward references
   */

  class CWidgetControl;
  class CRect;
  class CStickyButtonArray;
  
  /**
   * Tab panel, with tabs at the top and a panel at the bottom.
   */

  class CTabPanel : public CNxWidget, public CWidgetEventHandler
  {
  protected:
    TNxArray<CNxWidget*> m_tabpages;
    CLatchButtonArray *m_buttonbar;
    
    virtual void handleActionEvent(const CWidgetEventArgs &e);
    
    virtual void drawContents(CGraphicsPort* port) {}
    virtual void drawBorder(CGraphicsPort* port) {}
  
  public:
    CTabPanel(CWidgetControl *pWidgetControl, uint8_t numPages,
              nxgl_coord_t x, nxgl_coord_t y,
              nxgl_coord_t width, nxgl_coord_t height,
              nxgl_coord_t buttonHeight,
              FAR const CWidgetStyle *style = (FAR const CWidgetStyle *)NULL
             );
    
    inline CNxWidget &page(uint8_t index) { return *m_tabpages.at(index); }
    
    void setPageName(uint8_t index, const CNxString &name);
    
    void showPage(uint8_t index);
  };
}

#endif
#endif
