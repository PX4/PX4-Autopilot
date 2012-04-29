/********************************************************************************************
 * NxWidgets/nxwm/src/cnxtaskbar.cxx
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
 ********************************************************************************************/

/********************************************************************************************
 * Included Files
 ********************************************************************************************/
 
#include <nuttx/config.h>

#include "nxwmconfig.hxx"
#include "nxwmglyphs.hxx"
#include "cappliationwinow.hxx"

/********************************************************************************************
 * Pre-Processor Definitions
 ********************************************************************************************/

/********************************************************************************************
 * CNxApplicationWindow Method Implementations
 ********************************************************************************************/

 using namespace NxWM;

/**
 * CNxApplicationWindow Constructor
 *
 * @param taskbar.  A pointer to the parent task bar instance
 * @param window.  The window to be used by this application.
 */

CNxApplicationWindow::CNxApplicationWindow(NxWidgets::CNxTkWindow *window);
{
  // Save the window for later use

  m_window          = window;

  // These will be created with the open method is called

  m_toolbar         = (NxWidgets::CNxToolbar        *)0;

  m_minimizeImage   = (NxWidgets::CImage            *)0;
  m_stopImage       = (NxWidgets::CImage            *)0;
  m_windowLabel     = (NxWidgets::CLabel            *)0;

  m_minimizeBitmap  = (NxWidgets::CRlePaletteBitmap *)0;
  m_stopBitmap      = (NxWidgets::CRlePaletteBitmap *)0;

  m_windowFont      = (NxWidgets::CNxFont           *)0;

  // This will be initialized when the registerCallbacks() method is called

  m_callback        = (IApplicationCallback         *)0;
}

/**
 * CNxApplicationWindow Destructor
 */

CNxApplicationWindow::~CNxApplicationWindow(void)
{
  // Free the resources that we are responsible for

  if (m_minimizeImage)
    {
      delete m_minimizeImage;
    }

  if (m_stopImage)
    {
      delete m_stopImage;
    }

  if (m_windowLabel)
    {
      delete m_windowLabel;
    }

  if (m_minimizeBitmap)
    {
      delete m_minimizeBitmap;
    }

  if (m_stopBitmap)
    {
      delete m_stopBitmap;
    }

  if (m_windowFont)
    {
      delete m_windowFont;
    }

  if (m_toolbar)
    {
      delete m_toolbar;
    }

  // We didn't create the window.  That was done by the task bar,
  // Be we will handle destruction of with window as a courtesy.

  if (m_window)
    {
      delete m_window;
    }
}

/**
 * Initialize window.  Window initialization is separate from
 * object instantiation so that failures can be reported.
 *
 * @return True if the window was successfully initialized.
 */

bool CNxApplicationWindow::open(void)
{
  /* Configure the toolbar */

  if (!configureToolbar())
    {
      return false;
    }

  return true;
}

/**
 * Configure the standard application toolbar
 *
 * @return True if the toolcar was successfully initialized.
 */
 
bool configureToolbar(void)
{
  // Open the toolbar

  m_toolbar = m_window->openToolbar();
  if (!m_toolbar)
    {
      // We failed to open the toolbar

      return false;
    }

  // Get the width of the display

  struct nxgl_size_s windowSize;
  if (!m_toolbar->getSize(&windowSize))
    {
      return false;
    }

  // Get the CWidgetControl associated with this window

  NxWidgets::CWidgetControl *control = m_toolbar->getWidgetControl();
  if (control)
    {
      return false;
    }

  // Create STOP bitmap container

  m_stopBitmap = new NxWidgets::CRlePaletteBitmap(&g_stopBitmap);
  if (!m_stopBitmap)
    {
      return false;
    }

  // Create the STOP application icon at the right of the toolbar

  nxgl_point_t iconPos;
  nxgl_size_t  iconSize;

  // Get the height and width of the stop bitmap

  iconSize.w = m_stopBitmap->getWidth();
  iconSize.h = m_stopBitmap->getHeight();

  // The default CImage has borders enabled with thickness of the border
  // width.  Add twice the thickness of the border to the width and height.
  // (We could let CImage do this for us by calling
  // CImage::getPreferredDimensions())

  iconSize.w += 2 * 1;
  iconSize.h += 2 * 1;

  // Pick an X/Y position such that the image will position at the right of
  // the toolbar and centered vertically.

  iconPos.x = windowSize.w - iconsize.w;

  if (iconSize.h >= windowSize.h)
    {
      iconPos.y = 0;
    }
  else
    {
      iconPos.y = (windowSize.h - iconSize.h) >> 1;
    }

  // Now we have enough information to create the image

  m_stopImage = new CImage(control, iconPos.x, iconPos.y, iconSize.w, iconSize.h, m_stopBitmap);
  if (!m_stopImage)
    {
      return false;
    }

  // Create MINIMIZE application bitmap container

  m_minimizeBitmap = new NxWidgets::CRlePaletteBitmap(&g_minimizeBitmap);
  if (!m_minimizeBitmap)
    {
      return false;
    }

  // Get the height and width of the stop bitmap

  iconSize.w = m_minimizeBitmap->getWidth();
  iconSize.h = m_minimizeBitmap->getHeight();

  // The default CImage has borders enabled with thickness of the border
  // width.  Add twice the thickness of the border to the width and height.
  // (We could let CImage do this for us by calling
  // CImage::getPreferredDimensions())

  iconSize.w += 2 * 1;
  iconSize.h += 2 * 1;

  // Pick an X/Y position such that the image will position at the right of
  // the toolbar and centered vertically.

  iconPos.x -= iconsize.w;

  if (iconSize.h >= windowSize.h)
    {
      iconPos.y = 0;
    }
  else
    {
      iconPos.y = (windowSize.h - iconSize.h) >> 1;
    }

  // Now we have enough information to create the image

  m_minimizeImage = new CImage(control, iconPos.x, iconPos.y, iconSize.w, iconSize.h, m_minimizeBitmap);
  if (!m_minimizeImage)
    {
      return false;
    }

  // The rest of the toolbar will hold the left-justified application label
  // Create the default font instance

  m_windowFont = new CNxFont(CONFIG_NXWM_DEFAULT_FONTID,
                             CONFIG_NXWM_DEFAULT_FONTCOLOR,
                             CONFIG_NXWM_TRANSPARENT_COLOR);
  if (!m_windowFont)
    {
      return false;
    }
  // Get the width of the display

  struct nxgl_size_s windowSize;
  if (!m_bgWindow->getSize(&windowSize))
    {
      printf("CLabelTest::createGraphics: Failed to get window size\n");
      return (CLabel *)NULL;
    }

  // Get the height and width of the text display area

  size.w = pos.x
  size.h = windowSize.h;

  pos.x  = 0;
  pos.y  = 0;

  // Now we have enough information to create the label

  m_windowLabel = new CLabel(control, pos.x, pos.y, size.w, size.h, "");
  if (!m_windowLabel)
    {
      return false;
    }

  // Configure the label

  m_windowLabel->setBorderLess(true);
  m_windowLabel->setTextAlignmentHoriz(NxWidgets::TEXT_ALIGNMENT_HORIZ_LEFT);
  m_windowLabel->setTextAlignmentVert(NxWidgets::TEXT_ALIGNMENT_VERT_CENTER);
  return true;
}


