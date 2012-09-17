/////////////////////////////////////////////////////////////////////////////
// NxWidgets/UnitTests/CLabel/clabeltest.cxx
//
//   Copyright (C) 2012 Gregory Nutt. All rights reserved.
//   Author: Gregory Nutt <gnutt@nuttx.org>
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the
//    distribution.
// 3. Neither the name NuttX, NxWidgets, nor the names of its contributors
//    me be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
// OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
// AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
//////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
// Included Files
/////////////////////////////////////////////////////////////////////////////

#include <nuttx/config.h>

#include <nuttx/init.h>
#include <cstdio>
#include <cerrno>
#include <debug.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxfonts.h>

#include "nxconfig.hxx"
#include "clabeltest.hxx"
#include "cbgwindow.hxx"

/////////////////////////////////////////////////////////////////////////////
// Definitions
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
// Private Classes
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
// Private Data
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
// Public Data
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
// Public Function Prototypes
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
// CLabelTest Method Implementations
/////////////////////////////////////////////////////////////////////////////

// CLabelTest Constructor

CLabelTest::CLabelTest()
{
  m_bgWindow = (CBgWindow *)NULL;
  m_nxFont   = (CNxFont *)NULL;
  m_text     = (CNxString *)NULL;
}

// CLabelTest Descriptor

CLabelTest::~CLabelTest()
{
  disconnect();
}

// Connect to the NX server

bool CLabelTest::connect(void)
{
  // Connect to the server

  bool nxConnected = CNxServer::connect();
  if (nxConnected)
    {
      // Create the default font instance

      m_nxFont = new CNxFont(NXFONT_DEFAULT,
                            CONFIG_NXWIDGETS_DEFAULT_FONTCOLOR,
                            CONFIG_NXWIDGETS_TRANSPARENT_COLOR);
      if (!m_nxFont)
        {
          printf("CLabelTest::connect: Failed to create the default font\n");
        }

      // Set the background color

      if (!setBackgroundColor(CONFIG_CLABELTEST_BGCOLOR))
        {
          printf("CLabelTest::connect: setBackgroundColor failed\n");
        }
    }

  return nxConnected;
}

// Disconnect from the NX server

void CLabelTest::disconnect(void)
{
  // Close the window

  if (m_bgWindow)
    {
      delete m_bgWindow;
    }

  // Free the display string

  if (m_text)
    {
      delete m_text;
      m_text = (CNxString *)NULL;
    }

  // Free the default font

  if (m_nxFont)
    {
      delete m_nxFont;
      m_nxFont = (CNxFont *)NULL;
    }

  // And disconnect from the server

  CNxServer::disconnect();
}

// Create the background window instance.  This function illustrates
// the basic steps to instantiate any window:
//
// 1) Create a dumb CWigetControl instance
// 2) Pass the dumb CWidgetControl instance to the window constructor
//    that inherits from INxWindow.  This will "smarten" the CWidgetControl
//    instance with some window knowlede
// 3) Call the open() method on the window to display the window.
// 4) After that, the fully smartened CWidgetControl instance can
//    be used to generate additional widgets by passing it to the
//    widget constructor

bool CLabelTest::createWindow(void)
{
  // Initialize the widget control using the default style

  m_widgetControl = new CWidgetControl((CWidgetStyle *)NULL);

  // Get an (uninitialized) instance of the background window as a class
  // that derives from INxWindow.

  m_bgWindow = getBgWindow(m_widgetControl);
  if (!m_bgWindow)
    {
      printf("CLabelTest::createGraphics: Failed to create CBgWindow instance\n");
      delete m_widgetControl;
      return false;
    }

  // Open (and initialize) the window

  bool success = m_bgWindow->open();
  if (!success)
    {
      printf("CLabelTest::createGraphics: Failed to open background window\n");
      delete m_bgWindow;
      m_bgWindow = (CBgWindow*)0;
      return false;
    }

  return true;
}

// Create a CLabel instance

CLabel *CLabelTest::createLabel(FAR const char *text)
{
  // Get the width of the display

  struct nxgl_size_s windowSize;
  if (!m_bgWindow->getSize(&windowSize))
    {
      printf("CLabelTest::createGraphics: Failed to get window size\n");
      return (CLabel *)NULL;
    }

  // Create a CNxString instance to contain the C string

  m_text = new CNxString(text);

  // Get the height and width of the text display area

  nxgl_coord_t stringWidth  = m_nxFont->getStringWidth(*m_text);
  nxgl_coord_t stringHeight = (nxgl_coord_t)m_nxFont->getHeight();

  // The default CLabel has borders enabled with thickness of the border
  // width.  Add twice the thickness of the border to the width and height. (We
  // could let CLabel do this for us by calling CLabel::getPreferredDimensions())

  stringWidth  += 2 * 1;
  stringHeight += 2 * 1;

  // Pick an X/Y position such that the label will be centered in the display

  nxgl_coord_t labelX;
  if (stringWidth >= windowSize.w)
    {
      labelX = 0;
    }
  else
    {
      labelX = (windowSize.w - stringWidth) >> 1;
    }

  nxgl_coord_t labelY;
  if (stringHeight >= windowSize.h)
    {
      labelY = 0;
    }
  else
    {
      labelY = (windowSize.h - stringHeight) >> 1;
    }

  // Now we have enough information to create the label

  return new CLabel(m_widgetControl, labelX, labelY, stringWidth, stringHeight, *m_text);
}

// Draw the label

void CLabelTest::showLabel(CLabel *label)
{
  label->enable();
  label->enableDrawing();
  label->redraw();
  label->disableDrawing();
}
