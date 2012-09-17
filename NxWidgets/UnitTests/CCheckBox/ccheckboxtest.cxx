/////////////////////////////////////////////////////////////////////////////
// NxWidgets/UnitTests/CCheckBox/ccheckboxtest.cxx
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
#include "cbgwindow.hxx"
#include "ccheckboxtest.hxx"
#include "cbitmap.hxx"
#include "glyphs.hxx"

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
// CCheckBoxTest Method Implementations
/////////////////////////////////////////////////////////////////////////////

// CCheckBoxTest Constructor

CCheckBoxTest::CCheckBoxTest()
{
  // Initialize state data

  m_widgetControl = (CWidgetControl *)NULL;
  m_bgWindow      = (CBgWindow *)NULL;
  m_checkBox      = (CCheckBox *)NULL;
}

// CCheckBoxTest Descriptor

CCheckBoxTest::~CCheckBoxTest(void)
{
  disconnect();
}

// Connect to the NX server

bool CCheckBoxTest::connect(void)
{
  // Connect to the server

  bool nxConnected = CNxServer::connect();
  if (nxConnected)
    {
      // Set the background color

      if (!setBackgroundColor(CONFIG_CCHECKBOXTEST_BGCOLOR))
        {
          message("CCheckBoxTest::connect: setBackgroundColor failed\n");
        }
    }

  return nxConnected;
}

// Disconnect from the NX server

void CCheckBoxTest::disconnect(void)
{
  // Free the radiobutton group

  if (m_checkBox)
    {
      delete m_checkBox;
      m_checkBox = (CCheckBox *)NULL;
    }

  // Close the window

  if (m_bgWindow)
    {
      delete m_bgWindow;
      m_bgWindow = (CBgWindow *)NULL;
    }

  // Free the widget control instance

  if (m_widgetControl)
    {
      delete m_widgetControl;
      m_widgetControl = (CWidgetControl *)NULL;
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

bool CCheckBoxTest::createWindow(void)
{
  // Initialize the widget control using the default style

  m_widgetControl = new CWidgetControl((CWidgetStyle *)NULL);

  // Get an (uninitialized) instance of the background window as a class
  // that derives from INxWindow.

  m_bgWindow = getBgWindow(m_widgetControl);
  if (!m_bgWindow)
    {
      message("CCheckBoxTest::createWindow: Failed to create CBgWindow instance\n");
      disconnect();
      return false;
    }

  // Open (and initialize) the window

  bool success = m_bgWindow->open();
  if (!success)
    {
      message("CCheckBoxTest::createWindow: Failed to open background window\n");
      disconnect();
      return false;
    }

  // Get the size of the display

  struct nxgl_size_s windowSize;
  if (!m_bgWindow->getSize(&windowSize))
    {
      printf("CCheckBoxTest::createWindow: Failed to get window size\n");
      disconnect();
      return false;
    }

  // Use the the size of the ON checkbox glyph. (adding twice the border thickness)

  nxgl_coord_t width  = g_checkBoxOn.width + 2 * 1;
  nxgl_coord_t height = g_checkBoxOn.height + 2 * 1;

  nxgl_coord_t checkboxX = (windowSize.w - width) >> 1;
  nxgl_coord_t checkboxY = (windowSize.h - height) >> 1;

  // Create the checkbox

  m_checkBox = new CCheckBox(m_widgetControl, checkboxX, checkboxY,
                             width, height, (CWidgetStyle *)NULL);
  if (!m_checkBox)
    {
      printf("CCheckBoxTest::createWindow: Failed to create CCheckBox\n");
      disconnect();
      return false;
    }

  return true;
}

// (Re-)draw the check box.

void CCheckBoxTest::showCheckBox(void)
{
  m_checkBox->enable();        // Un-necessary, the widget is enabled by default
  m_checkBox->enableDrawing();
  m_checkBox->redraw();
}

// Push the radio button

void CCheckBoxTest::clickCheckBox(void)
{
  // Get the checkbox center coordinates

  nxgl_coord_t checkboxX = m_checkBox->getX() + (m_checkBox->getWidth() >> 1);
  nxgl_coord_t checkboxY = m_checkBox->getY() + (m_checkBox->getHeight() >> 1);

  // Click the checkbox by calling nx_mousein.  nx_mousein is meant to be
  // called by mouse handling software. Here we just inject a left-button click
  // directly in the center of the radio button.

  // First, get the server handle.  Graphics software will never care
  // about the server handle.  Here we need it to get access to the
  // low-level mouse interface

  NXHANDLE handle = getServer();

  // Then inject the mouse click

  (void)nx_mousein(handle, checkboxX, checkboxY, NX_MOUSE_LEFTBUTTON);

  // Poll for mouse events
  //
  // Widget events are normally handled in a modal loop.
  // However, for this case we know that we just pressed the mouse button
  // so we don't have to poll.  We can just perform a one pass poll then
  // then check if the mouse event was processed corredly.

  m_widgetControl->pollEvents(m_checkBox);

  // Then inject the mouse release

  (void)nx_mousein(handle, checkboxX, checkboxY, 0);

  // And poll for more mouse events

  m_widgetControl->pollEvents(m_checkBox);

  // And re-draw the buttons (the mouse click event should have automatically
  // triggered the re-draw)
  //
  // showCheckBox();
}

// Show the state of the radio button group

void CCheckBoxTest::showCheckBoxState(void)
{
  CCheckBox::CheckBoxState state = m_checkBox->getState();
  switch (state)
    {
      case CCheckBox::CHECK_BOX_STATE_OFF: // Checkbox is unticked
        message("CCheckBoxTest::showCheckBoxState Checkbox is in the unticked state\n");
        break;

      case CCheckBox::CHECK_BOX_STATE_ON:  // Checkbox is ticked
        message("CCheckBoxTest::showCheckBoxState Check is in the ticked state\n");
        break;

      default:
      case CCheckBox::CHECK_BOX_STATE_MU:  // Checkbox is in the third state
        message("CCheckBoxTest::showCheckBoxState Checkbox is in the 3rd state\n");
        break;
    }
}
