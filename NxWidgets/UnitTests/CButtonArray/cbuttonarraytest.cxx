/////////////////////////////////////////////////////////////////////////////
// NxWidgets/UnitTests/CButtonArray/cbuttonarraytest.cxx
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
#include "cbuttonarraytest.hxx"
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
// CButtonArrayTest Method Implementations
/////////////////////////////////////////////////////////////////////////////

// CButtonArrayTest Constructor

CButtonArrayTest::CButtonArrayTest()
{
  m_widgetControl = (CWidgetControl *)NULL;
  m_bgWindow      = (CBgWindow *)NULL;
}

// CButtonArrayTest Descriptor

CButtonArrayTest::~CButtonArrayTest()
{
  disconnect();
}

// Connect to the NX server

bool CButtonArrayTest::connect(void)
{
  // Connect to the server

  bool nxConnected = CNxServer::connect();
  if (nxConnected)
    {
      // Set the background color

      if (!setBackgroundColor(CONFIG_CBUTTONARRAYTEST_BGCOLOR))
        {
          message("CButtonArrayTest::connect: setBackgroundColor failed\n");
        }
    }

  return nxConnected;
}

// Disconnect from the NX server

void CButtonArrayTest::disconnect(void)
{
  // Close the window

  if (m_bgWindow)
    {
      delete m_bgWindow;
    }

  // Destroy the widget control

  if (m_widgetControl)
    {
      delete m_widgetControl;
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

bool CButtonArrayTest::createWindow(void)
{
  // Initialize the widget control using the default style

  m_widgetControl = new CWidgetControl((CWidgetStyle *)NULL);

  // Get an (uninitialized) instance of the background window as a class
  // that derives from INxWindow.

  m_bgWindow = getBgWindow(m_widgetControl);
  if (!m_bgWindow)
    {
      message("CButtonArrayTest::createGraphics: Failed to create CBgWindow instance\n");
      delete m_widgetControl;
      return false;
    }

  // Open (and initialize) the window

  bool success = m_bgWindow->open();
  if (!success)
    {
      message("CButtonArrayTest::createGraphics: Failed to open background window\n");
      delete m_bgWindow;
      m_bgWindow = (CBgWindow*)0;
      return false;
    }

  return true;
}

// Create a CButtonArray instance

CButtonArray *CButtonArrayTest::createButtonArray(void)
{
  // Get the width of the display

  struct nxgl_size_s windowSize;
  if (!m_bgWindow->getSize(&windowSize))
    {
      message("CButtonArrayTest::createGraphics: Failed to get window size\n");
      return (CButtonArray *)NULL;
    }

  // Pick an X/Y position such that the button array will be centered in the display

  nxgl_coord_t buttonArrayX;
  if (BUTTONARRAY_WIDTH >= windowSize.w)
    {
      buttonArrayX = 0;
    }
  else
    {
      buttonArrayX = (windowSize.w - BUTTONARRAY_WIDTH) >> 1;
    }

  nxgl_coord_t buttonArrayY;
  if (BUTTONARRAY_HEIGHT >= windowSize.h)
    {
      buttonArrayY = 0;
    }
  else
    {
      buttonArrayY = (windowSize.h - BUTTONARRAY_HEIGHT) >> 1;
    }

  // Now we have enough information to create the button array

  return new CButtonArray(m_widgetControl,
                          buttonArrayX, buttonArrayY,
                          BUTTONARRAY_NCOLUMNS, BUTTONARRAY_NROWS,
                          BUTTONARRAY_BUTTONWIDTH, BUTTONARRAY_BUTTONHEIGHT);
}

// Draw the button array

void CButtonArrayTest::showButton(CButtonArray *buttonArray)
{
  buttonArray->enable();        // Un-necessary, the widget is enabled by default
  buttonArray->enableDrawing();
  buttonArray->redraw();
}

// Perform a simulated mouse click on a button in the array.  This method injects
// the mouse click through the NX heirarchy just as would real mouse
// hardward.

void CButtonArrayTest::click(CButtonArray *buttonArray, int column, int row)
{
  // nx_mousein is meant to be called by mouse handling software.
  // Here we just inject a left-button click directly in the center of
  // the selected button.

  // First, get the server handle.  Graphics software will never care
  // about the server handle.  Here we need it to get access to the
  // low-level mouse interface

  NXHANDLE handle = getServer();

  // The the coorinates of the center of the button

  nxgl_coord_t buttonX = buttonArray->getX() +
                         column * BUTTONARRAY_BUTTONWIDTH +
                         BUTTONARRAY_BUTTONWIDTH/2;
  nxgl_coord_t buttonY = buttonArray->getY() +
                         row * BUTTONARRAY_BUTTONHEIGHT +
                         BUTTONARRAY_BUTTONHEIGHT/2;

  // Then inject the mouse click

  (void)nx_mousein(handle, buttonX, buttonY, NX_MOUSE_LEFTBUTTON);
}

// The counterpart to click.  This simulates a button release through
// the same mechanism.

void CButtonArrayTest::release(CButtonArray *buttonArray, int column, int row)
{
  // nx_mousein is meant to be called by mouse handling software.
  // Here we just inject a left-button click directly in the center of
  // the button.

  // First, get the server handle.  Graphics software will never care
  // about the server handle.  Here we need it to get access to the
  // low-level mouse interface

  NXHANDLE handle = getServer();

  // The the coorinates of the center of the button

  nxgl_coord_t buttonX = buttonArray->getX() +
                         column * BUTTONARRAY_BUTTONWIDTH +
                         BUTTONARRAY_BUTTONWIDTH/2;
  nxgl_coord_t buttonY = buttonArray->getY() +
                         row * BUTTONARRAY_BUTTONHEIGHT +
                         BUTTONARRAY_BUTTONHEIGHT/2;

  // Then inject the mouse release

  (void)nx_mousein(handle, buttonX, buttonY, NX_MOUSE_NOBUTTONS);
}

// Widget events are normally handled in a modal loop.
// However, for this case we know when there should be press and release
// events so we don't have to poll.  We can just perform a one pass poll
// then check if the event was processed corredly.

void CButtonArrayTest::poll(CButtonArray *button)
{
  // Poll for mouse events

  m_widgetControl->pollEvents(button);
}

