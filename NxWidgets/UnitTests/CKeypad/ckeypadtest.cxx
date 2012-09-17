/////////////////////////////////////////////////////////////////////////////
// NxWidgets/UnitTests/CKeypad/ckeypadtest.cxx
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
#include "ckeypadtest.hxx"
#include "cbgwindow.hxx"
#include "cnxstring.hxx"
#include "cnxfont.hxx"
#include "crect.hxx"

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
// CKeypadTest Method Implementations
/////////////////////////////////////////////////////////////////////////////

// CKeypadTest Constructor

CKeypadTest::CKeypadTest()
{
  m_widgetControl = (CWidgetControl *)NULL;
  m_bgWindow      = (CBgWindow *)NULL;
  m_buttonWidth   = 0;
  m_buttonHeight  = 0;
  m_displayHeight = 0;
}

// CKeypadTest Descriptor

CKeypadTest::~CKeypadTest()
{
  disconnect();
}

// Connect to the NX server

bool CKeypadTest::connect(void)
{
  // Connect to the server

  bool nxConnected = CNxServer::connect();
  if (nxConnected)
    {
      // Set the background color

      if (!setBackgroundColor(CONFIG_CKEYPADTEST_BGCOLOR))
        {
          message("CKeypadTest::connect: setBackgroundColor failed\n");
        }
    }

  return nxConnected;
}

// Disconnect from the NX server

void CKeypadTest::disconnect(void)
{
  // Destroy the text box

  if (m_textbox)
    {
      delete m_textbox;
    }

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

bool CKeypadTest::createWindow(void)
{
  // Initialize the widget control using the default style

  m_widgetControl = new CWidgetControl((CWidgetStyle *)NULL);

  // Get an (uninitialized) instance of the background window as a class
  // that derives from INxWindow.

  m_bgWindow = getBgWindow(m_widgetControl);
  if (!m_bgWindow)
    {
      message("CKeypadTest::createGraphics: Failed to create CBgWindow instance\n");
      delete m_widgetControl;
      return false;
    }

  // Open (and initialize) the window

  bool success = m_bgWindow->open();
  if (!success)
    {
      message("CKeypadTest::createGraphics: Failed to open background window\n");
      delete m_bgWindow;
      m_bgWindow = (CBgWindow*)0;
      return false;
    }

  // Then determine the display size

  setDisplaySize();
  return true;
}

// Pick size of the display

void CKeypadTest::setDisplaySize(void)
{
  // Get the height and width of the display

  struct nxgl_size_s windowSize;
  if (!m_bgWindow->getSize(&windowSize))
    {
      message("CKeypadTest::createGraphics: Failed to get window size\n");
      return;
    }

  // Pick a height and width of a button.  Here we use inside information
  // that the number of rows and columns in the keypad.  This should not matter in
  // a "real" application.

  // Lets aim for a width of 4*60 = 240

  if (windowSize.w > KEYPAD_NCOLUMNS*60)
    {
      m_buttonWidth = 60;
    }
  else
    {
      // Otherwise, let's use what we have

      m_buttonWidth = windowSize.w >> 2;
    }
  
  // Lets aim for a height of 7*32 = 224.  But lets bump up the number of rows
  // to allow one for the text box.

  if (windowSize.h > (KEYPAD_NROWS+1)*32)
    {
      m_buttonHeight  = 32;
      m_displayHeight = (KEYPAD_NROWS+1)*32;
    }
  else
    {
      // Otherwise, let's use what we have

      m_buttonHeight  = windowSize.h >> 3;
      m_displayHeight = windowSize.h;
    }
}

// Create a CKeypad instance

CKeypad *CKeypadTest::createKeypad(void)
{
  // Get the height and width of the display

  struct nxgl_size_s windowSize;
  if (!m_bgWindow->getSize(&windowSize))
    {
      message("CKeypadTest::createGraphics: Failed to get window size\n");
      return (CKeypad *)NULL;
    }

  // Pick a height and width.  Here we use inside information that the number
  // of rows in the keypad is 7.  This should not matter in a "real" application.

  nxgl_coord_t keypadWidth  = KEYPAD_NCOLUMNS * m_buttonWidth;
  nxgl_coord_t keypadHeight = KEYPAD_NROWS * m_buttonHeight;

  // Pick an X/Y position such that the keypad will be centered in the display

  nxgl_coord_t keypadOffset = m_displayHeight - keypadHeight;

  nxgl_coord_t keypadX = (windowSize.w - keypadWidth) >> 1;
  nxgl_coord_t keypadY = keypadOffset + ((windowSize.h - m_displayHeight) >> 1);

  // Now we have enough information to create the keypad

  CKeypad *keypad = new CKeypad(m_widgetControl, getServer(), keypadX, keypadY,
                                keypadWidth, keypadHeight);
  if (keypad)
    {
      // Create a text box to catch the keyboard inputs

      m_textbox = createTextBox();
      if (!m_textbox)
        {
          delete keypad;
          keypad = (CKeypad *)NULL;
        }
      else
        {
          // Always show the cursor and wrap the cursor if it goes past the end

          m_textbox->showCursor(SHOW_CURSOR_ALWAYS);
          m_textbox->wrapCursor(true);

          // Align text on the left

          m_textbox->setTextAlignmentHoriz(CTextBox::TEXT_ALIGNMENT_HORIZ_LEFT);

          // Configure the text box to receive the keyboard input

          keypad->addWidgetEventHandler(m_textbox);
       }
    }

  return keypad;
}

// Create a CTextBox instance so that we can see the keypad output

CTextBox *CKeypadTest::createTextBox(void)
{
  // Get the height and width of the display

  struct nxgl_size_s windowSize;
  if (!m_bgWindow->getSize(&windowSize))
    {
      message("CKeypadTest::createGraphics: Failed to get window size\n");
      return (CTextBox *)NULL;
    }

  // Pick a height and width.  Here we use inside information that the number
  // of rows in the keypad is 7.  This should not matter in a "real" application.

  nxgl_coord_t textboxWidth  = KEYPAD_NCOLUMNS * m_buttonWidth;
  nxgl_coord_t textboxHeight = m_displayHeight - KEYPAD_NROWS * m_buttonHeight;

  // Pick an X/Y position such that the keypad will be centered in the display

  nxgl_coord_t textboxX = (windowSize.w - textboxWidth) >> 1;
  nxgl_coord_t textboxY = (windowSize.h - m_displayHeight) >> 1;

  // Now we have enough information to create the TextBox

  return new CTextBox(m_widgetControl, textboxX, textboxY,
                      textboxWidth, textboxHeight, "");
}

// Draw the keypad

void CKeypadTest::showKeypad(CKeypad *keypad)
{
  // Re-draw the keypad

  keypad->enable();        // Un-necessary, the widget is enabled by default
  keypad->enableDrawing();
  keypad->redraw();

  // Then redraw the text box
  
  m_textbox->enable();        // Un-necessary, the widget is enabled by default
  m_textbox->enableDrawing();
  m_textbox->redraw();
}

// Perform a simulated mouse click on a button in the array.  This method injects
// the mouse click through the NX heirarchy just as would real mouse
// hardward.

void CKeypadTest::click(CKeypad *keypad, int column, int row)
{
  // nx_mousein is meant to be called by mouse handling software.
  // Here we just inject a left-button click directly in the center of
  // the selected button.

  // First, get the server handle.  Graphics software will never care
  // about the server handle.  Here we need it to get access to the
  // low-level mouse interface

  NXHANDLE handle = getServer();

  // The the coorinates of the center of the button

  nxgl_coord_t buttonX = keypad->getX() + column * m_buttonWidth  + (m_buttonWidth  >> 1);
  nxgl_coord_t buttonY = keypad->getY() +    row * m_buttonHeight + (m_buttonHeight >> 1);

  // Then inject the mouse click

  (void)nx_mousein(handle, buttonX, buttonY, NX_MOUSE_LEFTBUTTON);
}

// The counterpart to click.  This simulates a button release through
// the same mechanism.

void CKeypadTest::release(CKeypad *keypad, int column, int row)
{
  // nx_mousein is meant to be called by mouse handling software.
  // Here we just inject a left-button click directly in the center of
  // the button.

  // First, get the server handle.  Graphics software will never care
  // about the server handle.  Here we need it to get access to the
  // low-level mouse interface

  NXHANDLE handle = getServer();

  // The the coorinates of the center of the button

  nxgl_coord_t buttonX = keypad->getX() +
                         column * m_buttonWidth +
                         m_buttonWidth/2;
  nxgl_coord_t buttonY = keypad->getY() +
                         row * m_buttonHeight +
                         m_buttonHeight/2;

  // Then inject the mouse release

  (void)nx_mousein(handle, buttonX, buttonY, NX_MOUSE_NOBUTTONS);
}

// Widget events are normally handled in a modal loop.
// However, for this case we know when there should be press and release
// events so we don't have to poll.  We can just perform a one pass poll
// then check if the event was processed corredly.

void CKeypadTest::poll(CKeypad *button)
{
  // Poll for mouse events

  m_widgetControl->pollEvents(button);

  // Limit the amount of text in the TextBox

  reverseAlignment();
}

// Start with left text alignment.  When the text reaches the right sice
// of the text box, switch to right text alignment.

void CKeypadTest::reverseAlignment(void)
{
  // Get the current horizontal text alignment

   CTextBox::TextAlignmentHoriz hAlign = m_textbox->getTextAlignmentHoriz();

  // Are we still using left text alignment?

  if (hAlign == CTextBox::TEXT_ALIGNMENT_HORIZ_LEFT)
    {
      // Yes.. Get the string in the text box

      CNxString string = m_textbox->getText();

      // Get the font

      CNxFont *font = m_textbox->getFont();

      // Get the TextBox bounding box

      CRect rect;
      m_textbox->getRect(rect);

      // When the length of string approaches the width of the display
      // region, then switch to right text alignment

      int mxWidth = font->getMaxWidth();
      if (font->getStringWidth(string) + mxWidth >= rect.getWidth())
        {
          // Switch to right text alignment

          m_textbox->setTextAlignmentHoriz(CTextBox::TEXT_ALIGNMENT_HORIZ_RIGHT);
        }
    }
}

