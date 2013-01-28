/////////////////////////////////////////////////////////////////////////////
// NxWidgets/UnitTests/CButtonArray/cbuttonarraytest.hxx
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

#ifndef __UNITTESTS_CBUTTONARRAY_CBUTTONARRAYTEST_HXX
#define __UNITTESTS_CBUTTONARRAY_CBUTTONARRAYTEST_HXX

/////////////////////////////////////////////////////////////////////////////
// Included Files
/////////////////////////////////////////////////////////////////////////////

#include <nuttx/config.h>

#include <nuttx/init.h>
#include <cstdio>
#include <semaphore.h>
#include <debug.h>

#include <nuttx/nx/nx.h>

#include "nxconfig.hxx"
#include "cwidgetcontrol.hxx"
#include "ccallback.hxx"
#include "cbgwindow.hxx"
#include "cnxserver.hxx"
#include "cnxfont.hxx"
#include "cnxstring.hxx"
#include "cbuttonarray.hxx"

/////////////////////////////////////////////////////////////////////////////
// Definitions
/////////////////////////////////////////////////////////////////////////////
// Configuration ////////////////////////////////////////////////////////////

#ifndef CONFIG_HAVE_CXX
#  error "CONFIG_HAVE_CXX must be defined"
#endif

#ifndef CONFIG_CBUTTONARRAYTEST_BGCOLOR
#  define CONFIG_CBUTTONARRAYTEST_BGCOLOR CONFIG_NXWIDGETS_DEFAULT_BACKGROUNDCOLOR
#endif

#ifndef CONFIG_CBUTTONARRAYTEST_FONTCOLOR
#  define CONFIG_CBUTTONARRAYTEST_FONTCOLOR CONFIG_NXWIDGETS_DEFAULT_FONTCOLOR
#endif

// If debug is enabled, use the debug function, syslog() instead
// of printf() so that the output is synchronized.

#ifdef CONFIG_DEBUG
#  define message lowsyslog
#else
#  define message printf
#endif

// The geometry of the button array

#define BUTTONARRAY_NCOLUMNS      4
#define BUTTONARRAY_NROWS         7
#define BUTTONARRAY_BUTTONWIDTH  60
#define BUTTONARRAY_BUTTONHEIGHT 32
#define BUTTONARRAY_WIDTH        (BUTTONARRAY_BUTTONWIDTH * BUTTONARRAY_NCOLUMNS)
#define BUTTONARRAY_HEIGHT       (BUTTONARRAY_BUTTONHEIGHT * BUTTONARRAY_NROWS)

/////////////////////////////////////////////////////////////////////////////
// Public Classes
/////////////////////////////////////////////////////////////////////////////

using namespace NXWidgets;

class CButtonArrayTest : public CNxServer
{
private:
  CWidgetControl     *m_widgetControl;  // The widget control for the window
  CBgWindow          *m_bgWindow;       // Background window instance

public:
  // Constructor/destructors

  CButtonArrayTest();
  ~CButtonArrayTest();

  // Initializer/unitializer.  These methods encapsulate the basic steps for
  // starting and stopping the NX server

  bool connect(void);
  void disconnect(void);

  // Create a window.  This method provides the general operations for
  // creating a window that you can draw within.
  //
  // Those general operations are:
  // 1) Create a dumb CWigetControl instance
  // 2) Pass the dumb CWidgetControl instance to the window constructor
  //    that inherits from INxWindow.  This will "smarten" the CWidgetControl
  //    instance with some window knowlede
  // 3) Call the open() method on the window to display the window.
  // 4) After that, the fully smartened CWidgetControl instance can
  //    be used to generate additional widgets by passing it to the
  //    widget constructor

  bool createWindow(void);

  // Create a CButtonArray instance.  This method will show you how to create
  // a CButtonArray widget

  CButtonArray *createButtonArray(void);

  // Draw the button array.  This method illustrates how to draw the CButtonArray widget.

  void showButton(CButtonArray *buttonArray);

  // Perform a simulated mouse click on a button in the array.  This method injects
  // the mouse click through the NX heirarchy just as would real mouse
  // hardward.

  void click(CButtonArray *buttonArray, int column, int row);

  // The counterpart to click.  This simulates a button release through
  // the same mechanism.

  void release(CButtonArray *buttonArray, int column, int row);

  // Widget events are normally handled in a model loop (by calling goModel()).
  // However, for this case we know when there should be press and release
  // events so we don't have to poll.  We can just perform a one pass poll
  // then check if the event was processed corredly.

  void poll(CButtonArray *buttonArray);
};

/////////////////////////////////////////////////////////////////////////////
// Public Data
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
// Public Function Prototypes
/////////////////////////////////////////////////////////////////////////////


#endif // __UNITTESTS_CBUTTONARRAY_CBUTTONARRAYTEST_HXX
