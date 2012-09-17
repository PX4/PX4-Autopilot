/////////////////////////////////////////////////////////////////////////////
// NxWidgets/UnitTests/CGlyphButton/cglyphbutton_main.cxx
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
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <debug.h>

#include <nuttx/nx/nx.h>

#include "cglyphbuttontest.hxx"
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

static unsigned int g_mmInitial;
static unsigned int g_mmprevious;

/////////////////////////////////////////////////////////////////////////////
// Public Function Prototypes
/////////////////////////////////////////////////////////////////////////////

// Suppress name-mangling

extern "C" int cglyphbutton_main(int argc, char *argv[]);

/////////////////////////////////////////////////////////////////////////////
// Private Functions
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
// Name: updateMemoryUsage
/////////////////////////////////////////////////////////////////////////////

static void updateMemoryUsage(unsigned int previous,
                              FAR const char *msg)
{
  struct mallinfo mmcurrent;

  /* Get the current memory usage */

#ifdef CONFIG_CAN_PASS_STRUCTS
  mmcurrent = mallinfo();
#else
  (void)mallinfo(&mmcurrent);
#endif

  /* Show the change from the previous time */

  message("\n%s:\n", msg);
  message("  Before: %8d After: %8d Change: %8d\n\n",
          previous, mmcurrent.uordblks, mmcurrent.uordblks - previous);

  /* Set up for the next test */

  g_mmprevious = mmcurrent.uordblks;
}

/////////////////////////////////////////////////////////////////////////////
// Name: initMemoryUsage
/////////////////////////////////////////////////////////////////////////////

static void initMemoryUsage(void)
{
  struct mallinfo mmcurrent;

  /* Get the current memory usage */

#ifdef CONFIG_CAN_PASS_STRUCTS
  mmcurrent = mallinfo();
#else
  (void)mallinfo(&mmcurrent);
#endif

  g_mmInitial  = mmcurrent.uordblks;
  g_mmprevious = mmcurrent.uordblks;
}

/////////////////////////////////////////////////////////////////////////////
// Public Functions
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
// nxheaders_main
/////////////////////////////////////////////////////////////////////////////

int cglyphbutton_main(int argc, char *argv[])
{
  // Initialize memory monitor logic

  initMemoryUsage();

  // Create an instance of the font test

  message("cglyphbutton_main: Create CGlyphButtonTest instance\n");
  CGlyphButtonTest *test = new CGlyphButtonTest();
  updateMemoryUsage(g_mmprevious, "After creating CGlyphButtonTest");

  // Connect the NX server

  message("cglyphbutton_main: Connect the CGlyphButtonTest instance to the NX server\n");
  if (!test->connect())
    {
      message("cglyphbutton_main: Failed to connect the CGlyphButtonTest instance to the NX server\n");
      delete test;
      return 1;
    }
  updateMemoryUsage(g_mmprevious, "After connecting to the server");

  // Create a window to draw into

  message("cglyphbutton_main: Create a Window\n");
  if (!test->createWindow())
    {
      message("cglyphbutton_main: Failed to create a window\n");
      delete test;
      return 1;
    }
  updateMemoryUsage(g_mmprevious, "After creating a window");

  // Create a CGlyphButton instance

  CGlyphButton *button = test->createButton(&g_arrowDown, &g_arrowUp);
  if (!button)
    {
      message("cglyphbutton_main: Failed to create a button\n");
      delete test;
      return 1;
    }
  updateMemoryUsage(g_mmprevious, "After creating the glyph button");

  // Show the button

  message("cglyphbutton_main: Show the button\n");
  test->showButton(button);
  updateMemoryUsage(g_mmprevious, "After showing the glyph button");

  // Wait two seconds, then perform a simulated mouse click on the button

  sleep(2);
  message("cglyphbutton_main: Click the button\n");
  test->click();
  updateMemoryUsage(g_mmprevious, "After clicking glyph button");

  // Poll for the mouse click event (of course this can hang if something fails)

  bool clicked = test->poll(button);
  message("cglyphbutton_main: Button is %s\n", clicked ? "clicked" : "released");

  // Wait a second, then release the mouse buttone

  sleep(1);
  test->release();
  updateMemoryUsage(g_mmprevious, "After releasing glyph button");

  // Poll for the mouse release event (of course this can hang if something fails)

  clicked = test->poll(button);
  message("cglyphbutton_main: Button is %s\n", clicked ? "clicked" : "released");

  // Wait a few more seconds so that the tester can ponder the result

  sleep(3);

  // Clean up and exit

  message("cglyphbutton_main: Clean-up and exit\n");
  delete button;
  updateMemoryUsage(g_mmprevious, "After deleting the glyph button");
  delete test;
  updateMemoryUsage(g_mmprevious, "After deleting the test");
  updateMemoryUsage(g_mmInitial, "Final memory usage");
  return 0;
}

