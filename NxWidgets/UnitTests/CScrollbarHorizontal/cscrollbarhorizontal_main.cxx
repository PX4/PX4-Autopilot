/////////////////////////////////////////////////////////////////////////////
// NxWidgets/UnitTests/CScrollbarHorizontal/cscrollbarhorizontal_main.cxx
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

#include "cscrollbarhorizontaltest.hxx"

/////////////////////////////////////////////////////////////////////////////
// Definitions
/////////////////////////////////////////////////////////////////////////////

#define MAX_SCROLLBAR 50

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

extern "C" int cscrollbarhorizontal_main(int argc, char *argv[]);

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
// Name: nxheaders_main
/////////////////////////////////////////////////////////////////////////////

int cscrollbarhorizontal_main(int argc, char *argv[])
{
  // Initialize memory monitor logic

  initMemoryUsage();

  // Create an instance of the checkbox test

  message("cscrollbarhorizontal_main: Create CScrollbarHorizontalTest instance\n");
  CScrollbarHorizontalTest *test = new CScrollbarHorizontalTest();
  updateMemoryUsage(g_mmprevious, "After creating CScrollbarHorizontalTest");

  // Connect the NX server

  message("cscrollbarhorizontal_main: Connect the CScrollbarHorizontalTest instance to the NX server\n");
  if (!test->connect())
    {
      message("cscrollbarhorizontal_main: Failed to connect the CScrollbarHorizontalTest instance to the NX server\n");
      delete test;
      return 1;
    }
  updateMemoryUsage(g_mmprevious, "cscrollbarhorizontal_main: After connecting to the server");

  // Create a window to draw into

  message("cscrollbarhorizontal_main: Create a Window\n");
  if (!test->createWindow())
    {
      message("cscrollbarhorizontal_main: Failed to create a window\n");
      delete test;
      return 1;
    }
  updateMemoryUsage(g_mmprevious, "cscrollbarhorizontal_main: After creating a window");

  // Create a scrollbar

  message("cscrollbarhorizontal_main: Create a Scrollbar\n");
  CScrollbarHorizontal *scrollbar = test->createScrollbar();
  if (!scrollbar)
    {
      message("cscrollbarhorizontal_main: Failed to create a scrollbar\n");
      delete test;
      return 1;
    }
  updateMemoryUsage(g_mmprevious, "cscrollbarhorizontal_main: After creating a scrollbar");

  // Set the scrollbar minimum and maximum values

  scrollbar->setMinimumValue(0);
  scrollbar->setMaximumValue(MAX_SCROLLBAR);
  scrollbar->setValue(0);
  message("cscrollbarhorizontal_main: Scrollbar range %d->%d Initial value %d\n",
          scrollbar->getMinimumValue(), scrollbar->getMaximumValue(),
          scrollbar->getValue());

  // Show the initial state of the checkbox

  test->showScrollbar(scrollbar);
  sleep(1);

  // Now move the scrollbar up

  for (int i = 0; i <= MAX_SCROLLBAR; i++)
    {
      scrollbar->setValue(i);
      test->showScrollbar(scrollbar);
      message("cscrollbarhorizontal_main: %d. New value %d\n", i, scrollbar->getValue());
      usleep(1000); // The simulation needs this to let the X11 event loop run
    }
  updateMemoryUsage(g_mmprevious, "cscrollbarhorizontal_main: After moving the scrollbar up");

  // And move the scrollbar down

  for (int i = MAX_SCROLLBAR; i >= 0; i--)
    {
      scrollbar->setValue(i);
      test->showScrollbar(scrollbar);
      message("cscrollbarhorizontal_main: %d. New value %d\n", i, scrollbar->getValue());
      usleep(1000); // The simulation needs this to let the X11 event loop run
    }
  updateMemoryUsage(g_mmprevious, "cscrollbarhorizontal_main: After moving the scrollbar down");
  sleep(1);

  // Clean up and exit

  message("cscrollbarhorizontal_main: Clean-up and exit\n");
  delete scrollbar;
  updateMemoryUsage(g_mmprevious, "After deleting the scrollbar");
  delete test;
  updateMemoryUsage(g_mmprevious, "After deleting the test");
  updateMemoryUsage(g_mmInitial, "Final memory usage");
  return 0;
}

