/////////////////////////////////////////////////////////////////////////////
// NxWidgets/UnitTests/CRadioButton/cradiobutton_main.cxx
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

#include "crlepalettebitmap.hxx"
#include "glyphs.hxx"
#include "cradiobuttontest.hxx"

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

extern "C" int cradiobutton_main(int argc, char *argv[]);

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

int cradiobutton_main(int argc, char *argv[])
{
  // Initialize memory monitor logic

  initMemoryUsage();

  // Create an instance of the radio button test

  message("cradiobutton_main: Create CRadioButtonTest instance\n");
  CRadioButtonTest *test = new CRadioButtonTest();
  updateMemoryUsage(g_mmprevious, "After creating CRadioButtonTest");

  // Connect the NX server

  message("cradiobutton_main: Connect the CRadioButtonTest instance to the NX server\n");
  if (!test->connect())
    {
      message("cradiobutton_main: Failed to connect the CRadioButtonTest instance to the NX server\n");
      delete test;
      return 1;
    }
  updateMemoryUsage(g_mmprevious, "cradiobutton_main: After connecting to the server");

  // Create a window to draw into

  message("cradiobutton_main: Create a Window\n");
  if (!test->createWindow())
    {
      message("cradiobutton_main: Failed to create a window\n");
      delete test;
      return 1;
    }
  updateMemoryUsage(g_mmprevious, "cradiobutton_main: After creating a window");

  // Create three radio buttons

  CRadioButton *button1 = test->newRadioButton();
  if (!button1)
    {
      message("cradiobutton_main: Failed to create radio button 1\n");
      delete test;
      return 1;
    }
  updateMemoryUsage(g_mmprevious, "cradiobutton_main: After creating radio button 1");

  CRadioButton *button2 = test->newRadioButton();
  if (!button2)
    {
      message("cradiobutton_main: Failed to create radio button 2\n");
      delete test;
      return 1;
    }
  updateMemoryUsage(g_mmprevious, "cradiobutton_main: After creating radio button 2");

  CRadioButton *button3 = test->newRadioButton();
  if (!button3)
    {
      message("cradiobutton_main: Failed to create radio button 3\n");
      delete test;
      return 1;
    }
  updateMemoryUsage(g_mmprevious, "cradiobutton_main: After creating radio button 3");

  // Show the initial state of the buttons

  test->showButtons();
  test->showButtonState();
  sleep(1);

  // Now push some buttons

  message("cradiobutton_main: Pushing button 1\n");
  test->pushButton(button1);
  usleep(500*1000);
  test->showButtonState();
  updateMemoryUsage(g_mmprevious, "After pushing button 1");
  usleep(500*1000);

  message("cradiobutton_main: Pushing button 2\n");
  test->pushButton(button2);
  usleep(500*1000);
  test->showButtonState();
  updateMemoryUsage(g_mmprevious, "After pushing button 2");
  usleep(500*1000);

  message("cradiobutton_main: Pushing button 3\n");
  test->pushButton(button3);
  usleep(500*1000);
  test->showButtonState();
  updateMemoryUsage(g_mmprevious, "After pushing button 3");
  sleep(2);

  // Clean up and exit

  message("cradiobutton_main: Clean-up and exit\n");
  delete test;
  updateMemoryUsage(g_mmprevious, "After deleting the test");
  updateMemoryUsage(g_mmInitial, "Final memory usage");
  return 0;
}

