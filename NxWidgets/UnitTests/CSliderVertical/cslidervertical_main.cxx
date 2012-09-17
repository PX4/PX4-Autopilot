/////////////////////////////////////////////////////////////////////////////
// NxWidgets/UnitTests/CSliderVertical/cslidervertical_main.cxx
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

#include "csliderverticaltest.hxx"

/////////////////////////////////////////////////////////////////////////////
// Definitions
/////////////////////////////////////////////////////////////////////////////

#define MAX_SLIDER 50

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

extern "C" int cslidervertical_main(int argc, char *argv[]);

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

int cslidervertical_main(int argc, char *argv[])
{
  // Initialize memory monitor logic

  initMemoryUsage();

  // Create an instance of the checkbox test

  message("cslidervertical_main: Create CSliderVerticalTest instance\n");
  CSliderVerticalTest *test = new CSliderVerticalTest();
  updateMemoryUsage(g_mmprevious, "After creating CSliderVerticalTest");

  // Connect the NX server

  message("cslidervertical_main: Connect the CSliderVerticalTest instance to the NX server\n");
  if (!test->connect())
    {
      message("cslidervertical_main: Failed to connect the CSliderVerticalTest instance to the NX server\n");
      delete test;
      return 1;
    }
  updateMemoryUsage(g_mmprevious, "cslidervertical_main: After connecting to the server");

  // Create a window to draw into

  message("cslidervertical_main: Create a Window\n");
  if (!test->createWindow())
    {
      message("cslidervertical_main: Failed to create a window\n");
      delete test;
      return 1;
    }
  updateMemoryUsage(g_mmprevious, "cslidervertical_main: After creating a window");

  // Create a slider

  message("cslidervertical_main: Create a Slider\n");
  CSliderVertical *slider = test->createSlider();
  if (!slider)
    {
      message("cslidervertical_main: Failed to create a slider\n");
      delete test;
      return 1;
    }
  updateMemoryUsage(g_mmprevious, "cslidervertical_main: After creating a slider");

  // Set the slider minimum and maximum values

  slider->setMinimumValue(0);
  slider->setMaximumValue(MAX_SLIDER);
  slider->setValue(0);
  message("cslidervertical_main: Slider range %d->%d Initial value %d\n",
          slider->getMinimumValue(), slider->getMaximumValue(),
          slider->getValue());

  // Show the initial state of the checkbox

  test->showSlider(slider);
  sleep(1);

  // Now move the slider up

  for (int i = 0; i <= MAX_SLIDER; i++)
    {
      slider->setValue(i);
      test->showSlider(slider);
      message("cslidervertical_main: %d. New value %d\n", i, slider->getValue());
      usleep(1000); // The simulation needs this to let the X11 event loop run
    }
  updateMemoryUsage(g_mmprevious, "cslidervertical_main: After moving the slider up");

  // And move the slider down

  for (int i = MAX_SLIDER; i >= 0; i--)
    {
      slider->setValue(i);
      test->showSlider(slider);
      message("cslidervertical_main: %d. New value %d\n", i, slider->getValue());
      usleep(1000); // The simulation needs this to let the X11 event loop run
    }
  updateMemoryUsage(g_mmprevious, "cslidervertical_main: After moving the slider down");
  sleep(1);

  // Clean up and exit

  message("cslidervertical_main: Clean-up and exit\n");
  delete slider;
  updateMemoryUsage(g_mmprevious, "After deleting the slider");
  delete test;
  updateMemoryUsage(g_mmprevious, "After deleting the test");
  updateMemoryUsage(g_mmInitial, "Final memory usage");
  return 0;
}

