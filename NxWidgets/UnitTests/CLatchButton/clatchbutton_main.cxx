/////////////////////////////////////////////////////////////////////////////
// NxWidgets/UnitTests/CLatchButton/clatchbutton_main.cxx
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
#include <unistd.h>
#include <debug.h>

#include <nuttx/nx/nx.h>

#include "clatchbuttontest.hxx"

/////////////////////////////////////////////////////////////////////////////
// Definitions
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
// Private Classes
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
// Private Data
/////////////////////////////////////////////////////////////////////////////

static const char g_pushme[] = "Push Me";

/////////////////////////////////////////////////////////////////////////////
// Public Function Prototypes
/////////////////////////////////////////////////////////////////////////////

// Suppress name-mangling

extern "C" int clatchbutton_main(int argc, char *argv[]);

/////////////////////////////////////////////////////////////////////////////
// Private Functions
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
// Name: showButtonState
/////////////////////////////////////////////////////////////////////////////

static void showButtonState(CLatchButton *button, bool &clicked, bool &latched)
{
  bool nowClicked = button->isClicked();
  bool nowLatched = button->isLatched();

  printf("showButtonState: Button state: %s and %s\n",
    nowClicked ? "clicked" : "released",
    nowLatched ? "latched" : "unlatched");

  if (clicked != nowClicked || latched != nowLatched)
    {
      printf("showButtonState: ERROR: Expected %s and %s\n",
        clicked ? "clicked" : "released",
        latched ? "latched" : "unlatched");

      clicked = nowClicked;
      latched = nowLatched;
    }
}

/////////////////////////////////////////////////////////////////////////////
// Public Functions
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
// nxheaders_main
/////////////////////////////////////////////////////////////////////////////

int clatchbutton_main(int argc, char *argv[])
{
  // Create an instance of the font test

  printf("clatchbutton_main: Create CLatchButtonTest instance\n");
  CLatchButtonTest *test = new CLatchButtonTest();

  // Connect the NX server

  printf("clatchbutton_main: Connect the CLatchButtonTest instance to the NX server\n");
  if (!test->connect())
    {
      printf("clatchbutton_main: Failed to connect the CLatchButtonTest instance to the NX server\n");
      delete test;
      return 1;
    }

  // Create a window to draw into

  printf("clatchbutton_main: Create a Window\n");
  if (!test->createWindow())
    {
      printf("clatchbutton_main: Failed to create a window\n");
      delete test;
      return 1;
    }

  // Create a CLatchButton instance

  CLatchButton *button = test->createButton(g_pushme);
  if (!button)
    {
      printf("clatchbutton_main: Failed to create a button\n");
      delete test;
      return 1;
    }

  // Show the button

  printf("clatchbutton_main: Show the button\n");
  test->showButton(button);

  bool clicked = false;
  bool latched = false;
  showButtonState(button, clicked, latched);

  // Toggle the button state a few times

  for (int i = 0; i < 8; i++)
    {
      // Wait two seconds, then perform a simulated mouse click on the button

      sleep(2);
      printf("clatchbutton_main: Click the button\n");
      test->click();
      test->poll(button);

      // Test the button state it should be clicked with the latch state
      // toggled

      clicked = true;
      latched = !latched;
      showButtonState(button, clicked, latched);

      // And release the button after 0.5 seconds

      usleep(500 * 1000);
      printf("clatchbutton_main: Release the button\n");
      test->release();
      test->poll(button);

      // Test the button state it should be unclicked with the latch state
      // unchanged

      clicked = false;
      showButtonState(button, clicked, latched);
      fflush(stdout);
    }

  // Wait a few more seconds so that the tester can ponder the result

  sleep(3);

  // Clean up and exit

  printf("clatchbutton_main: Clean-up and exit\n");
  delete button;
  delete test;
  return 0;
}

