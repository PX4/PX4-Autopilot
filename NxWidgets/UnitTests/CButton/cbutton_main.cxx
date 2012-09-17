/////////////////////////////////////////////////////////////////////////////
// NxWidgets/UnitTests/CButton/cbutton_main.cxx
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

#include "cbuttontest.hxx"

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

extern "C" int cbutton_main(int argc, char *argv[]);

/////////////////////////////////////////////////////////////////////////////
// Public Functions
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
// nxheaders_main
/////////////////////////////////////////////////////////////////////////////

int cbutton_main(int argc, char *argv[])
{
  // Create an instance of the font test

  printf("cbutton_main: Create CButtonTest instance\n");
  CButtonTest *test = new CButtonTest();

  // Connect the NX server

  printf("cbutton_main: Connect the CButtonTest instance to the NX server\n");
  if (!test->connect())
    {
      printf("cbutton_main: Failed to connect the CButtonTest instance to the NX server\n");
      delete test;
      return 1;
    }

  // Create a window to draw into

  printf("cbutton_main: Create a Window\n");
  if (!test->createWindow())
    {
      printf("cbutton_main: Failed to create a window\n");
      delete test;
      return 1;
    }

  // Create a CButton instance

  CButton *button = test->createButton(g_pushme);
  if (!button)
    {
      printf("cbutton_main: Failed to create a button\n");
      delete test;
      return 1;
    }

  // Show the button

  printf("cbutton_main: Show the button\n");
  test->showButton(button);

  // Wait two seconds, then perform a simulated mouse click on the button

  sleep(2);
  printf("cbutton_main: Click the button\n");
  test->click();

  // Poll for the mouse click event (of course this can hang if something fails)

  bool clicked = test->poll(button);
  printf("cbutton_main: Button is %s\n", clicked ? "clicked" : "released");

  // Wait a second, then release the mouse buttone

  sleep(1);
  test->release();

  // Poll for the mouse release event (of course this can hang if something fails)

  clicked = test->poll(button);
  printf("cbutton_main: Button is %s\n", clicked ? "clicked" : "released");

  // Wait a few more seconds so that the tester can ponder the result

  sleep(3);

  // Clean up and exit

  printf("cbutton_main: Clean-up and exit\n");
  delete button;
  delete test;
  return 0;
}

