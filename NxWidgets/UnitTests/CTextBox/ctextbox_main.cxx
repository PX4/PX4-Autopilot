/////////////////////////////////////////////////////////////////////////////
// NxWidgets/UnitTests/CTextBox/ctextbox_main.cxx
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

#include "ctextboxtest.hxx"

/////////////////////////////////////////////////////////////////////////////
// Definitions
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
// Private Classes
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
// Private Data
/////////////////////////////////////////////////////////////////////////////

static const char string1[] = "Johhn ";
static const char string2[] = "\b\b\bn Doe\r";

/////////////////////////////////////////////////////////////////////////////
// Public Function Prototypes
/////////////////////////////////////////////////////////////////////////////

// Suppress name-mangling

extern "C" int ctextbox_main(int argc, char *argv[]);

/////////////////////////////////////////////////////////////////////////////
// Public Functions
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
// nxheaders_main
/////////////////////////////////////////////////////////////////////////////

int ctextbox_main(int argc, char *argv[])
{
  // Create an instance of the font test

  printf("ctextbox_main: Create CTextBoxTest instance\n");
  CTextBoxTest *test = new CTextBoxTest();

  // Connect the NX server

  printf("ctextbox_main: Connect the CTextBoxTest instance to the NX server\n");
  if (!test->connect())
    {
      printf("ctextbox_main: Failed to connect the CTextBoxTest instance to the NX server\n");
      delete test;
      return 1;
    }

  // Create a window to draw into

  printf("ctextbox_main: Create a Window\n");
  if (!test->createWindow())
    {
      printf("ctextbox_main: Failed to create a window\n");
      delete test;
      return 1;
    }

  // Create a CTextBox instance

  CTextBox *textbox = test->createTextBox();
  if (!textbox)
    {
      printf("ctextbox_main: Failed to create a text box\n");
      delete test;
      return 1;
    }

  // Show the text box

  test->showTextBox(textbox);

  // Wait a bit, then inject a string with a typo

  sleep(1);
  test->injectChars(textbox, sizeof(string1), (FAR const uint8_t*)string1);

  // Now fix the string with backspaces and finish it correctly

  usleep(500*1000);
  test->injectChars(textbox, sizeof(string2), (FAR const uint8_t*)string2);
  
  // Clean up and exit

  sleep(2);
  printf("ctextbox_main: Clean-up and exit\n");
  delete textbox;
  delete test;
  return 0;
}

