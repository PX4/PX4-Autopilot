/////////////////////////////////////////////////////////////////////////////
// NxWidgets/UnitTests/CLabel/clabel_main.cxx
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

#include "clabeltest.hxx"

/////////////////////////////////////////////////////////////////////////////
// Definitions
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
// Private Classes
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
// Private Data
/////////////////////////////////////////////////////////////////////////////

static const char g_hello[] = "Hello, World!";

/////////////////////////////////////////////////////////////////////////////
// Public Function Prototypes
/////////////////////////////////////////////////////////////////////////////

// Suppress name-mangling

extern "C" int clabel_main(int argc, char *argv[]);

/////////////////////////////////////////////////////////////////////////////
// Public Functions
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
// nxheaders_main
/////////////////////////////////////////////////////////////////////////////

int clabel_main(int argc, char *argv[])
{
  // Create an instance of the font test

  printf("clabel_main: Create CLabelTest instance\n");
  CLabelTest *test = new CLabelTest();

  // Connect the NX server

  printf("clabel_main: Connect the CLabelTest instance to the NX server\n");
  if (!test->connect())
    {
      printf("clabel_main: Failed to connect the CLabelTest instance to the NX server\n");
      delete test;
      return 1;
    }

  // Create a window to draw into

  printf("clabel_main: Create a Window\n");
  if (!test->createWindow())
    {
      printf("clabel_main: Failed to create a window\n");
      delete test;
      return 1;
    }

  // Create a CLabel instance

  CLabel *label = test->createLabel(g_hello);
  if (!label)
    {
      printf("clabel_main: Failed to create a label\n");
      delete test;
      return 1;
    }

  // Show the label

  test->showLabel(label);
  sleep(5);

  // Clean up and exit

  printf("clabel_main: Clean-up and exit\n");
  delete label;
  delete test;
  return 0;
}

