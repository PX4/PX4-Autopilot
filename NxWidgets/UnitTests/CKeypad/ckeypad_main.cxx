/////////////////////////////////////////////////////////////////////////////
// NxWidgets/UnitTests/CKeypad/ckeypad_main.cxx
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
#include <unistd.h>
#include <debug.h>

#include <nuttx/nx/nx.h>

#include "cnxstring.hxx"
#include "ckeypadtest.hxx"

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
static unsigned int g_mmPrevious;
static unsigned int g_mmPeak;

/////////////////////////////////////////////////////////////////////////////
// Public Function Prototypes
/////////////////////////////////////////////////////////////////////////////

// Suppress name-mangling

extern "C" int ckeypad_main(int argc, char *argv[]);

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

  message("%s: Before: %8d After: %8d Change: %8d\n",
           msg, previous, mmcurrent.uordblks, mmcurrent.uordblks - previous);

  /* Set up for the next test */

  g_mmPrevious = mmcurrent.uordblks;
  if ((unsigned int)mmcurrent.uordblks > g_mmPeak)
    {
      g_mmPeak = mmcurrent.uordblks;
    }
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
  g_mmPrevious = mmcurrent.uordblks;
  g_mmPeak     = mmcurrent.uordblks;
}

/////////////////////////////////////////////////////////////////////////////
// Name: clickButtons
/////////////////////////////////////////////////////////////////////////////

static void clickButtons(CKeypadTest *test, CKeypad *keypad)
{
  // Perform a simulated mouse click on a button in the keypad

  for (int j = 0; j < KEYPAD_NROWS; j++)
    {
      for (int i = 0; i < KEYPAD_NCOLUMNS; i++)
        {
          printf("clickButtons: Click the button (%d,%d)\n", i, j);
          test->click(keypad, i, j);

          // Poll for the mouse click event

          test->poll(keypad);

          // Is anything clicked?

          int clickColumn;
          int clickRow;
          if (keypad->isButtonClicked(clickColumn, clickRow))
            {
              printf("clickButtons: %s: Button (%d, %d) is clicked\n", 
                     clickColumn == i && clickRow == j ? "OK" : "ERROR",
                     clickColumn, clickRow);
            }
          else
            {
              printf("clickButtons: ERROR: No button is clicked\n");
            }

          // Wait a bit, then release the mouse button

          usleep(250*1000);
          test->release(keypad, i, j);

          // Poll for the mouse release event (of course this can hang if something fails)

          test->poll(keypad);
          if (keypad->isButtonClicked(clickColumn, clickRow))
            {
              printf("clickButtons: ERROR: Button (%d, %d) is clicked\n", 
                     clickColumn, clickRow);
            }
 
          usleep(500*1000);
        }
    }
  updateMemoryUsage(g_mmPrevious, "After pushing buttons");
}

/////////////////////////////////////////////////////////////////////////////
// Public Functions
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
// nxheaders_main
/////////////////////////////////////////////////////////////////////////////

int ckeypad_main(int argc, char *argv[])
{
  // Initialize memory monitor logic

  initMemoryUsage();

  // Create an instance of the keypad test

  printf("ckeypad_main: Create CKeypadTest instance\n");
  CKeypadTest *test = new CKeypadTest();
  updateMemoryUsage(g_mmPrevious, "After creating CKeypadTest");

  // Connect the NX server

  printf("ckeypad_main: Connect the CKeypadTest instance to the NX server\n");
  if (!test->connect())
    {
      printf("ckeypad_main: Failed to connect the CKeypadTest instance to the NX server\n");
      delete test;
      return 1;
    }
  updateMemoryUsage(g_mmPrevious, "After connecting to the server");

  // Create a window to draw into

  printf("ckeypad_main: Create a Window\n");
  if (!test->createWindow())
    {
      printf("ckeypad_main: Failed to create a window\n");
      delete test;
      return 1;
    }
  updateMemoryUsage(g_mmPrevious, "After creating a window");

  // Create a CKeypad instance

  CKeypad *keypad = test->createKeypad();
  if (!keypad)
    {
      printf("ckeypad_main: Failed to create a keypad\n");
      delete test;
      return 1;
    }
  updateMemoryUsage(g_mmPrevious, "After creating CKeypad");

  // Show the keypad in alphabetic mode

  printf("ckeypad_main: Show the keypad in alphabetic mode\n");
  keypad->setKeypadMode(false);
  test->showKeypad(keypad);
  sleep(1);

  // Then click some buttons

  clickButtons(test, keypad);
  sleep(1);

  // Show the keypad in numeric mode

  printf("ckeypad_main: Show the keypad in numeric mode\n");
  keypad->setKeypadMode(true);
  sleep(1);

  // Then click some buttons

  clickButtons(test, keypad);
  sleep(1);

  // Clean up and exit

  printf("ckeypad_main: Clean-up and exit\n");
  delete keypad;
  updateMemoryUsage(g_mmPrevious, "After deleting the keypad");
  delete test;
  updateMemoryUsage(g_mmPrevious, "After deleting the test");
  updateMemoryUsage(g_mmInitial, "Final memory usage");
  message("Peak memory usage: %8d\n", g_mmPeak - g_mmInitial);
  return 0;
}

