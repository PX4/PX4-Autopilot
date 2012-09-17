/////////////////////////////////////////////////////////////////////////////
// NxWidgets/UnitTests/CButtonArray/cbuttonarray_main.cxx
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
#include "cbuttonarraytest.hxx"

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

static FAR const char *g_buttonLabels[BUTTONARRAY_NCOLUMNS*BUTTONARRAY_NROWS] = {
 "=>", "A", "B", "<DEL", 
 "C", "D", "E", "F",
 "G", "H", "I", "J",
 "K", "L", "M", "N",
 "O", "P", "Q", "R",
 "S", "T", "U", "V",
 "W", "X", "Y", "Z"
};

/////////////////////////////////////////////////////////////////////////////
// Public Function Prototypes
/////////////////////////////////////////////////////////////////////////////

// Suppress name-mangling

extern "C" int cbuttonarray_main(int argc, char *argv[]);

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
// Name: checkHighlighting
/////////////////////////////////////////////////////////////////////////////

static void checkHighlighting(CButtonArray *buttonArray)
{
  // Turn highlighting on

  buttonArray->setCursorPosition(0, 0);
  buttonArray->cursor(true);

  // Then test the cursor movement

  for (int row = 0; row < BUTTONARRAY_NROWS; row++)
    {
      for (int column = 0; column < BUTTONARRAY_NCOLUMNS; column++)
        {
          // Set cursor position

          buttonArray->setCursorPosition(column, row);

          // Check cursor position

          int checkColumn;
          int checkRow;
          if (buttonArray->isCursorPosition(checkColumn, checkRow))
            {
              printf("ERROR: Not button selected\n");
              printf("       Expected (%d,%d)\n", column, row);
            }
          else if (checkColumn != column || checkRow != row)
            {
              printf("ERROR: Wrong button selected\n");
              printf("       Expected (%d,%d)\n", column, row);
              printf("       Selected (%d,%d)\n", checkColumn, checkRow);
            }

          // Wait a bit so that we can see the highlighting

          usleep(500*1000);
        }
    }

  // Turn highlighting off

  buttonArray->cursor(false);
}

/////////////////////////////////////////////////////////////////////////////
// Public Functions
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
// nxheaders_main
/////////////////////////////////////////////////////////////////////////////

int cbuttonarray_main(int argc, char *argv[])
{
  // Initialize memory monitor logic

  initMemoryUsage();

  // Create an instance of the button array test

  printf("cbuttonarray_main: Create CButtonArrayTest instance\n");
  CButtonArrayTest *test = new CButtonArrayTest();
  updateMemoryUsage(g_mmPrevious, "After creating CButtonArrayTest");

  // Connect the NX server

  printf("cbuttonarray_main: Connect the CButtonArrayTest instance to the NX server\n");
  if (!test->connect())
    {
      printf("cbuttonarray_main: Failed to connect the CButtonArrayTest instance to the NX server\n");
      delete test;
      return 1;
    }
  updateMemoryUsage(g_mmPrevious, "After connecting to the server");

  // Create a window to draw into

  printf("cbuttonarray_main: Create a Window\n");
  if (!test->createWindow())
    {
      printf("cbuttonarray_main: Failed to create a window\n");
      delete test;
      return 1;
    }
  updateMemoryUsage(g_mmPrevious, "After creating a window");

  // Create a CButtonArray instance

  CButtonArray *buttonArray = test->createButtonArray();
  if (!buttonArray)
    {
      printf("cbuttonarray_main: Failed to create a button array\n");
      delete test;
      return 1;
    }
  updateMemoryUsage(g_mmPrevious, "After creating CButtonArray");

  // Add the labels to each button

  FAR const char **ptr = g_buttonLabels;
  for (int j = 0; j < BUTTONARRAY_NROWS; j++)
    {
      for (int i = 0; i < BUTTONARRAY_NCOLUMNS; i++)
        {
          printf("cbuttonarray_main: Label (%d,%d): %s\n", i, j, *ptr);
          CNxString string = *ptr++;
          buttonArray->setText(i, j, string);
        }
    }
  updateMemoryUsage(g_mmPrevious, "After adding labels to the buttons");

  // Show the button array

  printf("cbuttonarray_main: Show the button array\n");
  test->showButton(buttonArray);
  sleep(1);

  // Verify that button highlighting works

  checkHighlighting(buttonArray);
  updateMemoryUsage(g_mmPrevious, "After highliting");

  // Then perform a simulated mouse click on a button in the array

  for (int j = 0; j < BUTTONARRAY_NROWS; j++)
    {
      for (int i = 0; i < BUTTONARRAY_NCOLUMNS; i++)
        {
          printf("cbuttonarray_main: Click the button (%d,%d)\n", i, j);
          test->click(buttonArray, i, j);

          // Poll for the mouse click event

          test->poll(buttonArray);

          // Is anything clicked?

          int clickColumn;
          int clickRow;
          if (buttonArray->isButtonClicked(clickColumn, clickRow))
            {
              printf("cbuttonarray_main: %s: Button (%d, %d) is clicked\n", 
                     clickColumn == i && clickRow == j ? "OK" : "ERROR",
                     clickColumn, clickRow);
            }
          else
            {
              printf("cbuttonarray_main: ERROR: No button is clicked\n");
            }

          // Wait a bit, then release the mouse button

          usleep(500*1000);
          test->release(buttonArray, i, j);

          // Poll for the mouse release event (of course this can hang if something fails)

          test->poll(buttonArray);
          if (buttonArray->isButtonClicked(clickColumn, clickRow))
            {
              printf("cbuttonarray_main: ERROR: Button (%d, %d) is clicked\n", 
                     clickColumn, clickRow);
            }
 
          usleep(500*1000);
        }
    }
  updateMemoryUsage(g_mmPrevious, "After pushing buttons");

  // Clean up and exit

  printf("cbuttonarray_main: Clean-up and exit\n");
  delete buttonArray;
  updateMemoryUsage(g_mmPrevious, "After deleting the button array");
  delete test;
  updateMemoryUsage(g_mmPrevious, "After deleting the test");
  updateMemoryUsage(g_mmInitial, "Final memory usage");
  message("Peak memory usage: %8d\n", g_mmPeak - g_mmInitial);
  return 0;
}

