/////////////////////////////////////////////////////////////////////////////
// NxWidgets/UnitTests/CListBox/clistbox_main.cxx
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

#include "clistboxtest.hxx"

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

static FAR const char *g_options[] =
{
  "American groundnut (Apios americana)",
  "Azuki bean (Vigna angularis)",
  "Black-eyed pea (Vigna unguiculata subsp. unguiculata)",
  "Chickpea (Cicer arietinum)",
  "Common bean (Phaseolus vulgaris)",
  "Drumstick (Moringa oleifera)",
  "Dolichos bean (Lablab purpureus)",
  "Fava bean (Vicia faba)",
  "Garbanzo (Cicer arietinum)",
  "Green bean (Phaseolus vulgaris)",
  "Guar (Cyamopsis tetragonoloba)",
  "Gumbo (Abelmoschus esculentus)",
  "Horse gram (Macrotyloma uniflorum)",
  "Indian pea (Lathyrus sativus)",
  "Lentil (Lens culinaris)",
  "Lima Bean (Phaseolus lunatus)",
  "Moth bean (Vigna acontifolia)",
  "Mung bean (Vigna radiata)",
  "Okra (Abelmoschus esculentus)",
  "Pea (Pisum sativum)",
  "Peanut (Arachis hypogaea)",
  "Pigeon pea (Cajanus cajan)",
  "Ricebean (Vigna umbellata)",
  "Runner bean (Phaseolus coccineus)",
  "Soybean (Glycine max)",
  "Tarwi (tarhui, chocho; Lupinus mutabilis)",
  "Tepary bean (Phaseolus acutifolius)",
  "Urad bean (Vigna mungo)",
  "Velvet bean (Mucuna pruriens)",
  "Winged bean (Psophocarpus tetragonolobus)",
  "Yardlong bean (Vigna unguiculata subsp. sesquipedalis)"
};
#define NOPTIONS (sizeof(g_options)/sizeof(FAR const char *))

/////////////////////////////////////////////////////////////////////////////
// Public Function Prototypes
/////////////////////////////////////////////////////////////////////////////

// Suppress name-mangling

extern "C" int clistbox_main(int argc, char *argv[]);

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
// Public Functions
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
// Name: nxheaders_main
/////////////////////////////////////////////////////////////////////////////

int clistbox_main(int argc, char *argv[])
{
  // Initialize memory monitor logic

  initMemoryUsage();

  // Create an instance of the listbox test

  message("clistbox_main: Create CListBoxTest instance\n");
  CListBoxTest *test = new CListBoxTest();
  updateMemoryUsage(g_mmPrevious, "After creating CListBoxTest");

  // Connect the NX server

  message("clistbox_main: Connect the CListBoxTest instance to the NX server\n");
  if (!test->connect())
    {
      message("clistbox_main: Failed to connect the CListBoxTest instance to the NX server\n");
      delete test;
      return 1;
    }
  updateMemoryUsage(g_mmPrevious, "clistbox_main: After connecting to the server");

  // Create a window to draw into

  message("clistbox_main: Create a Window\n");
  if (!test->createWindow())
    {
      message("clistbox_main: Failed to create a window\n");
      delete test;
      return 1;
    }
  updateMemoryUsage(g_mmPrevious, "clistbox_main: After creating a window");

  // Create a listbox

  message("clistbox_main: Create a ListBox\n");
  CListBox *listbox = test->createListBox();
  if (!listbox)
    {
      message("clistbox_main: Failed to create a listbox\n");
      delete test;
      return 1;
    }
  updateMemoryUsage(g_mmPrevious, "clistbox_main: After creating a listbox");

  // Show the initial state of the listbox

  listbox->setAllowMultipleSelections(true);
  test->showListBox(listbox);
  sleep(1);

  // Now add items to the list box (in reverse alphabetical order)

  message("clistbox_main: Add options to the ListBox\n");
  for (int i = NOPTIONS - 1; i >= 0; i--)
    {
      listbox->addOption(g_options[i],i);
      test->showListBox(listbox);
      message("clistbox_main: %d. New option %s\n", i, g_options[i]);
      usleep(500000); // The simulation needs this to let the X11 event loop run
    }
  updateMemoryUsage(g_mmPrevious, "clistbox_main: After adding the listbox items");
  sleep(1);

  // Sort the list box

  message("clistbox_main: Sort the ListBox\n");
  listbox->sort();
  test->showListBox(listbox);
  updateMemoryUsage(g_mmPrevious, "clistbox_main: After sorting the listbox");
  sleep(1);

  // Select and remove items from the listbox

  srand(1978);
  int nOptions;
  while ((nOptions = listbox->getOptionCount()) > 0)
    {
      message("clistbox_main: Option count: %d\n", nOptions);
      if (nOptions <= 5)
        {
          message("clistbox_main: Selecting all remaining options\n");
          listbox->selectAllOptions();
          test->showListBox(listbox);
          updateMemoryUsage(g_mmPrevious, "clistbox_main: After selecting all options");
          sleep(1);

          message("clistbox_main: Removing all remaining options\n");
          listbox->removeAllOptions();
          updateMemoryUsage(g_mmPrevious, "clistbox_main: After removing all options");
          test->showListBox(listbox);
        }
      else
        {
          int selected[5];

          message("clistbox_main: Selecting five options\n");
          for (int i = 0; i < 5; i++)
            {
              selected[i] = ((nOptions - 1) * rand()) / MAX_RAND;
              message("clistbox_main: Selecting option %d\n", selected[i]);
              listbox->removeOption(selected[i]);
              test->showListBox(listbox);
              usleep(500000);
            }
          updateMemoryUsage(g_mmPrevious, "clistbox_main: After selecting five options");

          message("clistbox_main: De-selecting options\n");
          int index;
          int count = 0;
          while ((index = listbox->getSelectedIndex()) >= 0)
            {
              message("clistbox_main: De-selecting option %d\n", index);
              listbox->deselectOption(index);
              test->showListBox(listbox);
              count++;
              usleep(500000);
            }

          message("clistbox_main: %s: %d options de-selected\n",
                  count == 5 ? "OK" : "ERROR", count);
          updateMemoryUsage(g_mmPrevious, "clistbox_main: After de-selecting options");
 
          message("clistbox_main: Removing the selected options\n");
          for (int i = 0; i < 5; i++)
            {
              message("clistbox_main: Removing option %d\n", selected[i]);
              listbox->removeOption(selected[i]);
              test->showListBox(listbox);
              usleep(500000);
            }
          updateMemoryUsage(g_mmPrevious, "clistbox_main: After removing five options");
        }
      sleep(1);
    }
  updateMemoryUsage(g_mmPrevious, "clistbox_main: After the listbox is empty again");
  sleep(1);

  // Clean up and exit

  message("clistbox_main: Clean-up and exit\n");
  delete listbox;
  updateMemoryUsage(g_mmPrevious, "After deleting the listbox");
  delete test;
  updateMemoryUsage(g_mmPrevious, "After deleting the test");
  updateMemoryUsage(g_mmInitial, "Final memory usage");
  message("Peak memory usage: %8d\n", g_mmPeak - g_mmInitial);
  return 0;
}

