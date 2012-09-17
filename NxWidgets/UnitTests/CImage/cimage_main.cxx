/////////////////////////////////////////////////////////////////////////////
// NxWidgets/UnitTests/CImage/cimage_main.cxx
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
#include "cimagetest.hxx"

/////////////////////////////////////////////////////////////////////////////
// Definitions
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
// Private Classes
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
// Private Data
/////////////////////////////////////////////////////////////////////////////

static struct mallinfo g_mmInitial;
static struct mallinfo g_mmprevious;

/////////////////////////////////////////////////////////////////////////////
// Public Function Prototypes
/////////////////////////////////////////////////////////////////////////////

// Suppress name-mangling

extern "C" int cimage_main(int argc, char *argv[]);

/////////////////////////////////////////////////////////////////////////////
// Private Functions
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
// Name: showMemoryUsage
/////////////////////////////////////////////////////////////////////////////

static void showMemoryUsage(FAR struct mallinfo *mmbefore,
                            FAR struct mallinfo *mmafter)
{
  message("VARIABLE  BEFORE   AFTER\n");
  message("======== ======== ========\n");
  message("arena    %8d %8d\n", mmbefore->arena,    mmafter->arena);
  message("ordblks  %8d %8d\n", mmbefore->ordblks,  mmafter->ordblks);
  message("mxordblk %8d %8d\n", mmbefore->mxordblk, mmafter->mxordblk);
  message("uordblks %8d %8d\n", mmbefore->uordblks, mmafter->uordblks);
  message("fordblks %8d %8d\n", mmbefore->fordblks, mmafter->fordblks);
}

/////////////////////////////////////////////////////////////////////////////
// Name: updateMemoryUsage
/////////////////////////////////////////////////////////////////////////////

static void updateMemoryUsage(FAR struct mallinfo *previous,
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
  showMemoryUsage(previous, &mmcurrent);

  /* Set up for the next test */

#ifdef CONFIG_CAN_PASS_STRUCTS
  g_mmprevious = mmcurrent;
#else
  memcpy(&g_mmprevious, &mmcurrent, sizeof(struct mallinfo));
#endif
}

/////////////////////////////////////////////////////////////////////////////
// Name: initMemoryUsage
/////////////////////////////////////////////////////////////////////////////

static void initMemoryUsage(void)
{
#ifdef CONFIG_CAN_PASS_STRUCTS
  g_mmInitial = mallinfo();
  g_mmprevious = g_mmInitial;
#else
  (void)mallinfo(&g_mmInitial);
  memcpy(&g_mmprevious, &g_mmInitial, sizeof(struct mallinfo));
#endif
}

/////////////////////////////////////////////////////////////////////////////
// Public Functions
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
// Name: nxheaders_main
/////////////////////////////////////////////////////////////////////////////

int cimage_main(int argc, char *argv[])
{
  // Initialize memory monitor logic

  initMemoryUsage();

  // Create an instance of the font test

  message("cimage_main: Create CImageTest instance\n");
  CImageTest *test = new CImageTest();
  updateMemoryUsage(&g_mmprevious, "After creating CImageTest");

  // Connect the NX server

  message("cimage_main: Connect the CImageTest instance to the NX server\n");
  if (!test->connect())
    {
      message("cimage_main: Failed to connect the CImageTest instance to the NX server\n");
      delete test;
      return 1;
    }
  updateMemoryUsage(&g_mmprevious, "After connecting to the server");

  // Create a window to draw into

  message("cimage_main: Create a Window\n");
  if (!test->createWindow())
    {
      message("cimage_main: Failed to create a window\n");
      delete test;
      return 1;
    }
  updateMemoryUsage(&g_mmprevious, "After creating a window");

  // Create an instance of the NuttX logo

  CRlePaletteBitmap *nuttxBitmap = new CRlePaletteBitmap(&g_nuttxBitmap);
  updateMemoryUsage(&g_mmprevious, "After creating the bitmap");

  // Create a CImage instance

  CImage *image = test->createImage(static_cast<IBitmap*>(nuttxBitmap));
  if (!image)
    {
      message("cimage_main: Failed to create a image\n");
      delete test;
      return 1;
    }
  updateMemoryUsage(&g_mmprevious, "After creating CImage");

  // Show the image

  test->showImage(image);
  updateMemoryUsage(&g_mmprevious, "After showing the image");
  sleep(5);

  // Clean up and exit

  message("cimage_main: Clean-up and exit\n");
  delete image;
  updateMemoryUsage(&g_mmprevious, "After deleting CImage");

  delete nuttxBitmap;
  updateMemoryUsage(&g_mmprevious, "After deleting the bitmap");

  delete test;
  updateMemoryUsage(&g_mmprevious, "After deleting the test");
  updateMemoryUsage(&g_mmInitial, "Final memory usage");
  return 0;
}

