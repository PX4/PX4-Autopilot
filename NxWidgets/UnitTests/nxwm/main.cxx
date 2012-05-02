/////////////////////////////////////////////////////////////////////////////
// NxWidgets/UnitTests/nxwm/main.cxx
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
#include <nuttx/arch.h>

#include <cstdio>
#include <cstdlib>

#include "ctaskbar.hxx"
#include "cstartwindow.hxx"
#include "cnxconsole.hxx"

/////////////////////////////////////////////////////////////////////////////
// Pre-processor Definitions
/////////////////////////////////////////////////////////////////////////////

// What is the entry point called?

#ifdef CONFIG_NSH_BUILTIN_APPS
#  define MAIN_NAME nxwm_main
#  define MAIN_STRING "nxwm_main: "
#else
#  define MAIN_NAME user_start
#  define MAIN_STRING "user_start: "
#endif

/////////////////////////////////////////////////////////////////////////////
// Private Types
/////////////////////////////////////////////////////////////////////////////

struct SNxWmTest
{
  NxWM::CTaskbar     *taskbar;     // The task bar
  NxWM::CStartWindow *startwindow; // The start window
};

/////////////////////////////////////////////////////////////////////////////
// Private Data
/////////////////////////////////////////////////////////////////////////////

static struct SNxWmTest g_nxwmtest;

/////////////////////////////////////////////////////////////////////////////
// Public Function Prototypes
/////////////////////////////////////////////////////////////////////////////

// Suppress name-mangling

extern "C" int MAIN_NAME(int argc, char *argv[]);

/////////////////////////////////////////////////////////////////////////////
// Public Functions
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
// user_start/nxwm_main
/////////////////////////////////////////////////////////////////////////////

int MAIN_NAME(int argc, char *argv[])
{
  // Call all C++ static constructors

#if defined(CONFIG_HAVE_CXX) && defined(CONFIG_HAVE_CXXINITIALIZE)
  up_cxxinitialize();
#endif

  // Create an instance of the Task Bar.
  //
  // The general sequence for initializing the task bar is:
  //
  // 1. Create the CTaskbar instance,
  // 2. Call the CTaskbar::connect() method to connect to the NX server (CTaskbar
  //    inherits the connect method from CNxServer),
  // 3. Call the CTaskbar::initWindowManager() method to initialize the task bar.
  // 3. Call CTaskBar::startApplication repeatedly to add applications to the task bar
  // 4. Call CTaskBar::startWindowManager to start the display with applications in place

  printf(MAIN_STRING "Create CTaskbar instance\n");
  g_nxwmtest.taskbar = new NxWM::CTaskbar();
  if (!g_nxwmtest.taskbar)
    {
      printf(MAIN_STRING "ERROR: Failed to instantiate CTaskbar\n");
      return EXIT_FAILURE;
    }

  // Connect to the NX server

  printf(MAIN_STRING "Connect the CTaskbar instance to the NX server\n");
  if (!g_nxwmtest.taskbar->connect())
    {
      printf(MAIN_STRING "ERROR: Failed to connect the CTaskbar instance to the NX server\n");
      delete g_nxwmtest.taskbar;
      return EXIT_FAILURE;
    }

  // Initialize the task bar
  //
  // Taskbar::initWindowManager() prepares the task bar to receive applications.
  // CTaskBar::startWindowManager() brings the window manager up with those applications
  // in place.

  printf(MAIN_STRING "Initialize the CTaskbar instance\n");
  if (!g_nxwmtest.taskbar->initWindowManager())
    {
      printf(MAIN_STRING "ERROR: Failed to intialize the CTaskbar instance\n");
      delete g_nxwmtest.taskbar;
      return EXIT_FAILURE;
    }
  
  // Create the start window.  The general sequence for setting up the start window is:
  //
  // 1. Call CTaskBar::openApplicationWindow to create a window for the start window,
  // 2. Use the window to instantiate Cstartwindow
  // 3. Call Cstartwindow::addApplication numerous times to install applications
  //    in the start window.
  // 4. Call CTaskBar::startApplication (initially minimized) to start the start
  //    window application.

  printf(MAIN_STRING "Opening the start window application window\n");
   NxWM::CApplicationWindow *window = g_nxwmtest.taskbar->openApplicationWindow();
  if (!window)
    {
      printf(MAIN_STRING "ERROR: Failed to create CApplicationWindow for the start window\n");
      delete g_nxwmtest.taskbar;
      return EXIT_FAILURE;
    }

  printf(MAIN_STRING "Creating the start window application\n");
  g_nxwmtest.startwindow = new NxWM::CStartWindow(g_nxwmtest.taskbar, window);
  if (!g_nxwmtest.taskbar)
    {
      printf(MAIN_STRING "ERROR: Failed to instantiate CStartWindow\n");
      delete window;
      delete g_nxwmtest.taskbar;
      return EXIT_FAILURE;
    }

  // Initialize the NSH library

  printf(MAIN_STRING "Initialize the NSH library\n");
  if (!NxWM::nshlibInitialize())
    {
      printf(MAIN_STRING "ERROR: Failed to initialize the NSH library\n");
      delete window;
      delete g_nxwmtest.taskbar;
      return EXIT_FAILURE;
    }

  // Add the NxConsole application to the start window

  NxWM::CNxConsole *console = (NxWM::CNxConsole *)0; // Avoid compiler complaint

  printf(MAIN_STRING "Opening the NxConsole application window\n");
  window = g_nxwmtest.taskbar->openApplicationWindow();
  if (!window)
    {
      printf(MAIN_STRING "ERROR: Failed to create CApplicationWindow for the NxConsole\n");
      goto noconsole;
    }

  printf(MAIN_STRING "Creating the NxConsole application\n");
  console = new  NxWM::CNxConsole(g_nxwmtest.taskbar, window);
  if (!console)
    {
      printf(MAIN_STRING "ERROR: Failed to instantiate CNxConsole\n");
      delete window;
      goto noconsole;
    }

  printf(MAIN_STRING "Adding the NxConsole application to the start window\n");
  if (!g_nxwmtest.startwindow->addApplication(console))
    {
      printf(MAIN_STRING "ERROR: Failed to add CNxConsole to the start window\n");
      delete window;
    }

noconsole:

  // Add the calculator application to the start window

#if 0
  NxWM::CCalculator *calculator = (NxWM::CCalculator *)0; // Avoid compiler complaint

  printf(MAIN_STRING "Opening the calculator application window\n");
  window = g_nxwmtest.taskbar->openApplicationWindow();
  if (!window)
    {
      printf(MAIN_STRING "ERROR: Failed to create CApplicationWindow for the calculator\n");
      goto nocalculator;
    }

  printf(MAIN_STRING "Creating the calculator application\n");
  calculator = new  NxWM::CCalculator(g_nxwmtest.taskbar, window);
  if (!calculator)
    {
      printf(MAIN_STRING "ERROR: Failed to instantiate calculator\n");
      delete window;
      goto nocalculator;
    }

  printf(MAIN_STRING "Adding the calculator application to the start window\n");
  if (!g_nxwmtest.startwindow->addApplication(calculator))
    {
      printf(MAIN_STRING "ERROR: Failed to add calculator to the start window\n");
      delete window;
    }

nocalculator:
#endif

  // Call CTaskBar::startApplication to start the start window application.  The initial
  // state of the start window is minimized.

  printf(MAIN_STRING "Start the start window application\n");
  if (!g_nxwmtest.taskbar->startApplication(g_nxwmtest.startwindow, true))
    {
      printf(MAIN_STRING "ERROR: Failed to start the start window application\n");

      // Delete the task bar then the start window.  the order is important because
      // we must bet all of the application references out of the task bar before
      // deleting the start window.  When the start window is deleted, it will
      // also delete of of the resouces contained within the start window.

      delete g_nxwmtest.taskbar;
      delete g_nxwmtest.startwindow;
      return EXIT_FAILURE;
    }

  // Call CTaskBar::startWindowManager to start the display with applications in place.
  // This method will not return but will enter the task bar's modal loop.

  printf(MAIN_STRING "Start the window manager\n");
  if (!g_nxwmtest.taskbar->startWindowManager())
    {
      printf(MAIN_STRING "ERROR: Failed to start the window manager\n");

      // Delete the task bar then the start window.  the order is important because
      // we must bet all of the application references out of the task bar before
      // deleting the start window.  When the start window is deleted, it will
      // also delete of of the resouces contained within the start window.

      delete g_nxwmtest.taskbar;
      delete g_nxwmtest.startwindow;
      return EXIT_FAILURE;
    }

  return EXIT_SUCCESS;    
}

