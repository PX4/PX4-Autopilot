/********************************************************************************************
 * NxWidgets/nxwm/src/cnxconsole.cxx
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX, NxWidgets, nor the names of its contributors
 *    me be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ********************************************************************************************/

/********************************************************************************************
 * Included Files
 ********************************************************************************************/
 
#include <nuttx/config.h>

#include <cstdio>
#include <cstdlib>
#include <cunistd>
#include <ctime>

#include <fcntl.h>
#include <semaphore.h>
#include <sched.h>

#include <apps/nsh.h>

#include "cwidgetcontrol.hxx"

#include "nxwmconfig.hxx"
#include "nxwmglyphs.hxx"
#include "cnxconsole.hxx"

/********************************************************************************************
 * Pre-Processor Definitions
 ********************************************************************************************/

/********************************************************************************************
 * Private Types
 ********************************************************************************************/

namespace NxWM
{
    /**
     * This structure is used to pass start up parameters to the NxConsole task and to assure the
     * the NxConsole is successfully started.
     */

    struct SNxConsole
    {
      sem_t                 sem;    /**< Sem that posted when the task is initialized */
      NXTKWINDOW            hwnd;   /**< Window handle */
      NXCONSOLE             nxcon;  /**< NxConsole handle */
      int                   minor;  /**< Next device minor number */
      struct nxcon_window_s wndo;   /**< Describes the NxConsole window */
      bool                  result; /**< True if successfully initialized */
    };

/********************************************************************************************
 * Private Data
 ********************************************************************************************/

  /**
   * This global data structure is used to pass start parameters to NxConsole task and to
   * assure that the NxConsole is successfully started.
   */

  static struct SNxConsole g_nxconvars;
}

/********************************************************************************************
 * Private Functions
 ********************************************************************************************/

/********************************************************************************************
 * CNxConsole Method Implementations
 ********************************************************************************************/

using namespace NxWM;

/**
 * CNxConsole constructor
 *
 * @param window.  The application window
 */

CNxConsole::CNxConsole(CTaskbar *taskbar, CApplicationWindow *window)
{
  // Save the constructor data

  m_taskbar = taskbar;
  m_window  = window;

  // The NxConsole is not runing

  m_pid    = -1;
  m_nxcon   = 0;

  // Add our personalized window label

  NXWidgets::CNxString myName = getName();
  window->setWindowLabel(myName);

  // Add our callbacks to the application window

  window->registerCallbacks(static_cast<IApplicationCallback *>(this));
}

/**
 * CNxConsole destructor
 *
 * @param window.  The application window
 */

CNxConsole::~CNxConsole(void)
{
  // Although we didn't create it, we are responsible for deleting the
  // application window

  if (m_window)
    {
      delete m_window;
    }
}

/**
 * Each implementation of IApplication must provide a method to recover
 * the contained CApplicationWindow instance.
 */

CApplicationWindow *CNxConsole::getWindow(void) const
{
  return m_window;
}

/**
 * Get the icon associated with the application
 *
 * @return An instance if IBitmap that may be used to rend the
 *   application's icon.  This is an new IBitmap instance that must
 *   be deleted by the caller when it is no long needed.
 */

NXWidgets::IBitmap *CNxConsole::getIcon(void)
{
  NXWidgets::CRlePaletteBitmap *bitmap =
    new NXWidgets::CRlePaletteBitmap(&g_nshBitmap);

  return bitmap;
}

/**
 * Get the name string associated with the application
 *
 * @return A copy if CNxString that contains the name of the application.
 */

NXWidgets::CNxString CNxConsole::getName(void)
{
  return NXWidgets::CNxString("NuttShell");
}

/**
 * Start the application (perhaps in the minimized state).
 *
 * @return True if the application was successfully started.
 */

bool CNxConsole::run(void)
{
  // Some sanity checking

  if (m_pid >= 0 || m_nxcon != 0)
    {
      return false;
    }

  // Recover the NXTK window instance contained in the application window

  NXWidgets::CNxTkWindow *window = m_window->getWindow();

  // Get the widget control associated with the NXTK window

  NXWidgets::CWidgetControl *control =  window->getWidgetControl();

  // Get the window handle from the widget control

  g_nxconvars.hwnd = control->getWindowHandle();

  // Describe the NxConsole

  g_nxconvars.wndo.wcolor[0] = CONFIG_NXWM_NXCONSOLE_WCOLOR;
  g_nxconvars.wndo.fcolor[0] = CONFIG_NXWM_NXCONSOLE_FONTCOLOR;
  g_nxconvars.wndo.fontid    = CONFIG_NXWM_NXCONSOLE_FONTID;

  // Get the size of the window

  (void)window->getSize(&g_nxconvars.wndo.wsize);

  // Start the NxConsole task

  g_nxconvars.result = false;
  g_nxconvars.nxcon  = 0;

  sched_lock();
  m_pid = TASK_CREATE("NxConsole", CONFIG_NXWM_NXCONSOLE_PRIO,
                      CONFIG_NXWM_NXCONSOLE_STACKSIZE, nxconsole,
                      (FAR const char **)0);

  // Did we successfully start the NxConsole task?

  if (m_pid < 0)
    {
      return false;
    }

  // Wait for up to two second for the task to initialize

  struct timespec abstime;
  clock_gettime(CLOCK_REALTIME, &abstime);
  abstime.tv_sec += 2;

  int ret = sem_timedwait(&g_nxconvars.sem, &abstime);
  sched_unlock();

  if (ret == OK && g_nxconvars.result)
    {
      // Save the handle to use in the stop method

      m_nxcon = g_nxconvars.nxcon;
      return true;
    }
  else
    {
      // Stop the application

      stop();
      return false;
    }
}

/**
 * Stop the application.
 */

void CNxConsole::stop(void)
{
  // Delete the NxConsole task if it is still running (this could strand resources)

  if (m_pid >= 0)
    {
      task_delete(m_pid);
      m_pid = -1;
    }
 
  // Destroy the NX console device
 
  if (m_nxcon)
    {
      nxcon_unregister(m_nxcon);
      m_nxcon = 0;
    }
}

/**
 * The application window is hidden (either it is minimized or it is
 * maximized, but not at the top of the hierarchy
 */

void CNxConsole::hide(void)
{
  // Disable drawing and events
#warning "Missing logic"
}

/**
 * Redraw the entire window.  The application has been maximized or
 * otherwise moved to the top of the hierarchy.  This method is call from
 * CTaskbar when the application window must be displayed
 */

void CNxConsole::redraw(void)
{
  // Recover the NXTK window instance contained in the application window

  NXWidgets::CNxTkWindow *window = m_window->getWindow();

  // Get the size of the window

  struct nxgl_size_s windowSize;
  (void)window->getSize(&windowSize);

  // Redraw the entire NxConsole window

  struct nxgl_rect_s rect;
  rect.pt1.x = 0;
  rect.pt1.y = 0;
  rect.pt2.x = windowSize.w - 1;
  rect.pt2.y = windowSize.h - 1;

  nxcon_redraw(m_nxcon, &rect, false);
}

/**
 * This is the NxConsole task.  This function first redirects output to the
 * console window.
 */

int CNxConsole::nxconsole(int argc, char *argv[])
{
  // To stop compiler complaining about "jump to label crosses initialization
  // of 'int fd'

  int fd = -1;

  // Use the window handle to create the NX console

  g_nxconvars.nxcon = nxtk_register(g_nxconvars.hwnd, &g_nxconvars.wndo,
                                    g_nxconvars.minor);
  if (!g_nxconvars.nxcon)
    {
      goto errout;
    }

  // Construct the driver name using this minor number

  char devname[32];
  snprintf(devname, 32, "/dev/nxcon%d", g_nxconvars.minor);

  // Increment the minor number while it is protect by the semaphore

  g_nxconvars.minor++;

  // Open the NxConsole driver

  fd = open(devname, O_WRONLY);
  if (fd < 0)
    {
      goto errout_with_nxcon;
    }

  // Now re-direct stdout and stderr so that they use the NX console driver.
  // Notes: (1) stdin is retained (file descriptor 0, probably the the serial
  // console).  (2) Don't bother trying to put debug instrumentation in the
  // following becaue it will end up in the NxConsole window.

  (void)std::fflush(stdout);
  (void)std::fflush(stderr);

  (void)std::fclose(stdout);
  (void)std::fclose(stderr);

  (void)std::dup2(fd, 1);
  (void)std::dup2(fd, 2);

  (void)std::fdopen(1, "w");
  (void)std::fdopen(2, "w");

  // And we can close our original driver file descriptor

  std::close(fd);

  // Inform the parent thread that we successfully initialized

  g_nxconvars.result = true;
  sem_post(&g_nxconvars.sem);

  // Run the NSH console

#ifdef CONFIG_NSH_CONSOLE
  (void)nsh_consolemain(argc, argv);
#endif

  // We get here if console exits
#warning "Missing logic"
  return EXIT_SUCCESS;

errout_with_nxcon:
  nxcon_unregister(g_nxconvars.nxcon);

errout:
  g_nxconvars.nxcon  = 0;
  g_nxconvars.result = false;
  sem_post(&g_nxconvars.sem);
  return EXIT_FAILURE;
}

/**
 * Called when the window minimize button is pressed.
 */

void CNxConsole::minimize(void)
{
  m_taskbar->minimizeApplication(static_cast<IApplication*>(this));
}

/**
 * Called when the window minimize close is pressed.
 */

void CNxConsole::close(void)
{
  m_taskbar->stopApplication(static_cast<IApplication*>(this));
}

/**
 * One time NSH initialization. This function must be called exactly
 * once during the boot-up sequence to initialize the NSH library.
 *
 * @return True on successful initialization
 */

bool NxWM::nshlibInitialize(void)
{
  // Initialize the global data structure

  sem_init(&g_nxconvars.sem, 0, 0);

  // Initialize the NSH library

  nsh_initialize();

  // If the Telnet console is selected as a front-end, then start the
  // Telnet daemon.

#ifdef CONFIG_NSH_TELNET
  int ret = nsh_telnetstart();
  if (ret < 0)
    {
      // The daemon is NOT running!

      return false;
   }
#endif
  return true;
}

