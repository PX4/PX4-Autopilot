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
#include <debug.h>

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
      FAR void             *console;  /**< The console 'this' pointer use with on_exit() */
      sem_t                 exclSem;  /**< Sem that gives exclusive access to this structure */
      sem_t                 waitSem;  /**< Sem that posted when the task is initialized */
      NXTKWINDOW            hwnd;     /**< Window handle */
      NXCONSOLE             nxcon;    /**< NxConsole handle */
      int                   minor;    /**< Next device minor number */
      struct nxcon_window_s wndo;     /**< Describes the NxConsole window */
      bool                  result;   /**< True if successfully initialized */
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
  // There would be a problem if we were stopped with the NxConsole task
  // running... that should never happen but we'll check anyway:

  stop();

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

IApplicationWindow *CNxConsole::getWindow(void) const
{
  return static_cast<IApplicationWindow*>(m_window);
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
    new NXWidgets::CRlePaletteBitmap(&CONFIG_NXWM_NXCONSOLE_ICON);

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

  // Get exclusive access to the global data structure

  if (sem_wait(&g_nxconvars.exclSem) != 0)
    {
      // This might fail if a signal is received while we are waiting.

      return false;
    }

  // Recover the NXTK window instance contained in the application window

  NXWidgets::INxWindow *window = m_window->getWindow();

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

  g_nxconvars.console = (FAR void *)this;
  g_nxconvars.result  = false;
  g_nxconvars.nxcon   = 0;

  sched_lock();
  m_pid = TASK_CREATE("NxConsole", CONFIG_NXWM_NXCONSOLE_PRIO,
                      CONFIG_NXWM_NXCONSOLE_STACKSIZE, nxconsole,
                      (FAR const char **)0);

  // Did we successfully start the NxConsole task?

  bool result = true;
  if (m_pid < 0)
    {
      result = false;
    }
  else
    {
      // Wait for up to two seconds for the task to initialize

      struct timespec abstime;
      clock_gettime(CLOCK_REALTIME, &abstime);
      abstime.tv_sec += 2;

      int ret = sem_timedwait(&g_nxconvars.waitSem, &abstime);
      sched_unlock();

      if (ret == OK && g_nxconvars.result)
        {
          // Save the handle to use in the stop method

          m_nxcon = g_nxconvars.nxcon;
        }
      else
        {
          // Stop the application

          stop();
          result = false;
        }
    }

  sem_post(&g_nxconvars.exclSem);
  return result;
}

/**
 * Stop the application.
 */

void CNxConsole::stop(void)
{
  // Delete the NxConsole task if it is still running (this could strand
  // resources). If we get here due to CTaskbar::stopApplication() processing
  // initialed by CNxConsole::exitHandler, then do *not* delete the task (it
  // is already being delete).

  if (m_pid >= 0)
    {
      // Calling task_delete() will also invoke the on_exit() handler.  We se
      // m_pid = -1 before calling task_delete() to let the on_exit() handler,
      // CNxConsole::exitHandler(), know that it should not do anything
 
      pid_t pid = m_pid;
      m_pid = -1;

      // Then delete the NSH task, possibly stranding resources

      task_delete(pid);
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
}

/**
 * Redraw the entire window.  The application has been maximized or
 * otherwise moved to the top of the hierarchy.  This method is call from
 * CTaskbar when the application window must be displayed
 */

void CNxConsole::redraw(void)
{
  // Recover the NXTK window instance contained in the application window

  NXWidgets::INxWindow *window = m_window->getWindow();

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
 * Report of this is a "normal" window or a full screen window.  The
 * primary purpose of this method is so that window manager will know
 * whether or not it show draw the task bar.
 *
 * @return True if this is a full screen window.
 */

bool CNxConsole::isFullScreen(void) const
{
  return m_window->isFullScreen();
}

/**
 * This is the NxConsole task.  This function first redirects output to the
 * console window then calls to start the NSH logic.
 */

int CNxConsole::nxconsole(int argc, char *argv[])
{
  // To stop compiler complaining about "jump to label crosses initialization
  // of 'int fd'

  int fd = -1;

  // Set up an on_exit() event that will be called when this task exits

  if (on_exit(exitHandler, g_nxconvars.console) != 0)
    {
      goto errout;
    }

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
  sem_post(&g_nxconvars.waitSem);

  // Run the NSH console

#ifdef CONFIG_NSH_CONSOLE
  (void)nsh_consolemain(argc, argv);
#endif

  // We get here if the NSH console should exits.  nsh_consolemain() ALWAYS
  // exits by calling nsh_exit() (which is a pointer to nsh_consoleexit())
  // which, in turn, calls exit()

  return EXIT_SUCCESS;

errout_with_nxcon:
  nxcon_unregister(g_nxconvars.nxcon);

errout:
  g_nxconvars.nxcon  = 0;
  g_nxconvars.result = false;
  sem_post(&g_nxconvars.waitSem);
  return EXIT_FAILURE;
}

/**
 * This is the NxConsole task exit handler.  It registered with on_exit()
 * and called automatically when the nxconsole task exits.
 */

void CNxConsole::exitHandler(int code, FAR void *arg)
{
  CNxConsole *This = (CNxConsole *)arg;

  // If we got here because of the task_delete() call in CNxConsole::stop(),
  // then m_pid will be set to -1 to let us know that we do not need to do
  // anything

  if (This->m_pid >= 0)
    {
      // Set m_pid to -1 to prevent calling detlete_task() in CNxConsole::stop().
      // CNxConsole::stop() is called by the processing initiated by the following
      // call to CTaskbar::stopApplication()

      This->m_pid = -1;

      // Remove the NxConsole application from the taskbar

      This->m_taskbar->stopApplication(This);
    }
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
 * CNxConsoleFactory Constructor
 *
 * @param taskbar.  The taskbar instance used to terminate the console
 */

CNxConsoleFactory::CNxConsoleFactory(CTaskbar *taskbar)
{
  m_taskbar = taskbar;
}

/**
 * Create a new instance of an CNxConsole (as IApplication).
 */

IApplication *CNxConsoleFactory::create(void)
{
  // Call CTaskBar::openFullScreenWindow to create a full screen window for
  // the NxConsole application

  CApplicationWindow *window = m_taskbar->openApplicationWindow();
  if (!window)
    {
      gdbg("ERROR: Failed to create CApplicationWindow\n");
      return (IApplication *)0;
    }

  // Open the window (it is hot in here)

  if (!window->open())
    {
      gdbg("ERROR: Failed to open CApplicationWindow\n");
      delete window;
      return (IApplication *)0;
    }

  // Instantiate the application, providing the window to the application's
  // constructor

  CNxConsole *nxconsole = new CNxConsole(m_taskbar, window);
  if (!nxconsole)
    {
      gdbg("ERROR: Failed to instantiate CNxConsole\n");
      delete window;
      return (IApplication *)0;
    }

  return static_cast<IApplication*>(nxconsole);
}

/**
 * Get the icon associated with the application
 *
 * @return An instance if IBitmap that may be used to rend the
 *   application's icon.  This is an new IBitmap instance that must
 *   be deleted by the caller when it is no long needed.
 */

NXWidgets::IBitmap *CNxConsoleFactory::getIcon(void)
{
  NXWidgets::CRlePaletteBitmap *bitmap =
    new NXWidgets::CRlePaletteBitmap(&CONFIG_NXWM_NXCONSOLE_ICON);

  return bitmap;
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

  sem_init(&g_nxconvars.exclSem, 0, 1);
  sem_init(&g_nxconvars.waitSem, 0, 0);

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

