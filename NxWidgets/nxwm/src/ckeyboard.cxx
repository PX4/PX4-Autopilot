/********************************************************************************************
 * NxWidgets/nxwm/src/ckeyboard.cxx
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

#include <cunistd>
#include <cerrno>
#include <cfcntl>

#include <sched.h>
#include <pthread.h>
#include <assert.h>
#include <debug.h>

#include "nxwmconfig.hxx"
#include "ckeyboard.hxx"

/********************************************************************************************
 * Pre-Processor Definitions
 ********************************************************************************************/

/********************************************************************************************
 * CKeyboard Method Implementations
 ********************************************************************************************/

using namespace NxWM;

/**
 * CKeyboard Constructor
 *
 * @param server. An instance of the NX server.  This will be needed for
 *   injecting mouse data.
 */

CKeyboard::CKeyboard(NXWidgets::CNxServer *server)
{
  m_server      = server;              // Save the NX server
  m_kbdFd     = -1;                    // Device driver is not opened
  m_state       = LISTENER_NOTRUNNING; // The listener thread is not running yet

  // Initialize the semaphore used to synchronize with the listener thread

  sem_init(&m_waitSem, 0, 0);
}

/**
 * CKeyboard Destructor
 */

CKeyboard::~CKeyboard(void)
{
  // Stop the listener thread

  m_state = LISTENER_STOPREQUESTED;

  // Wake up the listener thread so that it will use our buffer
  // to receive data
  // REVISIT:  Need wait here for the listener thread to terminate

  (void)pthread_kill(m_thread, CONFIG_NXWM_KEYBOARD_SIGNO);

  // Close the keyboard device (or should these be done when the thread exits?)

  if (m_kbdFd >= 0)
    {
      std::close(m_kbdFd);
    }
}

/**
 * Start the keyboard listener thread.
 *
 * @return True if the keyboard listener thread was correctly started.
 */

bool CKeyboard::start(void)
{
  pthread_attr_t attr;

  gvdbg("Starting listener\n");

  // Start a separate thread to listen for keyboard events

  (void)pthread_attr_init(&attr);

  struct sched_param param;
  param.sched_priority = CONFIG_NXWM_KEYBOARD_LISTENERPRIO;
  (void)pthread_attr_setschedparam(&attr, &param);

  (void)pthread_attr_setstacksize(&attr, CONFIG_NXWM_KEYBOARD_LISTENERSTACK);

  m_state  = LISTENER_STARTED; // The listener thread has been started, but is not yet running

  int ret = pthread_create(&m_thread, &attr, listener, (FAR void *)this);
  if (ret != 0)
    {
      gdbg("CKeyboard::start: pthread_create failed: %d\n", ret);
      return false;
    }

  // Detach from the thread

  (void)pthread_detach(m_thread);

  // Don't return until we are sure that the listener thread is running
  // (or until it reports an error).

  while (m_state == LISTENER_STARTED)
    {
      // Wait for the listener thread to wake us up when we really
      // are connected.

      (void)sem_wait(&m_waitSem);
    }

  // Then return true only if the listener thread reported successful
  // initialization.

  gvdbg("Listener m_state=%d\n", (int)m_state);
  return m_state == LISTENER_RUNNING;
}

 /**
 * The keyboard listener thread.  This is the entry point of a thread that
 * listeners for and dispatches keyboard events to the NX server.
 *
 * @param arg.  The CKeyboard 'this' pointer cast to a void*.
 * @return This function normally does not return but may return NULL on
 *   error conditions.
 */

FAR void *CKeyboard::listener(FAR void *arg)
{
  CKeyboard *This = (CKeyboard *)arg;

  gvdbg("Listener started\n");

  // Open the keyboard device

  This->m_kbdFd = std::open(CONFIG_NXWM_KEYBOARD_DEVPATH, O_RDONLY);
  if (This->m_kbdFd < 0)
    {
      gdbg("ERROR Failed to open %s for reading: %d\n",
           CONFIG_NXWM_KEYBOARD_DEVPATH, errno);
      This->m_state = LISTENER_FAILED;
      sem_post(&This->m_waitSem);
      return (FAR void *)0;
    }

  // Indicate that we have successfully initialized

  This->m_state = LISTENER_RUNNING;
  sem_post(&This->m_waitSem);

  // Now loop, reading and dispatching keyboard data

  while (This->m_state == LISTENER_RUNNING)
    {
      // Read one keyboard sample

      gvdbg("Listening for keyboard input\n");

      uint8_t rxbuffer[CONFIG_NXWM_KEYBOARD_BUFSIZE];
      ssize_t nbytes = read(This->m_kbdFd, rxbuffer, CONFIG_NXWM_KEYBOARD_BUFSIZE);

      // Check for errors

      if (nbytes < 0)
        {
          // The only expect error is to be interrupt by a signal
#ifdef CONFIG_DEBUG
          int errval = errno;

          gdbg("ERROR: read %s failed: %d\n", CONFIG_NXWM_KEYBOARD_DEVPATH, errval);
          DEBUGASSERT(errval == EINTR);
#endif
        }

      // Give the keyboard input to NX

      else if (nbytes > 0)
        {
          // Looks like good keyboard input... process it.
          // First, get the server handle

          NXHANDLE handle = This->m_server->getServer();

          // Then inject the keyboard input into NX

          int ret = nx_kbdin(handle, (uint8_t)nbytes, rxbuffer);
          if (ret < 0)
            {
              gdbg("ERROR: nx_kbdin failed\n");
            }
        }
    }

  // We should get here only if we were asked to terminate via
  // m_state = LISTENER_STOPREQUESTED

  gvdbg("Listener exiting\n");
  This->m_state = LISTENER_TERMINATED;
  return (FAR void *)0;
}
