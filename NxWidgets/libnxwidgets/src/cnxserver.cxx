/****************************************************************************
 * NxWidgets/libnxwidgets/src/cnxserver.cxx
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <cstdlib>
#include <cerrno>
#include <debug.h>

#ifdef CONFIG_NX_MULTIUSER
#  include <sched.h>
#  include <pthread.h>
#endif

#include "nxconfig.hxx"
#include "singletons.hxx"
#include "cnxserver.hxx"

/****************************************************************************
 * Static Data Members
 ****************************************************************************/

using namespace NXWidgets;

uint8_t CNxServer::m_nServers;   /**< The number of NX server instances */

/****************************************************************************
 * Method Implementations
 ****************************************************************************/

/**
 * CNXServer constructor
 */

CNxServer::CNxServer(void)
{
  // Initialize server instance state data

  m_hDevice    = (FAR NX_DRIVERTYPE *)NULL;  // LCD/Framebuffer device handle 
  m_hNxServer  = (NXHANDLE)NULL;             // NX server handle
#ifdef CONFIG_NX_MULTIUSER
  m_connected  = false;                      // True:  Connected to the server
  sem_init(&m_connsem, 0, 0);                // Wait for server connection
#endif

  // Increment the global count of NX servers.  Normally there is only one
  // but we don't want to preclude the case where there might be multiple
  // displays, each with its own NX server instance

  m_nServers++;

  // Create miscellaneous singleton instances.  Why is this done here? 
  // Because this needs to be done once before any widgets are created and we
  // don't want to rely on static constructors.

  instantiateSingletons();
}

/**
 * CNXServer destructor
 */

CNxServer::~CNxServer(void)
{
  // Disconnect from the server

  disconnect();

  // Decrement the count of NX servers.  When that count goes to zero,
  // delete all of the fake static instances

  if (--m_nServers == 0)
    {
      freeSingletons();
    }
}

/**
 * Connect to the NX Server -- Single user version
 */

#ifndef CONFIG_NX_MULTIUSER
bool CNxServer::connect(void)
{
#if defined(CONFIG_NXWIDGETS_EXTERNINIT)
  // Use external graphics driver initialization

  m_hDevice = up_nxdrvinit(CONFIG_NXWIDGETS_DEVNO);
  if (!m_hDevice)
    {
      gdbg("up_nxdrvinit failed, devno=%d\n", CONFIG_NXWIDGETS_DEVNO);
      return false;
    }

#elif defined(CONFIG_NX_LCDDRIVER)
  int ret;

  // Initialize the LCD device

  ret = up_lcdinitialize();
  if (ret < 0)
    {
      gdbg("up_lcdinitialize failed: %d\n", -ret);
      return false;
    }

  // Get the device instance

  m_hDevice = up_lcdgetdev(CONFIG_NXWIDGETS_DEVNO);
  if (!m_hDevice)
    {
      gdbg("up_lcdgetdev failed, devno=%d\n", CONFIG_NXWIDGETS_DEVNO);
      return false;
    }

  // Turn the LCD on at 75% power

  (void)m_hDevice->setpower(m_hDevice, ((3*CONFIG_LCD_MAXPOWER + 3)/4));
#else
  int ret;

  // Initialize the frame buffer device

  ret = up_fbinitialize();
  if (ret < 0)
    {
      gdbg("up_fbinitialize failed: %d\n", -ret);
      return false;
    }

  m_hDevice = up_fbgetvplane(CONFIG_NXWIDGETS_VPLANE);
  if (!m_hDevice)
    {
      gdbg("CNxServer::connect: up_fbgetvplane failed, vplane=%d\n",
             CONFIG_NXWIDGETS_VPLANE);
      return false;
    }
#endif

  // Then open NX

  m_hNxServer = nx_open(m_hDevice);
  if (!m_hNxServer)
    {
      gdbg("CNxServer::connect: nx_open failed: %d\n", errno);
      return false;
    }

  return true;
}
#endif

/**
 * Connect to the NX Server -- Multi user version
 */

#ifdef CONFIG_NX_MULTIUSER
bool CNxServer::connect(void)
{
  struct sched_param param;
  pthread_t thread;
  pid_t serverId;
  int ret;

  // Set the client task priority

  param.sched_priority = CONFIG_NXWIDGETS_CLIENTPRIO;
  ret = sched_setparam(0, &param);
  if (ret < 0)
    {
      gdbg("CNxServer::connect: sched_setparam failed: %d\n" , ret);
      return false;
    }

  // Start the server task

  gvdbg("NxServer::connect: Starting server task\n");
  serverId = TASK_CREATE("NX Server", CONFIG_NXWIDGETS_SERVERPRIO,
                         CONFIG_NXWIDGETS_SERVERSTACK, server, (FAR const char **)0);
  if (serverId < 0)
    {
      gdbg("NxServer::connect: Failed to create nx_servertask task: %d\n", errno);
      return false;
    }

  // Wait a bit to let the server get started

  sleep(1);

  // Connect to the server

  m_hNxServer = nx_connect();
  if (m_hNxServer)
    {
       pthread_attr_t attr;

       // Start a separate thread to listen for server events.  This is probably
       // the least efficient way to do this, but it makes this logic flow more
       // smoothly.

       (void)pthread_attr_init(&attr);
       param.sched_priority = CONFIG_NXWIDGETS_LISTENERPRIO;
       (void)pthread_attr_setschedparam(&attr, &param);
       (void)pthread_attr_setstacksize(&attr, CONFIG_NXWIDGETS_LISTENERSTACK);

       m_stop    = false;
       m_running = true;

       ret = pthread_create(&thread, &attr, listener, (FAR void *)this);
       if (ret != 0)
         {
            gdbg("NxServer::connect: pthread_create failed: %d\n", ret);
            m_running = false;
            disconnect();
            return false;
         }

       // Don't return until we are connected to the server

       while (!m_connected && m_running)
         {
           // Wait for the listener thread to wake us up when we really
           // are connected.

           (void)sem_wait(&m_connsem);
         }

       // In the successful case, the listener is still running (m_running)
       // and the server is connected (m_connected).  Anything else is a failure.

       if (!m_connected || !m_running)
         {
           disconnect();
           return false;
         }
    }
  else
    {
      gdbg("NxServer::connect: nx_connect failed: %d\n", errno);
      return false;
    }

  return true;
}
#endif

/**
 * Disconnect to the NX Server -- Single user version
 */

#ifndef CONFIG_NX_MULTIUSER
void CNxServer::disconnect(void)
{
  // Close the server

  if (m_hNxServer)
    {
      nx_close(m_hNxServer);
      m_hNxServer = NULL;
    }
}
#endif

/**
 * Disconnect to the NX Server -- Single user version
 */

#ifdef CONFIG_NX_MULTIUSER
void CNxServer::disconnect(void)
{
  // Is the listener running?
  // Hmm.. won't this hang is the listener is in a blocking call?

  while (m_running)
    {
      // Yes.. stop the listener thread

      m_stop = true;
      while (m_running)
        {
          // Wait for the listener thread to stop

          (void)sem_wait(&m_connsem);
        }
    }

  // Disconnect from the server

  if (m_hNxServer)
    {
      nx_disconnect(m_hNxServer);
      m_hNxServer = NULL;
    }
}
#endif

/**
 * NX server thread.  This is the entry point into the server thread that
 * serializes the multi-threaded accesses to the display.
 */

#ifdef CONFIG_NX_MULTIUSER
int CNxServer::server(int argc, char *argv[])
{
  FAR NX_DRIVERTYPE *dev;
  int ret;

#if defined(CONFIG_NXWIDGETS_EXTERNINIT)
  // Use external graphics driver initialization

  dev = up_nxdrvinit(CONFIG_NXWIDGETS_DEVNO);
  if (!dev)
    {
      gdbg("up_nxdrvinit failed, devno=%d\n", CONFIG_NXWIDGETS_DEVNO);
      return EXIT_FAILURE;
    }

#elif defined(CONFIG_NX_LCDDRIVER)
  // Initialize the LCD device

  ret = up_lcdinitialize();
  if (ret < 0)
    {
      gdbg("up_lcdinitialize failed: %d\n", -ret);
      return EXIT_FAILURE;
    }

  // Get the device instance

  dev = up_lcdgetdev(CONFIG_NXWIDGETS_DEVNO);
  if (!dev)
    {
      gdbg("up_lcdgetdev failed, devno=%d\n", CONFIG_NXWIDGETS_DEVNO);
      return EXIT_FAILURE;
    }

  // Turn the LCD on at 75% power

  (void)dev->setpower(dev, ((3*CONFIG_LCD_MAXPOWER + 3)/4));
#else
  // Initialize the frame buffer device

  ret = up_fbinitialize();
  if (ret < 0)
    {
      gdbg("nxcon_server: up_fbinitialize failed: %d\n", -ret);
      return EXIT_FAILURE;
    }

  dev = up_fbgetvplane(CONFIG_NXWIDGETS_VPLANE);
  if (!dev)
    {
      gdbg("up_fbgetvplane failed, vplane=%d\n", CONFIG_NXWIDGETS_VPLANE);
      return 2;
    }
#endif

  // Then start the server

  ret = nx_run(dev);
  gvdbg("nx_run returned: %d\n", errno);
  return EXIT_FAILURE;
}
#endif

/**
 * This is the entry point of a thread that listeners for and dispatches
 * events from the NX server.
 */

#ifdef CONFIG_NX_MULTIUSER
FAR void *CNxServer::listener(FAR void *arg)
{
  // The argument must be the CNxServer instance

  CNxServer *This = (CNxServer*)arg;

  // Process events forever 

  while (!This->m_stop)
    {
      // Handle the next event.  If we were configured blocking, then
      // we will stay right here until the next event is received.  Since
      // we have dedicated a while thread to servicing events, it would
      // be most natural to also select CONFIG_NX_BLOCKING -- if not, the
      // following would be a tight infinite loop (unless we added addition
      // logic with nx_eventnotify and sigwait to pace it).

      int ret = nx_eventhandler(This->m_hNxServer);
      if (ret < 0)
        {
          // An error occurred... assume that we have lost connection with
          // the server.

          gdbg("Lost server connection: %d\n", errno);
          break;
        }

      // If we received a message, we must be connected

      if (!This->m_connected)
        {
          This->m_connected = true;
          sem_post(&This->m_connsem);
          gvdbg("Connected\n");
        }
    }

  // We fall out of the loop when either (1) the server has died or
  // we have been requested to stop

  This->m_running   = false;
  This->m_connected = false;
  sem_post(&This->m_connsem);
  return NULL;
}
#endif
