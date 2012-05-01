/****************************************************************************
 * NxWidgets/libnxwidgets/include/cnxserver.hxx
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

#ifndef __INCLUDE_CNXSERVER_HXX
#define __INCLUDE_CNXSERVER_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>

#include <nuttx/nx/nx.h>

#include "cnxwindow.hxx"
#include "cnxtkwindow.hxx"
#include "cbgwindow.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Implementation Classes
 ****************************************************************************/
 
#if defined(__cplusplus)

namespace NXWidgets
{
  /**
   * Forward references
   */

  class CBgWindow;

  /**
   * This class represents the NX server.  It includes methods to connect to
   * and disconnect form the NX server, methods to manage the background, and
   * "factory" methods to create window objects on the NX server.  NXWidget
   * objects them may be created on the window objects.
   */

  class CNxServer
  {
  private:
    FAR NX_DRIVERTYPE *m_hDevice;    /**< LCD/Framebuffer device handle */
    NXHANDLE           m_hNxServer;  /**< NX server handle */
#ifdef CONFIG_NX_MULTIUSER
    volatile bool      m_running;    /**< True: The listener thread is running */
    volatile bool      m_connected;  /**< True: Connected to the server */
    volatile bool      m_stop;       /**< True: Waiting for the listener thread to stop */
    sem_t              m_connsem;    /**< Wait for server connection */
#endif
    static uint8_t     m_nServers;   /**< The number of NX server instances */

    /**
     * NX server thread.  This is the entry point into the server thread that
     * serializes the multi-threaded accesses to the display.
     */

#ifdef CONFIG_NX_MULTIUSER
    static int server(int argc, char *argv[]);
#endif

    /**
     * NX listener thread.  This is the entry point of a thread that listeners for and
     * dispatches events from the NX server.
     */

#ifdef CONFIG_NX_MULTIUSER
    static FAR void *listener(FAR void *arg);
#endif

  public:

    /**
     * CNXServer constructor.  The CNxServer is a normally singleton.  However, that
     * not enforced:  This constructor could run more than one in the situation where
     * there are multiple physical displays.  However, that configuration has never
     * been texted.
     */

    CNxServer(void);

    /**
     * CNXServer destructor
     */

    ~CNxServer(void);

    /**
     * Connect to the NX Server
     */

    virtual bool connect(void);

    /**
     * Disconnect from the NX Server
     */

    virtual void disconnect(void);

    /**
     * Get the NX server handle
     *
     * @return The NX server handler (NULL is not connected)
     */

    inline NXHANDLE getServer(void)
    {
      return m_hNxServer;
    }

    /**
     * Test if we are connected to the NX server.
     *
     * @return True is connected; false is not connected.
     */

    inline bool connected(void)
    {
#ifdef CONFIG_NX_MULTIUSER
      return m_connected;
#else
      return true;
#endif
    }

    /**
     * Set the background color
     */

     inline bool setBackgroundColor(nxgl_mxpixel_t color)
     {
       return nx_setbgcolor(m_hNxServer, &color) == OK;
     }

    /**
     * Get an instance of a raw NX window.
     */

    inline CNxWindow *createRawWindow(CWidgetControl *widgetControl)
    {
      return new CNxWindow(m_hNxServer, widgetControl);
    }

    /**
     * Get an instance of the framed NX window.
     */

    inline CNxTkWindow *createFramedWindow(CWidgetControl *widgetControl)
    {
      return new CNxTkWindow(m_hNxServer, widgetControl);
    }

    /**
     * Get an instance of the background window.
     */

    inline CBgWindow *getBgWindow(CWidgetControl *widgetControl)
    {
      return new CBgWindow(m_hNxServer, widgetControl);
    }
  };
}

#endif // __cplusplus

#endif // __INCLUDE_CNXSERVER_HXX
