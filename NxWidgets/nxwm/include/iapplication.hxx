/****************************************************************************
 * NxWidgets/nxwm/include/iapplication.hxx
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

#ifndef __INCLUDE_IAPPLICATION_NXX
#define __INCLUDE_IAPPLICATION_NXX

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include "cnxstring.hxx"
#include "ibitmap.hxx"
#include "capplicationwindow.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Abstract Base Classes
 ****************************************************************************/

#if defined(__cplusplus)

namespace NxWM
{
  /**
   * IApplication provides the abstract base class for each NxWM application.
   */

  class IApplication
  {
    protected:
      // These values (and the accessors that go with them) violate the "purity"
      // of the base class.  These are really part of the task bar implementation:
      // Each application provides this state information needed by the taskbar.

      bool m_minimized; /**< True if the application is minimized */
      bool m_topapp;    /**< True if this application is at the top in the hiearchy */

    public:
      /**
       * Each implementation of IApplication must provide a method to recover
       * the contained CApplicationWindow instance.
       */

      virtual CApplicationWindow *getWindow(void) const = 0;

      /**
       * Get the icon associated with the application
       *
       * @return An instance if IBitmap that may be used to rend the
       *   application's icon.  This is an new IBitmap instance that must
       *   be deleted by the caller when it is no long needed.
       */

      virtual NXWidgets::IBitmap *getIcon(void) = 0;

      /**
       * Get the name string associated with the application
       *
       * @return A copy if CNxString that contains the name of the application.
       */

      virtual NXWidgets::CNxString getName(void) = 0;

      /**
       * Start the application (perhaps in the minimized state).
       *
       * @return True if the application was successfully started.
       */

      virtual bool run(void) = 0;

      /**
       * Stop the application.
       */

      virtual void stop(void) = 0;

      /**
       * The application window is hidden (either it is minimized or it is
       * maximized, but not at the top of the hierarchy
       */

      virtual void hide(void) = 0;

      /**
       * Redraw the entire window.  The application has been maximized or
       * otherwise moved to the top of the hierarchy.  This method is call from
       * CTaskbar when the application window must be displayed
       */

      virtual void redraw(void) = 0;

      /**
       * Set the application's minimized state
       *
       * @param minimized. True if the application is minimized
       */

      inline void setMinimized(bool minimized)
      {
        m_minimized = minimized;
      }

      /**
       * Set the application's top state
       *
       * @param topapp. True if the application is the new top application
       */

      inline void setTopApplication(bool topapp)
      {
        m_topapp = topapp;
      }

      /**
       * Get the application's minimized state
       *
       * @return True if the application is minimized
       */

      inline bool isMinimized(void) const
      {
        return m_minimized;
      }

      /**
       * Return true if this is the top application
       *
       * @return True if the application is the new top application
       */

      inline bool isTopApplication(void) const
      {
        return m_topapp;
      }
  };
}

#endif // __cplusplus

#endif // __INCLUDE_IAPPLICATION_NXX
