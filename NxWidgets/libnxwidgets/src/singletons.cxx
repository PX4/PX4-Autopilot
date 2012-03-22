/****************************************************************************
 * NxWidgets/libnxwidgets/src/singletons.cxx
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
 ****************************************************************************
 *
 * Portions of this package derive from Woopsi (http://woopsi.org/) and
 * portions are original efforts.  It is difficult to determine at this
 * point what parts are original efforts and which parts derive from Woopsi.
 * However, in any event, the work of  Antony Dzeryn will be acknowledged
 * in most NxWidget files.  Thanks Antony!
 *
 *   Copyright (c) 2007-2011, Antony Dzeryn
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * * Neither the names "Woopsi", "Simian Zombie" nor the
 *   names of its contributors may be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY Antony Dzeryn ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Antony Dzeryn BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <sched.h>
#include <debug.h>

#include "nxconfig.hxx"
#include "cnxserver.hxx"
#include "cnxstring.hxx"
#include "cwidgetstyle.hxx"
#include "cnxfont.hxx"
#include "singletons.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Global Static Data
 ****************************************************************************/

using namespace NXWidgets;

CWidgetStyle        *NXWidgets::g_defaultWidgetStyle; /**< The default widget style */
CNxString           *NXWidgets::g_nullString;         /**< The reusable empty string */
TNxArray<CNxTimer*> *NXWidgets::g_nxTimers;           /**< An array of all timers */

/****************************************************************************
 * Method Implementations
 ****************************************************************************/

/**
 * Setup misc singleton instances.  Why is there here?  Because it needs to be
 * done one time before any widgets are created.  So why isn't it just the
 * default style just a statically constructed class?  Because not all platforms
 * will support static class constructions.
 */

void NXWidgets::instantiateSingletons(void)
{
  // Disable pre-emption.  Since we are dealing with global resources here we
  // are not inherently thread-safe.

  sched_lock();

  // Create a global, empty string that may be used whereever a string is
  // required, but not needed.

  if (!g_nullString)
    {
      g_nullString = new CNxString();
    }

  // Setup the default widget style

  if (!g_defaultWidgetStyle)
    {
      // Create the singleton
 
      g_defaultWidgetStyle                   = new CWidgetStyle();

      // Default colors

      g_defaultWidgetStyle->colors.background         = CONFIG_NXWIDGETS_DEFAULT_BACKGROUNDCOLOR;
      g_defaultWidgetStyle->colors.selectedBackground = CONFIG_NXWIDGETS_DEFAULT_SELECTEDBACKGROUNDCOLOR;
      g_defaultWidgetStyle->colors.shineEdge          = CONFIG_NXWIDGETS_DEFAULT_SHINEEDGECOLOR;
      g_defaultWidgetStyle->colors.shadowEdge         = CONFIG_NXWIDGETS_DEFAULT_SHADOWEDGECOLOR;
      g_defaultWidgetStyle->colors.highlight          = CONFIG_NXWIDGETS_DEFAULT_HIGHLIGHTCOLOR;

      g_defaultWidgetStyle->colors.disabledText       = CONFIG_NXWIDGETS_DEFAULT_DISABLEDTEXTCOLOR;
      g_defaultWidgetStyle->colors.enabledText        = CONFIG_NXWIDGETS_DEFAULT_ENABLEDTEXTCOLOR;
      g_defaultWidgetStyle->colors.selectedText       = CONFIG_NXWIDGETS_DEFAULT_SELECTEDTEXTCOLOR;

      // Default font using the default font ID and the default font colors

      g_defaultWidgetStyle->font = new CNxFont((enum nx_fontid_e)CONFIG_NXWIDGETS_DEFAULT_FONTID,
                                               CONFIG_NXWIDGETS_DEFAULT_FONTCOLOR,
                                               CONFIG_NXWIDGETS_TRANSPARENT_COLOR);
    }

  // Create the timer list

  if (!g_nxTimers)
    {
      g_nxTimers = new TNxArray<CNxTimer*>();
    }

  sched_unlock();
}

/**
 * Free the singleton instances when the last NX server is destroyed.
 */

void NXWidgets::freeSingletons(void)
{
  // Delete the null string singleton

  if (g_nullString)
    {
      delete g_nullString;
      g_nullString = (CNxString *)NULL;
    }

  // Delete the default widget style singleton

  if (g_defaultWidgetStyle)
    {
      if (g_defaultWidgetStyle->font)
        {
          delete g_defaultWidgetStyle->font;
        }

      delete g_defaultWidgetStyle;
      g_defaultWidgetStyle = (CWidgetStyle *)NULL;
    }

  // Free the timer list

  if (g_nxTimers)
    {
      delete g_nxTimers;
      g_nxTimers = (TNxArray<CNxTimer*> *)NULL;
    }

}

