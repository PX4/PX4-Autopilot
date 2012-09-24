/****************************************************************************
 * drivers/power/pm_activity.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
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

#include <nuttx/power/pm.h>
#include <nuttx/clock.h>
#include <arch/irq.h>

#include "pm_internal.h"

#ifdef CONFIG_PM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pm_activity
 *
 * Description:
 *   This function is called by a device driver to indicate that it is
 *   performing meaningful activities (non-idle).  This increments an activity
 *   count and/or will restart a idle timer and prevent entering reduced
 *   power states.
 *
 * Input Parameters:
 *   priority - Activity priority, range 0-9.  Larger values correspond to
 *     higher priorities.  Higher priority activity can prevent the system
 *     from entering reduced power states for a longer period of time.
 *
 *     As an example, a button press might be higher priority activity because
 *     it means that the user is actively interacting with the device.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   This function may be called from an interrupt handler (this is the ONLY
 *   PM function that may be called from an interrupt handler!).
 *
 ****************************************************************************/

void pm_activity(int priority)
{
  uint32_t now;
  uint32_t accum;
  irqstate_t flags;

  /* Just increment the activity count in the current time slice. The priority
   * is simply the number of counts that are added.
   */

  if (priority > 0)
    {
      /* Add the priority to the accumulated counts in a critical section. */

      flags = irqsave();
      accum = (uint32_t)g_pmglobals.accum + priority;

      /* Make sure that we do not overflow the underlying uint16_t representation */

      if (accum > INT16_MAX)
        {
          accum = INT16_MAX;
        }

      /* Save the updated count */

      g_pmglobals.accum = (int16_t)accum;

      /* Check the elapsed time.  In periods of low activity, time slicing is
       * controlled by IDLE loop polling; in periods of higher activity, time
       * slicing is controlled by driver activity.  In either case, the duration
       * of the time slice is only approximate; during times of heavy activity,
       * time slices may be become longer and the activity level may be over-
       * estimated.
       */

      now = clock_systimer();
      if (now - g_pmglobals.stime >= TIME_SLICE_TICKS)
        {
          int16_t tmp;

          /* Sample the count, reset the time and count, and assess the PM
           * state.  This is an atomic operation because interrupts are
           * still disabled.
           */

          tmp               = g_pmglobals.accum;
          g_pmglobals.stime = now;
          g_pmglobals.accum = 0;

          /* Reassessing the PM state may require some computation.  However,
           * the work will actually be performed on a worker thread at a user-
           * controlled priority.
           */

          (void)pm_update(accum);
        }

      irqrestore(flags);
    }
}

#endif /* CONFIG_PM */