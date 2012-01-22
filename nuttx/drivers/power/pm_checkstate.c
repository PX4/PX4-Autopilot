/****************************************************************************
 * drivers/power/pm_checkstate.c
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
 * Name: pm_checkstate
 *
 * Description:
 *   This function is called from the MCU-specific IDLE loop to monitor the
 *   the power management conditions.  This function returns the "recommended"
 *   power management state based on the PM configuration and activity
 *   reported in the last sampling periods.  The power management state is
 *   not automatically changed, however.  The IDLE loop must call
 *   pm_changestate() in order to make the state change.
 *
 *   These two steps are separated because the plaform-specific IDLE loop may
 *   have additional situational information that is not available to the
 *   the PM sub-system.  For example, the IDLE loop may know that the
 *   battery charge level is very low and may force lower power states
 *   even if there is activity.
 *
 *   NOTE: That these two steps are separated in time and, hence, the IDLE
 *   loop could be suspended for a long period of time between calling
 *   pm_checkstate() and pm_changestate().  The IDLE loop may need to make
 *   these calls atomic by either disabling interrupts until the state change
 *   is completed.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The recommended power management state.
 *
 ****************************************************************************/

enum pm_state_e pm_checkstate(void)
{
  uint32_t now;
  irqstate_t flags;

  /* Check for the end of the current time slice.  This must be performed
   * with interrupts disabled so that it does not conflict with the similar
   * logic in pm_activity().
   */

  flags = irqsave();

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
       int16_t accum;

       /* Sample the count, reset the time and count, and assess the PM
        * state.  This is an atomic operation because interrupts are
        * still disabled.
        */

       accum             = g_pmglobals.accum;
       g_pmglobals.stime = now;
       g_pmglobals.accum = 0;

       /* Reassessing the PM state may require some computation.  However,
        * the work will actually be performed on a worker thread at a user-
        * controlled priority.
        */

       (void)pm_update(accum);
    }
  irqrestore(flags);

  /* Return the recommended state.  Assuming that we are called from the
   * IDLE thread at the lowest priority level, any updates scheduled on the
   * worker thread above should have already been peformed and the recommended
   * state should be current:
   */

  return g_pmglobals.recommended;
}

#endif /* CONFIG_PM */
