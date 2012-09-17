/****************************************************************************
 * drivers/power/pm_update.c
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

#include <assert.h>

#include <nuttx/power/pm.h>
#include <nuttx/wqueue.h>

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
/* CONFIG_PM_MEMORY is the total number of time slices (including the current
 * time slice.  The histor or previous values is then CONFIG_PM_MEMORY-1.
 */

#if CONFIG_PM_MEMORY > 1
static const int16_t g_pmcoeffs[CONFIG_PM_MEMORY-1] =
{
  CONFIG_PM_COEF1
#if CONFIG_PM_MEMORY > 2
  , CONFIG_PM_COEF2
#endif
#if CONFIG_PM_MEMORY > 3
  , CONFIG_PM_COEF3
#endif
#if CONFIG_PM_MEMORY > 4
  , CONFIG_PM_COEF4
#endif
#if CONFIG_PM_MEMORY > 5
  , CONFIG_PM_COEF5
#endif
#if CONFIG_PM_MEMORY > 6
#  warning "This logic needs to be extended"
#endif
};
#endif

/* Threshold activity values to enter into the next lower power consumption
 * state. Indexing is next state 0:IDLE, 1:STANDBY, 2:SLEEP.
 */
 
static const int16_t g_pmenterthresh[3] =
{
  CONFIG_PM_IDLEENTER_THRESH,
  CONFIG_PM_STANDBYENTER_THRESH,
  CONFIG_PM_SLEEPENTER_THRESH
};

/* Threshold activity values to leave the current low power consdumption
 * state. Indexing is current state 0:IDLE, 1: STANDBY, 2: SLEEP.
 */

static const int16_t g_pmexitthresh[3] =
{
  CONFIG_PM_IDLEEXIT_THRESH,
  CONFIG_PM_STANDBYEXIT_THRESH,
  CONFIG_PM_SLEEPEXIT_THRESH
};

/* Threshold time slice count to enter the next low power consdumption
 * state. Indexing is next state 0:IDLE, 1: STANDBY, 2: SLEEP.
 */

static const uint16_t g_pmcount[3] =
{
  CONFIG_PM_IDLEENTER_COUNT,
  CONFIG_PM_STANDBYENTER_COUNT,
  CONFIG_PM_SLEEPENTER_COUNT
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pm_worker
 *
 * Description:
 *   This worker function is queue at the end of a time slice in order to
 *   update driver activity metrics and recommended states.
 *
 * Input Parameters:
 *   arg - The value of the activity accumulator at the end of the time
 *     slice.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   This function runs on the worker thread.
 *
 ****************************************************************************/

void pm_worker(FAR void *arg)
{
  int16_t accum = (int16_t)((intptr_t)arg);
  int32_t Y;
  int index;

#if CONFIG_PM_MEMORY > 1
  int32_t denom;
  int i, j;

  /* We won't bother to do anything until we have accumulated
   * CONFIG_PM_MEMORY-1 samples.
   */

  if (g_pmglobals.mcnt < CONFIG_PM_MEMORY-1)
    {
      g_pmglobals.memory[g_pmglobals.mcnt] = accum;
      g_pmglobals.mcnt++;
      return;
    }

  /* The averaging algorithm is simply: Y = (An*X + SUM(Ai*Yi))/SUM(Aj), where
   * i = 1..n-1 and j= 1..n, n is the length of the "memory", Ai is the
   * weight applied to each value, and X is the current activity.
   *
   * CONFIG_PM_MEMORY provides the memory for the algorithm.  Default: 2
   * CONFIG_PM_COEFn provides weight for each sample.  Default: 1
   *
   * First, calclate Y = An*X
   */

  Y     = CONFIG_PM_COEFN * accum;
  denom = CONFIG_PM_COEFN;

  /* Then calculate Y +=  SUM(Ai*Yi), i = 1..n-1.  The oldest sample will
   * reside at g_pmglobals.mndx (and this is the value that we will overwrite
   * with the new value).
   */

  for (i = 0, j =  g_pmglobals.mndx; i < CONFIG_PM_MEMORY-1; i++, j++)
    {
      if (j >= CONFIG_PM_MEMORY-1)
        {
          j = 0;
        }

      Y     += g_pmcoeffs[i] * g_pmglobals.memory[j];
      denom += g_pmcoeffs[i];
    }

  /* Compute and save the new activity value */

  Y /= denom;
  g_pmglobals.memory[g_pmglobals.mndx] = Y;
  g_pmglobals.mndx++;
  if (g_pmglobals.mndx >= CONFIG_PM_MEMORY-1)
    {
      g_pmglobals.mndx = 0;
    }

#else

  /* No smoothing */

  Y = accum;

#endif

  /* First check if increased activity should cause us to return to the
   * normal operating state.  This would be unlikely for the lowest power
   * consumption states because the CPU is probably asleep.  However this
   * probably does apply for the IDLE state.
   */

  if (g_pmglobals.state > PM_NORMAL)
    {
      /* Get the table index for the current state (which will be the
       * current state minus one)
       */

      index = g_pmglobals.state - 1;

      /* Has the threshold to return to normal power consumption state been
       * exceeded?
       */

      if (Y > g_pmexitthresh[index])
        {
          /* Yes... reset the count and recommend the normal state. */

          g_pmglobals.thrcnt = 0;
          g_pmglobals.recommended = PM_NORMAL;
          return;
        }
    }

  /* Now, compare this new activity level to the thresholds and counts for
   * the next lower power consumption state. If we are already in the SLEEP
   * state, then there is nothing more to be done (in fact, I would be
   * surprised to be executing!).
   */

  if (g_pmglobals.state < PM_SLEEP)
    {
      unsigned int nextstate;

      /* Get the next state and the table index for the next state (which will
       * be the current state)
       */

      index     = g_pmglobals.state;
      nextstate = g_pmglobals.state + 1;

      /* Has the threshold to enter the next lower power consumption state
       * been exceeded?
       */

      if (Y > g_pmenterthresh[index])
        {
          /* No... reset the count and recommend the current state */

          g_pmglobals.thrcnt      = 0;
          g_pmglobals.recommended = g_pmglobals.state;
        }

      /* Yes.. have we already recommended this state? If so, do nothing */

      else if (g_pmglobals.recommended < nextstate)
        {
          /* No.. increment the count.  Has is passed the the count required
           * for a state transition?
           */

          if (++g_pmglobals.thrcnt >= g_pmcount[index])
            {
              /* Yes, recommend the new state and set up for the next
               * transition.
               */

              g_pmglobals.thrcnt      = 0;
              g_pmglobals.recommended = nextstate;
            }
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pm_update
 *
 * Description:
 *   This internal function is called at the end of a time slice in order to
 *   update driver activity metrics and recommended states.
 *
 * Input Parameters:
 *   accum - The value of the activity accumulator at the end of the time
 *     slice.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   This function may be called from a driver, perhaps even at the interrupt
 *   level.  It may also be called from the IDLE loop at the lowest possible
 *   priority level.  To reconcile these various conditions, all work is
 *   performed on the worker thread at a user-selectable priority.  This will
 *   also serialize all of the updates and eliminate any need for additional
 *   protection.
 *
 ****************************************************************************/

void pm_update(int16_t accum)
{
  /* The work will be performed on the worker thread */

  DEBUGASSERT(g_pmglobals.work.worker == NULL);
  (void)work_queue(HPWORK, &g_pmglobals.work, pm_worker, (FAR void*)((intptr_t)accum), 0);
}

#endif /* CONFIG_PM */
