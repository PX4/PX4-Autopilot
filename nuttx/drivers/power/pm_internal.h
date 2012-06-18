/****************************************************************************
 * drivers/power/pm_internal.h
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

#ifndef __DRIVERS_POWER_PM_INTERNAL_H
#define __DRIVERS_POWER_PM_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <semaphore.h>
#include <queue.h>

#include <nuttx/power/pm.h>
#include <nuttx/wqueue.h>

#ifdef CONFIG_PM

/****************************************************************************
 * Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_SCHED_WORKQUEUE
#  warning "Worker thread support is required (CONFIG_SCHED_WORKQUEUE)"
#endif

/* Convert the time slice interval into system clock ticks.
 *
 * CONFIG_PM_SLICEMS provides the duration of one time slice in milliseconds.
 * CLOCKS_PER_SEC provides the number of timer ticks in one second.
 *
 * slice ticks = (CONFIG_PM_SLICEMS msec / 1000 msec/sec) /
 *               (CLOCKS_PER_SEC ticks/sec)
 */
 
#define TIME_SLICE_TICKS ((CONFIG_PM_SLICEMS * CLOCKS_PER_SEC) /  1000)

/* Function-like macros *****************************************************/
/****************************************************************************
 * Name: pm_lock
 *
 * Descripton:
 *   Lock the power management registry.  NOTE: This function may return
 *   an error if a signal is received while what (errno == EINTR).
 *
 ****************************************************************************/

#define pm_lock() sem_wait(&g_pmglobals.regsem);

/****************************************************************************
 * Name: pm_unlock
 *
 * Descripton:
 *   Unlock the power management registry.
 *
 ****************************************************************************/

#define pm_unlock() sem_post(&g_pmglobals.regsem);

/****************************************************************************
 * Public Types
 ****************************************************************************/
/* This structure encapsulates all of the global data used by the PM module */

struct pm_global_s
{
  /* state       - The current state (as determined by an explicit call to
   *             pm_changestate()
   * recommended - The recommended state based on the PM algorithm in
   *               function pm_update().
   * mndex       - The index to the next slot in the memory[] array to use.
   * mcnt        - A tiny counter used only at start up.  The actual
   *               algorithm cannot be applied until CONFIG_PM_MEMORY
   *               samples have been collected.
   */

  uint8_t state;
  uint8_t recommended;
  uint8_t mndx;
  uint8_t mcnt;

  /* accum - The accumulated counts in this time interval
   * thrcnt - The number of below threshold counts seen.
   */

  int16_t accum;
  uint16_t thrcnt;

  /* This is the averaging "memory."  The averaging algorithm is simply:
   * Y = (An*X + SUM(Ai*Yi))/SUM(Aj), where i = 1..n-1 and j= 1..n, n is the
   * length of the "memory", Ai is the weight applied to each value, and X is
   * the current activity.
   *
   * CONFIG_PM_MEMORY provides the memory for the algorithm.  Default: 2
   * CONFIG_PM_COEFn provides weight for each sample.  Default: 1
   */

#if CONFIG_PM_MEMORY > 1
  int16_t memory[CONFIG_PM_MEMORY-1];
#endif

  /* stime - The time (in ticks) at the start of the current time slice */

  uint32_t stime;

  /* This semaphore manages mutually exclusive access to the power management
   * registry.  It must be initialized to the value 1.
   */

  sem_t regsem;

  /* For work that has been deferred to the worker thread */

  struct work_s work;

  /* registry is a singly-linked list of registered power management
   * callback structures.  To ensure mutually exclusive access, this list
   * must be locked by calling pm_lock() before it is accessed.
   */

  sq_queue_t registry;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

/* All PM global data: */

EXTERN struct pm_global_s g_pmglobals;

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

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
 *   performed on the worker thread at a user-selectable priority.
 *
 ****************************************************************************/

EXTERN void pm_update(int16_t accum);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_PM */
#endif /* #define __DRIVERS_POWER_PM_INTERNAL_H */
