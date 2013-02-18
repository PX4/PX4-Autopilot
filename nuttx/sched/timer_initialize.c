/********************************************************************************
 * timer_initialize.c
 *
 *   Copyright (C) 2007, 2009 Gregory Nutt. All rights reserved.
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
 ********************************************************************************/

/********************************************************************************
 * Included Files
 ********************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <time.h>
#include <queue.h>
#include <errno.h>

#include "timer_internal.h"

#ifndef CONFIG_DISABLE_POSIX_TIMERS

/********************************************************************************
 * Definitions
 ********************************************************************************/

/********************************************************************************
 * Private Data
 ********************************************************************************/

/* These are the preallocated times */

#if CONFIG_PREALLOC_TIMERS > 0
static struct posix_timer_s g_prealloctimers[CONFIG_PREALLOC_TIMERS];
#endif

/********************************************************************************
 * Public Data
 ********************************************************************************/

/* This is a list of free, preallocated timer structures */

#if CONFIG_PREALLOC_TIMERS > 0
volatile sq_queue_t g_freetimers;
#endif

/* This is a list of instantiated timer structures -- active and inactive.  The
 * timers are place on this list by timer_create() and removed from the list by
 * timer_delete() or when the owning thread exits.
 */

volatile sq_queue_t g_alloctimers;

/********************************************************************************
 * Private Functions
 ********************************************************************************/

/********************************************************************************
 * Public Functions
 ********************************************************************************/

/********************************************************************************
 * Name: timer_initialize
 *
 * Description:
 *   Boot up configuration of the POSIX timer facility.
 *
 * Parameters:
 *   None
 *
 * Return Value:
 *   None
 *
 * Assumptions:
 *
 ********************************************************************************/

void weak_function timer_initialize(void)
{
#if CONFIG_PREALLOC_TIMERS > 0
  int i;

  /* Place all of the pre-allocated timers into the free timer list */

  sq_init((sq_queue_t*)&g_freetimers);

  for (i = 0; i < CONFIG_PREALLOC_TIMERS; i++)
    {
      g_prealloctimers[i].pt_flags = PT_FLAGS_PREALLOCATED;
      sq_addlast((FAR sq_entry_t*)&g_prealloctimers[i], (FAR sq_queue_t*)&g_freetimers);
    }
#endif

  /* Initialize the list of allocated timers */

  sq_init((sq_queue_t*)&g_alloctimers);
}

/********************************************************************************
 * Name: timer_deleteall
 *
 * Description:
 *   This function is called whenever a thread exits.  Any timers owned by that
 *   thread are deleted as though called by timer_delete().
 *
 *   It is provided in this file so that it can be weakly defined but also,
 *   like timer_intitialize(), be brought into the link whenever the timer
 *   resources are referenced.
 *
 * Parameters:
 *   pid - the task ID of the thread that exited
 *
 * Return Value:
 *   None
 *
 * Assumptions:
 *
 ********************************************************************************/

void weak_function timer_deleteall(pid_t pid)
{
  FAR struct posix_timer_s *timer;
  FAR struct posix_timer_s *next;
  irqstate_t flags;

  flags = irqsave();
  for (timer = (FAR struct posix_timer_s*)g_alloctimers.head; timer; timer = next)
    {
      next = timer->flink;
      if (timer->pt_owner == pid)
        {
          timer_delete((timer_t)timer);
        }
    }

  irqrestore(flags);
}

#endif /* CONFIG_DISABLE_POSIX_TIMERS */
