/********************************************************************************
 * timer_release.c
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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

#include <queue.h>
#include <errno.h>

#include <nuttx/kmalloc.h>

#include "timer_internal.h"

#ifndef CONFIG_DISABLE_POSIX_TIMERS

/********************************************************************************
 * Definitions
 ********************************************************************************/

/********************************************************************************
 * Private Data
 ********************************************************************************/

/********************************************************************************
 * Public Data
 ********************************************************************************/

/********************************************************************************
 * Private Functions
 ********************************************************************************/

/********************************************************************************
 * Name: timer_free
 *
 * Description:
 *   Remove the timer from the allocated timer list and free it or return it to
 *   the free list (depending on whether or not the timer is one of the
 *   preallocated timers)
 *
 ********************************************************************************/

static inline void timer_free(struct posix_timer_s *timer)
{
  irqstate_t flags;

  /* Remove the timer from the allocated list */

  flags = irqsave();
  sq_rem((FAR sq_entry_t*)timer, (sq_queue_t*)&g_alloctimers);

  /* Return it to the free list if it is one of the preallocated timers */

#if CONFIG_PREALLOC_TIMERS > 0
  if ((timer->pt_flags & PT_FLAGS_PREALLOCATED) != 0)
    {
      sq_addlast((FAR sq_entry_t*)timer, (FAR sq_queue_t*)&g_freetimers);
      irqrestore(flags);
    }
  else
#endif
    {
      /* Otherwise, return it to the heap */

      irqrestore(flags);
      sched_free(timer);
    }
}

/********************************************************************************
 * Public Functions
 ********************************************************************************/

/********************************************************************************
 * Name: timer_release
 *
 * Description:
 *   timer_release implements the heart of timer_delete.  It is private to the
 *   the OS internals and differs only in that return value of 1 means that the
 *   timer was not actually deleted.
 *
 * Parameters:
 *   timer - The per-thread timer, previously created by the call to
 *     timer_create(), to be deleted.
 *
 * Return Value:
 *   If the call succeeds, timer_release() will return 0 (OK) or 1 (meaning that
 *   the timer is still valid).  Otherwise, the function will return a negated errno:
 *
 *   -EINVAL - The timer specified timerid is not valid.
 *
 ********************************************************************************/

int timer_release(FAR struct posix_timer_s *timer)
{
  /* Some sanity checks */

  if (!timer)
    {
      return -EINVAL;
    }

  /* Release one reference to timer.  Don't delete the timer until the count
   * would decrement to zero.
   */

  if (timer->pt_crefs > 1)
    {
      timer->pt_crefs--;
      return 1;
    }

  /* Free the underlying watchdog instance (the timer will be canceled by the
   * watchdog logic before it is actually deleted)
   */

  (void)wd_delete(timer->pt_wdog);

  /* Release the timer structure */

  timer_free(timer);
  return OK;
}

#endif /* CONFIG_DISABLE_POSIX_TIMERS */
