/****************************************************************************
 * sched/work_cancel.c
 *
 *   Copyright (C) 2009-2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include <queue.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>

#include "work_internal.h"

#ifdef CONFIG_SCHED_WORKQUEUE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: work_cancel
 *
 * Description:
 *   Cancel previously queued work.  This removes work from the work queue.
 *   After work has been canceled, it may be re-queue by calling work_queue()
 *   again.
 *
 * Input parameters:
 *   work   - The previously queue work structure to cancel
 *
 * Returned Value:
 *   Zero on success, a negated errno on failure
 *
 ****************************************************************************/

int work_cancel(struct work_s *work)
{
  irqstate_t flags;

  DEBUGASSERT(work != NULL);

  /* Cancelling the work is simply a matter of removing the work structure
   * from the work queue.  This must be done with interrupts disabled because
   * new work is typically added to the work queue from interrupt handlers.
   */

  flags = irqsave();
  if (work->worker != NULL)
    {
      DEBUGASSERT(work->dq.flink || (FAR dq_entry_t *)work == g_work.head);
      DEBUGASSERT(work->dq.blink || (FAR dq_entry_t *)work == g_work.tail);
      dq_rem((FAR dq_entry_t *)work, &g_work);

      work->worker = NULL;
    }
  irqrestore(flags);
  return OK;
}
#endif /* CONFIG_SCHED_WORKQUEUE */
