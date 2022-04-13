/****************************************************************************
 * libc/wqueue/work_queue.c
 *
 *   Copyright (C) 2009-2011, 2013 Gregory Nutt. All rights reserved.
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

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/workqueue.h>
#include <px4_platform_common/tasks.h>

#include <signal.h>
#include <stdint.h>
#include <queue.h>
#include <stdio.h>
#include <semaphore.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/workqueue.h>
#include <px4_platform_common/posix.h>
#include "hrt_work.h"

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
 * Name: hrt_work_queue
 *
 * Description:
 *   Queue work to be performed at a later time.  All queued work will be
 *   performed on the worker thread of of execution (not the caller's).
 *
 *   The work structure is allocated by caller, but completely managed by
 *   the work queue logic.  The caller should never modify the contents of
 *   the work queue structure; the caller should not call work_queue()
 *   again until either (1) the previous work has been performed and removed
 *   from the queue, or (2) work_cancel() has been called to cancel the work
 *   and remove it from the work queue.
 *
 * Input parameters:
 *   work   - The work structure to queue
 *   worker - The worker callback to be invoked.  The callback will invoked
 *            on the worker thread of execution.
 *   arg    - The argument that will be passed to the workder callback when
 *            int is invoked.
 *   delay  - Delay (in microseconds) from the time queue until the worker
 *            is invoked. Zero means to perform the work immediately.
 *
 * Returned Value:
 *   Zero on success, a negated errno on failure
 *
 ****************************************************************************/

int hrt_work_queue(struct work_s *work, worker_t worker, void *arg, uint32_t delay)
{
	struct wqueue_s *wqueue = &g_hrt_work;

	/* First, initialize the work structure */

	work->worker = worker;           /* Work callback */
	work->arg    = arg;              /* Callback argument */
	work->delay  = delay;            /* Delay until work performed */

	/* Now, time-tag that entry and put it in the work queue.  This must be
	 * done with interrupts disabled.  This permits this function to be called
	 * from with task logic or interrupt handlers.
	 */

	hrt_work_lock();
	work->qtime  = hrt_absolute_time(); /* Time work queued */
	//PX4_INFO("hrt work_queue adding work delay=%u time=%lu", delay, work->qtime);

	dq_addlast(&work->dq, &wqueue->q);

	if (px4_getpid() != wqueue->pid) { /* only need to wake up if called from a different thread */
#ifdef __PX4_QURT
		px4_task_kill(wqueue->pid, SIGALRM);      /* Wake up the worker thread */
#else
		//wqueue->pid == own task? -> don't signal
		px4_task_kill(wqueue->pid, SIGCONT);      /* Wake up the worker thread */
#endif
	}

	hrt_work_unlock();
	return PX4_OK;
}

