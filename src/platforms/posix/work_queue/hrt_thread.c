/****************************************************************************
 * libc/wqueue/work_thread.c
 *
 *   Copyright (C) 2009-2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *   Modified by: Mark Charlebois to use hrt compatible times
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

#include <px4_config.h>
#include <px4_defines.h>
#include <stdint.h>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <queue.h>
#include <px4_workqueue.h>
#include <drivers/drv_hrt.h>
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

/* The state of each work queue. */
struct wqueue_s g_hrt_work;

/****************************************************************************
 * Private Variables
 ****************************************************************************/
sem_t _hrt_work_lock;

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static void hrt_work_process(void);

static void _sighandler(int sig_num);

/****************************************************************************
 * Name: _sighandler
 *
 * Description:
 *   This is the handler for the signal to wake the queue processing thread
 *
 * Input parameters:
 *   sig_num - the received signal
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
static void _sighandler(int sig_num)
{
	PX4_DEBUG("RECEIVED SIGNAL %d", sig_num);
}

/****************************************************************************
 * Name: work_process
 *
 * Description:
 *   This is the logic that performs actions placed on any work list.
 *
 * Input parameters:
 *   wqueue - Describes the work queue to be processed
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void hrt_work_process()
{
	struct wqueue_s *wqueue = &g_hrt_work;
	volatile struct work_s *work;
	worker_t  worker;
	void *arg;
	uint64_t elapsed;
	uint32_t remaining;
	uint32_t next;

	/* Then process queued work.  We need to keep interrupts disabled while
	 * we process items in the work list.
	 */

	/* Default to sleeping for 1 sec */
	next  = 1000000;

	hrt_work_lock();

	work  = (struct work_s *)wqueue->q.head;

	while (work) {
		/* Is this work ready?  It is ready if there is no delay or if
		 * the delay has elapsed. qtime is the time that the work was added
		 * to the work queue.  It will always be greater than or equal to
		 * zero.  Therefore a delay of zero will always execute immediately.
		 */

		elapsed = hrt_absolute_time() - work->qtime;

		//PX4_INFO("hrt work_process: in usec elapsed=%lu delay=%u work=%p", elapsed, work->delay, work);
		if (elapsed >= work->delay) {
			/* Remove the ready-to-execute work from the list */

			(void)dq_rem((struct dq_entry_s *)work, &wqueue->q);
			//PX4_INFO("Dequeued work=%p", work);

			/* Extract the work description from the entry (in case the work
			 * instance by the re-used after it has been de-queued).
			 */

			worker = work->worker;
			arg    = work->arg;

			/* Mark the work as no longer being queued */

			work->worker = NULL;

			/* Do the work.  Re-enable interrupts while the work is being
			 * performed... we don't have any idea how long that will take!
			 */

			hrt_work_unlock();

			if (!worker) {
				PX4_ERR("MESSED UP: worker = 0");

			} else {
				worker(arg);
			}

			/* Now, unfortunately, since we re-enabled interrupts we don't
			 * know the state of the work list and we will have to start
			 * back at the head of the list.
			 */

			hrt_work_lock();
			work  = (struct work_s *)wqueue->q.head;

		} else {
			/* This one is not ready.. will it be ready before the next
			 * scheduled wakeup interval?
			 */

			/* Here: elapsed < work->delay */
			remaining = work->delay - elapsed;

			//PX4_INFO("remaining=%u delay=%u elapsed=%lu", remaining, work->delay, elapsed);
			if (remaining < next) {
				/* Yes.. Then schedule to wake up when the work is ready */

				next = remaining;
			}

			/* Then try the next in the list. */

			work = (struct work_s *)work->dq.flink;
			//PX4_INFO("next %u work %p", next, work);
		}
	}

	/* Wait awhile to check the work list.  We will wait here until either
	 * the time elapses or until we are awakened by a signal.
	 */
	hrt_work_unlock();

	/* might sleep less if a signal received and new item was queued */
	//PX4_INFO("Sleeping for %u usec", next);
	usleep(next);
}

/****************************************************************************
 * Name: work_hrtthread
 *
 * Description:
 *   This is the worker threads that performs actions placed on the ISR work
 *   list.
 *
 *   work_hpthread and work_lpthread:  These are the kernel mode work queues
 *     (also build in the flat build).  One of these threads also performs
 *     periodic garbage collection (that is otherwise performed by the idle
 *     thread if CONFIG_SCHED_WORKQUEUE is not defined).
 *
 *     These worker threads are started by the OS during normal bringup.
 *
 *   All of these entrypoints are referenced by OS internally and should not
 *   not be accessed by application logic.
 *
 * Input parameters:
 *   argc, argv (not used)
 *
 * Returned Value:
 *   Does not return
 *
 ****************************************************************************/

static int work_hrtthread(int argc, char *argv[])
{
	/* Loop forever */

	for (;;) {
		/* First, perform garbage collection.  This cleans-up memory de-allocations
		 * that were queued because they could not be freed in that execution
		 * context (for example, if the memory was freed from an interrupt handler).
		 * NOTE: If the work thread is disabled, this clean-up is performed by
		 * the IDLE thread (at a very, very low priority).
		 */

		/* Then process queued work.  We need to keep interrupts disabled while
		 * we process items in the work list.
		 */

		hrt_work_process();
	}

	return PX4_OK; /* To keep some compilers happy */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void hrt_work_queue_init(void)
{
	sem_init(&_hrt_work_lock, 0, 1);

	// Create high priority worker thread
	g_hrt_work.pid = px4_task_spawn_cmd("wkr_hrt",
					    SCHED_DEFAULT,
					    SCHED_PRIORITY_MAX,
					    2000,
					    work_hrtthread,
					    (char *const *)NULL);

	
#ifdef __PX4_QURT
	signal(SIGALRM, _sighandler);
#else
	signal(SIGCONT, _sighandler);
#endif
}

