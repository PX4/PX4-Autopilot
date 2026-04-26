/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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

#include <px4_platform_common/px4_work_queue/WorkQueue.hpp>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>

#include <string.h>

#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>
#include <drivers/drv_hrt.h>

namespace px4
{

WorkQueue::WorkQueue(const wq_config_t &config) :
	_config(config)
{
	// WorkQueue is constructed on its own worker thread (see
	// WorkQueueRunner in WorkQueueManager.cpp), so pthread_self() here
	// captures the worker's tid for later self-wait checks in
	// WorkItem::Deinit.
	_runner_tid = pthread_self();

	// set the threads name
#ifdef __PX4_DARWIN
	pthread_setname_np(_config.name);
#else
	pthread_setname_np(pthread_self(), _config.name);
#endif

#ifndef __PX4_NUTTX
	px4_sem_init(&_qlock, 0, 1);
#endif /* __PX4_NUTTX */

	px4_sem_init(&_process_lock, 0, 0);
	px4_sem_setprotocol(&_process_lock, SEM_PRIO_NONE);

	px4_sem_init(&_exit_lock, 0, 1);
	px4_sem_setprotocol(&_exit_lock, SEM_PRIO_NONE);
}

WorkQueue::~WorkQueue()
{

	work_lock();

	// Synchronize with ::Detach
	px4_sem_wait(&_exit_lock);
	px4_sem_destroy(&_exit_lock);

	px4_sem_destroy(&_process_lock);
	work_unlock();

#ifndef __PX4_NUTTX
	px4_sem_destroy(&_qlock);
#endif /* __PX4_NUTTX */
}

bool WorkQueue::Attach(WorkItem *item)
{
	work_lock();

	if (!should_exit()) {
		_work_items.add(item);
		work_unlock();
		return true;
	}

	work_unlock();

	return false;
}

void WorkQueue::Detach(WorkItem *item)
{
	bool exiting = false;

	work_lock();

	_work_items.remove(item);

	if (_work_items.size() == 0) {
		// shutdown, no active WorkItems
		PX4_DEBUG("stopping: %s, last active WorkItem closing", _config.name);

		// Deletion of this work queue might happen right after request_stop or
		// SignalWorkerThread. Use a separate lock to prevent premature deletion
		px4_sem_wait(&_exit_lock);
		exiting = true;
		request_stop();
		SignalWorkerThread();
	}

	work_unlock();

	// In case someone is deleting this wq already, signal
	// that it is now allowed
	if (exiting) {
		px4_sem_post(&_exit_lock);
	}
}

void WorkQueue::Add(WorkItem *item)
{
	work_lock();

#if defined(ENABLE_LOCKSTEP_SCHEDULER)

	if (_lockstep_component == -1) {
		_lockstep_component = px4_lockstep_register_component();
	}

#endif // ENABLE_LOCKSTEP_SCHEDULER

	_q.push(item);
	work_unlock();

	SignalWorkerThread();
}

void WorkQueue::SignalWorkerThread()
{
	int sem_val;

	if (px4_sem_getvalue(&_process_lock, &sem_val) == 0 && sem_val <= 0) {
		px4_sem_post(&_process_lock);
	}
}

void WorkQueue::Remove(WorkItem *item)
{
	work_lock();
	_q.remove(item);
	work_unlock();
}

void WorkQueue::Clear()
{
	work_lock();

	while (!_q.empty()) {
		_q.pop();
	}

	work_unlock();
}

void WorkQueue::Run()
{
#if 0 // diagnostics: warn on large gaps between Run() invocations
	hrt_abstime _diag_last_run_time = 0;
	static hrt_abstime _diag_last_warn_time = 0;
#endif

	while (!should_exit()) {
		// loop as the wait may be interrupted by a signal
		do {} while (px4_sem_wait(&_process_lock) != 0);

#if 0 // diagnostics
		hrt_abstime now = hrt_absolute_time();
		hrt_abstime gap = (_diag_last_run_time > 0) ? (now - _diag_last_run_time) : 0;
		_diag_last_run_time = now;
#endif

		work_lock();

#if 0 // diagnostics: count queue depth
		int queue_depth = (int)_q.size();
#endif

		// process queued work
		while (!_q.empty()) {
			WorkItem *work = _q.pop();

			// Mark as in-flight so ~WorkItem / Deinit on another thread
			// can wait for us to return before the item's memory is torn
			// down. Without this, a Run() executing after Deinit has
			// popped the item will call Schedule* on dead memory,
			// corrupting the hrt callout queue.
			work->_run_in_progress.store(true);

			work_unlock(); // unlock work queue to run (item may requeue itself)
			work->RunPreamble();
			work->Run();
			work->_run_in_progress.store(false);
			work_lock(); // re-lock
		}

#if defined(ENABLE_LOCKSTEP_SCHEDULER)

		if (_q.empty()) {
			px4_lockstep_unregister_component(_lockstep_component);
			_lockstep_component = -1;
		}

#endif // ENABLE_LOCKSTEP_SCHEDULER

		work_unlock();

#if 0 // diagnostics: warn when work queue has large gaps

		if (gap > 100000) { // > 100ms sim time gap
			hrt_abstime warn_now = hrt_absolute_time();

			if (warn_now - _diag_last_warn_time > 1000000) { // max once per second sim time
				PX4_WARN("wq '%s': gap=%llums depth=%d sim_t=%.3fs",
					 _config.name,
					 (unsigned long long)(gap / 1000),
					 queue_depth,
					 (double)warn_now / 1e6);
				_diag_last_warn_time = warn_now;
			}
		}

#endif
	}

	PX4_DEBUG("%s: exiting", _config.name);
}

void WorkQueue::print_status(bool last)
{
	const size_t num_items = _work_items.size();
	PX4_INFO_RAW("%-16s\n", get_name());
	unsigned i = 0;

	for (WorkItem *item : _work_items) {
		i++;

		if (last) {
			PX4_INFO_RAW("    ");

		} else {
			PX4_INFO_RAW("|   ");
		}

		if (i < num_items) {
			PX4_INFO_RAW("|__%2d) ", i);

		} else {
			PX4_INFO_RAW("\\__%2d) ", i);
		}

		item->print_run_status();
	}
}

} // namespace px4
