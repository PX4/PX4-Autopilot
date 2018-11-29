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

#pragma once

#include "WorkQueueManager.hpp"

#include <containers/List.hpp>
#include <containers/Queue.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_defines.h>
#include <px4_sem.h>
#include <px4_tasks.h>

namespace px4
{

class WorkItem;

class WorkQueue : public ListNode<WorkQueue *>
{

public:

	WorkQueue(const wq_config &wq_config);
	~WorkQueue();

	const char *get_name() { return _config.name; }

	void Add(WorkItem *item);

	// TODO: need helpers to handle clean shutdown - remove and clear?
	//void remove(WorkItem *item);
	//void clear();

	void Run();

	void request_stop() { _should_exit = true; }

	void print_status();

private:

	bool should_exit() const { return _should_exit; }

	const wq_config &_config;

#ifdef __PX4_NUTTX
	void work_lock() { _flags = enter_critical_section(); }
	void work_unlock() { leave_critical_section(_flags); }
	irqstate_t _flags;
#else
	void work_lock() { px4_sem_wait(&_qlock); }
	void work_unlock() { px4_sem_post(&_qlock); }
	px4_sem_t _qlock;
#endif

	px4_sem_t _process_lock;

	Queue<WorkItem *>	_q;

	perf_counter_t	_perf_latency;

	bool	_should_exit{false};

};

} // namespace px4
