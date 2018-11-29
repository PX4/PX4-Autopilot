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

#include "WorkItem.hpp"

#include "WorkQueue.hpp"
#include "WorkQueueManager.hpp"

#include <px4_log.h>
#include <drivers/drv_hrt.h>

namespace px4
{

WorkItem::WorkItem(const wq_config &config)
{
#if WQ_ITEM_PERF
	_perf_cycle_time = perf_alloc(PC_ELAPSED, "wq_cycle_run_time");
	_perf_latency = perf_alloc(PC_ELAPSED, "wq_run_latency");
	_perf_interval = perf_alloc(PC_INTERVAL, "wq_run_interval");
#endif /* WQ_ITEM_PERF */

	if (!Init(config)) {
		PX4_ERR("init fail");
	}
}

WorkItem::~WorkItem()
{
#if WQ_ITEM_PERF
	perf_free(_perf_cycle_time);
	perf_free(_perf_latency);
	perf_free(_perf_interval);
#endif /* WQ_ITEM_PERF */
}

bool WorkItem::Init(const wq_config &config)
{
	px4::WorkQueue *wq = work_queue_create(config);

	if (wq != nullptr) {
		_wq = wq;

		return true;
	}

	return false;
}

void WorkItem::ScheduleNow()
{
	if (_wq != nullptr && !_queued) {
		_queued = true;
		_wq->Add(this);
	}
};

void WorkItem::pre_run()
{
	_queued = false;
#if WQ_ITEM_PERF
	perf_set_elapsed(_perf_latency, hrt_elapsed_time(&_qtime));
	perf_count(_perf_interval);
	perf_begin(_perf_cycle_time);
#endif /* WQ_ITEM_PERF */
}

void WorkItem::post_run()
{
#if WQ_ITEM_PERF
	perf_end(_perf_cycle_time);
#endif /* WQ_ITEM_PERF */
}

#if WQ_ITEM_PERF
void WorkItem::print_status() const
{
	perf_print_counter(_perf_cycle_time);
	perf_print_counter(_perf_interval);
	perf_print_counter(_perf_latency);
}
#endif /* WQ_ITEM_PERF */

} // namespace px4
