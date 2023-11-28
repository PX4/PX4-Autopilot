/****************************************************************************
 *
 * Copyright (C) 2022 ModalAI, Inc. All rights reserved.
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

#include <px4_platform_common/time.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/workqueue.h>
#include <drivers/drv_hrt.h>

#include <semaphore.h>
#include <time.h>
#include <string.h>
#include <errno.h>

#include "hrt_work.h"

static constexpr unsigned HRT_INTERVAL_MIN = 50;
static constexpr unsigned HRT_INTERVAL_MAX = 50000000;

static struct sq_queue_s	callout_queue;

static uint64_t			latency_baseline;

static uint64_t			latency_actual;

const uint16_t latency_bucket_count = LATENCY_BUCKET_COUNT;
const uint16_t latency_buckets[LATENCY_BUCKET_COUNT] = { 1, 2, 5, 10, 20, 50, 100, 1000 };
__EXPORT uint32_t latency_counters[LATENCY_BUCKET_COUNT + 1];

static px4_sem_t 	_hrt_lock;
static struct work_s	_hrt_work;

static int32_t dsp_offset = 0;

static void hrt_latency_update();

static void hrt_call_reschedule();
static void hrt_call_invoke();

hrt_abstime hrt_absolute_time_offset()
{
	return 0;
}

static void hrt_lock()
{
	px4_sem_wait(&_hrt_lock);
}

static void hrt_unlock()
{
	px4_sem_post(&_hrt_lock);
}

int px4_clock_settime(clockid_t clk_id, struct timespec *tp)
{
	return 0;
}

int px4_clock_gettime(clockid_t clk_id, struct timespec *tp)
{
	int rv = clock_gettime(clk_id, tp);
	hrt_abstime temp_abstime = ts_to_abstime(tp);

	if (dsp_offset < 0) {
		hrt_abstime temp_offset = -dsp_offset;

		if (temp_offset >= temp_abstime) { temp_abstime = 0; }

		else { temp_abstime -= temp_offset; }

	} else {
		temp_abstime += (hrt_abstime) dsp_offset;
	}

	tp->tv_sec = temp_abstime / 1000000;
	tp->tv_nsec = (temp_abstime % 1000000) * 1000;
	return rv;
}

hrt_abstime hrt_absolute_time()
{
	struct timespec ts;
	px4_clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts_to_abstime(&ts);
}

int hrt_set_absolute_time_offset(int32_t time_diff_us)
{
	// dsp_offset = time_diff_us;
	return 0;
}

void hrt_store_absolute_time(volatile hrt_abstime *t)
{
	*t = hrt_absolute_time();
}

bool	hrt_called(struct hrt_call *entry)
{
	return (entry->deadline == 0);
}

void	hrt_cancel(struct hrt_call *entry)
{
	hrt_lock();
	sq_rem(&entry->link, &callout_queue);
	entry->deadline = 0;
	entry->period = 0;
	hrt_unlock();
}

static void hrt_latency_update()
{
	uint16_t latency = latency_actual - latency_baseline;
	unsigned	index;

	for (index = 0; index < LATENCY_BUCKET_COUNT; index++) {
		if (latency <= latency_buckets[index]) {
			latency_counters[index]++;
			return;
		}
	}

	latency_counters[index]++;
}

void	hrt_call_init(struct hrt_call *entry)
{
	memset(entry, 0, sizeof(*entry));
}

void	hrt_call_delay(struct hrt_call *entry, hrt_abstime delay)
{
	entry->deadline = hrt_absolute_time() + delay;
}

void	hrt_init()
{
	sq_init(&callout_queue);

	int sem_ret = px4_sem_init(&_hrt_lock, 0, 1);

	if (sem_ret) {
		PX4_ERR("SEM INIT FAIL: %s", strerror(errno));
	}

	memset(&_hrt_work, 0, sizeof(_hrt_work));
}

static void
hrt_call_enter(struct hrt_call *entry)
{
	struct hrt_call	*call, *next;

	call = (struct hrt_call *)sq_peek(&callout_queue);

	if ((call == nullptr) || (entry->deadline < call->deadline)) {
		sq_addfirst(&entry->link, &callout_queue);
		hrt_call_reschedule();

	} else {
		do {
			next = (struct hrt_call *)sq_next(&call->link);

			if ((next == nullptr) || (entry->deadline < next->deadline)) {
				//lldbg("call enter after head\n");
				sq_addafter(&call->link, &entry->link, &callout_queue);
				break;
			}
		} while ((call = next) != nullptr);
	}
}

static void
hrt_tim_isr(void *p)
{
	latency_actual = hrt_absolute_time();
	hrt_latency_update();
	hrt_call_invoke();
	hrt_lock();
	hrt_call_reschedule();
	hrt_unlock();
}

static void
hrt_call_reschedule()
{
	hrt_abstime	now = hrt_absolute_time();
	hrt_abstime	delay = HRT_INTERVAL_MAX;
	struct hrt_call	*next = (struct hrt_call *)sq_peek(&callout_queue);
	hrt_abstime	deadline = now + HRT_INTERVAL_MAX;

	if (next != nullptr) {
		if (next->deadline <= (now + HRT_INTERVAL_MIN)) {
			delay = HRT_INTERVAL_MIN;

		} else if (next->deadline < deadline) {
			delay = next->deadline - now;
		}
	}

	latency_baseline = now + delay;
	hrt_work_cancel(&_hrt_work);
	hrt_work_queue(&_hrt_work, (worker_t)&hrt_tim_isr, nullptr, delay);
}

static void
hrt_call_internal(struct hrt_call *entry, hrt_abstime deadline, hrt_abstime interval, hrt_callout callout, void *arg)
{
	PX4_DEBUG("hrt_call_internal deadline=%lu interval = %lu", deadline, interval);
	hrt_lock();

	if (entry->deadline != 0) {
		sq_rem(&entry->link, &callout_queue);
	}

	entry->deadline = deadline;
	entry->period = interval;
	entry->callout = callout;
	entry->arg = arg;

	hrt_call_enter(entry);
	hrt_unlock();
}

void	hrt_call_after(struct hrt_call *entry, hrt_abstime delay, hrt_callout callout, void *arg)
{
	hrt_call_internal(entry,
			  hrt_absolute_time() + delay,
			  0,
			  callout,
			  arg);
}

void	hrt_call_every(struct hrt_call *entry, hrt_abstime delay, hrt_abstime interval, hrt_callout callout, void *arg)
{
	hrt_call_internal(entry,
			  hrt_absolute_time() + delay,
			  interval,
			  callout,
			  arg);
}

void	hrt_call_at(struct hrt_call *entry, hrt_abstime calltime, hrt_callout callout, void *arg)
{
	hrt_call_internal(entry, calltime, 0, callout, arg);
}

static void
hrt_call_invoke()
{
	struct hrt_call	*call;
	hrt_abstime deadline;

	hrt_lock();

	while (true) {
		hrt_abstime now = hrt_absolute_time();

		call = (struct hrt_call *)sq_peek(&callout_queue);

		if (call == nullptr) {
			break;
		}

		if (call->deadline > now) {
			break;
		}

		sq_rem(&call->link, &callout_queue);
		deadline = call->deadline;
		call->deadline = 0;

		if (call->callout) {
			hrt_unlock();
			call->callout(call->arg);
			hrt_lock();
		}

		if (call->period != 0) {
			if (call->deadline <= now) {
				call->deadline = deadline + call->period;
			}

			hrt_call_enter(call);
		}
	}

	hrt_unlock();
}
