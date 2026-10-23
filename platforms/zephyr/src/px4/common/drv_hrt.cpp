/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

/**
 * @file drv_hrt.cpp
 *
 * High-resolution timer on top of the Zephyr kernel clock. Callouts are
 * invoked from the k_timer expiry function, in interrupt context.
 */

#include <drivers/drv_hrt.h>
#include <px4_platform_common/time.h>

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>

static sq_queue_t callout_queue;
static struct k_timer hrt_timer;

const uint16_t latency_bucket_count = LATENCY_BUCKET_COUNT;
const uint16_t latency_buckets[LATENCY_BUCKET_COUNT] = { 1, 2, 5, 10, 20, 50, 100, 1000 };
__EXPORT uint32_t latency_counters[LATENCY_BUCKET_COUNT + 1];

static void hrt_call_reschedule()
{
	struct hrt_call *next = (struct hrt_call *)(void *)sq_peek(&callout_queue);

	if (next == nullptr) {
		k_timer_stop(&hrt_timer);
		return;
	}

	k_timer_start(&hrt_timer, K_TIMEOUT_ABS_TICKS(k_us_to_ticks_ceil64(next->deadline)), K_NO_WAIT);
}

static void hrt_call_enter(struct hrt_call *entry)
{
	struct hrt_call *call = (struct hrt_call *)(void *)sq_peek(&callout_queue);

	if ((call == nullptr) || (entry->deadline < call->deadline)) {
		sq_addfirst(&entry->link, &callout_queue);
		hrt_call_reschedule();
		return;
	}

	struct hrt_call *next;

	while ((next = (struct hrt_call *)(void *)sq_next(&call->link)) != nullptr) {
		if (entry->deadline < next->deadline) {
			break;
		}

		call = next;
	}

	sq_addafter(&call->link, &entry->link, &callout_queue);
}

static void hrt_latency_update(hrt_abstime latency)
{
	for (unsigned index = 0; index < LATENCY_BUCKET_COUNT; index++) {
		if (latency <= latency_buckets[index]) {
			latency_counters[index]++;
			return;
		}
	}

	latency_counters[LATENCY_BUCKET_COUNT]++;
}

static void hrt_timer_expiry(struct k_timer *timer)
{
	// interrupts of a higher priority than the system timer can schedule calls too
	unsigned int key = irq_lock();
	struct hrt_call *call;

	while ((call = (struct hrt_call *)(void *)sq_peek(&callout_queue)) != nullptr) {
		const hrt_abstime now = hrt_absolute_time();

		if (call->deadline > now) {
			break;
		}

		sq_rem(&call->link, &callout_queue);
		hrt_latency_update(now - call->deadline);

		const hrt_abstime deadline = call->deadline;

		// zero the deadline, as the call has occurred
		call->deadline = 0;

		const hrt_callout callout = call->callout;
		void *const arg = call->arg;

		// the callout runs with interrupts enabled, as on NuttX
		irq_unlock(key);

		if (callout) {
			callout(arg);
		}

		key = irq_lock();

		// periodic calls are re-entered, unless the callout cancelled them
		if (call->period != 0) {
			// the callout may have rescheduled the call with hrt_call_delay()
			if (call->deadline <= now) {
				call->deadline = deadline + call->period;
			}

			// or queued it again with hrt_call_every(): enter it once
			sq_rem(&call->link, &callout_queue);
			hrt_call_enter(call);
		}
	}

	hrt_call_reschedule();

	irq_unlock(key);
}

static void hrt_call_internal(struct hrt_call *entry, hrt_abstime deadline, hrt_abstime interval,
			      hrt_callout callout, void *arg)
{
	const unsigned int key = irq_lock();

	// the entry may be uninitialised, sq_rem() only dereferences queued entries
	if (entry->deadline != 0) {
		sq_rem(&entry->link, &callout_queue);
	}

	entry->deadline = deadline;
	entry->period = interval;
	entry->callout = callout;
	entry->arg = arg;

	hrt_call_enter(entry);

	irq_unlock(key);
}

hrt_abstime hrt_absolute_time()
{
#if defined(CONFIG_TIMER_HAS_64BIT_CYCLE_COUNTER)
	// the cycles of the system timer, which also counts the kernel ticks, of 10 us by default
	return k_cyc_to_us_floor64(k_cycle_get_64());
#else
	return k_ticks_to_us_floor64(k_uptime_ticks());
#endif
}

void hrt_store_absolute_time(volatile hrt_abstime *time)
{
	*time = hrt_absolute_time();
}

void hrt_init()
{
	sq_init(&callout_queue);
	k_timer_init(&hrt_timer, hrt_timer_expiry, nullptr);
}

void hrt_call_after(struct hrt_call *entry, hrt_abstime delay, hrt_callout callout, void *arg)
{
	hrt_call_internal(entry, hrt_absolute_time() + delay, 0, callout, arg);
}

void hrt_call_every(struct hrt_call *entry, hrt_abstime delay, hrt_abstime interval, hrt_callout callout, void *arg)
{
	hrt_call_internal(entry, hrt_absolute_time() + delay, interval, callout, arg);
}

void hrt_call_at(struct hrt_call *entry, hrt_abstime calltime, hrt_callout callout, void *arg)
{
	hrt_call_internal(entry, calltime, 0, callout, arg);
}

bool hrt_called(struct hrt_call *entry)
{
	// the 64-bit deadline is zeroed in interrupt context, by hrt_timer_expiry()
	const unsigned int key = irq_lock();
	const bool called = (entry->deadline == 0);
	irq_unlock(key);

	return called;
}

void hrt_cancel(struct hrt_call *entry)
{
	const unsigned int key = irq_lock();

	sq_rem(&entry->link, &callout_queue);
	entry->deadline = 0;

	// prevent a periodic call from being re-entered when cancelled from its own callout
	entry->period = 0;

	irq_unlock(key);
}

void hrt_call_init(struct hrt_call *entry)
{
	memset(entry, 0, sizeof(*entry));
}

void hrt_call_delay(struct hrt_call *entry, hrt_abstime delay)
{
	const unsigned int key = irq_lock();
	entry->deadline = hrt_absolute_time() + delay;
	irq_unlock(key);
}

int px4_clock_gettime(clockid_t clk_id, struct timespec *tp)
{
	if (tp == nullptr) {
		errno = EINVAL;
		return -1;
	}

	if (clk_id == CLOCK_MONOTONIC) {
		abstime_to_ts(tp, hrt_absolute_time());
		return 0;
	}

	return system_clock_gettime(clk_id, tp);
}
