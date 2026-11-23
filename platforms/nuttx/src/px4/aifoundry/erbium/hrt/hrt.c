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
 * @file hrt.c
 *
 * High-resolution timer for the AIFoundry Erbium Minion.
 *
 * hrt_absolute_time() reads the machine timer (mtime) directly. Callouts
 * run in a high-priority task instead of an interrupt handler: the NuttX
 * RISC-V trap path does not save the floating-point registers, so callouts,
 * which may use floating point, must not run in interrupt context. The
 * task wakes at least once per millisecond, which bounds callout latency.
 */

#include <px4_platform_common/px4_config.h>
#include <drivers/drv_hrt.h>

#include <nuttx/arch.h>
#include <nuttx/queue.h>

#include <sched.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

#include "hardware/erbium_memorymap.h"

#if (CONFIG_ERBIUM_MTIMER_FREQ % 1000000) != 0
#  error "CONFIG_ERBIUM_MTIMER_FREQ must be a multiple of 1 MHz"
#endif

#define HRT_MTIME_TICKS_PER_US   (CONFIG_ERBIUM_MTIMER_FREQ / 1000000)
#define HRT_TASK_STACKSIZE       1536
#define HRT_MAX_SLEEP_US         1000

static struct sq_queue_s callout_queue;
static bool hrt_initialized;

const uint16_t latency_bucket_count = LATENCY_BUCKET_COUNT;
const uint16_t latency_buckets[LATENCY_BUCKET_COUNT] = {1, 2, 5, 10, 20, 50, 100, 1000};
__EXPORT uint32_t latency_counters[LATENCY_BUCKET_COUNT + 1];

static void hrt_call_enter(struct hrt_call *entry);
static void hrt_call_invoke(void);

hrt_abstime hrt_absolute_time(void)
{
	return *(volatile uint64_t *)ERBIUM_MTIME_BASE / HRT_MTIME_TICKS_PER_US;
}

void hrt_store_absolute_time(volatile hrt_abstime *time)
{
	irqstate_t flags = px4_enter_critical_section();
	*time = hrt_absolute_time();
	px4_leave_critical_section(flags);
}

static int hrt_task(int argc, char *argv[])
{
	for (;;) {
		hrt_call_invoke();

		irqstate_t flags = px4_enter_critical_section();
		struct hrt_call *next = (struct hrt_call *)sq_peek(&callout_queue);
		hrt_abstime now = hrt_absolute_time();
		hrt_abstime delay = HRT_MAX_SLEEP_US;

		if (next != NULL && next->deadline > now && next->deadline - now < delay) {
			delay = next->deadline - now;

		} else if (next != NULL && next->deadline <= now) {
			delay = 0;
		}

		px4_leave_critical_section(flags);

		if (delay > 0) {
			usleep(delay);
		}
	}

	return 0;
}

void hrt_init(void)
{
	if (hrt_initialized) {
		return;
	}

	sq_init(&callout_queue);
	hrt_initialized = true;

	if (task_create("hrt", SCHED_PRIORITY_MAX, HRT_TASK_STACKSIZE, hrt_task, NULL) < 0) {
		PANIC();
	}
}

void hrt_call_after(struct hrt_call *entry, hrt_abstime delay, hrt_callout callout, void *arg)
{
	hrt_call_every(entry, delay, 0, callout, arg);
}

void hrt_call_at(struct hrt_call *entry, hrt_abstime calltime, hrt_callout callout, void *arg)
{
	irqstate_t flags = px4_enter_critical_section();

	sq_rem(&entry->link, &callout_queue);
	entry->deadline = calltime;
	entry->period = 0;
	entry->callout = callout;
	entry->arg = arg;
	hrt_call_enter(entry);

	px4_leave_critical_section(flags);
}

void hrt_call_every(struct hrt_call *entry, hrt_abstime delay, hrt_abstime interval,
		    hrt_callout callout, void *arg)
{
	irqstate_t flags = px4_enter_critical_section();

	sq_rem(&entry->link, &callout_queue);
	entry->deadline = hrt_absolute_time() + delay;
	entry->period = interval;
	entry->callout = callout;
	entry->arg = arg;
	hrt_call_enter(entry);

	px4_leave_critical_section(flags);
}

bool hrt_called(struct hrt_call *entry)
{
	return entry->deadline == 0;
}

void hrt_cancel(struct hrt_call *entry)
{
	irqstate_t flags = px4_enter_critical_section();

	sq_rem(&entry->link, &callout_queue);
	entry->deadline = 0;
	entry->period = 0;

	px4_leave_critical_section(flags);
}

void hrt_call_init(struct hrt_call *entry)
{
	memset(entry, 0, sizeof(*entry));
}

void hrt_call_delay(struct hrt_call *entry, hrt_abstime delay)
{
	entry->deadline = hrt_absolute_time() + delay;
}

/* Insert an entry in deadline order. Called with the critical section held. */

static void hrt_call_enter(struct hrt_call *entry)
{
	struct hrt_call *call = (struct hrt_call *)sq_peek(&callout_queue);

	if (call == NULL || entry->deadline < call->deadline) {
		sq_addfirst(&entry->link, &callout_queue);
		return;
	}

	struct hrt_call *next;

	while ((next = (struct hrt_call *)sq_next(&call->link)) != NULL &&
	       next->deadline <= entry->deadline) {
		call = next;
	}

	sq_addafter(&call->link, &entry->link, &callout_queue);
}

static void hrt_call_invoke(void)
{
	for (;;) {
		irqstate_t flags = px4_enter_critical_section();
		struct hrt_call *call = (struct hrt_call *)sq_peek(&callout_queue);
		hrt_abstime now = hrt_absolute_time();

		if (call == NULL || call->deadline > now) {
			px4_leave_critical_section(flags);
			return;
		}

		sq_rem(&call->link, &callout_queue);
		hrt_abstime deadline = call->deadline;
		call->deadline = 0;
		hrt_callout callout = call->callout;
		void *arg = call->arg;

		px4_leave_critical_section(flags);

		if (callout != NULL) {
			callout(arg);
		}

		flags = px4_enter_critical_section();

		/* Re-arm a periodic call unless the callout rescheduled it */

		if (call->period != 0 && call->deadline == 0) {
			call->deadline = deadline + call->period;

			if (call->deadline <= now) {
				call->deadline = now + call->period;
			}

			hrt_call_enter(call);
		}

		px4_leave_critical_section(flags);
	}
}
