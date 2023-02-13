/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * High-resolution timer with callouts and timekeeping.
 */

#include <drivers/drv_hrt.h>

#include <semaphore.h>
#include <time.h>
#include <string.h>
#include <errno.h>

#include <stdint.h>

#include <px4_platform_common/sem.h>
#include <queue.h>

sq_entry_t *sq_remafter(sq_entry_t *node, sq_queue_t *queue)
{
	sq_entry_t *ret = node->flink;

	if (queue->head && ret) {
		if (queue->tail == ret) {
			queue->tail = node;
			node->flink = NULL;

		} else {
			node->flink = ret->flink;
		}

		ret->flink = NULL;
	}

	return ret;
}

void sq_rem(sq_entry_t *node, sq_queue_t *queue)
{
	if (queue->head && node) {
		if (node == queue->head) {
			queue->head = node->flink;

			if (node == queue->tail) {
				queue->tail = NULL;
			}

		} else {
			sq_entry_t *prev;

			for (prev = (sq_entry_t *)queue->head;
			     prev && prev->flink != node;
			     prev = prev->flink) {}

			if (prev) {
				sq_remafter(prev, queue);
			}
		}
	}
}


// Intervals in usec
static constexpr unsigned HRT_INTERVAL_MIN = 50;
static constexpr unsigned HRT_INTERVAL_MAX = 50000000;

/*
 * Queue of callout entries.
 */
sq_entry_t callout_queue{};

/* latency baseline (last compare value applied) */
//static uint64_t latency_baseline;

/* latency histogram */
// const uint16_t latency_bucket_count = LATENCY_BUCKET_COUNT;
// const uint16_t latency_buckets[LATENCY_BUCKET_COUNT] = { 1, 2, 5, 10, 20, 50, 100, 1000 };
// __EXPORT uint32_t latency_counters[LATENCY_BUCKET_COUNT + 1];

static px4_sem_t 	_hrt_lock;
//static struct work_s	_hrt_work;

static void hrt_lock()
{
	px4_sem_wait(&_hrt_lock);
}

static void hrt_unlock()
{
	px4_sem_post(&_hrt_lock);
}

/*
 * Get absolute time.
 */
hrt_abstime hrt_absolute_time()
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts_to_abstime(&ts);
}

/*
 * Store the absolute time in an interrupt-safe fashion.
 *
 * This function ensures that the timestamp cannot be seen half-written by an interrupt handler.
 */
void hrt_store_absolute_time(volatile hrt_abstime *t)
{
	*t = hrt_absolute_time();
}

/*
 * If this returns true, the entry has been invoked and removed from the callout list,
 * or it has never been entered.
 *
 * Always returns false for repeating callouts.
 */
bool hrt_called(struct hrt_call *entry)
{
	return (entry->deadline == 0);
}

/*
 * Remove the entry from the callout list.
 */
void hrt_cancel(struct hrt_call *entry)
{
	hrt_lock();
	//sq_rem(&entry->link, &callout_queue);
	entry->deadline = 0;

	/* if this is a periodic call being removed by the callout, prevent it from
	 * being re-entered when the callout returns.
	 */
	entry->period = 0;
	hrt_unlock();
	// endif
}

/*
 * initialise a hrt_call structure
 */
void hrt_call_init(struct hrt_call *entry)
{
	memset(entry, 0, sizeof(*entry));
}

/*
 * delay a hrt_call_every() periodic call by the given number of
 * microseconds. This should be called from within the callout to
 * cause the callout to be re-scheduled for a later time. The periodic
 * callouts will then continue from that new base time at the
 * previously specified period.
 */
void hrt_call_delay(struct hrt_call *entry, hrt_abstime delay)
{
	entry->deadline = hrt_absolute_time() + delay;
}

/*
 * Initialise the HRT.
 */
void hrt_init()
{
	//sq_init(&callout_queue);

	int sem_ret = px4_sem_init(&_hrt_lock, 0, 1);

	if (sem_ret) {
		//PX4_ERR("SEM INIT FAIL: %s", strerror(errno));
	}
}

int px4_clock_gettime(clockid_t clk_id, struct timespec *tp)
{
	return system_clock_gettime(clk_id, tp);
}
