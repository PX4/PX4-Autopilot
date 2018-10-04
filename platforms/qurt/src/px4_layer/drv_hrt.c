/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
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
 * @file drv_hrt.c
 *
 * High-resolution timer with callouts and timekeeping.
 */

#include <px4_defines.h>
#include <px4_workqueue.h>
#include <drivers/drv_hrt.h>
#include <semaphore.h>
#include <time.h>
#include <sys/time.h>
#include <dspal_time.h>
#include <string.h>
#include <stdio.h>

static struct sq_queue_s	callout_queue;

/* latency histogram */
#define LATENCY_BUCKET_COUNT 8
__EXPORT const uint16_t latency_bucket_count = LATENCY_BUCKET_COUNT;
__EXPORT const uint16_t	latency_buckets[LATENCY_BUCKET_COUNT] = { 1, 2, 5, 10, 20, 50, 100, 1000 };
__EXPORT uint32_t	latency_counters[LATENCY_BUCKET_COUNT + 1];

static void		hrt_call_reschedule(void);

// Intervals in ms
#define HRT_INTERVAL_MIN	50
#define HRT_INTERVAL_MAX	50000

static sem_t 	_hrt_lock;
static struct work_s	_hrt_work;

#define CLOCK_REALTIME 0

#ifndef CLOCK_MONOTONIC
#define CLOCK_MONOTONIC 1
#endif

int px4_clock_gettime(clockid_t clk_id, struct timespec *tp)
{
	struct timeval now;
	int rv = gettimeofday(&now, NULL);

	if (rv) {
		return rv;
	}

	tp->tv_sec = now.tv_sec;
	tp->tv_nsec = now.tv_usec * 1000;

	return 0;
}

int px4_clock_settime(clockid_t clk_id, struct timespec *tp)
{
	/* do nothing right now */
	return 0;
}

static void hrt_call_invoke(void);

static void hrt_lock(void)
{
	//printf("hrt_lock\n");
	sem_wait(&_hrt_lock);
}

static void hrt_unlock(void)
{
	//printf("hrt_unlock\n");
	sem_post(&_hrt_lock);
}

/*
 * Get absolute time.
 */
hrt_abstime hrt_absolute_time(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts_to_abstime(&ts);
}

/*
 * Convert a timespec to absolute time.
 */
hrt_abstime ts_to_abstime(struct timespec *ts)
{
	hrt_abstime	result;

	result = (hrt_abstime)(ts->tv_sec) * 1000000;
	result += ts->tv_nsec / 1000;

	return result;
}

/*
 * Compute the delta between a timestamp taken in the past
 * and now.
 *
 * This function is safe to use even if the timestamp is updated
 * by an interrupt during execution.
 */
hrt_abstime hrt_elapsed_time(const volatile hrt_abstime *then)
{
	hrt_abstime delta = hrt_absolute_time() - *then;
	return delta;
}

/*
 * Store the absolute time in an interrupt-safe fashion.
 *
 * This function ensures that the timestamp cannot be seen half-written by an interrupt handler.
 */
hrt_abstime hrt_store_absolute_time(volatile hrt_abstime *now)
{
	hrt_abstime ts = hrt_absolute_time();
	return ts;
}


/*
 * If this returns true, the entry has been invoked and removed from the callout list,
 * or it has never been entered.
 *
 * Always returns false for repeating callouts.
 */
bool	hrt_called(struct hrt_call *entry)
{
	return (entry->deadline == 0);
}

/*
 * Remove the entry from the callout list.
 */
void	hrt_cancel(struct hrt_call *entry)
{
	hrt_lock();
	sq_rem(&entry->link, &callout_queue);
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
void	hrt_call_init(struct hrt_call *entry)
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
void	hrt_call_delay(struct hrt_call *entry, hrt_abstime delay)
{
	entry->deadline = hrt_absolute_time() + delay;
}

/*
 * Initialise the HRT.
 */
void	hrt_init(void)
{
	//printf("hrt_init\n");
	sq_init(&callout_queue);
	sem_init(&_hrt_lock, 0, 1);
	memset(&_hrt_work, 0, sizeof(_hrt_work));
}

static void
hrt_call_enter(struct hrt_call *entry)
{
	struct hrt_call	*call, *next;

	//printf("hrt_call_enter\n");
	call = (struct hrt_call *)sq_peek(&callout_queue);

	if ((call == NULL) || (entry->deadline < call->deadline)) {
		sq_addfirst(&entry->link, &callout_queue);
		//lldbg("call enter at head, reschedule\n");
		/* we changed the next deadline, reschedule the timer event */
		hrt_call_reschedule();

	} else {
		do {
			next = (struct hrt_call *)sq_next(&call->link);

			if ((next == NULL) || (entry->deadline < next->deadline)) {
				//lldbg("call enter after head\n");
				sq_addafter(&call->link, &entry->link, &callout_queue);
				break;
			}
		} while ((call = next) != NULL);
	}

	//lldbg("scheduled\n");
}

/**
 * Timer interrupt handler
 *
 * This routine simulates a timer interrupt handler
 */
static void
hrt_tim_isr(void *p)
{

	//printf("hrt_tim_isr\n");
	/* run any callouts that have met their deadline */
	hrt_call_invoke();

	hrt_lock();

	/* and schedule the next interrupt */
	hrt_call_reschedule();

	hrt_unlock();
}

/**
 * Reschedule the next timer interrupt.
 *
 * This routine must be called with interrupts disabled.
 */
static void
hrt_call_reschedule()
{
	hrt_abstime	now = hrt_absolute_time();
	struct hrt_call	*next = (struct hrt_call *)sq_peek(&callout_queue);
	hrt_abstime	deadline = now + HRT_INTERVAL_MAX;
	uint32_t	ticks = USEC2TICK(HRT_INTERVAL_MAX);

	//printf("hrt_call_reschedule\n");

	/*
	 * Determine what the next deadline will be.
	 *
	 * Note that we ensure that this will be within the counter
	 * period, so that when we truncate all but the low 16 bits
	 * the next time the compare matches it will be the deadline
	 * we want.
	 *
	 * It is important for accurate timekeeping that the compare
	 * interrupt fires sufficiently often that the base_time update in
	 * hrt_absolute_time runs at least once per timer period.
	 */
	if (next != NULL) {
		//lldbg("entry in queue\n");
		if (next->deadline <= (now + HRT_INTERVAL_MIN)) {
			//lldbg("pre-expired\n");
			/* set a minimal deadline so that we call ASAP */
			ticks = USEC2TICK(HRT_INTERVAL_MIN);

		} else if (next->deadline < deadline) {
			//lldbg("due soon\n");
			ticks = USEC2TICK((next->deadline - now));
		}
	}

	// There is no timer ISR, so simulate one by putting an event on the
	// high priority work queue
	//printf("ticks = %u\n", ticks);
	work_queue(HPWORK, &_hrt_work, (worker_t)&hrt_tim_isr, NULL, ticks);
}

static void
hrt_call_internal(struct hrt_call *entry, hrt_abstime deadline, hrt_abstime interval, hrt_callout callout, void *arg)
{
	//printf("hrt_call_internal\n");
	hrt_lock();

	//printf("hrt_call_internal after lock\n");
	/* if the entry is currently queued, remove it */
	/* note that we are using a potentially uninitialised
	   entry->link here, but it is safe as sq_rem() doesn't
	   dereference the passed node unless it is found in the
	   list. So we potentially waste a bit of time searching the
	   queue for the uninitialised entry->link but we don't do
	   anything actually unsafe.
	*/
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

/*
 * Call callout(arg) after delay has elapsed.
 *
 * If callout is NULL, this can be used to implement a timeout by testing the call
 * with hrt_called().
 */
void	hrt_call_after(struct hrt_call *entry, hrt_abstime delay, hrt_callout callout, void *arg)
{
	//printf("hrt_call_after\n");
	hrt_call_internal(entry,
			  hrt_absolute_time() + delay,
			  0,
			  callout,
			  arg);
}

/*
 * Call callout(arg) after delay, and then after every interval.
 *
 * Note thet the interval is timed between scheduled, not actual, call times, so the call rate may
 * jitter but should not drift.
 */
void	hrt_call_every(struct hrt_call *entry, hrt_abstime delay, hrt_abstime interval, hrt_callout callout, void *arg)
{
	hrt_call_internal(entry,
			  hrt_absolute_time() + delay,
			  interval,
			  callout,
			  arg);
}

/*
 * Call callout(arg) at absolute time calltime.
 */
void	hrt_call_at(struct hrt_call *entry, hrt_abstime calltime, hrt_callout callout, void *arg)
{
	hrt_call_internal(entry, calltime, 0, callout, arg);
}


static void
hrt_call_invoke(void)
{
	struct hrt_call	*call;
	hrt_abstime deadline;

	hrt_lock();

	while (true) {
		/* get the current time */
		hrt_abstime now = hrt_absolute_time();

		call = (struct hrt_call *)sq_peek(&callout_queue);

		if (call == NULL) {
			break;
		}

		if (call->deadline > now) {
			break;
		}

		sq_rem(&call->link, &callout_queue);
		//lldbg("call pop\n");

		/* save the intended deadline for periodic calls */
		deadline = call->deadline;

		/* zero the deadline, as the call has occurred */
		call->deadline = 0;

		/* invoke the callout (if there is one) */
		if (call->callout) {
			// Unlock so we don't deadlock in callback
			hrt_unlock();

			//lldbg("call %p: %p(%p)\n", call, call->callout, call->arg);
			call->callout(call->arg);

			hrt_lock();
		}

		/* if the callout has a non-zero period, it has to be re-entered */
		if (call->period != 0) {
			// re-check call->deadline to allow for
			// callouts to re-schedule themselves
			// using hrt_call_delay()
			if (call->deadline <= now) {
				call->deadline = deadline + call->period;
			}

			hrt_call_enter(call);
		}
	}

	hrt_unlock();
}

