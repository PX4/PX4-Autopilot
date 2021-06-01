/****************************************************************************
 *
 *   Copyright (c) 2021 Technology Innovation Institute. All rights reserved.
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
 * Author: Jani Paalijärvi
 *
 * High-resolution timer callouts and timekeeping.
 * The implementation is based on 32-bit decrementing counter (MSTIMER)
 * which is used in periodic mode and generates an interrupt
 * when timer reaches zero.
 *
 */

#include <px4_platform_common/px4_config.h>
#include <systemlib/px4_macros.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include <stdbool.h>

#include <assert.h>
#include <debug.h>
#include <time.h>
#include <queue.h>
#include <errno.h>
#include <string.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>
#include <nuttx/board.h>

#include "mpfs_memorymap.h"
#include "hardware/mpfs_timer.h"

#define getreg32(a)          (*(volatile uint32_t *)(a))
#define putreg32(v,a)        (*(volatile uint32_t *)(a) = (v))

#ifdef CONFIG_DEBUG_HRT
#  define hrtinfo _info
#else
#  define hrtinfo(x...)
#endif

#define CLOCK_RATE_MHZ          (MPFS_MSS_APB_AHB_CLK / 1000000)

#define HRT_INTERVAL_MIN	50UL // in microseconds
#define HRT_INTERVAL_MAX	28633115UL // in microsecond

#define HRT_COUNTER_MIN	        (HRT_INTERVAL_MIN * CLOCK_RATE_MHZ)
#define HRT_COUNTER_MAX         (HRT_INTERVAL_MAX * CLOCK_RATE_MHZ)

/*
 * Scaling factor(s) for the free-running counter; convert an input
 * in counts to a time in microseconds.
 */
#define HRT_COUNTER_SCALE(_c)	(_c / CLOCK_RATE_MHZ)

#define HRT_TIME_TO_COUNTS(_a)  (_a * CLOCK_RATE_MHZ)

/*
 * Queue of callout entries.
 */
static struct sq_queue_s	callout_queue;

/* latency baseline (last compare value applied) */
static uint32_t			latency_baseline;

/* timer count at interrupt (for latency purposes) */
static uint32_t			latency_actual;

/* latency histogram */
const uint16_t latency_bucket_count = LATENCY_BUCKET_COUNT;
const uint16_t latency_buckets[LATENCY_BUCKET_COUNT] = { 1, 2, 5, 10, 20, 50, 100, 1000 };
__EXPORT uint32_t latency_counters[LATENCY_BUCKET_COUNT + 1];

/* timer-specific functions */
static void hrt_tim_init(void);
static int  hrt_tim_isr(int irq, void *context, void *args);
static void hrt_latency_update(void);

/* callout list manipulation */
static void hrt_call_internal(struct hrt_call *entry, hrt_abstime deadline, hrt_abstime interval, hrt_callout callout,
			      void *arg);
static void hrt_call_enter(struct hrt_call *entry);
static void hrt_call_reschedule(void);
static void hrt_call_invoke(void);
static inline hrt_abstime hrt_calc_base_time(uint32_t newloadval);
static void hrt_set_new_deadline(uint32_t deadline);

/**
 * Initialize the timer we are going to use.
 */
static void
hrt_tim_init(void)
{
	/* e.g. connect hrt_tim_isr to a timer vector, initialize the timer */

	/* attach irq */
	int ret;
	ret = irq_attach(MPFS_IRQ_TIMER1, hrt_tim_isr, NULL);

	if (ret == OK) {

		/* Assumes that the clock for timer is enabled and not in reset */

		/* set an initial timeout */
		putreg32(0x0fff, MPFS_MSTIMER_LO_BASE + MPFS_MSTIMER_TIM1LOADVAL_OFFSET);

		/* enable interrupt for timer, set periodic mode and enable timer */
		putreg32((MPFS_MSTIMER_INTEN_MASK | MPFS_MSTIMER_ENABLE_MASK) & ~(MPFS_MSTIMER_MODE_MASK),
			 MPFS_MSTIMER_LO_BASE + MPFS_MSTIMER_TIM1CONTROL_OFFSET);

		/* enable interrupts */
		up_enable_irq(MPFS_IRQ_TIMER1);

	}
}


/**
 * Handle the compare interrupt by calling the callout dispatcher
 * and then re-scheduling the next deadline.
 */
static int
hrt_tim_isr(int irq, void *context, void *arg)
{
	uint32_t status;

	status = getreg32(MPFS_MSTIMER_LO_BASE + MPFS_MSTIMER_TIM1RIS_OFFSET);

	/* was this a timer tick? */
	if (status & MPFS_MSTIMER_RIS_MASK) {

		/* do latency calculations */
		hrt_latency_update();

		/* run any callouts that have met their deadline */
		hrt_call_invoke();

		/* and schedule the next interrupt */
		hrt_call_reschedule();

		/* clear the interrupt */
		putreg32((MPFS_MSTIMER_RIS_MASK), MPFS_MSTIMER_LO_BASE + MPFS_MSTIMER_TIM1RIS_OFFSET);
	}

	return OK;
}


/**
 * Fetch a never-wrapping absolute time value in microseconds from
 * some arbitrary epoch shortly after system start.
 */
hrt_abstime
hrt_absolute_time(void)
{
	hrt_abstime	abstime;
	hrt_abstime	base_time;

	base_time = hrt_calc_base_time(0);
	abstime = HRT_COUNTER_SCALE(base_time);

	return abstime;
}

/**
 * Convert a timespec to absolute time
 */
hrt_abstime
ts_to_abstime(const struct timespec *ts)
{
	hrt_abstime	result;

	result = (hrt_abstime)(ts->tv_sec) * 1000000;
	result += ts->tv_nsec / 1000;

	return result;
}

/**
 * Convert absolute time to a timespec.
 */
void
abstime_to_ts(struct timespec *ts, hrt_abstime abstime)
{
	ts->tv_sec = abstime / 1000000;
	abstime -= ts->tv_sec * 1000000;
	ts->tv_nsec = abstime * 1000;
}

/**
 * Compare a time value with the current time as atomic operation
 */
hrt_abstime
hrt_elapsed_time_atomic(const volatile hrt_abstime *then)
{
	irqstate_t flags = px4_enter_critical_section();

	hrt_abstime delta = hrt_absolute_time() - *then;

	px4_leave_critical_section(flags);

	return delta;
}

/**
 * Store the absolute time in an interrupt-safe fashion
 */
void
hrt_store_absolute_time(volatile hrt_abstime *t)
{
	irqstate_t flags = px4_enter_critical_section();
	*t = hrt_absolute_time();
	px4_leave_critical_section(flags);
}


static inline hrt_abstime hrt_calc_base_time(uint32_t newloadval)
{
	uint32_t	count;
	uint32_t	loadval;
	static volatile hrt_abstime base_time;
	static volatile uint32_t last_count;
	irqstate_t	flags;

	/* prevent re-entry */
	flags = px4_enter_critical_section();

	/* get the current counter value */
	count = getreg32(MPFS_MSTIMER_LO_BASE + MPFS_MSTIMER_TIM1VALUE_OFFSET);

	/* get the previous loaded value */
	loadval = getreg32(MPFS_MSTIMER_LO_BASE + MPFS_MSTIMER_TIM1LOADVAL_OFFSET);


	/* Determine whether the counter has wrapped since the
	 * last time we're called.
	 *
	 * This simple test is sufficient due to the guarantee that
	 * we are always called at least once per counter period.
	 */
	if (count > last_count) {
		base_time += loadval;
	}

	/* set new last counter val if needed */
	if (newloadval != 0) {
		last_count = newloadval;
		/* store current counter value into base_time */
		base_time += (loadval - count);

	} else {
		/* save the count for next time */
		last_count = count;
	}

	px4_leave_critical_section(flags);

	return base_time;
}

static void hrt_set_new_deadline(uint32_t deadline)
{
	/* calculate base time at this point because counter value will be changed */
	hrt_calc_base_time(deadline);

	putreg32(deadline, MPFS_MSTIMER_LO_BASE + MPFS_MSTIMER_TIM1LOADVAL_OFFSET);
}

/**
 * Initialise the high-resolution timing module.
 */
void
hrt_init(void)
{
	sq_init(&callout_queue);
	hrt_tim_init();
}

/**
 * Call callout(arg) after interval has elapsed.
 */
void
hrt_call_after(struct hrt_call *entry, hrt_abstime delay, hrt_callout callout, void *arg)
{
	hrt_call_internal(entry,
			  hrt_absolute_time() + delay,
			  0,
			  callout,
			  arg);
}

/**
 * Call callout(arg) at calltime.
 */
void
hrt_call_at(struct hrt_call *entry, hrt_abstime calltime, hrt_callout callout, void *arg)
{
	hrt_call_internal(entry, calltime, 0, callout, arg);
}

/**
 * Call callout(arg) every period.
 */
void
hrt_call_every(struct hrt_call *entry, hrt_abstime delay, hrt_abstime interval, hrt_callout callout, void *arg)
{
	hrt_call_internal(entry,
			  hrt_absolute_time() + delay,
			  interval,
			  callout,
			  arg);
}

static void
hrt_call_internal(struct hrt_call *entry, hrt_abstime deadline, hrt_abstime interval, hrt_callout callout, void *arg)
{
	irqstate_t flags = px4_enter_critical_section();

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

	px4_leave_critical_section(flags);
}

/**
 * If this returns true, the call has been invoked and removed from the callout list.
 *
 * Always returns false for repeating callouts.
 */
bool
hrt_called(struct hrt_call *entry)
{
	return (entry->deadline == 0);
}

/**
 * Remove the entry from the callout list.
 */
void
hrt_cancel(struct hrt_call *entry)
{
	irqstate_t flags = px4_enter_critical_section();

	sq_rem(&entry->link, &callout_queue);
	entry->deadline = 0;

	/* if this is a periodic call being removed by the callout, prevent it from
	 * being re-entered when the callout returns.
	 */
	entry->period = 0;

	px4_leave_critical_section(flags);
}

static void
hrt_call_enter(struct hrt_call *entry)
{
	struct hrt_call	*call, *next;

	call = (struct hrt_call *)sq_peek(&callout_queue);

	if ((call == NULL) || (entry->deadline < call->deadline)) {
		sq_addfirst(&entry->link, &callout_queue);
		hrtinfo("call enter at head, reschedule\n");
		/* we changed the next deadline, reschedule the timer event */
		hrt_call_reschedule();

	} else {
		do {
			next = (struct hrt_call *)sq_next(&call->link);

			if ((next == NULL) || (entry->deadline < next->deadline)) {
				hrtinfo("call enter after head\n");
				sq_addafter(&call->link, &entry->link, &callout_queue);
				break;
			}
		} while ((call = next) != NULL);
	}

	hrtinfo("scheduled\n");
}

static void
hrt_call_invoke(void)
{
	struct hrt_call	*call;
	hrt_abstime deadline;

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
		hrtinfo("call pop\n");

		/* save the intended deadline for periodic calls */
		deadline = call->deadline;

		/* zero the deadline, as the call has occurred */
		call->deadline = 0;

		/* invoke the callout (if there is one) */
		if (call->callout) {
			hrtinfo("call %p: %p(%p)\n", call, call->callout, call->arg);
			call->callout(call->arg);
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
	hrt_abstime	absdeadline = now + HRT_INTERVAL_MAX;
	uint32_t	deadline = HRT_COUNTER_MAX;

	/*
	 * Determine what the next deadline will be.
	 *
	 * It is important for accurate timekeeping that
	 * the interrupt fires sufficiently often that the base_time update in
	 * hrt_absolute_time runs at least once per timer period.
	 */
	if (next != NULL) {
		hrtinfo("entry in queue\n");

		if (next->deadline <= (now + HRT_INTERVAL_MIN)) {
			hrtinfo("pre-expired\n");
			/* set a minimal deadline so that we call ASAP */
			deadline = HRT_COUNTER_MIN;

		} else if (next->deadline < absdeadline) {
			hrtinfo("due soon\n");
			/* calculate how much time we have to the next deadline */
			uint32_t deadline_us = (next->deadline - now);
			/* convert to the counter counts */
			deadline = HRT_TIME_TO_COUNTS(deadline_us);
		}
	}


	hrtinfo("schedule for %"PRIu64" at %"PRIu64"\n", next->deadline, now);

	/* set the new compare value and remember it for latency tracking */
	latency_baseline = deadline;

	/* set next deadline */
	hrt_set_new_deadline(deadline);
}

static void
hrt_latency_update(void)
{
	uint16_t latency = latency_actual - latency_baseline;
	unsigned	index;

	/* bounded buckets */
	for (index = 0; index < LATENCY_BUCKET_COUNT; index++) {
		if (latency <= latency_buckets[index]) {
			latency_counters[index]++;
			return;
		}
	}

	/* catch-all at the end */
	latency_counters[index]++;
}

void
hrt_call_init(struct hrt_call *entry)
{
	memset(entry, 0, sizeof(*entry));
}

void
hrt_call_delay(struct hrt_call *entry, hrt_abstime delay)
{
	entry->deadline = hrt_absolute_time() + delay;
}
