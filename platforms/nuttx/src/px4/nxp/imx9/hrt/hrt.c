/****************************************************************************
 *
 *   Copyright (c) 2018-2024 PX4 Development Team. All rights reserved.
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
 * Author: David Sidrane <david_s5@nscdg.com>
 *
 * High-resolution timer callouts and timekeeping.
 *
 * Uses one LPTMR as the free-running 1 MHz counter and one LPIT channel as the deadline interrupt.
 *
 * Note that really, this could use systick too, but that's
 * monopolised by NuttX and stealing it would just be awkward.
 *
 */

#include <px4_platform_common/px4_config.h>
#include <systemlib/px4_macros.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <arm_internal.h>
#include <hardware/imx9_lptmr.h>
#include <hardware/imx9_lpit.h>
#include <imx9_clockconfig.h>

#include <sys/types.h>
#include <stdbool.h>

#include <assert.h>
#include <debug.h>
#include <time.h>
#include <nuttx/queue.h>
#include <errno.h>
#include <string.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include "chip.h"

#ifdef HRT_PPM_CHANNEL
# error HRT_PPM_CHANNEL not supported
#endif

#ifdef CONFIG_DEBUG_HRT
#  define hrtinfo _info
#else
#  define hrtinfo(x...)
#endif

#define hrtwarn _warn

#define HRT_TIMER_FREQ   1000000

/* lpit value has to be multiplied by 133 because bus_wakeup runs 133MHz
 * ponytail: assumes the LPIT runs from the 133.33 MHz bus_wakeup root; a different LPIT clock
 * root silently skews deadlines. Upgrade path: read the root with imx9_get_rootclock at init.
 */
#define LPIT_VALUE_FACTOR(a) ((a*400) / 3)  /* == a * 133.333333... */

/* HRT configuration */
#define HRT_LPTMR_INSTANCE 2
#define HRT_LPIT_INSTANCE 2

#if HRT_LPTMR_INSTANCE==1
#  define HRT_LPTMR	 IMX9_LPTMR1_BASE
#  define HRT_CLK_LPTMR	 LPTMR1_CLK
#elif HRT_LPTMR_INSTANCE==2
#  define HRT_LPTMR	 IMX9_LPTMR2_BASE
#  define HRT_CLK_LPTMR	 LPTMR2_CLK
#else
#  error Assign HRT_LPTMR_INSTANCE with valid LPTMR instance to use for HRT
#endif

#if HRT_LPIT_INSTANCE==1
#  define HRT_LPIT	 IMX9_LPIT1_BASE
#  define HRT_IRQ_LPIT	 IMX9_IRQ_LPIT1
#elif HRT_LPIT_INSTANCE==2
#  define HRT_LPIT	 IMX9_LPIT2_BASE
#  define HRT_IRQ_LPIT	 IMX9_IRQ_LPIT2
#else
#  error Assign HRT_LPIT_INSTANCE with valid LPIT instance to use for HRT
#endif


/**
* Minimum/maximum deadlines.
*
* These are suitable for use with a 32-bit timer/counter clocked
* at 1MHz.  The high-resolution timer need only guarantee that it
* not wrap more than once in the 4294.967296s period for absolute
* time to be consistently maintained.
*
* The minimum deadline must be such that the time taken between
* reading a time and writing a deadline to the timer cannot
* result in missing the deadline.
*/
#define HRT_INTERVAL_MIN	50
#define HRT_INTERVAL_MAX	10000000LL

/*
* Period of the free-running counter, in microseconds.
*/
#define HRT_COUNTER_PERIOD	4294967296LL

/*
* Scaling factor(s) for the free-running counter; convert an input
* in counts to a time in microseconds.
*/
#define HRT_COUNTER_SCALE(_c)	(_c)

/*
 * Queue of callout entries.
 */
static struct sq_queue_s  callout_queue;

/* latency baseline (last compare value applied) */
static uint32_t           latency_baseline;

/* timer count at interrupt (for latency purposes) */
static uint32_t           latency_actual;

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

/**
 * Initialize the timer we are going to use.
 */
static void hrt_tim_init(void)
{
	uint32_t reg;
	int ret;
	uint32_t freq = 0;

	/* Configure the LPTMR clock for HRT */
	ret = imx9_configure_clock(HRT_CLK_LPTMR, true);

	if (ret != 0) {
		hrtwarn("Setup HRT LPTMR clock: imx9_configure_clock() returned %d", ret);
	}

	if (imx9_get_rootclock(GET_CLOCK_ROOT(HRT_CLK_LPTMR), &freq) != 0 || freq != HRT_TIMER_FREQ) {
		hrtwarn("HRT LPTMR clock frequency != 1MHz\n");
	}

	/* LPTMR */

	/* Timer source 0, Time counter mode, Free-running Reset on overflow */
	putreg32(LPTMR_CSR_TPS0 | LPTMR_CSR_TFC, LPTMR_CSR(HRT_LPTMR));

	/* Bypass prescaler/glitch filter */
	putreg32(LPTMR_PSR_PBYP | LPTMR_PSR_PCS_REF_INT, LPTMR_PSR(HRT_LPTMR));

	/* For set compare to 1000 */
	putreg32(1000, LPTMR_CMR(HRT_LPTMR));

	reg = getreg32(LPTMR_CSR(HRT_LPTMR));
	reg |= (LPTMR_CSR_TEN);

	/* Start timer */
	putreg32(reg, LPTMR_CSR(HRT_LPTMR));

	/* LPIT */

	/* claim our interrupt vector */
	irq_attach(HRT_IRQ_LPIT, hrt_tim_isr, NULL);

	/* Reset the LPIT */
	putreg32(LPIT_MCR_SW_RST, LPIT_MCR(HRT_LPIT));

	up_udelay(10);

	/* enable the peripheral clock */
	putreg32(LPIT_MCR_M_CEN, LPIT_MCR(HRT_LPIT));

	/* wait 4 peripheral clock cycles
	   enable interrupts in the meantime */
	up_enable_irq(HRT_IRQ_LPIT);

	/* set the timer timeout value */
	putreg32(LPIT_VALUE_FACTOR(1000), LPIT_TVAL0(HRT_LPIT));

	/* configure MIER[TIEn] */
	putreg32(LPIT_MIER_TIE0, LPIT_MIER(HRT_LPIT));

	/* configure the low-power modes of the module */
	putreg32(LPIT_MCR_M_CEN | LPIT_MCR_DBG_EN | LPIT_MCR_DOZE_EN, LPIT_MCR(HRT_LPIT));

	/* enable the channel timers */
	putreg32(LPIT_TCTRL_TSOI | LPIT_TCTRL_T_EN, LPIT_TCTRL0(HRT_LPIT));
}


/**
 * Handle the compare interrupt by calling the callout dispatcher
 * and then re-scheduling the next deadline.
 */
static int
hrt_tim_isr(int irq, void *context, void *arg)
{

	/* Must first write any value to the CNR. This synchronizes and registers the current value of the CNR */
	putreg32(0x0, LPTMR_CNR(HRT_LPTMR));

	/* grab the timer for latency tracking purposes */
	latency_actual = getreg32(LPTMR_CNR(HRT_LPTMR));

	bool status = (getreg32(LPIT_MSR(HRT_LPIT)) & LPIT_MSR_TIF0) == LPIT_MSR_TIF0;

	/* was this a timer tick? */
	if (status) {
		/* clear flag */
		putreg32(LPIT_MSR_TIF0, LPIT_MSR(HRT_LPIT));

		/* do latency calculations */
		hrt_latency_update();

		/* run any callouts that have met their deadline */
		hrt_call_invoke();

		/* and schedule the next interrupt */
		hrt_call_reschedule();
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
	uint32_t	count;
	irqstate_t	flags;

	/*
	 * Counter state.  Marked volatile as they may change
	 * inside this routine but outside the irqsave/restore
	 * pair.  Discourage the compiler from moving loads/stores
	 * to these outside of the protected range.
	 */
	static volatile hrt_abstime base_time;
	static volatile uint32_t last_count;

	/* prevent re-entry */
	flags = px4_enter_critical_section();

	/* Must first write any value to the CNR. This synchronizes and registers the current value of the CNR */
	putreg32(0x0, LPTMR_CNR(HRT_LPTMR));

	/* get the current counter value */
	count = getreg32(LPTMR_CNR(HRT_LPTMR));

	/*
	 * Determine whether the counter has wrapped since the
	 * last time we're called.
	 *
	 * This simple test is sufficient due to the guarantee that
	 * we are always called at least once per counter period.
	 */
	if (count < last_count) {
		base_time += HRT_COUNTER_PERIOD;
	}

	/* save the count for next time */
	last_count = count;

	/* compute the current time */
	abstime = HRT_COUNTER_SCALE(base_time + count);

	px4_leave_critical_section(flags);

	return abstime;
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

/**
 * Initialize the high-resolution timing module.
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
	/* note that we are using a potentially uninitialized
	   entry->link here, but it is safe as sq_rem() doesn't
	   dereference the passed node unless it is found in the
	   list. So we potentially waste a bit of time searching the
	   queue for the uninitialized entry->link but we don't do
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

	call = (struct hrt_call *)(void *)sq_peek(&callout_queue);

	if ((call == NULL) || (entry->deadline < call->deadline)) {
		sq_addfirst(&entry->link, &callout_queue);
		hrtinfo("call enter at head, reschedule\n");
		/* we changed the next deadline, reschedule the timer event */
		hrt_call_reschedule();

	} else {
		do {
			next = (struct hrt_call *)(void *)sq_next(&call->link);

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

		call = (struct hrt_call *)(void *)sq_peek(&callout_queue);

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
	struct hrt_call	*next = (struct hrt_call *)(void *)sq_peek(&callout_queue);
	hrt_abstime	deadline = HRT_INTERVAL_MAX;

	/*
	 * Determine what the next deadline will be.
	 *
	 * Note that we ensure that this will be within the counter
	 * period, so that when we truncate all but the low 32 bits
	 * the next time the compare matches it will be the deadline
	 * we want.
	 *
	 * It is important for accurate timekeeping that the compare
	 * interrupt fires sufficiently often that the base_time update in
	 * hrt_absolute_time runs at least once per timer period.
	 */
	if (next != NULL) {
		hrtinfo("entry in queue\n");

		if (next->deadline <= (now + HRT_INTERVAL_MIN)) {
			hrtinfo("pre-expired\n");
			/* set a minimal deadline so that we call ASAP */
			deadline = HRT_INTERVAL_MIN;

		} else if (next->deadline < (now + deadline)) {
			hrtinfo("due soon\n");
			deadline = next->deadline - now;
		}
	}

	hrtinfo("schedule for %ul at %ul\n", (unsigned long)(deadline & 0xffffffff), (unsigned long)(now & 0xffffffff));

	/* set the new compare value and remember it for latency tracking */

	latency_baseline = deadline + now;

	/* Disable the channel timers */
	putreg32(LPIT_TCTRL_TSOI, LPIT_TCTRL0(HRT_LPIT));

	/* set the new compare value */
	putreg32(LPIT_VALUE_FACTOR(deadline), LPIT_TVAL0(HRT_LPIT));

	/* enable the channel timers */
	putreg32(LPIT_TCTRL_TSOI | LPIT_TCTRL_T_EN, LPIT_TCTRL0(HRT_LPIT));
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
