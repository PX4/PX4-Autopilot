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
 * High-resolution timer callouts and timekeeping.
 *
 * This can use any general or advanced STM32 timer.
 *
 * Note that really, this could use systick too, but that's
 * monopolised by NuttX and stealing it would just be awkward.
 *
 * We don't use the NuttX STM32 driver per se; rather, we
 * claim the timer and then drive it directly.
 */

#include <px4_platform_common/px4_config.h>
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


#include "hardware/esp32_soc.h"
#include "hardware/esp32_tim.h"
#include "hardware/esp32_dport.h"

#ifdef CONFIG_DEBUG_HRT
#  define hrtinfo _info
#else
#  define hrtinfo(x...)
#endif

#ifdef HRT_TIMER

#if HRT_TIMER_GROUP == 0
# define TIM0_BASE	DR_REG_TIMERGROUP0_BASE
# define TIM1_BASE	DR_REG_TIMERGROUP1_BASE
#else
# error HRT_TIMER_GROUP must be a value between 0 and 0
#endif

/* HRT configuration */
#if   HRT_TIMER == 0
# define HRT_TIMER_VECTOR
# define HRT_TIMER_CLOCK	APB_CLK_FREQ
# define HRT_TIMER_BASE		TIM0_BASE
# define HRT_TIMER_CONFIG_REG	TIM0_BASE + TIM_CONFIG_OFFSET
# define HRT_TIMER_LO_REG	TIM0_BASE + TIM_LO_OFFSET
# define HRT_TIMER_HI_REG	TIM0_BASE + TIM_HI_OFFSET
# define HRT_TIMER_UPDATE_REG	TIM0_BASE + TIM_UPDATE_OFFSET
# define HRT_TIMER_ALARM_LO_REG	TIM0_BASE + TIMG_ALARM_LO_OFFSET
# define HRT_TIMER_ALARM_HI_REG	TIM0_BASE + TIMG_ALARM_HI_OFFSET
# define HRT_TIMER_LOAD_LO_REG	TIM0_BASE + TIM_LOAD_LO_OFFSET
# define HRT_TIMER_LOAD_HI_REG	TIM0_BASE + TIM_LOAD_HI_OFFSET
# define HRT_TIMER_LOAD_REG	TIM0_BASE + TIM_LOAD_OFFSET
# if CONFIG_STM32_TIM1
#  error must not set CONFIG_STM32_TIM1=y and HRT_TIMER=1
# endif
#elif HRT_TIMER == 1
# define HRT_TIMER_CLOCK	APB_CLK_FREQ
# define HRT_TIMER_BASE		TIM1_BASE
# define HRT_TIMER_CONFIG_REG	TIM1_BASE + TIM_CONFIG_OFFSET
# define HRT_TIMER_LO_REG	TIM1_BASE + TIM_LO_OFFSET
# define HRT_TIMER_HI_REG	TIM1_BASE + TIM_HI_OFFSET
# define HRT_TIMER_UPDATE_REG	TIM1_BASE + TIM_UPDATE_OFFSET
# define HRT_TIMER_ALARM_LO_REG	TIM1_BASE + TIMG_ALARM_LO_OFFSET
# define HRT_TIMER_ALARM_HI_REG	TIM1_BASE + TIMG_ALARM_HI_OFFSET
# define HRT_TIMER_LOAD_LO_REG	TIM1_BASE + TIM_LOAD_LO_OFFSET
# define HRT_TIMER_LOAD_HI_REG	TIM1_BASE + TIM_LOAD_HI_OFFSET
# define HRT_TIMER_LOAD_REG	TIM1_BASE + TIM_LOAD_OFFSET
# if CONFIG_STM32_TIM2
#  error must not set CONFIG_STM32_TIM2=y and HRT_TIMER=2
# endif
#else
# error HRT_TIMER must be a value between 0 and 1
#endif

/*
 * HRT clock must be a multiple of 1MHz greater than 1MHz
 */
#if (HRT_TIMER_CLOCK % 1000000) != 0
# error HRT_TIMER_CLOCK must be a multiple of 1MHz
#endif
#if HRT_TIMER_CLOCK <= 1000000
# error HRT_TIMER_CLOCK must be greater than 1MHz
#endif

/**
 * Minimum/maximum deadlines.
 *
 * These are suitable for use with a 16-bit timer/counter clocked
 * at 1MHz.  The high-resolution timer need only guarantee that it
 * not wrap more than once in the 50ms period for absolute time to
 * be consistently maintained.
 *
 * The minimum deadline must be such that the time taken between
 * reading a time and writing a deadline to the timer cannot
 * result in missing the deadline.
 */
#define HRT_INTERVAL_MIN	50
#define HRT_INTERVAL_MAX	50000

/*
 * Period of the free-running counter, in microseconds.
 */
#define HRT_COUNTER_PERIOD	18446744073709551615

/*
 * Scaling factor(s) for the free-running counter; convert an input
 * in counts to a time in microseconds.
 */
#define HRT_COUNTER_SCALE(_c)	(_c)

/*
 * Queue of callout entries.
 */
static struct sq_queue_s	callout_queue;

/* latency baseline (last compare value applied) */
static uint64_t			latency_baseline;

/* timer count at interrupt (for latency purposes) */
static uint64_t			latency_actual;

/* latency histogram */
const uint16_t latency_bucket_count = LATENCY_BUCKET_COUNT;
const uint16_t latency_buckets[LATENCY_BUCKET_COUNT] = { 1, 2, 5, 10, 20, 50, 100, 1000 };
__EXPORT uint32_t latency_counters[LATENCY_BUCKET_COUNT + 1];

/* timer-specific functions */
static void		hrt_tim_init(void);
static int		hrt_tim_isr(int irq, void *context, void *arg);
static void		hrt_latency_update(void);

/* callout list manipulation */
static void		hrt_call_internal(struct hrt_call *entry,
		hrt_abstime deadline,
		hrt_abstime interval,
		hrt_callout callout,
		void *arg);
static void		hrt_call_enter(struct hrt_call *entry);
static void		hrt_call_reschedule(void);
static void		hrt_call_invoke(void);


int hrt_ioctl(unsigned int cmd, unsigned long arg);

__EXPORT uint16_t ppm_buffer[0];
__EXPORT uint16_t ppm_frame_length = 0;
__EXPORT unsigned ppm_decoded_channels = 0;
__EXPORT uint64_t ppm_last_valid_decode = 0;

#define PPM_DEBUG 0

/**
 * Initialise the timer we are going to use.
 *
 * We expect that we'll own one of the reduced-function STM32 general
 * timers, and that we can use channel 1 in compare mode.
 */
static void
hrt_tim_init(void)
{
	/* claim our interrupt vector */
	irq_attach(DPORT_APP_TIMER_INT1_MAP_REG, hrt_tim_isr, NULL);

	/* clock/power on our timer */
	// modifyreg32(HRT_TIMER_POWER_REG, 0, HRT_TIMER_POWER_BIT);

	/* disable and configure the timer */
	REG_CLR_BIT(HRT_TIMER_CONFIG_REG, TIMG_T0_EN);

	// time-base counter will increment every clock tick
	REG_SET_BIT(HRT_TIMER_CONFIG_REG, TIMG_T0_INCREASE);

	// auto-reload at alarm is enabled
	REG_SET_BIT(HRT_TIMER_CONFIG_REG, TIMG_T0_AUTORELOAD);

	// configure timer to free run at 1Mhz
	REG_SET_BITS(HRT_TIMER_CONFIG_REG, (HRT_TIMER_CLOCK / 1000000) - 1, TIMG_T0_DIVIDER);

	// an alarm will generate an edge type interrupt.
	REG_SET_BIT(HRT_TIMER_CONFIG_REG, TIMG_T0_EDGE_INT_EN);

	// an alarm will not generate a level type interrupt
	REG_CLR_BIT(HRT_TIMER_CONFIG_REG, TIMG_T0_LEVEL_INT_EN);

	// the alarm is enabled.
	REG_SET_BIT(HRT_TIMER_CONFIG_REG, TIMG_T0_ALARM_EN);

	// set the alarm at 1000
	// set the alarm just infront of the current timer value
	REG_WRITE(HRT_TIMER_ALARM_LO_REG, 0x3e8);
	REG_WRITE(HRT_TIMER_ALARM_HI_REG, 0x00);

	// reload value will be zero
	REG_WRITE(HRT_TIMER_LOAD_LO_REG, 0x00);
	REG_WRITE(HRT_TIMER_LOAD_HI_REG, 0x00);

	// set the timer to zero!
	REG_WRITE(HRT_TIMER_LOAD_REG, 0x01);

	/* enable the timer */
	REG_SET_BIT(HRT_TIMER_CONFIG_REG, TIMG_T0_EN);

	/* enable interrupts */
	up_enable_irq(DPORT_APP_TIMER_INT1_MAP_REG);
}

/**
 * Handle the compare interrupt by calling the callout dispatcher
 * and then re-scheduling the next deadline.
 */
static int
hrt_tim_isr(int irq, void *context, void *arg)
{
	// uint32_t status;

	/* grab the timer for latency tracking purposes */
	REG_WRITE(HRT_TIMER_UPDATE_REG, 0x1);

	latency_actual = (uint64_t)(REG_READ(HRT_TIMER_HI_REG)) << 32 | REG_READ(HRT_TIMER_LO_REG);

	// /* copy interrupt status */
	// status = REG_READ(HRT_TIMER_HI_REG);

	// /* ack the interrupts we just read */
	// rSR = ~status;

	/* was this a timer tick? */
	// if (status & SR_INT_HRT) {

		/* do latency calculations */
		hrt_latency_update();

		/* run any callouts that have met their deadline */
		hrt_call_invoke();

		/* and schedule the next interrupt */
		hrt_call_reschedule();
	// }

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
	uint64_t	count;
	irqstate_t	flags;

	/*
	 * Counter state.  Marked volatile as they may change
	 * inside this routine but outside the irqsave/restore
	 * pair.  Discourage the compiler from moving loads/stores
	 * to these outside of the protected range.
	 */
	// static volatile uint64_t last_count;

	/* prevent re-entry */
	flags = px4_enter_critical_section();

	/* get the current counter value */
	REG_WRITE(HRT_TIMER_UPDATE_REG, 0x1);
	count = (uint64_t)(HRT_TIMER_HI_REG) << 32 | REG_READ(HRT_TIMER_LO_REG);


	/* compute the current time */
	abstime = HRT_COUNTER_SCALE(count);

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
	hrt_abstime	deadline = now + HRT_INTERVAL_MAX;

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
		hrtinfo("entry in queue\n");

		if (next->deadline <= (now + HRT_INTERVAL_MIN)) {
			hrtinfo("pre-expired\n");
			/* set a minimal deadline so that we call ASAP */
			deadline = now + HRT_INTERVAL_MIN;

		} else if (next->deadline < deadline) {
			hrtinfo("due soon\n");
			deadline = next->deadline;
		}
	}

	hrtinfo("schedule for %u at %u\n", (unsigned)(deadline & 0xffffffff), (unsigned)(now & 0xffffffff));

	/* set the new compare value and remember it for latency tracking */
	latency_baseline = deadline & 0xffff;

	REG_WRITE(HRT_TIMER_ALARM_LO_REG, latency_baseline & 0xffff);
	REG_WRITE(HRT_TIMER_ALARM_HI_REG, latency_baseline >> 32);
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

#endif /* HRT_TIMER */
