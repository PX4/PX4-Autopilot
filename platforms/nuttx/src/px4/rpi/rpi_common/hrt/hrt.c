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
 * RP2040's internal 64 bit timer can be used for this purpose.
 *
 * Note that really, this could use systick too, but that's
 * monopolised by NuttX and stealing it would just be awkward.
 *
 * We don't use the NuttX RP2040 driver per se; rather, we
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


// #include "rp2040_gpio.h"

#ifdef CONFIG_DEBUG_HRT
#  define hrtinfo _info
#else
#  define hrtinfo(x...)
#endif

// RP2040 has a dedicated 64-bit timer which updates at 1MHz. This timer can be used here as hrt.
// The advantage is that this timer will not overflow as it can run for thousands of years.
// This timer is activated when the clk_ref is configured for watchdog and TICK is enabled.
// Fortunately, nuttx by default does this for us in rp2040_clock.c file. Thus, no init required.
// Four individual interrupts can be configured which are triggered when the lower 32-bits of the timer
// matches with the value in ALARMx register. This allows for the interrupt to fire at ~72 min in future.
// Take a look at src/drivers/drv_hrt.h to find all the necessary functions required to be implemented.

#ifdef HRT_TIMER

/* HRT configuration */
#if   HRT_TIMER == 1
# define HRT_TIMER_BASE		RP2040_TIMER_BASE
#else
# error HRT_TIMER must have a value of 1 because there is only one timer in RP2040
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
#define HRT_INTERVAL_MAX	4000000000

/*
 * Scaling factor(s) for the free-running counter; convert an input
 * in counts to a time in microseconds.
 */
#define HRT_COUNTER_SCALE(_c)	(_c)

/*
 * Timer register accessors
 */
#define REG(_reg)	(*(volatile uint32_t *)(HRT_TIMER_BASE + _reg))

#define rTIMEHW		REG(0x0)	// Write to bits 63:32 of time always write timelw before timehw
#define rTIMELW		REG(0x4)	// Write to bits 31:0 of time writes do not get copied to time until timehw is written
#define rTIMEHR		REG(0x8)	// Read from bits 63:32 of time always read timelr before timehr
#define rTIMELR		REG(0xc)	// Read from bits 31:0 of time
#define rALARM0		REG(0x10)	// Arm alarm 0, and configure the time it will fire
#define rALARM1		REG(0x14)	// Arm alarm 1, and configure the time it will fire
#define rALARM2		REG(0x18)	// Arm alarm 2, and configure the time it will fire
#define rALARM3		REG(0x1c)	// Arm alarm 3, and configure the time it will fire
#define rARMED		REG(0x20)	// Indicates the armed/disarmed status of each alarm
#define rTIMERAWH	REG(0x24)	// Raw read from bits 63:32 of time
#define rTIMERAWL	REG(0x28)	// Raw read from bits 31:0 of time
#define rDBGPAUSE	REG(0x2c)	// Set bits high to enable pause when the corresponding debug ports are active
#define rPAUSE		REG(0x30)	// Set high to pause the timer
#define rINTR		REG(0x34)	// Raw Interrupts
#define rINTE		REG(0x38)	// Interrupt Enable
#define rINTF		REG(0x3c)	// Interrupt Force
#define rINTS		REG(0x40)	// Interrupt status after masking & forcing

/*
 * Specific registers and bits used by HRT sub-functions
 */
#if HRT_TIMER_CHANNEL == 1
# define HRT_TIMER_VECTOR	RP2040_TIMER_IRQ_0	// Timer alarm interrupt vector //
# define HRT_ALARM_VALUE	rALARM0			// Alarm register for HRT (similar to compare register for other MCUs) //
# define HRT_ALARM_ENABLE	(1 << 0)		// Bit-0 for alarm 0 //
#elif HRT_TIMER_CHANNEL == 2
# define HRT_TIMER_VECTOR	RP2040_TIMER_IRQ_1	// Timer alarm interrupt vector //
# define HRT_ALARM_VALUE	rALARM1			// Alarm register for HRT (similar to compare register for other MCUs) //
# define HRT_ALARM_ENABLE	(1 << 1)		// Bit-1 for alarm 1 //
#elif HRT_TIMER_CHANNEL == 3
# define HRT_TIMER_VECTOR	RP2040_TIMER_IRQ_2	// Timer alarm interrupt vector //
# define HRT_ALARM_VALUE	rALARM2			// Alarm register for HRT (similar to compare register for other MCUs) //
# define HRT_ALARM_ENABLE	(1 << 2)		// Bit-2 for alarm 2 //
#elif HRT_TIMER_CHANNEL == 4
# define HRT_TIMER_VECTOR	RP2040_TIMER_IRQ_3	// Timer alarm interrupt vector //
# define HRT_ALARM_VALUE	rALARM3			// Alarm register for HRT (similar to compare register for other MCUs) //
# define HRT_ALARM_ENABLE	(1 << 3)		// Bit-3 for alarm 3 //
#else
# error HRT_TIMER_CHANNEL must be a value between 1 and 4
#endif

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

// For PPM, a PWM timer is not required to be used. This will be handled by activating
// edge detection interrupts on the gpio. This way, the value of hrt can be recorded
// when irq_handler is called and the PPM signal can be decoded. Also, there is no way
// to check wheter the interrupt was caused by a rising edge or a falling edge. This can
// be done by changing the interrupt type (rising/falling) every time the interrupt is
// triggered.

/*
 * Specific registers and bits used by PPM sub-functions
 */
#ifdef HRT_PPM_CHANNEL
/*
 * PPM decoder tuning parameters
 */
# define PPM_MIN_PULSE_WIDTH	200		/**< minimum width of a valid first pulse */
# define PPM_MAX_PULSE_WIDTH	600		/**< maximum width of a valid first pulse */
# define PPM_MIN_CHANNEL_VALUE	800		/**< shortest valid channel signal */
# define PPM_MAX_CHANNEL_VALUE	2200		/**< longest valid channel signal */
# define PPM_MIN_START		2300		/**< shortest valid start gap (only 2nd part of pulse) */

/* decoded PPM buffer */
#define PPM_MIN_CHANNELS	5
#define PPM_MAX_CHANNELS	20

/** Number of same-sized frames required to 'lock' */
#define PPM_CHANNEL_LOCK	4		/**< should be less than the input timeout */

__EXPORT uint16_t ppm_buffer[PPM_MAX_CHANNELS];
__EXPORT uint16_t ppm_frame_length = 0;
__EXPORT unsigned ppm_decoded_channels = 0;
__EXPORT uint64_t ppm_last_valid_decode = 0;

#define PPM_DEBUG 0

#if PPM_DEBUG
/* PPM edge history */
__EXPORT uint16_t ppm_edge_history[32];
unsigned ppm_edge_next;

/* PPM pulse history */
__EXPORT uint16_t ppm_pulse_history[32];
unsigned ppm_pulse_next;
#endif

static uint16_t ppm_temp_buffer[PPM_MAX_CHANNELS];

/** PPM decoder state machine */
struct {
	uint16_t	last_edge;	/**< last capture time */
	uint16_t	last_mark;	/**< last significant edge */
	uint16_t	frame_start;	/**< the frame width */
	unsigned	next_channel;	/**< next channel index */
	enum {
		UNSYNCH = 0,
		ARM,
		ACTIVE,
		INACTIVE
	} phase;
} ppm;

// Store the last detected edge type
enum edgeType {
	falling = 0,
	rising,
};
enum edgeType lastEdge = falling;

// PPM specific functions
static void hrt_ppm_decode(uint16_t counterVal);
static int hrt_ppm_isr(int irq, void *context, void *arg);

#endif /* HRT_PPM_CHANNEL */

/**
 * Initialise the timer we are going to use.
 */
static void
hrt_tim_init(void)
{
	/* claim our interrupt vector */
	irq_attach(HRT_TIMER_VECTOR, hrt_tim_isr, NULL);

	/* Enable alarm interrupt */
	rINTE = HRT_ALARM_ENABLE;

	/* Enable vector interrupt */
	up_enable_irq(HRT_TIMER_VECTOR);

	/* Set Initial capture a little ways off */
	HRT_ALARM_VALUE = rTIMERAWL + 1000;
}

#ifdef HRT_PPM_CHANNEL
/**
 * Handle the PPM decoder state machine.
 */
static void
hrt_ppm_decode(uint16_t counterVal)
{
	uint16_t count = counterVal;
	uint16_t width;
	uint16_t interval;
	unsigned i;

	// If we miss an edge then we will miss a whole pulse.
	// So, frame discarding will be taken care of by the
	// width value.

	/* how long since the last edge? - this handles counter wrapping implicitly. */
	width = count - ppm.last_edge;

#if PPM_DEBUG
	ppm_edge_history[ppm_edge_next++] = width;

	if (ppm_edge_next >= 32) {
		ppm_edge_next = 0;
	}

#endif

	/*
	 * if this looks like a start pulse, then push the last set of values
	 * and reset the state machine
	 */
	if (width >= PPM_MIN_START) {

		/*
		 * If the number of channels changes unexpectedly, we don't want
		 * to just immediately jump on the new count as it may be a result
		 * of noise or dropped edges.  Instead, take a few frames to settle.
		 */
		if (ppm.next_channel != ppm_decoded_channels) {
			static unsigned new_channel_count;
			static unsigned new_channel_holdoff;

			if (new_channel_count != ppm.next_channel) {
				/* start the lock counter for the new channel count */
				new_channel_count = ppm.next_channel;
				new_channel_holdoff = PPM_CHANNEL_LOCK;

			} else if (new_channel_holdoff > 0) {
				/* this frame matched the last one, decrement the lock counter */
				new_channel_holdoff--;

			} else {
				/* we have seen PPM_CHANNEL_LOCK frames with the new count, accept it */
				ppm_decoded_channels = new_channel_count;
				new_channel_count = 0;
			}

		} else {
			/* frame channel count matches expected, let's use it */
			if (ppm.next_channel >= PPM_MIN_CHANNELS) {
				for (i = 0; i < ppm.next_channel; i++) {
					ppm_buffer[i] = ppm_temp_buffer[i];
				}

				ppm_last_valid_decode = hrt_absolute_time();

			}
		}

		/* reset for the next frame */
		ppm.next_channel = 0;

		/* next edge is the reference for the first channel */
		ppm.phase = ARM;

		ppm.last_edge = count;
		return;
	}

	switch (ppm.phase) {
	case UNSYNCH:
		/* we are waiting for a start pulse - nothing useful to do here */
		break;

	case ARM:

		/* we expect a pulse giving us the first mark */
		if (width < PPM_MIN_PULSE_WIDTH || width > PPM_MAX_PULSE_WIDTH) {
			goto error;        /* pulse was too short or too long */
		}

		/* record the mark timing, expect an inactive edge */
		ppm.last_mark = ppm.last_edge;

		/* frame length is everything including the start gap */
		ppm_frame_length = (uint16_t)(ppm.last_edge - ppm.frame_start);
		ppm.frame_start = ppm.last_edge;
		ppm.phase = ACTIVE;
		break;

	case INACTIVE:

		/* we expect a short pulse */
		if (width < PPM_MIN_PULSE_WIDTH || width > PPM_MAX_PULSE_WIDTH) {
			goto error;        /* pulse was too short or too long */
		}

		/* this edge is not interesting, but now we are ready for the next mark */
		ppm.phase = ACTIVE;
		break;

	case ACTIVE:
		/* determine the interval from the last mark */
		interval = count - ppm.last_mark;
		ppm.last_mark = count;

#if PPM_DEBUG
		ppm_pulse_history[ppm_pulse_next++] = interval;

		if (ppm_pulse_next >= 32) {
			ppm_pulse_next = 0;
		}

#endif

		/* if the mark-mark timing is out of bounds, abandon the frame */
		if ((interval < PPM_MIN_CHANNEL_VALUE) || (interval > PPM_MAX_CHANNEL_VALUE)) {
			goto error;
		}

		/* if we have room to store the value, do so */
		if (ppm.next_channel < PPM_MAX_CHANNELS) {
			ppm_temp_buffer[ppm.next_channel++] = interval;
		}

		ppm.phase = INACTIVE;
		break;

	}

	ppm.last_edge = count;
	return;

	/* the state machine is corrupted; reset it */

error:
	/* we don't like the state of the decoder, reset it and try again */
	ppm.phase = UNSYNCH;
	ppm_decoded_channels = 0;

}
#endif /* HRT_PPM_CHANNEL */

/**
 * Handle the compare interrupt by calling the callout dispatcher
 * and then re-scheduling the next deadline.
 */
static int
hrt_tim_isr(int irq, void *context, void *arg)
{
	/* grab the timer for latency tracking purposes */
	latency_actual = hrt_absolute_time();

	rINTR = rINTR;	// ack the interrupts we just read

	/* It is never a timer tick. It is always triggered by alarm. */
	/* do latency calculations */
	hrt_latency_update();

	/* run any callouts that have met their deadline */
	hrt_call_invoke();

	/* and schedule the next interrupt */
	hrt_call_reschedule();

	return OK;
}

/**
 * Handle the compare interrupt by calling the callout dispatcher
 * and then re-scheduling the next deadline.
 */
static int
hrt_ppm_isr(int irq, void *context, void *arg)
{
	// Read lower 16 bits of hrt
	uint16_t counter = rTIMERAWL & 0xffff;

	// Switch the next interrupt type
	px4_arch_gpiosetevent(GPIO_PPM_IN,lastEdge ? false : true,lastEdge ? true : false,true,hrt_ppm_isr,NULL);
	lastEdge = !lastEdge;

	hrt_ppm_decode(counter);
	return OK;
}

/**
 * Fetch a never-wrapping absolute time value in microseconds from
 * some arbitrary epoch shortly after system start.
 */
hrt_abstime hrt_absolute_time(void)
{
	/* Taken from rp2040 datasheet pg. 558 */
	uint32_t hi = rTIMERAWH;
	uint32_t lo;
	do
	{
		lo = rTIMERAWL;
		uint32_t next_hi = rTIMERAWH;
		if (hi == next_hi) break;
		hi = next_hi;
	} while (true);

	return ((uint64_t) hi << 32) | lo;
}

/**
 * Convert a timespec to absolute time
 */
hrt_abstime ts_to_abstime(const struct timespec *ts)
{
	hrt_abstime	result;

	result = (hrt_abstime)(ts->tv_sec) * 1000000;
	result += ts->tv_nsec / 1000;

	return result;
}

/**
 * Convert absolute time to a timespec.
 */
void abstime_to_ts(struct timespec *ts, hrt_abstime abstime)
{
	ts->tv_sec = abstime / 1000000;
	abstime -= ts->tv_sec * 1000000;
	ts->tv_nsec = abstime * 1000;
}

/**
 * Compare a time value with the current time as atomic operation
 */
hrt_abstime hrt_elapsed_time_atomic(const volatile hrt_abstime *then)
{
	irqstate_t flags = px4_enter_critical_section();

	hrt_abstime delta = hrt_absolute_time() - *then;

	px4_leave_critical_section(flags);

	return delta;
}

/**
 * Store the absolute time in an interrupt-safe fashion
 */
void hrt_store_absolute_time(volatile hrt_abstime *t)
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

#ifdef HRT_PPM_CHANNEL
	// Set up edge detection interrupt on the PPM gpio.
	px4_arch_gpiosetevent(GPIO_PPM_IN,lastEdge ? false : true,lastEdge ? true : false,true,hrt_ppm_isr,NULL);
	lastEdge = !lastEdge;

	/* configure the PPM input pin */
	px4_arch_configgpio(GPIO_PPM_IN);
#endif
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
	HRT_ALARM_VALUE = (latency_baseline = deadline) & 0xffffffff;
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
