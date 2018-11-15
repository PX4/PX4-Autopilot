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

#include <px4_config.h>
#include <px4_log.h>
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

#include "chip.h"
#include "up_internal.h"
#include "up_arch.h"

#include <chip.h>
#include "sam_gpio.h"
#include "sam_tc.h"

#ifdef HRT_TIMER_CHANNEL

#if defined(HRT_TIMER)
# error "HRT_TIMER should not be defined, instead define HRT_TIMER_CHANNEL from 0-11"
#endif

#if HRT_TIMER_CHANNEL == 0 || HRT_TIMER_CHANNEL == 1 || HRT_TIMER_CHANNEL == 2
# define HRT_TIMER 	0
#endif
#if HRT_TIMER_CHANNEL == 3 || HRT_TIMER_CHANNEL == 4 || HRT_TIMER_CHANNEL == 5
# define HRT_TIMER 	1
#endif
#if HRT_TIMER_CHANNEL == 6 || HRT_TIMER_CHANNEL == 7 || HRT_TIMER_CHANNEL == 8
# define HRT_TIMER 	2
#endif
#if HRT_TIMER_CHANNEL == 9 || HRT_TIMER_CHANNEL == 10 || HRT_TIMER_CHANNEL == 11
# define HRT_TIMER 	3
#endif

/* HRT configuration */
#if   HRT_TIMER == 0
# define HRT_TIMER_BASE			SAM_TC012_BASE
# if !defined(CONFIG_SAMV7_TC0)
#  error "HRT_TIMER_CHANNEL 0-2 Require CONFIG_SAMV7_TC0=y"
# endif
#elif HRT_TIMER == 1
# define HRT_TIMER_BASE			SAM_TC345_BASE
# if !defined(CONFIG_SAMV7_TC1)
#  error "HRT_TIMER_CHANNEL 3-5 Require CONFIG_SAMV7_TC1=y"
# endif
#elif HRT_TIMER == 2
# define HRT_TIMER_BASE			SAM_TC678_BASE
# if !defined(CONFIG_SAMV7_TC2)
#  error "HRT_TIMER_CHANNEL 6-8 Require CONFIG_SAMV7_TC2=y"
# endif
#elif HRT_TIMER == 3
# define HRT_TIMER_BASE			SAM_TC901_BASE
# if !defined(CONFIG_SAMV7_TC3)
#  error "HRT_TIMER_CHANNEL 9-11 Require CONFIG_SAMV7_TC3=y"
# endif
#else
# error "HRT_TIMER_CHANNEL should be defined valid value are from 0-11"
# endif
/*
 * We are going to request 1 1Mhz input to our counter
 * but on this platform we will most likely get a hotter value
 * We will test this by assertion.
 */
# define HRT_TIMER_CLOCK		(BOARD_MCK_FREQUENCY/128)
# define  HRT_DESIRED_FREQUENCY 1000000

/*
 * HRT clock must be a greater than 1MHz
 */

#if HRT_TIMER_CLOCK <= 1000000
# error HRT_TIMER_CLOCK must be greater than 1MHz
#endif

/*
 * Scaling factor for the free-running counter; convert an input
 * in counts to a time in microseconds.
 */
#define HRT_TIME_SCALE(_t)	((((_t) * HRT_TIMER_CLOCK) / USEC_PER_SEC) + 1)

/*
 * Scaling factor for the free-running counter; convert an input
 * in time in microseconds to counts.
 */
#define HRT_COUNTER_SCALE(_c)	((_c) * USEC_PER_SEC) / HRT_TIMER_CLOCK

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
#define HRT_COUNTER_PERIOD	65536

/*
 * Timer register accessors
 */
#define REG(_reg)	(*(volatile uint32_t *)(SAM_TC0_BASE + SAM_TC_CHAN_OFFSET(HRT_TIMER_CHANNEL) + _reg))

#define rCCR	REG(SAM_TC_CCR_OFFSET)
#define rCMR 	REG(SAM_TC_CMR_OFFSET)
#define rSMMR 	REG(SAM_TC_SMMR_OFFSET)
#define rRAB 	REG(SAM_TC_RAB_OFFSET)
#define rCV 	REG(SAM_TC_CV_OFFSET)
#define rRA 	REG(SAM_TC_RA_OFFSET)
#define rRB 	REG(SAM_TC_RB_OFFSET)
#define rRC 	REG(SAM_TC_RC_OFFSET)
#define rSR 	REG(SAM_TC_SR_OFFSET)
#define rIER 	REG(SAM_TC_IER_OFFSET)
#define rIDR 	REG(SAM_TC_IDR_OFFSET)
#define rIMR 	REG(SAM_TC_IMR_OFFSET)
#define rEMR 	REG(SAM_TC_EMR_OFFSET)
/*
*
 * Queue of callout entries.
 */
static struct sq_queue_s	callout_queue;

/* latency baseline (last compare value applied) */
static uint16_t			latency_baseline;

/* timer count at interrupt (for latency purposes) */
static uint16_t			latency_actual;

/* timer-specific functions */
static void		hrt_tim_init(void);
static void		hrt_tim_isr(TC_HANDLE tch, void *arg, uint32_t sr);
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

/*
 * Specific registers and bits used by PPM sub-functions
 */
#undef HRT_PPM_CHANNEL //todo:implement PPM
#ifdef HRT_PPM_CHANNEL

# if HRT_PPM_CHANNEL == 1
#  define rCCR_PPM	rCCR			/* capture register for PPM */
#  define DIER_PPM		TIM_DIER_CC1IE		/* capture interrupt (non-DMA mode) */
#  define SR_INT_PPM	GTIM_SR_CC1IF		/* capture interrupt (non-DMA mode) */
#  define SR_OVF_PPM	GTIM_SR_CC1OF		/* capture overflow (non-DMA mode) */
#  define CCMR1_PPM	1			/* not on TI1/TI2 */
#  define CCMR2_PPM	0			/* on TI3, not on TI4 */
#  define CCER_PPM	(GTIM_CCER_CC1E | GTIM_CCER_CC1P | GTIM_CCER_CC1NP) /* CC1, both edges */
#  define CCER_PPM_FLIP	GTIM_CCER_CC1P
# elif HRT_PPM_CHANNEL == 2
#  define rCCR_PPM	rCCR2			/* capture register for PPM */
#  define DIER_PPM	GTIM_DIER_CC2IE		/* capture interrupt (non-DMA mode) */
#  define SR_INT_PPM	GTIM_SR_CC2IF		/* capture interrupt (non-DMA mode) */
#  define SR_OVF_PPM	GTIM_SR_CC2OF		/* capture overflow (non-DMA mode) */
#  define CCMR1_PPM	2			/* not on TI1/TI2 */
#  define CCMR2_PPM	0			/* on TI3, not on TI4 */
#  define CCER_PPM	(GTIM_CCER_CC2E | GTIM_CCER_CC2P | GTIM_CCER_CC2NP) /* CC2, both edges */
#  define CCER_PPM_FLIP	GTIM_CCER_CC2P
# elif HRT_PPM_CHANNEL == 3
#  define rCCR_PPM	rCCR3			/* capture register for PPM */
#  define DIER_PPM	GTIM_DIER_CC3IE		/* capture interrupt (non-DMA mode) */
#  define SR_INT_PPM	GTIM_SR_CC3IF		/* capture interrupt (non-DMA mode) */
#  define SR_OVF_PPM	GTIM_SR_CC3OF		/* capture overflow (non-DMA mode) */
#  define CCMR1_PPM	0			/* not on TI1/TI2 */
#  define CCMR2_PPM	1			/* on TI3, not on TI4 */
#  define CCER_PPM	(GTIM_CCER_CC3E | GTIM_CCER_CC3P | GTIM_CCER_CC3NP) /* CC3, both edges */
#  define CCER_PPM_FLIP	GTIM_CCER_CC3P
# elif HRT_PPM_CHANNEL == 4
#  define rCCR_PPM	rCCR4			/* capture register for PPM */
#  define DIER_PPM	GTIM_DIER_CC4IE		/* capture interrupt (non-DMA mode) */
#  define SR_INT_PPM	GTIM_SR_CC4IF		/* capture interrupt (non-DMA mode) */
#  define SR_OVF_PPM	GTIM_SR_CC4OF		/* capture overflow (non-DMA mode) */
#  define CCMR1_PPM	0			/* not on TI1/TI2 */
#  define CCMR2_PPM	2			/* on TI3, not on TI4 */
#  define CCER_PPM	(GTIM_CCER_CC4E | GTIM_CCER_CC4P | GTIM_CCER_CC4NP) /* CC4, both edges */
#  define CCER_PPM_FLIP	GTIM_CCER_CC4P
# else
#  error HRT_PPM_CHANNEL must be a value between 1 and 4
# endif

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

static void	hrt_ppm_decode(uint32_t status);

#else
/* disable the PPM configuration */
# define rCCR_PPM	0
# define DIER_PPM	0
# define SR_INT_PPM	0
# define SR_OVF_PPM	0
# define CCMR1_PPM	0
# define CCMR2_PPM	0
# define CCER_PPM	0
#endif /* HRT_PPM_CHANNEL */

/**
 * Initialise the timer we are going to use.
 *
 * We expect that we'll own one of the reduced-function STM32 general
 * timers, and that we can use channel 1 in compare mode.
 */

static TC_HANDLE hrt_tch = (TC_HANDLE) - 1;
static void
hrt_tim_init(void)
{
	uint32_t frequency;
	uint32_t actual;
	uint32_t cmr;
	int ret;

	/* We would like 1Mhz count rate */

	frequency = HRT_DESIRED_FREQUENCY;

	/* The pre-calculate values to use when we start the timer */

	ret = sam_tc_clockselect(frequency, &cmr, &actual);

	if (ret < 0) {
		printf("ERROR: Failed no divisor can be found (%d),for timer channel %d\n", ret, HRT_TIMER_CHANNEL);
		return;
	}

	/*  Verify that what we got was what we expected */

	ASSERT(actual == HRT_TIMER_CLOCK);

	/* Allocate the timer/counter and select its mode of operation
	 *
	 *   TC_CMR_TCCLKS       - Returned by sam_tc_clockselect
	 *   TC_CMR_CLKI=0       - Not inverted
	 *   TC_CMR_BURST_NONE   - Not gated by an external signal
	 *   TC_CMR_CPCSTOP=0    - Do not Stop the clock on an RC compare event
	 *   TC_CMR_CPCDIS=0     - Don't disable the clock on an RC compare event
	 *   TC_CMR_EEVTEDG_NONE - No external events (and, hence, no edges
	 *   TC_CMR_EEVT_TIOB    - ???? REVISIT
	 *   TC_CMR_ENET=0       - External event trigger disabled
	 *   TC_CMR_WAVSEL_UP    - TC_CV is incremented from 0 to 65535
	 *   TC_CMR_WAVE         - Waveform mode
	 *   TC_CMR_ACPA_NONE    - RA compare has no effect on TIOA
	 *   TC_CMR_ACPC_NONE    - RC compare has no effect on TIOA
	 *   TC_CMR_AEEVT_NONE   - No external event effect on TIOA
	 *   TC_CMR_ASWTRG_NONE  - No software trigger effect on TIOA
	 *   TC_CMR_BCPB_NONE    - RB compare has no effect on TIOB
	 *   TC_CMR_BCPC_NONE    - RC compare has no effect on TIOB
	 *   TC_CMR_BEEVT_NONE   - No external event effect on TIOB
	 *   TC_CMR_BSWTRG_NONE  - No software trigger effect on TIOB
	 */

	cmr |= (TC_CMR_BURST_NONE  | TC_CMR_EEVTEDG_NONE | TC_CMR_EEVT_TIOB   |
		TC_CMR_WAVSEL_UP   | TC_CMR_WAVE 		 | TC_CMR_ACPA_NONE   |
		TC_CMR_ACPC_NONE   | TC_CMR_AEEVT_NONE   | TC_CMR_ASWTRG_NONE |
		TC_CMR_BCPB_NONE   | TC_CMR_BCPC_NONE    | TC_CMR_BEEVT_NONE  |
		TC_CMR_BSWTRG_NONE);

	hrt_tch = sam_tc_allocate(HRT_TIMER_CHANNEL, cmr);

	if (!hrt_tch) {
		printf("ERROR: Failed to allocate timer channel %d\n", HRT_TIMER_CHANNEL);
		return;
	}

	irqstate_t flags = enter_critical_section();

	/* Start with 1 ms */

	uint64_t regval = HRT_TIME_SCALE(1000LL);

	ASSERT(regval <= UINT16_MAX);

	/* Set up to receive the callback when the Compare interrupt occurs */

	(void)sam_tc_attach(hrt_tch, hrt_tim_isr, hrt_tch, TC_INT_CPCS);

	/* Set RC so that an event will be triggered when TC_CV register counts
	 * up to RC.
	 */

	rRC = regval & UINT16_MAX;

	/* Start the counter */

	sam_tc_start(hrt_tch);

	/* Enable interrupts.  We should get the callback when the interrupt
	 * occurs.
	 */

	leave_critical_section(flags);
}

#ifdef HRT_PPM_CHANNEL
/**
 * Handle the PPM decoder state machine.
 */
static void
hrt_ppm_decode(uint32_t status)
{
	uint16_t count = rCCR_PPM;
	uint16_t width;
	uint16_t interval;
	unsigned i;

	/* if we missed an edge, we have to give up */
	if (status & SR_OVF_PPM) {
		goto error;
	}

	/* how long since the last edge? - this handles counter wrapping implicitely. */
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
			if (ppm.next_channel > PPM_MIN_CHANNELS) {
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
void hrt_tim_isr(TC_HANDLE tch, void *arg, uint32_t sr)
{
	/* grab the timer for latency tracking purposes */

	latency_actual = rCV;

#ifdef HRT_PPM_CHANNEL

	/* was this a PPM edge? */
	if (status & (SR_INT_PPM | SR_OVF_PPM)) {
		/* if required, flip edge sensitivity */
# ifdef PPM_EDGE_FLIP
		rCCER ^= CCER_PPM_FLIP;
# endif

		hrt_ppm_decode(status);
	}

#endif

	/* do latency calculations */
	hrt_latency_update();

	/* run any callouts that have met their deadline */
	hrt_call_invoke();

	/* and schedule the next interrupt */
	hrt_call_reschedule();
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
	flags = enter_critical_section();

	/* get the current counter value */
	count = rCV;

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

	leave_critical_section(flags);

	return abstime;
}

/**
 * Convert a timespec to absolute time
 */
hrt_abstime
ts_to_abstime(struct timespec *ts)
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
 * Compare a time value with the current time.
 */
hrt_abstime
hrt_elapsed_time(const volatile hrt_abstime *then)
{
	irqstate_t flags = enter_critical_section();

	hrt_abstime delta = hrt_absolute_time() - *then;

	leave_critical_section(flags);

	return delta;
}

/**
 * Store the absolute time in an interrupt-safe fashion
 */
hrt_abstime
hrt_store_absolute_time(volatile hrt_abstime *now)
{
	irqstate_t flags = enter_critical_section();

	hrt_abstime ts = hrt_absolute_time();

	leave_critical_section(flags);

	return ts;
}

/**
 * Initalise the high-resolution timing module.
 */
void
hrt_init(void)
{
	sq_init(&callout_queue);
	hrt_tim_init();

#ifdef HRT_PPM_CHANNEL
	/* configure the PPM input pin */
	stm32_configgpio(GPIO_PPM_IN);
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
	irqstate_t flags = enter_critical_section();

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

	leave_critical_section(flags);
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
	irqstate_t flags = enter_critical_section();

	sq_rem(&entry->link, &callout_queue);
	entry->deadline = 0;

	/* if this is a periodic call being removed by the callout, prevent it from
	 * being re-entered when the callout returns.
	 */
	entry->period = 0;

	leave_critical_section(flags);
}

static void
hrt_call_enter(struct hrt_call *entry)
{
	struct hrt_call	*call, *next;

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
		//lldbg("call pop\n");

		/* save the intended deadline for periodic calls */
		deadline = call->deadline;

		/* zero the deadline, as the call has occurred */
		call->deadline = 0;

		/* invoke the callout (if there is one) */
		if (call->callout) {
			//lldbg("call %p: %p(%p)\n", call, call->callout, call->arg);
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
		//lldbg("entry in queue\n");
		if (next->deadline <= (now + HRT_INTERVAL_MIN)) {
			//lldbg("pre-expired\n");
			/* set a minimal deadline so that we call ASAP */
			deadline = now + HRT_INTERVAL_MIN;

		} else if (next->deadline < deadline) {
			//lldbg("due soon\n");
			deadline = next->deadline;
		}
	}

	//lldbg("schedule for %u at %u\n", (unsigned)(deadline & 0xffffffff), (unsigned)(now & 0xffffffff));

	/* set the new compare value and remember it for latency tracking */
	rRC = latency_baseline = HRT_TIME_SCALE(deadline) & UINT16_MAX;
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

#endif /* HRT_TIMER_CHANNEL */
