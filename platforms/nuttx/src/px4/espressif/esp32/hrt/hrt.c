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

#include "esp32_irq.h"
#include <xtensa.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#ifdef CONFIG_DEBUG_HRT
#  define hrtinfo _info
#else
#  define hrtinfo(x...)
#endif

#ifdef	HRT_TIMER

/* HRT configuration */
#if HRT_TIMER == 0
# define HRT_TIM_BASE			0x3ff5f000
# define HRT_TIMER_PERIPH		14
# define HRT_TIMER_PRIO			1
# define HRT_TIMER_VECTOR	        5 + HRT_TIMER_PERIPH
# define HRT_TIMER_CLOCK	        80 * 1000000
# define HRT_TIMER_BASE                 0x3ff5f000
# define HRT_TIMER_CLR_OFFSET		0x00a4
# define HRT_TIMER_INT_ENA_OFFSET	0x0098
# define HRT_TIMER_INT_CLR		1 << 0
# if CONFIG_ESP32_WIFI
#  error must not set CONFIG_ESP32_WIFI=y and HRT_TIMER=0. WIFI makes use of TIMER=0
# endif
#elif HRT_TIMER == 1
# define HRT_TIM_BASE			0x3ff5f000 + 0x0024
# define HRT_TIMER_PERIPH		15
# define HRT_TIMER_PRIO			1
# define HRT_TIMER_VECTOR		5 + HRT_TIMER_PERIPH
# define HRT_TIMER_CLOCK		80 * 1000000
# define HRT_TIMER_BASE			0x3ff5f000 + 0x0024
# define HRT_TIMER_CLR_OFFSET		0x0080
# define HRT_TIMER_INT_ENA_OFFSET	0x0074
# define HRT_TIMER_INT_CLR		1 << 1
# if CONFIG_ESP32_TIMER1
#  error must not set CONFIG_ESP32_TIMER1=y and HRT_TIMER=1
# endif
#elif HRT_TIMER == 2
# define HRT_TIM_BASE			0x3ff5f000 + 0x1000
# define HRT_TIMER_PERIPH		18
# define HRT_TIMER_PRIO			1
# define HRT_TIMER_VECTOR		5 + HRT_TIMER_PERIPH
# define HRT_TIMER_CLOCK		80 * 1000000
# define HRT_TIMER_BASE			0x3ff5f000 + 0x1000
# define HRT_TIMER_CLR_OFFSET		0x00a4
# define HRT_TIMER_INT_ENA_OFFSET	0x0098
# define HRT_TIMER_INT_CLR		1 << 0
# if CONFIG_ESP32_TIMER2
#  error must not set CONFIG_ESP32_TIMER2=y and HRT_TIMER=2
# endif
#elif HRT_TIMER == 3
# define HRT_TIM_BASE			0x3ff5f000 + 0x0024 + 0x1000
# define HRT_TIMER_PERIPH		19
# define HRT_TIMER_PRIO			1
# define HRT_TIMER_VECTOR		5 + HRT_TIMER_PERIPH
# define HRT_TIMER_CLOCK		80 * 1000000
# define HRT_TIMER_BASE			0x3ff5f000 + 0x0024 + 0x1000
# define HRT_TIMER_CLR_OFFSET		0x0080
# define HRT_TIMER_INT_ENA_OFFSET	0x0074
# define HRT_TIMER_INT_CLR		1 << 1
# if CONFIG_ESP32_TIMER3
#  error must not set CONFIG_ESP32_TIMER3=y and HRT_TIMER=3
# endif
#else
# error HRT_TIMER must be a value between 0 and 3
#endif

#define REG(_reg)	(*(volatile uint32_t *)(HRT_TIMER_BASE + _reg))

#define HRT_CONFIG_OFFSET  		0x00
#define HRT_LOAD_LO_OFFSET 		0x0018
#define HRT_LOAD_HI_OFFSET 		0x001c
#define HRT_LOAD_OFFSET    		0x0020
#define HRT_ALARM_LO_OFFSET    		0x0010
#define HRT_ALARM_HI_OFFSET    		0x0014
#define HRT_UPDATE_OFFSET		0x000c
#define HRT_LO_OFFSET 			0x0004
#define HRT_HI_OFFSET 			0x0008
#define HRT_DIVIDER_S			13
#define HRT_DIVIDER_M   		0xffff << 13
#define HRT_ALARM_EN			1 << 10
#define HRT_AUTORELOAD			1 << 29
#define HRT_TIMER_LEVEL_INT_EN		1 << 11
#define HRT_TIMER_INT_ENA		1 << 0
#define HRT_TIMER_EN			1 << 31
#define HRT_INCREASE			1 << 30

#define rLO 		REG(HRT_LO_OFFSET)
#define rHI 		REG(HRT_HI_OFFSET)
#define rUPDATE 	REG(HRT_UPDATE_OFFSET)
#define rALARMLO 	REG(HRT_ALARM_LO_OFFSET)
#define rALARMHI 	REG(HRT_ALARM_HI_OFFSET)

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
static void 		esp32_tim_modifyreg32(uint32_t base, uint32_t offset, uint32_t clearbits, uint32_t setbits);
static void 		esp32_tim_putreg(uint32_t base, uint32_t offset, uint32_t value);
static uint32_t 	esp32_tim_getreg(uint32_t base, uint32_t offset);

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

static void esp32_tim_modifyreg32(uint32_t base, uint32_t offset, uint32_t clearbits, uint32_t setbits)
{
	modifyreg32(base + offset, clearbits, setbits);
}
static void esp32_tim_putreg(uint32_t base, uint32_t offset, uint32_t value)
{
	putreg32(value, base + offset);
}
static uint32_t esp32_tim_getreg(uint32_t base, uint32_t offset)
{

	return getreg32(base + offset);
}

/**
 * Initialise the timer we are going to use.
 *
 * We expect that we'll own one of the reduced-function STM32 general
 * timers, and that we can use channel 1 in compare mode.
 */

static void
hrt_tim_init(void)
{

	// ESP32_TIM_SETPRE(tim, ESP32_HRT_TIMER_PRESCALER);
	uint32_t mask = ((uint32_t)(HRT_TIMER_CLOCK / 1000000) - 1) << HRT_DIVIDER_S;
	esp32_tim_modifyreg32(HRT_TIM_BASE, HRT_CONFIG_OFFSET, HRT_DIVIDER_M, mask);

	// ESP32_TIM_SETMODE(tim, ESP32_TIM_MODE_UP);
	esp32_tim_modifyreg32(HRT_TIM_BASE, HRT_CONFIG_OFFSET, 0, HRT_INCREASE);

	// ESP32_TIM_CLEAR(tim);
	esp32_tim_putreg(HRT_TIM_BASE, HRT_LOAD_LO_OFFSET, 0);
	esp32_tim_putreg(HRT_TIM_BASE, HRT_LOAD_HI_OFFSET, 0);
	esp32_tim_putreg(HRT_TIM_BASE, HRT_LOAD_OFFSET, 1 << 0); //reload

	// ESP32_TIM_SETCTR(tim, 0); //set counter value
	esp32_tim_putreg(HRT_TIM_BASE, HRT_LOAD_LO_OFFSET, 0);
	esp32_tim_putreg(HRT_TIM_BASE, HRT_LOAD_HI_OFFSET, 0);

	// ESP32_TIM_RLD_NOW(tim);   //reload value now
	esp32_tim_putreg(HRT_TIM_BASE, HRT_LOAD_OFFSET, 1 << 0); //reload

	// ESP32_TIM_SETALRVL(tim, 1000);		//alarm value
	uint64_t val = 1000;
	uint64_t low_64 = val & 0xffffffff;
	uint64_t high_64 = (val >> 32) & 0xffffffff;
	esp32_tim_putreg(HRT_TIM_BASE, HRT_ALARM_LO_OFFSET, (uint32_t)low_64);
	esp32_tim_putreg(HRT_TIM_BASE, HRT_ALARM_HI_OFFSET, (uint32_t)high_64);

	// ESP32_TIM_SETALRM(tim, true);		//enable alarm
	esp32_tim_modifyreg32(HRT_TIM_BASE, HRT_CONFIG_OFFSET, 0, HRT_ALARM_EN);
	// ESP32_TIM_SETARLD(tim, false);		//auto reload
	esp32_tim_modifyreg32(HRT_TIM_BASE, HRT_CONFIG_OFFSET, HRT_AUTORELOAD, 0);

	// ESP32_TIM_SETISR(tim, hrt_tim_isr, NULL);
	esp32_setup_irq(0, HRT_TIMER_PERIPH, HRT_TIMER_PRIO, ESP32_CPUINT_LEVEL);
	irq_attach(HRT_TIMER_VECTOR, hrt_tim_isr, NULL);
	up_enable_irq(HRT_TIMER_VECTOR);

	// ESP32_TIM_ENABLEINT(tim);
	esp32_tim_modifyreg32(HRT_TIM_BASE, HRT_CONFIG_OFFSET, 0, HRT_TIMER_LEVEL_INT_EN);
	esp32_tim_modifyreg32(HRT_TIM_BASE, HRT_TIMER_INT_ENA_OFFSET, 0, HRT_TIMER_INT_ENA);

	// ESP32_TIM_START(tim);
	esp32_tim_modifyreg32(HRT_TIM_BASE, HRT_CONFIG_OFFSET, 0, HRT_TIMER_EN);

}

/**
 * Handle the compare interrupt by calling the callout dispatcher
 * and then re-scheduling the next deadline.
 */
static int IRAM_ATTR
hrt_tim_isr(int irq, void *context, void *arg)
{
	// uint32_t status = REG_READ(DPORT_APP_INTR_STATUS_1_REG);

	/* grab the timer for latency tracking purposes */
	uint32_t value_32;
	latency_actual = 0;
	/* Dummy value to latch the counter value to read it */
	esp32_tim_putreg(HRT_TIM_BASE, HRT_UPDATE_OFFSET, 1 << 0);
	/* Read value */
	value_32 = esp32_tim_getreg(HRT_TIM_BASE, HRT_HI_OFFSET); /* High 32 bits */
	latency_actual |= (uint64_t)value_32;
	latency_actual <<= 32;
	value_32 = esp32_tim_getreg(HRT_TIM_BASE, HRT_LO_OFFSET); /* Low 32 bits */
	latency_actual |= (uint64_t)value_32;

	/* do latency calculations */
	hrt_latency_update();

	/* run any callouts that have met their deadline */
	hrt_call_invoke();

	/* and schedule the next interrupt */
	hrt_call_reschedule();

	// acknowledge the interrupt
	esp32_tim_putreg(HRT_TIM_BASE, HRT_TIMER_CLR_OFFSET, HRT_TIMER_INT_CLR);
	esp32_tim_modifyreg32(HRT_TIM_BASE, HRT_CONFIG_OFFSET, 0, HRT_ALARM_EN);

	return OK;
}

/**
 * Fetch a never-wrapping absolute time value in microseconds from
 * some arbitrary epoch shortly after system start.
 */
hrt_abstime IRAM_ATTR
hrt_absolute_time(void)
{
	hrt_abstime	abstime;
	// uint64_t	count;
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
	rUPDATE = 1;
	abstime = (hrt_abstime)(((uint64_t)rHI << 32) | (uint64_t)rLO);

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
void __attribute__((section(".iram1")))
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
void __attribute__((section(".iram1")))
hrt_call_at(struct hrt_call *entry, hrt_abstime calltime, hrt_callout callout, void *arg)
{
	hrt_call_internal(entry, calltime, 0, callout, arg);
}

/**
 * Call callout(arg) every period.
 */
void __attribute__((section(".iram1")))
hrt_call_every(struct hrt_call *entry, hrt_abstime delay, hrt_abstime interval, hrt_callout callout, void *arg)
{
	hrt_call_internal(entry,
			  hrt_absolute_time() + delay,
			  interval,
			  callout,
			  arg);
}

static void __attribute__((section(".iram1")))
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
bool __attribute__((section(".iram1")))
hrt_called(struct hrt_call *entry)
{
	return (entry->deadline == 0);
}

/**
 * Remove the entry from the callout list.
 */
void __attribute__((section(".iram1")))
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

static void __attribute__((section(".iram1")))
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

static void __attribute__((section(".iram1")))
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
static void __attribute__((section(".iram1")))
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

	rALARMLO = (uint32_t)(deadline & 0xffffffff);
	rALARMHI = (uint32_t)((deadline >> 32) & 0xffffffff);
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

void __attribute__((section(".iram1")))
hrt_call_init(struct hrt_call *entry)
{
	memset(entry, 0, sizeof(*entry));
}

void __attribute__((section(".iram1")))
hrt_call_delay(struct hrt_call *entry, hrt_abstime delay)
{
	entry->deadline = hrt_absolute_time() + delay;
}

#endif /* HRT_TIMER */
