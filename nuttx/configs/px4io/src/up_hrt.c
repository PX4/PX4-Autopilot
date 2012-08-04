/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file High-resolution timer callouts and timekeeping.
 *
 * This can use any general or advanced STM32 timer.
 *
 * Note that really, this could use systick too, but that's
 * monopolised by NuttX and stealing it would just be awkward.
 *
 * We don't use the NuttX STM32 driver per se; rather, we 
 * claim the timer and then drive it directly.
 */

#include <nuttx/config.h>
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

#include <arch/board/board.h>
#include <arch/board/up_hrt.h>

#include "chip.h"
#include "up_internal.h"
#include "up_arch.h"

#include "stm32_internal.h"
#include "stm32_gpio.h"
#include "stm32_tim.h"

#ifdef CONFIG_HRT_TIMER

/* HRT configuration */
#if   HRT_TIMER == 1
# define HRT_TIMER_BASE		STM32_TIM1_BASE
# define HRT_TIMER_POWER_REG	STM32_RCC_APB2ENR
# define HRT_TIMER_POWER_BIT	RCC_APB2ENR_TIM1EN
# define HRT_TIMER_VECTOR	STM32_IRQ_TIM1CC
# define HRT_TIMER_CLOCK	STM32_APB2_TIM1_CLKIN
# if CONFIG_STM32_TIM1
#  error must not set CONFIG_STM32_TIM1=y and HRT_TIMER=1
# endif
#elif HRT_TIMER == 2
# define HRT_TIMER_BASE		STM32_TIM2_BASE
# define HRT_TIMER_POWER_REG	STM32_RCC_APB1ENR
# define HRT_TIMER_POWER_BIT	RCC_APB2ENR_TIM2EN
# define HRT_TIMER_VECTOR	STM32_IRQ_TIM2
# define HRT_TIMER_CLOCK	STM32_APB1_TIM2_CLKIN
# if CONFIG_STM32_TIM2
#  error must not set CONFIG_STM32_TIM2=y and HRT_TIMER=2
# endif
#elif HRT_TIMER == 3
# define HRT_TIMER_BASE		STM32_TIM3_BASE
# define HRT_TIMER_POWER_REG	STM32_RCC_APB1ENR
# define HRT_TIMER_POWER_BIT	RCC_APB2ENR_TIM3EN
# define HRT_TIMER_VECTOR	STM32_IRQ_TIM3
# define HRT_TIMER_CLOCK	STM32_APB1_TIM3_CLKIN
# if CONFIG_STM32_TIM3
#  error must not set CONFIG_STM32_TIM3=y and HRT_TIMER=3
# endif
#elif HRT_TIMER == 4
# define HRT_TIMER_BASE		STM32_TIM4_BASE
# define HRT_TIMER_POWER_REG	STM32_RCC_APB1ENR
# define HRT_TIMER_POWER_BIT	RCC_APB2ENR_TIM4EN
# define HRT_TIMER_VECTOR	STM32_IRQ_TIM4
# define HRT_TIMER_CLOCK	STM32_APB1_TIM4_CLKIN
# if CONFIG_STM32_TIM4
#  error must not set CONFIG_STM32_TIM4=y and HRT_TIMER=4
# endif
#elif HRT_TIMER == 5
# define HRT_TIMER_BASE		STM32_TIM5_BASE
# define HRT_TIMER_POWER_REG	STM32_RCC_APB1ENR
# define HRT_TIMER_POWER_BIT	RCC_APB2ENR_TIM5EN
# define HRT_TIMER_VECTOR	STM32_IRQ_TIM5
# define HRT_TIMER_CLOCK	STM32_APB1_TIM5_CLKIN
# if CONFIG_STM32_TIM5
#  error must not set CONFIG_STM32_TIM5=y and HRT_TIMER=5
# endif
#elif HRT_TIMER == 8
# define HRT_TIMER_BASE		STM32_TIM8_BASE
# define HRT_TIMER_POWER_REG	STM32_RCC_APB2ENR
# define HRT_TIMER_POWER_BIT	RCC_APB2ENR_TIM8EN
# define HRT_TIMER_VECTOR	STM32_IRQ_TIM8CC
# define HRT_TIMER_CLOCK	STM32_APB2_TIM8_CLKIN
# if CONFIG_STM32_TIM8
#  error must not set CONFIG_STM32_TIM8=y and HRT_TIMER=6
# endif
#elif HRT_TIMER == 9
# define HRT_TIMER_BASE		STM32_TIM9_BASE
# define HRT_TIMER_POWER_REG	STM32_RCC_APB1ENR
# define HRT_TIMER_POWER_BIT	RCC_APB2ENR_TIM9EN
# define HRT_TIMER_VECTOR	STM32_IRQ_TIM1BRK
# define HRT_TIMER_CLOCK	STM32_APB1_TIM9_CLKIN
# if CONFIG_STM32_TIM9
#  error must not set CONFIG_STM32_TIM9=y and HRT_TIMER=9
# endif
#elif HRT_TIMER == 10
# define HRT_TIMER_BASE		STM32_TIM10_BASE
# define HRT_TIMER_POWER_REG	STM32_RCC_APB1ENR
# define HRT_TIMER_POWER_BIT	RCC_APB2ENR_TIM10EN
# define HRT_TIMER_VECTOR	STM32_IRQ_TIM1UP
# define HRT_TIMER_CLOCK	STM32_APB1_TIM10_CLKIN
# if CONFIG_STM32_TIM10
#  error must not set CONFIG_STM32_TIM11=y and HRT_TIMER=10
# endif
#elif HRT_TIMER == 11
# define HRT_TIMER_BASE		STM32_TIM11_BASE
# define HRT_TIMER_POWER_REG	STM32_RCC_APB1ENR
# define HRT_TIMER_POWER_BIT	RCC_APB2ENR_TIM11EN
# define HRT_TIMER_VECTOR	STM32_IRQ_TIM1TRGCOM
# define HRT_TIMER_CLOCK	STM32_APB1_TIM11_CLKIN
# if CONFIG_STM32_TIM11
#  error must not set CONFIG_STM32_TIM11=y and HRT_TIMER=11
# endif
#else
# error HRT_TIMER must be set in board.h if CONFIG_HRT_TIMER=y
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

/*
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
 * Scaling factor(s) for the free-running counter; convert an input
 * in counts to a time in microseconds.
 */
#define HRT_COUNTER_SCALE(_c)	(_c)

/*
 * Timer register accessors
 */
#define REG(_reg)	(*(volatile uint32_t *)(HRT_TIMER_BASE + _reg))

#define rCR1     	REG(STM32_GTIM_CR1_OFFSET)
#define rCR2     	REG(STM32_GTIM_CR2_OFFSET)
#define rSMCR    	REG(STM32_GTIM_SMCR_OFFSET)
#define rDIER    	REG(STM32_GTIM_DIER_OFFSET)
#define rSR      	REG(STM32_GTIM_SR_OFFSET)
#define rEGR     	REG(STM32_GTIM_EGR_OFFSET)
#define rCCMR1   	REG(STM32_GTIM_CCMR1_OFFSET)
#define rCCMR2   	REG(STM32_GTIM_CCMR2_OFFSET)
#define rCCER    	REG(STM32_GTIM_CCER_OFFSET)
#define rCNT     	REG(STM32_GTIM_CNT_OFFSET)
#define rPSC     	REG(STM32_GTIM_PSC_OFFSET)
#define rARR     	REG(STM32_GTIM_ARR_OFFSET)
#define rCCR1    	REG(STM32_GTIM_CCR1_OFFSET)
#define rCCR2    	REG(STM32_GTIM_CCR2_OFFSET)
#define rCCR3    	REG(STM32_GTIM_CCR3_OFFSET)
#define rCCR4    	REG(STM32_GTIM_CCR4_OFFSET)
#define rDCR     	REG(STM32_GTIM_DCR_OFFSET)
#define rDMAR    	REG(STM32_GTIM_DMAR_OFFSET)

/*
 * Specific registers and bits used by HRT sub-functions
 */
#if HRT_TIMER_CHANNEL == 1
# define rCCR_HRT	rCCR1			/* compare register for HRT */
# define DIER_HRT	GTIM_DIER_CC1IE		/* interrupt enable for HRT */
# define SR_INT_HRT	GTIM_SR_CC1IF		/* interrupt status for HRT */
#elif HRT_TIMER_CHANNEL == 2
# define rCCR_HRT	rCCR2			/* compare register for HRT */
# define DIER_HRT	GTIM_DIER_CC2IE		/* interrupt enable for HRT */
# define SR_INT_HRT	GTIM_SR_CC2IF		/* interrupt status for HRT */
#elif HRT_TIMER_CHANNEL == 3
# define rCCR_HRT	rCCR3			/* compare register for HRT */
# define DIER_HRT	GTIM_DIER_CC3IE		/* interrupt enable for HRT */
# define SR_INT_HRT	GTIM_SR_CC3IF		/* interrupt status for HRT */
#elif HRT_TIMER_CHANNEL == 4
# define rCCR_HRT	rCCR4			/* compare register for HRT */
# define DIER_HRT	GTIM_DIER_CC4IE		/* interrupt enable for HRT */
# define SR_INT_HRT	GTIM_SR_CC4IF		/* interrupt status for HRT */
#else
# error HRT_TIMER_CHANNEL must be a value between 1 and 4
#endif

/*
 * Queue of callout entries.
 */
static struct sq_queue_s	callout_queue;

/*
 * The time corresponding to a counter value of zero, as of the
 * last time that hrt_absolute_time() was called.
 */
static hrt_abstime		base_time;

/* timer-specific functions */
static void		hrt_tim_init(int timer);
static int		hrt_tim_isr(int irq, void *context);

/* callout list manipulation */
static void		hrt_call_enter(struct hrt_call *entry);
static void		hrt_call_reschedule(void);
static void		hrt_call_invoke(void);

/*
 * Specific registers and bits used by PPM sub-functions
 */
#ifdef CONFIG_HRT_PPM
# include <arch/board/drv_ppm_input.h>

/* 
 * If the timer hardware doesn't support GTIM_CCER_CCxNP, then we will work around it.
 */
# ifndef GTIM_CCER_CC1NP
#  define GTIM_CCER_CC1NP 0
#  define GTIM_CCER_CC2NP 0
#  define GTIM_CCER_CC3NP 0
#  define GTIM_CCER_CC4NP 0
#  define PPM_EDGE_FLIP
# endif

# if HRT_PPM_CHANNEL == 1
#  define rCCR_PPM	rCCR1			/* capture register for PPM */
#  define DIER_PPM	GTIM_DIER_CC1IE		/* capture interrupt (non-DMA mode) */
#  define SR_INT_PPM	GTIM_SR_CC1IF		/* capture interrupt (non-DMA mode) */
#  define SR_OVF_PPM	GTIM_SR_CC1OF		/* capture overflow (non-DMA mode) */
#  define CCMR1_PPM	1			/* on TI1 */
#  define CCMR2_PPM	0
#  define CCER_PPM	(GTIM_CCER_CC1E | GTIM_CCER_CC1P | GTIM_CCER_CC1NP) /* CC1, both edges */
#  define CCER_PPM_FLIP	GTIM_CCER_CC1P
# elif HRT_PPM_CHANNEL == 2
#  define rCCR_PPM	rCCR2			/* capture register for PPM */
#  define DIER_PPM	GTIM_DIER_CC2IE		/* capture interrupt (non-DMA mode) */
#  define SR_INT_PPM	GTIM_SR_CC2IF		/* capture interrupt (non-DMA mode) */
#  define SR_OVF_PPM	GTIM_SR_CC2OF		/* capture overflow (non-DMA mode) */
#  define CCMR1_PPM	2			/* on TI2 */
#  define CCMR2_PPM	0
#  define CCER_PPM	(GTIM_CCER_CC2E | GTIM_CCER_CC2P | GTIM_CCER_CC2NP) /* CC2, both edges */
#  define CCER_PPM_FLIP	GTIM_CCER_CC2P
# elif HRT_PPM_CHANNEL == 3
#  define rCCR_PPM	rCCR3			/* capture register for PPM */
#  define DIER_PPM	GTIM_DIER_CC3IE		/* capture interrupt (non-DMA mode) */
#  define SR_INT_PPM	GTIM_SR_CC3IF		/* capture interrupt (non-DMA mode) */
#  define SR_OVF_PPM	GTIM_SR_CC3OF		/* capture overflow (non-DMA mode) */
#  define CCMR1_PPM	0
#  define CCMR2_PPM	1			/* on TI3 */
#  define CCER_PPM	(GTIM_CCER_CC3E | GTIM_CCER_CC3P | GTIM_CCER_CC3NP) /* CC3, both edges */
#  define CCER_PPM_FLIP	GTIM_CCER_CC3P
# elif HRT_PPM_CHANNEL == 4
#  define rCCR_PPM	rCCR4			/* capture register for PPM */
#  define DIER_PPM	GTIM_DIER_CC4IE		/* capture interrupt (non-DMA mode) */
#  define SR_INT_PPM	GTIM_SR_CC4IF		/* capture interrupt (non-DMA mode) */
#  define SR_OVF_PPM	GTIM_SR_CC4OF		/* capture overflow (non-DMA mode) */
#  define CCMR1_PPM	0
#  define CCMR2_PPM	2			/* on TI4 */
#  define CCER_PPM	(GTIM_CCER_CC4E | GTIM_CCER_CC4P | GTIM_CCER_CC4NP) /* CC4, both edges */
#  define CCER_PPM_FLIP	GTIM_CCER_CC4P
# else
#  error HRT_PPM_CHANNEL must be a value between 1 and 4 if CONFIG_HRT_PPM is set
# endif
#else
/* disable the PPM configuration */
# define rCCR_PPM	0
# define DIER_PPM	0
# define SR_INT_PPM	0
# define SR_OVF_PPM	0
# define CCMR1_PPM	0
# define CCMR2_PPM	0
# define CCER_PPM	0
#endif /* CONFIG_HRT_PPM */

/*
 * Initialise the timer we are going to use.
 *
 * We expect that we'll own one of the reduced-function STM32 general
 * timers, and that we can use channel 1 in compare mode.
 */
static void
hrt_tim_init(int timer)
{
	/* clock/power on our timer */
        modifyreg32(HRT_TIMER_POWER_REG, 0, HRT_TIMER_POWER_BIT);

        /* claim our interrupt vector */
        irq_attach(HRT_TIMER_VECTOR, hrt_tim_isr);

        /* disable and configure the timer */
        rCR1 = 0;
        rCR2 = 0;
        rSMCR = 0;
        rDIER = DIER_HRT | DIER_PPM;
        rCCER = 0;		/* unlock CCMR* registers */
        rCCMR1 = CCMR1_PPM;		
        rCCMR2 = CCMR2_PPM;
        rCCER = CCER_PPM;
        rDCR = 0;

        /* configure the timer to free-run at 1MHz */
        rPSC = (HRT_TIMER_CLOCK / 1000000) - 1;	/* this really only works for whole-MHz clocks */

        /* run the full span of the counter */
        rARR = 0xffff;

        /* set an initial capture a little ways off */
        rCCR_HRT = 1000;

        /* generate an update event; reloads the counter, all registers */
        rEGR = GTIM_EGR_UG;

        /* enable the timer */
        rCR1 = GTIM_CR1_CEN;

        /* enable interrupts */
        up_enable_irq(HRT_TIMER_VECTOR);
}

/*
 * Handle the compare interupt by calling the callout dispatcher
 * and then re-scheduling the next deadline.
 */
static int
hrt_tim_isr(int irq, void *context)
{
	uint32_t status;

	/* copy interrupt status */
	status = rSR;

	/* ack the interrupts we just read */
	rSR = ~status;

#ifdef CONFIG_HRT_PPM
	/* was this a PPM edge? */
	if (status & (SR_INT_PPM | SR_OVF_PPM)) {

		/* if required, flip edge sensitivity */
# ifdef PPM_EDGE_FLIP
		rCCER ^= CCER_PPM_FLIP;
# endif

		/* feed the edge to the PPM decoder */
		ppm_input_decode(status & SR_OVF_PPM, rCCR_PPM);
	}
#endif

	/* was this a timer tick? */
	if (status & SR_INT_HRT) {
		/* run any callouts that have met their deadline */
		hrt_call_invoke();

		/* and schedule the next interrupt */
		hrt_call_reschedule();
	}

	return OK;
}

/*
 * Fetch a never-wrapping absolute time value in microseconds from
 * some arbitrary epoch shortly after system start.
 */
hrt_abstime
hrt_absolute_time(void)
{
	static uint32_t last_count;
	uint32_t	count;
	uint32_t	flags = irqsave();

	count = rCNT;

	//lldbg("count %u last_count %u\n", count, last_count);

	/* This simple test is made possible by the guarantee that
	 * we are always called at least once per counter period.
	 */
	if (count < last_count)
		base_time += HRT_COUNTER_PERIOD;

	last_count = count;

	irqrestore(flags);

	return HRT_COUNTER_SCALE(base_time + count);
}

/*
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

/*
 * Convert absolute time to a timespec.
 */
void
abstime_to_ts(struct timespec *ts, hrt_abstime abstime)
{
	ts->tv_sec = abstime / 1000000;
	abstime -= ts->tv_sec * 1000000;
	ts->tv_nsec = abstime * 1000;
}

/*
 * Initalise the high-resolution timing module.
 */
void
hrt_init(int timer)
{
	sq_init(&callout_queue);
	hrt_tim_init(timer);
}

/*
 * Call callout(arg) after interval has elapsed.
 */
void
hrt_call_after(struct hrt_call *entry, hrt_abstime delay, hrt_callout callout, void *arg)
{
	entry->deadline = hrt_absolute_time() + delay;
	entry->period = 0;
	entry->callout = callout;
	entry->arg = arg;

	hrt_call_enter(entry);
}

/*
 * Call callout(arg) at calltime.
 */
void
hrt_call_at(struct hrt_call *entry, hrt_abstime calltime, hrt_callout callout, void *arg)
{
	entry->deadline = calltime;
	entry->period = 0;
	entry->callout = callout;
	entry->arg = arg;

	hrt_call_enter(entry);	
}

/*
 * Call callout(arg) every period.
 */
void
hrt_call_every(struct hrt_call *entry, hrt_abstime delay, hrt_abstime interval, hrt_callout callout, void *arg)
{
	entry->deadline = hrt_absolute_time() + delay;
	entry->period = interval;
	entry->callout = callout;
	entry->arg = arg;

	hrt_call_enter(entry);	
}

/*
 * If this returns true, the call has been invoked and removed from the callout list.
 *
 * Always returns false for repeating callouts.
 */
bool
hrt_called(struct hrt_call *entry)
{
	bool result;

	irqstate_t flags = irqsave();
	result = (entry->deadline == 0);
	irqrestore(flags);

	return result;	
}

/*
 * Remove the entry from the callout list.
 */
void
hrt_cancel(struct hrt_call *entry)
{
	irqstate_t flags = irqsave();

	sq_rem(&entry->link, &callout_queue);
	entry->deadline = 0;

	/* if this is a periodic call being removed by the callout, prevent it from
	 * being re-entered when the callout returns.
	 */
	entry->period = 0;

	irqrestore(flags);
}

static void
hrt_call_enter(struct hrt_call *entry)
{
	irqstate_t flags = irqsave();
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
	irqrestore(flags);
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
		if (call == NULL)
			break;
		if (call->deadline > now)
			break;

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
			call->deadline = deadline + call->period;
			hrt_call_enter(call);
		}
	}
}

/*
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

	/* set the new compare value */
	rCCR_HRT = deadline & 0xffff;
}

#endif /* CONFIG_HRT_TIMER */
