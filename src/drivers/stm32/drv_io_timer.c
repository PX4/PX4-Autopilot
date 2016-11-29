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

/*
 * @file drv_pwm_servo.c
 *
 * Servo driver supporting PWM servos connected to STM32 timer blocks.
 *
 * Works with any of the 'generic' or 'advanced' STM32 timers that
 * have output pins, does not require an interrupt.
 */

#include <px4_config.h>
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
#include <stdio.h>

#include <arch/board/board.h>
#include <drivers/drv_pwm_output.h>

#include "drv_io_timer.h"

#include <chip.h>
#include <up_internal.h>
#include <up_arch.h>

#include <stm32.h>
#include <stm32_gpio.h>
#include <stm32_tim.h>

#define arraySize(a) (sizeof((a))/sizeof(((a)[0])))

#define MAX_CHANNELS_PER_TIMER 4

#define _REG32(_base, _reg)	(*(volatile uint32_t *)(_base + _reg))
#define REG(_tmr, _reg)		_REG32(io_timers[_tmr].base, _reg)

#define rCR1(_tmr)    	REG(_tmr, STM32_GTIM_CR1_OFFSET)
#define rCR2(_tmr)    	REG(_tmr, STM32_GTIM_CR2_OFFSET)
#define rSMCR(_tmr)   	REG(_tmr, STM32_GTIM_SMCR_OFFSET)
#define rDIER(_tmr)   	REG(_tmr, STM32_GTIM_DIER_OFFSET)
#define rSR(_tmr)     	REG(_tmr, STM32_GTIM_SR_OFFSET)
#define rEGR(_tmr)    	REG(_tmr, STM32_GTIM_EGR_OFFSET)
#define rCCMR1(_tmr)  	REG(_tmr, STM32_GTIM_CCMR1_OFFSET)
#define rCCMR2(_tmr)  	REG(_tmr, STM32_GTIM_CCMR2_OFFSET)
#define rCCER(_tmr)   	REG(_tmr, STM32_GTIM_CCER_OFFSET)
#define rCNT(_tmr)    	REG(_tmr, STM32_GTIM_CNT_OFFSET)
#define rPSC(_tmr)    	REG(_tmr, STM32_GTIM_PSC_OFFSET)
#define rARR(_tmr)    	REG(_tmr, STM32_GTIM_ARR_OFFSET)
#define rCCR1(_tmr)   	REG(_tmr, STM32_GTIM_CCR1_OFFSET)
#define rCCR2(_tmr)   	REG(_tmr, STM32_GTIM_CCR2_OFFSET)
#define rCCR3(_tmr)   	REG(_tmr, STM32_GTIM_CCR3_OFFSET)
#define rCCR4(_tmr)   	REG(_tmr, STM32_GTIM_CCR4_OFFSET)
#define rDCR(_tmr)    	REG(_tmr, STM32_GTIM_DCR_OFFSET)
#define rDMAR(_tmr)   	REG(_tmr, STM32_GTIM_DMAR_OFFSET)
#define rBDTR(_tmr)		REG(_tmr, STM32_ATIM_BDTR_OFFSET)

#define GTIM_SR_CCIF (GTIM_SR_CC4IF|GTIM_SR_CC3IF|GTIM_SR_CC2IF|GTIM_SR_CC1IF)
#define GTIM_SR_CCOF (GTIM_SR_CC4OF|GTIM_SR_CC3OF|GTIM_SR_CC2OF|GTIM_SR_CC1OF)

#define CCMR_C1_RESET 		0x00ff
#define CCMR_C1_NUM_BITS 		8
#define CCER_C1_NUM_BITS 		4

#define CCMR_C1_CAPTURE_INIT (GTIM_CCMR_CCS_CCIN1  << GTIM_CCMR1_CC1S_SHIFT) | \
	(GTIM_CCMR_ICPSC_NOPSC << GTIM_CCMR1_IC1PSC_SHIFT) | \
	(GTIM_CCMR_ICF_NOFILT << GTIM_CCMR1_IC1F_SHIFT)

#define CCMR_C1_PWMOUT_INIT (GTIM_CCMR_MODE_PWM1 << GTIM_CCMR1_OC1M_SHIFT) | GTIM_CCMR1_OC1PE

#define CCMR_C1_PWMIN_INIT 0 // TBD

//												 				  NotUsed   PWMOut  PWMIn Capture
io_timer_channel_allocation_t channel_allocations[IOTimerChanModeSize] = { UINT16_MAX,   0  ,  0   ,  0 };

typedef uint8_t io_timer_allocation_t; /* big enough to hold MAX_IO_TIMERS */

static io_timer_allocation_t once = 0;

typedef struct channel_stat_t {
	uint32_t 			isr_cout;
	uint32_t 			overflows;
} channel_stat_t;

static channel_stat_t io_timer_channel_stats[MAX_TIMER_IO_CHANNELS];

static struct channel_handler_entry {
	channel_handler_t callback;
	void			  *context;
} channel_handlers[MAX_TIMER_IO_CHANNELS];


static int io_timer_handler(uint16_t timer_index)
{
	/* Read the count at the time of the interrupt */

	uint16_t count = rCNT(timer_index);

	/* Read the HRT at the time of the interrupt */

	hrt_abstime now = hrt_absolute_time();

	const io_timers_t *tmr = &io_timers[timer_index];

	/* What is pending and enabled? */

	uint16_t statusr = rSR(timer_index);
	uint16_t enabled =  rDIER(timer_index) & GTIM_SR_CCIF;

	/* Iterate over the timer_io_channels table */

	for (unsigned chan_index = tmr->first_channel_index; chan_index <= tmr->last_channel_index; chan_index++) {

		uint16_t masks = timer_io_channels[chan_index].masks;

		/* Do we have an enabled channel */

		if (enabled & masks) {


			if (statusr & masks & GTIM_SR_CCIF) {

				io_timer_channel_stats[chan_index].isr_cout++;

				/* Call the client to read the CCxR etc and clear the CCxIF */

				if (channel_handlers[chan_index].callback) {
					channel_handlers[chan_index].callback(channel_handlers[chan_index].context, tmr,
									      chan_index, &timer_io_channels[chan_index],
									      now , count);
				}
			}

			if (statusr & masks & GTIM_SR_CCOF) {

				/* Error we has a second edge before we cleared CCxR */

				io_timer_channel_stats[chan_index].overflows++;
			}
		}
	}

	/* Clear all the SR bits for interrupt enabled channels only */

	rSR(timer_index) = ~(statusr & (enabled | enabled << 8));
	return 0;
}

int io_timer_handler0(int irq, void *context)
{

	return io_timer_handler(0);
}

int io_timer_handler1(int irq, void *context)
{
	return io_timer_handler(1);

}

int io_timer_handler2(int irq, void *context)
{
	return io_timer_handler(2);

}

int io_timer_handler3(int irq, void *context)
{
	return io_timer_handler(3);

}

static inline int validate_timer_index(unsigned timer)
{
	return (timer < MAX_IO_TIMERS && io_timers[timer].base != 0) ? 0 : -EINVAL;
}

static inline int is_timer_uninitalized(unsigned timer)
{
	int rv = 0;

	if (once & 1 << timer) {
		rv = -EBUSY;
	}

	return rv;
}

static inline void set_timer_initalized(unsigned timer)
{
	once |= 1 << timer;
}

static inline void set_timer_deinitalized(unsigned timer)
{
	once &= ~(1 << timer);
}

static inline int channels_timer(unsigned channel)
{
	return timer_io_channels[channel].timer_index;
}

static uint32_t get_timer_channels(unsigned timer)
{
	uint32_t channels = 0;

	if (validate_timer_index(timer) == 0) {
		const io_timers_t *tmr = &io_timers[timer];
		/* Gather the channel bit that belong to the timer */

		for (unsigned chan_index = tmr->first_channel_index; chan_index <= tmr->last_channel_index; chan_index++) {
			channels |= 1 << chan_index;
		}
	}

	return channels;
}

static inline int is_channels_timer_uninitalized(unsigned channel)
{
	return is_timer_uninitalized(channels_timer(channel));
}

int io_timer_is_channel_free(unsigned channel)
{
	int rv = io_timer_validate_channel_index(channel);

	if (rv == 0) {
		if (0 == (channel_allocations[IOTimerChanMode_NotUsed] & (1 << channel))) {
			rv = -EBUSY;
		}
	}

	return rv;
}

int io_timer_validate_channel_index(unsigned channel)
{
	int rv = -EINVAL;

	if (channel < MAX_TIMER_IO_CHANNELS && timer_io_channels[channel].timer_channel != 0) {

		unsigned timer = timer_io_channels[channel].timer_index;

		/* test timer for validity */

		if ((io_timers[timer].base != 0) &&
		    (timer_io_channels[channel].gpio_out != 0) &&
		    (timer_io_channels[channel].gpio_in != 0)) {
			rv = 0;
		}
	}

	return rv;
}

int io_timer_get_mode_channels(io_timer_channel_mode_t mode)
{
	if (mode < IOTimerChanModeSize) {
		return channel_allocations[mode];
	}

	return 0;
}

int io_timer_get_channel_mode(unsigned channel)
{
	io_timer_channel_allocation_t bit = 1 << channel;

	for (int mode = IOTimerChanMode_NotUsed; mode < IOTimerChanModeSize; mode++) {
		if (bit & channel_allocations[mode]) {
			return mode;
		}
	}

	return -1;
}

static inline int allocate_channel_resource(unsigned channel, io_timer_channel_mode_t mode)
{
	int rv = io_timer_is_channel_free(channel);

	if (rv == 0) {
		io_timer_channel_allocation_t bit = 1 << channel;
		channel_allocations[IOTimerChanMode_NotUsed] &= ~bit;
		channel_allocations[mode] |= bit;
	}

	return rv;
}


static inline int free_channel_resource(unsigned channel)
{
	int mode = io_timer_get_channel_mode(channel);

	if (mode > IOTimerChanMode_NotUsed) {
		io_timer_channel_allocation_t bit = 1 << channel;
		channel_allocations[mode] &= ~bit;
		channel_allocations[IOTimerChanMode_NotUsed] |= bit;
	}

	return mode;
}

int io_timer_free_channel(unsigned channel)
{
	if (io_timer_validate_channel_index(channel) != 0) {
		return -EINVAL;
	}

	int mode = io_timer_get_channel_mode(channel);

	if (mode > IOTimerChanMode_NotUsed) {
		io_timer_set_enable(false, mode, 1 << channel);
		free_channel_resource(channel);

	}

	return 0;
}


static int allocate_channel(unsigned channel, io_timer_channel_mode_t mode)
{
	int rv = -EINVAL;

	if (mode != IOTimerChanMode_NotUsed) {
		rv = io_timer_validate_channel_index(channel);

		if (rv == 0) {
			rv = allocate_channel_resource(channel, mode);
		}
	}

	return rv;
}

static int timer_set_rate(unsigned timer, unsigned rate)
{
	/* configure the timer to update at the desired rate */
	rARR(timer) = 1000000 / rate;

	/* generate an update event; reloads the counter and all registers */
	rEGR(timer) = GTIM_EGR_UG;

	return 0;
}

static int timer_set_clock(unsigned timer, unsigned clock_MHz)
{
	rPSC(timer) = (io_timers[timer].clock_freq / (clock_MHz*1000000)) - 1;
	return 0;
}


static int io_timer_init_timer(unsigned timer)
{
	/* Do this only once per timer */

	int rv = is_timer_uninitalized(timer);

	if (rv == 0) {

		irqstate_t flags = irqsave();

		set_timer_initalized(timer);

		/* enable the timer clock before we try to talk to it */

		modifyreg32(io_timers[timer].clock_register, 0, io_timers[timer].clock_bit);

		/* disable and configure the timer */
		rCR1(timer) = 0;
		rCR2(timer) = 0;
		rSMCR(timer) = 0;
		rDIER(timer) = 0;
		rCCER(timer) = 0;
		rCCMR1(timer) = 0;
		rCCMR2(timer) = 0;
		rCCR1(timer) = 0;
		rCCR2(timer) = 0;
		rCCR3(timer) = 0;
		rCCR4(timer) = 0;
		rCCER(timer) = 0;
		rDCR(timer) = 0;

		if ((io_timers[timer].base == STM32_TIM1_BASE) || (io_timers[timer].base == STM32_TIM8_BASE)) {

			/* master output enable = on */

			rBDTR(timer) = ATIM_BDTR_MOE;
		}

		/* configure the timer to free-run at 1MHz */

		rPSC(timer) = (io_timers[timer].clock_freq / 1000000) - 1;


		/*
		 * Note we do the Standard PWM Out init here
		 * default to updating at 50Hz
		 */

		timer_set_rate(timer, 50);

		/*
		 * Note that the timer is left disabled with IRQ subs installed
		 * and active but DEIR bits are not set.
		 */

		irq_attach(io_timers[timer].vectorno, io_timers[timer].handler);

		up_enable_irq(io_timers[timer].vectorno);

		irqrestore(flags);
	}

	return rv;
}


int io_timer_set_rate(unsigned timer, unsigned rate)
{
	/* Gather the channel bit that belong to the timer */

	uint32_t channels = get_timer_channels(timer);

	/* Check ownership of PWM out */

	if ((channels & channel_allocations[IOTimerChanMode_PWMOut]) != 0) {

		/* Change only a timer that is owned by pwm */

		timer_set_rate(timer, rate);
	}

	return 0;
}

int io_timer_set_clock(unsigned timer, unsigned clock_MHz)
{
	/* Gather the channel bit that belong to the timer */
	uint32_t channels = get_timer_channels(timer);

	/* Check ownership of PWM out */
	if ((channels & channel_allocations[IOTimerChanMode_PWMOut]) != 0) {
		/* Change only a timer that is owned by pwm */
		timer_set_clock(timer, clock_MHz);
	}

	return 0;
}

int io_timer_channel_init(unsigned channel, io_timer_channel_mode_t mode,
			  channel_handler_t channel_handler, void *context)
{

	uint32_t gpio = 0;
	uint32_t clearbits = CCMR_C1_RESET;
	uint32_t setbits = CCMR_C1_CAPTURE_INIT;
	uint32_t ccer_setbits = GTIM_CCER_CC1E;
	uint32_t dier_setbits = GTIM_DIER_CC1IE;

	/* figure out the GPIO config first */

	switch (mode) {
	case IOTimerChanMode_PWMOut:
		ccer_setbits = 0;
		dier_setbits = 0;
		setbits = CCMR_C1_PWMOUT_INIT;
		break;

	case IOTimerChanMode_PWMIn:
		setbits = CCMR_C1_PWMIN_INIT;
		gpio = timer_io_channels[channel].gpio_in;
		break;

	case IOTimerChanMode_Capture:
		setbits = CCMR_C1_CAPTURE_INIT;
		gpio = timer_io_channels[channel].gpio_in;
		break;

	case IOTimerChanMode_NotUsed:
		setbits = 0;
		break;

	default:
		return -EINVAL;
	}

	int rv = allocate_channel(channel, mode);

	/* Valid channel should now be reserved in new mode */

	if (rv >= 0) {

		/* Blindly try to initialize the time - it will only do it once */

		io_timer_init_timer(channels_timer(channel));

		irqstate_t flags = irqsave();

		/* Set up IO */
		if (gpio) {
			stm32_configgpio(gpio);
		}


		unsigned timer = channels_timer(channel);


		/* configure the channel */

		uint32_t shifts = timer_io_channels[channel].timer_channel - 1;

		/* Map shifts timer channel 1-4 to 0-3 */

		uint32_t ccmr_offset = STM32_GTIM_CCMR1_OFFSET + ((shifts >> 1)  * sizeof(uint32_t));
		uint32_t ccr_offset = STM32_GTIM_CCR1_OFFSET + (shifts * sizeof(uint32_t));

		clearbits <<= (shifts & 1) * CCMR_C1_NUM_BITS;
		setbits <<= (shifts & 1) * CCMR_C1_NUM_BITS;

		uint16_t rvalue = REG(timer, ccmr_offset);
		rvalue &= ~clearbits;
		rvalue |=  setbits;
		REG(timer, ccmr_offset) = rvalue;

		/*
		 * The beauty here is that per DocID018909 Rev 8 18.3.5 Input capture mode
		 * As soon as CCxS (in SSMRx becomes different from 00, the channel is configured
		 * in input and the TIMx_CCR1 register becomes read-only.
		 * so the next line does nothing in capture mode and initializes an PWM out to
		 * 0
		 */

		REG(timer, ccr_offset) = 0;

		/* on PWM Out ccer_setbits is 0 */

		clearbits = (GTIM_CCER_CC1E | GTIM_CCER_CC1P | GTIM_CCER_CC1NP) << (shifts * CCER_C1_NUM_BITS);
		setbits  = ccer_setbits << (shifts * CCER_C1_NUM_BITS);
		rvalue = rCCER(timer);
		rvalue &= ~clearbits;
		rvalue |=  setbits;
		rCCER(timer) = rvalue;

		channel_handlers[channel].callback = channel_handler;
		channel_handlers[channel].context = context;
		rDIER(timer) |= dier_setbits << shifts;
		irqrestore(flags);
	}

	return rv;
}

int io_timer_set_enable(bool state, io_timer_channel_mode_t mode, io_timer_channel_allocation_t masks)
{

	struct action_cache_t {
		uint32_t ccer_clearbits;
		uint32_t ccer_setbits;
		uint32_t dier_setbits;
		uint32_t dier_clearbits;
		uint32_t base;
		uint32_t gpio[MAX_CHANNELS_PER_TIMER];
	} action_cache[MAX_IO_TIMERS];
	memset(action_cache, 0, sizeof(action_cache));

	uint32_t dier_bit = state ? GTIM_DIER_CC1IE : 0;
	uint32_t ccer_bit =  state ? GTIM_CCER_CC1E : 0;

	switch (mode) {
	case IOTimerChanMode_NotUsed:
	case IOTimerChanMode_PWMOut:
		dier_bit = 0;
		break;

	case IOTimerChanMode_PWMIn:
	case IOTimerChanMode_Capture:
		break;

	default:
		return -EINVAL;
	}

	/* Was the request for all channels in this mode ?*/

	if (masks == IO_TIMER_ALL_MODES_CHANNELS) {

		/* Yes - we provide them */

		masks = channel_allocations[mode];

	} else {

		/* No - caller provided mask */

		/* Only allow the channels in that mode to be affected */

		masks &= channel_allocations[mode];

	}

	/* Pre calculate all the changes */

	for (int chan_index = 0; masks != 0 && chan_index < MAX_TIMER_IO_CHANNELS; chan_index++) {
		if (masks & (1 << chan_index)) {
			masks &= ~(1 << chan_index);
			uint32_t shifts = timer_io_channels[chan_index].timer_channel - 1;
			uint32_t timer = channels_timer(chan_index);
			action_cache[timer].base  = io_timers[timer].base;
			action_cache[timer].ccer_clearbits |= GTIM_CCER_CC1E << (shifts * CCER_C1_NUM_BITS);
			action_cache[timer].ccer_setbits   |= ccer_bit  << (shifts * CCER_C1_NUM_BITS);
			action_cache[timer].dier_clearbits |= GTIM_DIER_CC1IE  << shifts;
			action_cache[timer].dier_setbits   |= dier_bit << shifts;

			if (state && mode == IOTimerChanMode_PWMOut) {
				action_cache[timer].gpio[shifts] = timer_io_channels[chan_index].gpio_out;
			}
		}
	}

	irqstate_t flags = irqsave();

	for (unsigned actions = 0; actions < arraySize(action_cache) && action_cache[actions].base != 0 ; actions++) {
		uint32_t rvalue = _REG32(action_cache[actions].base, STM32_GTIM_CCER_OFFSET);
		rvalue &= ~action_cache[actions].ccer_clearbits;
		rvalue |= action_cache[actions].ccer_setbits;
		_REG32(action_cache[actions].base, STM32_GTIM_CCER_OFFSET) = rvalue;
		uint32_t after = rvalue & (GTIM_CCER_CC1E | GTIM_CCER_CC2E | GTIM_CCER_CC3E | GTIM_CCER_CC4E);

		rvalue = _REG32(action_cache[actions].base, STM32_GTIM_DIER_OFFSET);
		rvalue &= ~action_cache[actions].dier_clearbits;
		rvalue |= action_cache[actions].dier_setbits;
		_REG32(action_cache[actions].base, STM32_GTIM_DIER_OFFSET) = rvalue;


		/* Any On ?*/

		if (after != 0) {

			/* force an update to preload all registers */
			rEGR(actions) = GTIM_EGR_UG;

			for (unsigned chan = 0; chan < arraySize(action_cache[actions].gpio); chan++) {
				if (action_cache[actions].gpio[chan]) {
					stm32_configgpio(action_cache[actions].gpio[chan]);
					action_cache[actions].gpio[chan] = 0;
				}
			}

			/* arm requires the timer be enabled */
			rCR1(actions) |= GTIM_CR1_CEN | GTIM_CR1_ARPE;

		} else 	{

			rCR1(actions) = 0;
		}
	}

	irqrestore(flags);

	return 0;
}

int io_timer_set_ccr(unsigned channel, uint16_t value)
{
	int rv = io_timer_validate_channel_index(channel);

	if (rv == 0) {
		if (io_timer_get_channel_mode(channel) != IOTimerChanMode_PWMOut) {

			rv = -EIO;

		} else {

			/* configure the channel */

			if (value > 0) {
				value--;
			}

			REG(channels_timer(channel), timer_io_channels[channel].ccr_offset) = value;
		}
	}

	return rv;
}

uint16_t io_channel_get_ccr(unsigned channel)
{
	uint16_t value = 0;

	if (io_timer_validate_channel_index(channel) == 0 &&
	    io_timer_get_channel_mode(channel) == IOTimerChanMode_PWMOut) {
		value = REG(channels_timer(channel), timer_io_channels[channel].ccr_offset) + 1;
	}

	return value;
}

uint32_t io_timer_get_group(unsigned timer)
{
	return get_timer_channels(timer);

}

/*
  trigger pwm output for all PWMOut channels. This is used in oneshot
  mode after the ccr values have been updated
 */
int io_timer_pwm_trigger(uint32_t channel_mask)
{
	int pwm_channels = io_timer_get_mode_channels(IOTimerChanMode_PWMOut) & channel_mask;
	uint32_t last_timer = 0xFFFFFFFF;

	for (int chan_index = 0; chan_index < MAX_TIMER_IO_CHANNELS; chan_index++) {
		if (pwm_channels & (1 << chan_index)) {
			uint32_t timer = channels_timer(chan_index);

			if (timer != last_timer) {
				rEGR(timer) = GTIM_EGR_UG;
				last_timer = timer;
			}
		}
	}

	return OK;
}

