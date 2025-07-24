/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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
 * @file io_timer.c
 *
 * Servo driver supporting PWM servos connected to RP2040 PWM blocks.
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
#include <stdio.h>

#include <arch/board/board.h>
#include <drivers/drv_pwm_output.h>

#include <px4_arch/io_timer.h>

// RP2040 PWM block has 8 slices (timers) and each has 2 independent outputs (channels) A and B.
// All the channels can output pwm. However, only channel B can be used for input where the timer
// will be working in edge sensitive mode or level sensitive mode. Be careful when choosing the
// timer and channel for input capture. Each timer has an independent 8.4 fractional divider.
// Each timer also has double buffered wrap (rTOP) and level (rCC) registers so the value can
// change while PWM is running.

#ifdef CONFIG_BOARD_PWM_FREQ
#define BOARD_PWM_FREQ CONFIG_BOARD_PWM_FREQ
#endif

#if !defined(BOARD_PWM_FREQ)
#define BOARD_PWM_FREQ 1000000
#endif

#if !defined(BOARD_ONESHOT_FREQ)
#define BOARD_ONESHOT_FREQ 8000000
#endif

#define TIM_SRC_CLOCK_FREQ 125000000

#define MAX_CHANNELS_PER_TIMER 2

#define _REG(_addr)	(*(volatile uint32_t *)(_addr))
#define _REG32(_base, _reg)	(*(volatile uint32_t *)(_base + _reg))
#define REG(_tmr, _reg)		_REG32(io_timers[_tmr].base, _reg)

// Register offsets
#define RP2040_PWM_CSR_OFFSET		0x00	// Control and status register
#define RP2040_PWM_DIV_OFFSET		0x04	// Clock divide register
#define RP2040_PWM_CTR_OFFSET		0x08	// Direct access to the PWM counter
#define RP2040_PWM_CCR_OFFSET		0x0c	// Counter compare values
#define RP2040_PWM_TOP_OFFSET		0x10	// Counter wrap value
#define RP2040_PWM_EN_OFFSET		0xa0	// This register aliases the CSR_EN bits for all channels
#define RP2040_PWM_INTR_OFFSET		0xa4	// Raw Interrupts
#define RP2040_PWM_INTE_OFFSET		0xa8	// Interrupt Enable
#define RP2040_PWM_INTF_OFFSET		0xac	// Interrupt Force
#define RP2040_PWM_INTS_OFFSET		0xb0	// Interrupt status after masking & forcing

/* Timer register accessors */
#define rCSR(_tmr)		REG(_tmr,RP2040_PWM_CSR_OFFSET)
#define rDIV(_tmr)		REG(_tmr,RP2040_PWM_DIV_OFFSET)
#define rCTR(_tmr)		REG(_tmr,RP2040_PWM_CTR_OFFSET)
#define rCCR(_tmr)		REG(_tmr,RP2040_PWM_CCR_OFFSET)
#define rTOP(_tmr)		REG(_tmr,RP2040_PWM_TOP_OFFSET)
#define rEN			_REG32(RP2040_PWM_BASE,RP2040_PWM_EN_OFFSET)
#define rINTR			_REG32(RP2040_PWM_BASE,RP2040_PWM_INTR_OFFSET)
#define rINTE			_REG32(RP2040_PWM_BASE,RP2040_PWM_INTE_OFFSET)
#define rINTF			_REG32(RP2040_PWM_BASE,RP2040_PWM_INTF_OFFSET)
#define rINTS			_REG32(RP2040_PWM_BASE,RP2040_PWM_INTS_OFFSET)

//					 				  NotUsed   PWMOut  PWMIn Capture OneShot Trigger
io_timer_channel_allocation_t channel_allocations[IOTimerChanModeSize] = { UINT16_MAX,   0,  0,  0, 0, 0 };

typedef uint8_t io_timer_allocation_t; /* big enough to hold MAX_IO_TIMERS */

static io_timer_allocation_t once = 0;	// Used to trace whether the timer is initialized or not

typedef struct channel_stat_t {
	uint32_t 			isr_cout;
	uint32_t 			overflows;
} channel_stat_t;

// static channel_stat_t io_timer_channel_stats[MAX_TIMER_IO_CHANNELS];

static struct channel_handler_entry {
	channel_handler_t callback;
	void			  *context;
} channel_handlers[MAX_TIMER_IO_CHANNELS];


static int io_timer_handler(int irq, void *context, void *arg)//uint16_t timer_index)
{
	/* Read the count at the time of the interrupt */

	// uint16_t count = rCNT(timer_index);

	// /* Read the HRT at the time of the interrupt */

	// hrt_abstime now = hrt_absolute_time();

	// const io_timers_t *tmr = &io_timers[timer_index];

	// /* What is pending */

	// uint32_t statusr = rSTATUS(timer_index);

	// /* Acknowledge all that are pending */

	// rSTATUS(timer_index) = 0;

	// /* Iterate over the timer_io_channels table */

	// uint32_t first_channel_index = io_timers_channel_mapping.element[timer_index].first_channel_index;
	// uint32_t last_channel_index = first_channel_index + io_timers_channel_mapping.element[timer_index].channel_count;

	// for (unsigned chan_index = first_channel_index; chan_index < last_channel_index; chan_index++) {

	// 	uint16_t chan = 1 << chan_index;

	// 	if (statusr & chan) {

	// 		io_timer_channel_stats[chan_index].isr_cout++;

	// 		/* Call the client to read the rCnV etc and clear the CHnF */

	// 		if (channel_handlers[chan_index].callback) {
	// 			channel_handlers[chan_index].callback(channel_handlers[chan_index].context, tmr,
	// 							      chan_index, &timer_io_channels[chan_index],
	// 							      now, count, _REG32(tmr->base, KINETIS_FTM_CV_OFFSET(chan_index)));
	// 		}
	// 	}

	// 	/* Did it set again during call out ?*/

	// 	if (rSTATUS(timer_index) & chan) {

	// 		/* Error we has a second edge before we serviced the fist */

	// 		io_timer_channel_stats[chan_index].overflows++;
	// 	}
	// }

	return 0;
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
	static uint32_t channels_cache[MAX_IO_TIMERS] = {0};

	if (validate_timer_index(timer) < 0) {
		return channels;

	} else {
		if (channels_cache[timer] == 0) {

			/* Gather the channel bits that belong to the timer */

			uint32_t first_channel_index = io_timers_channel_mapping.element[timer].first_channel_index;
			uint32_t last_channel_index = first_channel_index + io_timers_channel_mapping.element[timer].channel_count;

			for (unsigned chan_index = first_channel_index; chan_index < last_channel_index; chan_index++) {
				channels |= 1 << chan_index;
			}

			/* cache them */

			channels_cache[timer] = channels;
		}
	}

	return channels_cache[timer];
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

uint32_t io_timer_channel_get_gpio_output(unsigned channel)
{
	if (io_timer_validate_channel_index(channel) != 0) {
		return 0;
	}

	return timer_io_channels[channel].gpio_out;
}

uint32_t io_timer_channel_get_as_pwm_input(unsigned channel)
{
	if (io_timer_validate_channel_index(channel) != 0) {
		return 0;
	}

	return timer_io_channels[channel].gpio_in;
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

static int reallocate_channel_resources(uint32_t channels, io_timer_channel_mode_t mode,
					io_timer_channel_mode_t new_mode)
{
	/* If caller mode is not based on current setting adjust it */

	if ((channels & channel_allocations[IOTimerChanMode_NotUsed]) == channels) {
		mode = IOTimerChanMode_NotUsed;
	}

	/* Remove old set of channels from original */

	channel_allocations[mode] &= ~channels;

	/* Will this change ?*/

	uint32_t before = channel_allocations[new_mode] & channels;

	/* add in the new set */

	channel_allocations[new_mode] |= channels;

	/* Indicate a mode change */

	return before ^ channels;
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
	// RP2040 has a buffer for rTOP, so there shouldn't be any need to turn the timer off to change rTOP value
	rTOP(timer) = (BOARD_PWM_FREQ / rate) - 1;

	return 0;
}

static inline uint32_t freq2div(uint32_t freq)
{
	return (TIM_SRC_CLOCK_FREQ << 4) / freq;
}

static inline void io_timer_set_oneshot_mode(unsigned timer)
{
	/* Ideally, we would want per channel One pulse mode in HW
	 * Alas OPE stops the Timer not the channel
	 * todo:We can do this in an ISR later
	 * But since we do not have that
	 * We try to get the longest rate we can.
	 *  On 16 bit timers this is 8.1 Ms.
	 */

	rTOP(timer) = 0xffff;
	rDIV(timer) = freq2div(BOARD_ONESHOT_FREQ);
}

static inline void io_timer_set_PWM_mode(unsigned timer)
{
	rDIV(timer) = freq2div(BOARD_PWM_FREQ);
}

void io_timer_trigger(void)
{
	// This function is probably not important for RP2040 as the buffered registers are updated automatically on the timer wrap.

	// int oneshots = io_timer_get_mode_channels(IOTimerChanMode_OneShot);
	// uint32_t action_cache[MAX_IO_TIMERS] = {0};
	// int actions = 0;

	// /* Pre-calculate the list of timers to Trigger */

	// for (int timer = 0; timer < MAX_IO_TIMERS; timer++) {
	// 	if (validate_timer_index(timer) == 0) {
	// 		int channels = get_timer_channels(timer);

	// 		if (oneshots & channels) {
	// 			action_cache[actions++] = io_timers[timer].base;
	// 		}
	// 	}
	// }

	// /* Now do them all wit the shortest delay in between */

	// irqstate_t flags = px4_enter_critical_section();

	// for (actions = 0; actions < MAX_IO_TIMERS && action_cache[actions] != 0; actions++) {
	// 	_REG32(action_cache[actions], KINETIS_FTM_SYNC_OFFSET) |= FTM_SYNC;
	// }

	// px4_leave_critical_section(flags);
}

int io_timer_init_timer(unsigned timer)
{
	/* Do this only once per timer */

	int rv = is_timer_uninitalized(timer);

	if (rv == 0) {

		irqstate_t flags = px4_enter_critical_section();

		set_timer_initalized(timer);

		/* disable and configure the timer */

		rCSR(timer) = 0;
		rCTR(timer) = 0;

		/* enable the timer */

		io_timer_set_PWM_mode(timer);

		/*
		 * Note we do the Standard PWM Out init here
		 * default to updating at 50Hz
		 */

		timer_set_rate(timer, 50);

		/*
		 * Note that the timer is left disabled with IRQ subs installed
		 * and active but DEIR bits are not set.
		 */
		xcpt_t handler = io_timer_handler;

		if (handler) {
			irq_attach(RP2040_PWM_IRQ_WRAP, handler, NULL);
			up_enable_irq(RP2040_PWM_IRQ_WRAP);
		}

		px4_leave_critical_section(flags);
	}

	return rv;
}


int io_timer_set_rate(unsigned timer, unsigned rate)
{
	int rv = EBUSY;

	/* Get the channel bits that belong to the timer */

	uint32_t channels = get_timer_channels(timer);

	/* Check that all channels are either in PWM or Oneshot */

	if ((channels & (channel_allocations[IOTimerChanMode_PWMOut] |
			 channel_allocations[IOTimerChanMode_OneShot] |
			 channel_allocations[IOTimerChanMode_NotUsed])) ==
	    channels) {

		/* Change only a timer that is owned by pwm or one shot */

		/* Request to use OneShot ?*/

		if (rate == 0) {

			/* Request to use OneShot
			 *
			 * We are here because ALL these channels were either PWM or Oneshot
			 * Now they need to be Oneshot
			 */

			/* Did the allocation change */
			if (reallocate_channel_resources(channels, IOTimerChanMode_PWMOut, IOTimerChanMode_OneShot)) {
				io_timer_set_oneshot_mode(timer);
			}

		} else {

			/* Request to use PWM
			 *
			 * We are here because  ALL these channels were either PWM or Oneshot
			 * Now they need to be PWM
			 */

			if (reallocate_channel_resources(channels, IOTimerChanMode_OneShot, IOTimerChanMode_PWMOut)) {
				io_timer_set_PWM_mode(timer);
			}

			timer_set_rate(timer, rate);
		}

		rv = OK;
	}

	return rv;
}

int io_timer_channel_init(unsigned channel, io_timer_channel_mode_t mode,
			  channel_handler_t channel_handler, void *context)
{
	// RP2040 doesn't have any specific configuration for each channel except the CCR value.
	// And RP2040 doesn't really have the hardware capability for input capture.

	/* figure out the GPIO config first */
	switch (mode) {

	case IOTimerChanMode_OneShot:
	case IOTimerChanMode_PWMOut:
	case IOTimerChanMode_Trigger:
	case IOTimerChanMode_NotUsed:
		break;

	default:
		return -EINVAL;
	}

	int rv = allocate_channel(channel, mode);

	/* Valid channel should now be reserved in new mode */

	if (rv >= 0) {

		/* Blindly try to initialize the timer - it will only do it once */

		io_timer_init_timer(channels_timer(channel));

		/* configure the channel */ // Nothing to configure here really.

		// uint32_t chan = timer_io_channels[channel].timer_channel - 1;

		channel_handlers[channel].callback = channel_handler;
		channel_handlers[channel].context = context;
	}

	return rv;
}

int io_timer_set_enable(bool state, io_timer_channel_mode_t mode, io_timer_channel_allocation_t masks)
{
	typedef struct action_cache_rp_t {
		// uint32_t cnsc_offset;
		// uint32_t cnsc_value;
		uint32_t gpio;
	} action_cache_rp_t;
	struct action_cache_t {
		uint32_t base;
		uint32_t index;
		action_cache_rp_t cnsc[MAX_CHANNELS_PER_TIMER];
	} action_cache[MAX_IO_TIMERS];

	memset(action_cache, 0, sizeof(action_cache));

	// uint32_t bits =  state ? CnSC_PWMOUT_INIT : 0;

	switch (mode) {
	case IOTimerChanMode_NotUsed:
	case IOTimerChanMode_PWMOut:
	case IOTimerChanMode_OneShot:
	case IOTimerChanMode_Trigger:
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

			if (io_timer_validate_channel_index(chan_index) == 0) {
				// uint32_t chan = timer_io_channels[chan_index].timer_channel - 1;
				uint32_t timer = channels_timer(chan_index);
				action_cache[timer].base  = io_timers[timer].base;
				// action_cache[timer].cnsc[action_cache[timer].index].cnsc_offset = io_timers[timer].base + KINETIS_FTM_CSC_OFFSET(chan);
				// action_cache[timer].cnsc[action_cache[timer].index].cnsc_value = bits;

				if ((state &&
				     (mode == IOTimerChanMode_PWMOut ||
				      mode == IOTimerChanMode_OneShot ||
				      mode == IOTimerChanMode_Trigger))) {
					action_cache[timer].cnsc[action_cache[timer].index].gpio = timer_io_channels[chan_index].gpio_out;
				}

				action_cache[timer].index++;
			}
		}
	}

	irqstate_t flags = px4_enter_critical_section();


	for (unsigned actions = 0; actions < arraySize(action_cache); actions++) {
		// uint32_t any_on = 0;

		if (action_cache[actions].base != 0) {
			for (unsigned int index = 0; index < action_cache[actions].index; index++) {

				if (action_cache[actions].cnsc[index].gpio) {
					px4_arch_configgpio(action_cache[actions].cnsc[index].gpio);
				}

				// _REG(action_cache[actions].cnsc[index].cnsc_offset) = action_cache[actions].cnsc[index].cnsc_value;
				// any_on |= action_cache[actions].cnsc[index].cnsc_value;
			}

			/* Any On ?*/

			/* Assume not */
			// uint32_t regval = _REG32(action_cache[actions].base, KINETIS_FTM_SC_OFFSET);
			// regval &= ~(FTM_SC_CLKS_MASK);

			// if (any_on != 0) {

			// 	/* force an update to preload all registers */
			// 	_REG32(action_cache[actions].base, KINETIS_FTM_SYNC_OFFSET) |= FTM_SYNC;

			// 	/* arm requires the timer be enabled */
			// 	regval |= (FTM_SC_CLKS_EXTCLK);
			// }

			_REG32(action_cache[actions].base, RP2040_PWM_CSR_OFFSET) |= state;
		}
	}

	px4_leave_critical_section(flags);

	return 0;
}

int io_timer_set_ccr(unsigned channel, uint16_t value)
{
	int rv = io_timer_validate_channel_index(channel);
	int mode = io_timer_get_channel_mode(channel);

	if (rv == 0) {
		if ((mode != IOTimerChanMode_PWMOut) &&
		    (mode != IOTimerChanMode_OneShot) &&
		    (mode != IOTimerChanMode_Trigger)) {

			rv = -EIO;

		} else {

			/* configure the channel */
			int regVal = rCCR(channels_timer(channel));
			regVal &= ~(0xffff << (timer_io_channels[channel].timer_channel - 1) * 16);
			regVal |= value << (timer_io_channels[channel].timer_channel - 1) * 16;
			rCCR(channels_timer(channel)) = regVal;
		}
	}

	return rv;
}

uint16_t io_channel_get_ccr(unsigned channel)
{
	uint16_t value = 0;

	if (io_timer_validate_channel_index(channel) == 0) {
		int mode = io_timer_get_channel_mode(channel);

		if ((mode == IOTimerChanMode_PWMOut) ||
		    (mode == IOTimerChanMode_OneShot) ||
		    (mode == IOTimerChanMode_Trigger)) {
			value = rCCR(channels_timer(channel)) >> (timer_io_channels[channel].timer_channel - 1) * 16;
		}
	}

	return value;
}

uint32_t io_timer_get_group(unsigned timer)
{
	return get_timer_channels(timer);

}
