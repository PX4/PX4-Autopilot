/****************************************************************************
 *
 *   Copyright (C) 2017 PX4 Development Team. All rights reserved.
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
 * Servo driver supporting PWM servos connected to S32K3XX EMIOS blocks.
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

#include "s32k3xx_pin.h"
#include "hardware/s32k3xx_emios.h"


static int io_timer_handler0(int irq, void *context, void *arg);
static int io_timer_handler1(int irq, void *context, void *arg);
static int io_timer_handler2(int irq, void *context, void *arg);
static int io_timer_handler3(int irq, void *context, void *arg);
static int io_timer_handler4(int irq, void *context, void *arg);
static int io_timer_handler5(int irq, void *context, void *arg);
static int io_timer_handler6(int irq, void *context, void *arg);
static int io_timer_handler7(int irq, void *context, void *arg);

/* The FTM pre-scalers are limited to divide by 2^n where n={1-7}.
 *
 * All EMIOS blocks have their clock sources set to the core_clk
 * which should generate an 160 / 80 = 2 MHz clock.
 */

#define BOARD_PWM_PRESCALER 20

#if !defined(BOARD_PWM_FREQ)
#define BOARD_PWM_FREQ 160000000 / BOARD_PWM_PRESCALER
#endif

#if !defined(BOARD_ONESHOT_FREQ)
#define BOARD_ONESHOT_FREQ 160000000
#endif

#define FTM_SRC_CLOCK_FREQ 160000000

#define MAX_CHANNELS_PER_TIMER 24

#define _REG(_addr)		(*(volatile uint32_t *)(_addr))
#define _REG32(_base, _reg)	(*(volatile uint32_t *)(_base + _reg))
#define REG(_tmr, _reg)		_REG32(io_timers[_tmr].base, _reg)


/* Timer register accessors */

#define rMCR(_tmr)         REG(_tmr, S32K3XX_EMIOS_MCR_OFFSET)
#define rGLAG(_tmr)        REG(_tmr, S32K3XX_EMIOS_GFLAG_OFFSET)
#define rOUDIS(_tmr)       REG(_tmr, S32K3XX_EMIOS_OUDIS_OFFSET)
#define rUCDIS(_tmr)       REG(_tmr, S32K3XX_EMIOS_UCDIS_OFFSET)
#define rA(_tmr,c)         REG(_tmr, S32K3XX_EMIOS_A_OFFSET(c))
#define rB(_tmr,c)         REG(_tmr, S32K3XX_EMIOS_B_OFFSET(c))
#define rCNT(_tmr,c)       REG(_tmr, S32K3XX_EMIOS_CNT_OFFSET(c))
#define rC(_tmr,c)         REG(_tmr, S32K3XX_EMIOS_C_OFFSET(c))
#define rS(_tmr,c)         REG(_tmr, S32K3XX_EMIOS_S_OFFSET(c))
#define rALTA(_tmr,c)      REG(_tmr, S32K3XX_EMIOS_ALTA_OFFSET(c))
#define rC2(_tmr,c)        REG(_tmr, S32K3XX_EMIOS_C2_OFFSET(c))

//												 				  NotUsed   PWMOut  PWMIn Capture OneShot Trigger Dshot LED PPS Other
io_timer_channel_allocation_t channel_allocations[IOTimerChanModeSize] = { UINT16_MAX,   0,  0,  0, 0, 0, 0, 0, 0, 0 };

typedef uint8_t io_timer_allocation_t; /* big enough to hold MAX_IO_TIMERS */

io_timer_channel_allocation_t timer_allocations[MAX_IO_TIMERS] = { };

typedef struct channel_stat_t {
	uint32_t 			isr_cout;
	uint32_t 			overflows;
} channel_stat_t;

//static channel_stat_t io_timer_channel_stats[MAX_TIMER_IO_CHANNELS];

static struct channel_handler_entry {
	channel_handler_t callback;
	void			  *context;
} channel_handlers[MAX_TIMER_IO_CHANNELS];


static int io_timer_handler(uint16_t timer_index)
{
#if 0
	/* Read the count at the time of the interrupt */

	uint16_t count = rCNT(timer_index);

	/* Read the HRT at the time of the interrupt */

	hrt_abstime now = hrt_absolute_time();

	const io_timers_t *tmr = &io_timers[timer_index];

	/* What is pending */

	uint32_t statusr = rSTATUS(timer_index);

	/* Acknowledge all that are pending */

	rSTATUS(timer_index) = 0;

	/* Iterate over the timer_io_channels table */

	uint32_t first_channel_index = io_timers_channel_mapping.element[timer_index].first_channel_index;
	uint32_t last_channel_index = first_channel_index + io_timers_channel_mapping.element[timer_index].channel_count;

	for (unsigned chan_index = first_channel_index; chan_index < last_channel_index; chan_index++) {

		uint16_t chan = 1 << chan_index;

		if (statusr & chan) {

			io_timer_channel_stats[chan_index].isr_cout++;

			/* Call the client to read the rCnV etc and clear the CHnF */

			if (channel_handlers[chan_index].callback) {
				channel_handlers[chan_index].callback(channel_handlers[chan_index].context, tmr,
								      chan_index, &timer_io_channels[chan_index],
								      now, count, _REG32(tmr->base, S32K1XX_FTM_CNV_OFFSET(chan_index)));
			}
		}

		/* Did it set again during call out? */

		if (rSTATUS(timer_index) & chan) {

			/* Error - we had a second edge before we serviced the first */

			io_timer_channel_stats[chan_index].overflows++;
		}
	}

#endif
	return 0;
}

int io_timer_handler0(int irq, void *context, void *arg)
{
	return io_timer_handler(0);
}

int io_timer_handler1(int irq, void *context, void *arg)
{
	return io_timer_handler(1);
}

int io_timer_handler2(int irq, void *context, void *arg)
{
	return io_timer_handler(2);
}

int io_timer_handler3(int irq, void *context, void *arg)
{
	return io_timer_handler(3);
}

int io_timer_handler4(int irq, void *context, void *arg)
{
	return io_timer_handler(4);
}
int io_timer_handler5(int irq, void *context, void *arg)
{
	return io_timer_handler(5);
}
int io_timer_handler6(int irq, void *context, void *arg)
{
	return io_timer_handler(6);
}
int io_timer_handler7(int irq, void *context, void *arg)
{
	return io_timer_handler(7);
}

static inline int validate_timer_index(unsigned timer)
{
	return (timer < MAX_IO_TIMERS && io_timers[timer].base != 0) ? 0 : -EINVAL;
}

int io_timer_allocate_timer(unsigned timer, io_timer_channel_mode_t mode)
{
	int ret = -EINVAL;

	if (validate_timer_index(timer) == 0) {
		// check if timer is unused or already set to the mode we want
		if (timer_allocations[timer] == IOTimerChanMode_NotUsed || timer_allocations[timer] == mode) {
			timer_allocations[timer] = mode;
			ret = 0;

		} else {
			ret = -EBUSY;
		}
	}

	return ret;
}

int io_timer_unallocate_timer(unsigned timer)
{
	int ret = -EINVAL;

	if (validate_timer_index(timer) == 0) {
		timer_allocations[timer] = IOTimerChanMode_NotUsed;
		ret = 0;
	}

	return ret;
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

	return (timer_io_channels[channel].gpio_out & ~(_PIN_MODE_MASK | _PIN_OPTIONS_MASK)) | GPIO_HIGHDRIVE;
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

__EXPORT int io_timer_allocate_channel(unsigned channel, io_timer_channel_mode_t mode)
{
	irqstate_t flags = px4_enter_critical_section();
	int existing_mode = io_timer_get_channel_mode(channel);
	int ret = -EBUSY;

	if (existing_mode <= IOTimerChanMode_NotUsed || existing_mode == mode) {
		io_timer_channel_allocation_t bit = 1 << channel;
		channel_allocations[IOTimerChanMode_NotUsed] &= ~bit;
		channel_allocations[mode] |= bit;
		ret = 0;
	}

	px4_leave_critical_section(flags);

	return ret;
}


int io_timer_unallocate_channel(unsigned channel)
{
	int mode = io_timer_get_channel_mode(channel);

	if (mode > IOTimerChanMode_NotUsed) {
		io_timer_channel_allocation_t bit = 1 << channel;
		channel_allocations[mode] &= ~bit;
		channel_allocations[IOTimerChanMode_NotUsed] |= bit;
	}

	return mode;
}

static int allocate_channel(unsigned channel, io_timer_channel_mode_t mode)
{
	int rv = -EINVAL;

	if (mode != IOTimerChanMode_NotUsed) {
		rv = io_timer_validate_channel_index(channel);

		if (rv == 0) {
			rv = io_timer_allocate_channel(channel, mode);
		}
	}

	return rv;
}

static int timer_set_rate(unsigned channel, unsigned rate)
{

	irqstate_t flags = px4_enter_critical_section();

	int prediv = 1;

	if (rate < 500) {
		rC(channels_timer(channel), channel) = (rC(channels_timer(channel), channel) & ~EMIOS_C_UCPRE_MASK)
						       | EMIOS_C_UCPRE_DIV4;
		prediv = 4;

	} else {
		rC(channels_timer(channel), channel) = (rC(channels_timer(channel), channel) & ~EMIOS_C_UCPRE_MASK)
						       | EMIOS_C_UCPRE_DIV1;
	}


	/* configure the timer to update at the desired rate */
	rB(channels_timer(channel), channel) = ((BOARD_PWM_FREQ / prediv) / rate) - 1;

	px4_leave_critical_section(flags);

	return 0;
}

static inline uint32_t div2psc(int div)
{
	return 31 - __builtin_clz(div);
}

static inline void io_timer_set_oneshot_mode(unsigned timer)
{
	/* Ideally, we would want per channel One pulse mode in HW
	 * Alas The FTM anly support onshot capture)
	 * todo:We can do this in an ISR later
	 * But since we do not have that
	 * We try to get the longest rate we can.
	 *  On 16 bit timers this is 4.68 Ms.
	 */

	irqstate_t flags = px4_enter_critical_section();
	/*	rSC(timer) &= ~(FTM_SC_CLKS_MASK | FTM_SC_PS_MASK);
		rMOD(timer) = 0xffff;
		rSC(timer) |= (FTM_SC_CLKS_EXTCLK | div2psc(FTM_SRC_CLOCK_FREQ / BOARD_ONESHOT_FREQ));*/
	px4_leave_critical_section(flags);
}

static inline void io_timer_set_PWM_mode(unsigned channel)
{
	irqstate_t flags = px4_enter_critical_section();

	px4_leave_critical_section(flags);
}

void io_timer_trigger(unsigned channel_mask)
{
	int oneshots = io_timer_get_mode_channels(IOTimerChanMode_OneShot) & channel_mask;
	uint32_t action_cache[MAX_IO_TIMERS] = {0};
	int actions = 0;

	/* Pre-calculate the list of timers to Trigger */

	for (int timer = 0; timer < MAX_IO_TIMERS; timer++) {
		if (validate_timer_index(timer) == 0) {
			int channels = get_timer_channels(timer);

			if (oneshots & channels) {
				action_cache[actions++] = io_timers[timer].base;
			}
		}
	}

	/* Now do them all wit the shortest delay in between */

	irqstate_t flags = px4_enter_critical_section();

	for (actions = 0; actions < MAX_IO_TIMERS && action_cache[actions] != 0; actions++) {
		//_REG32(action_cache[actions], S32K1XX_FTM_SYNC_OFFSET) |= FTM_SYNC;
	}

	px4_leave_critical_section(flags);
}

int io_timer_init_timer(unsigned timer, io_timer_channel_mode_t mode)
{
	if (validate_timer_index(timer) != 0) {
		return -EINVAL;
	}

	io_timer_channel_mode_t previous_mode = timer_allocations[timer];
	int rv = io_timer_allocate_timer(timer, mode);

	/* Do this only once per timer */
	if (rv == 0 && previous_mode == IOTimerChanMode_NotUsed) {

		irqstate_t flags = px4_enter_critical_section();


		/* disable and configure the timer */

		rMCR(timer)     &= ~EMIOS_MCR_GPREN;

		/* Set EMIOS prescaler div by 1 clk_freq is 160Mhz */

		rMCR(timer)   |= EMIOS_MCR_GTBE;

		rMCR(timer)   |= EMIOS_MCR_GPRE(BOARD_PWM_PRESCALER - 1);

		/* Set to run in debug mode */

		rMCR(timer)   |= EMIOS_MCR_FRZ;

		/* Chn23 acts as EMIOS Global counter bus */

		rC(timer, 23) |= EMIOS_C_FREN;

		rC(timer, 23) |= EMIOS_C_UCPRE_DIV1;

		/* Default period hence clk_freq is 160Mhz */

		rA(timer, 23) = EMIOS_A(1);

		/* Offset at start */

		rCNT(timer, 23) = 0;

		/* Master mode */

		rC(timer, 23) |= EMIOS_C_MODE_MC_UPCNT_CLRSTRT_INTCLK;

		/* Prescaler enable */

		rC(timer, 23) |= EMIOS_C_UCPREN;

		/* Start EMIOS */

		rMCR(timer)   |= EMIOS_MCR_GPREN;
		/*
		 * Note we do the Standard PWM Out init here
		 * default to updating at 50Hz
		 */

		timer_set_rate(timer, 50);

		/*
		 * Note that the timer is left disabled with IRQ subs installed
		 * and active but DEIR bits are not set.
		 */
		xcpt_t handler;

		switch (timer) {
		case 0: handler = io_timer_handler0; break;

		case 1: handler = io_timer_handler1; break;

		case 2: handler = io_timer_handler2; break;

		case 3: handler = io_timer_handler3; break;

		case 4: handler = io_timer_handler4; break;

		case 5: handler = io_timer_handler5; break;

		case 6: handler = io_timer_handler6; break;

		case 7: handler = io_timer_handler7; break;

		default:
			handler = NULL;
			rv = -EINVAL;
			break;
		}

		if (handler) {
			irq_attach(io_timers[timer].vectorno_0_3, handler, NULL);
			up_enable_irq(io_timers[timer].vectorno_0_3);
			irq_attach(io_timers[timer].vectorno_4_7, handler, NULL);
			up_enable_irq(io_timers[timer].vectorno_4_7);
			irq_attach(io_timers[timer].vectorno_8_11, handler, NULL);
			up_enable_irq(io_timers[timer].vectorno_8_11);
			irq_attach(io_timers[timer].vectorno_12_15, handler, NULL);
			up_enable_irq(io_timers[timer].vectorno_12_15);
			irq_attach(io_timers[timer].vectorno_16_19, handler, NULL);
			up_enable_irq(io_timers[timer].vectorno_16_19);
			irq_attach(io_timers[timer].vectorno_20_23, handler, NULL);
			up_enable_irq(io_timers[timer].vectorno_20_23);
		}

		px4_leave_critical_section(flags);
	}

	return rv;
}


int io_timer_set_pwm_rate(unsigned timer, unsigned rate)
{
	/* Change only a timer that is owned by pwm or one shot */
	if (timer_allocations[timer] != IOTimerChanMode_PWMOut && timer_allocations[timer] != IOTimerChanMode_OneShot) {
		return -EINVAL;
	}

	/* Get the channel bits that belong to the timer and are in PWM or OneShot mode */

	uint32_t channels = get_timer_channels(timer) & (io_timer_get_mode_channels(IOTimerChanMode_OneShot) |
			    io_timer_get_mode_channels(IOTimerChanMode_PWMOut));

	/* Request to use OneShot ?*/

	if (PWM_RATE_ONESHOT == rate) {

		/* Request to use OneShot
		 */
		int changed_channels = reallocate_channel_resources(channels, IOTimerChanMode_PWMOut, IOTimerChanMode_OneShot);

		/* Did the allocation change */
		if (changed_channels) {
			io_timer_set_oneshot_mode(timer);
		}

	} else {

		/* Request to use PWM
		 */
		int changed_channels = reallocate_channel_resources(channels, IOTimerChanMode_OneShot, IOTimerChanMode_PWMOut);

		if (changed_channels) {
			io_timer_set_PWM_mode(timer);
		}

		timer_set_rate(timer, rate);
	}

	return OK;
}

int io_timer_channel_init(unsigned channel, io_timer_channel_mode_t mode,
			  channel_handler_t channel_handler, void *context)
{
	if (io_timer_validate_channel_index(channel) != 0) {
		return -EINVAL;
	}

	uint32_t gpio = timer_io_channels[channel].gpio_in;

	uint32_t clearbits = 0; //FIXME
	uint32_t setbits = 0;
	(void)setbits;
	(void)clearbits;

	/* figure out the GPIO config first */

	switch (mode) {

	case IOTimerChanMode_OneShot:
	case IOTimerChanMode_PWMOut:
	case IOTimerChanMode_Trigger:
		setbits = 0; //FIXME
		break;

	case IOTimerChanMode_PWMIn:
	case IOTimerChanMode_Capture:
		break;

	case IOTimerChanMode_NotUsed:
		setbits = 0;
		break;

	default:
		return -EINVAL;
	}

	irqstate_t flags = px4_enter_critical_section(); // atomic channel allocation and hw config

	int previous_mode = io_timer_get_channel_mode(channel);
	int rv = allocate_channel(channel, mode);
	unsigned timer = channels_timer(channel);

	if (rv == 0) {
		/* Try to reserve & initialize the timer - it will only do it once */

		rv = io_timer_init_timer(timer, mode);

		if (rv != 0 && previous_mode == IOTimerChanMode_NotUsed) {
			/* free the channel if it was not used before */
			io_timer_unallocate_channel(channel);
		}
	}

	/* Valid channel should now be reserved in new mode */

	if (rv == 0) {

		/* Set up IO */
		if (gpio) {
			px4_arch_configgpio(gpio);
		}

		/* configure the channel */

		uint32_t chan = timer_io_channels[channel].timer_channel - 1;

		//FIXME use setbits/clearbits for different modes

		/* Reset default */

		rC(timer, chan) = 0;

		/* Enable output update */

		rOUDIS(timer) &= ~EMIOS_OUDIS_OU(chan);

		/* OPWFMB mode initialization */

		rC(timer, chan) |= EMIOS_C_BSL_INTCNT;

		/* Duty cycle */

		rA(timer, chan) = EMIOS_A(0U); //Start at zero

		/* Period cycle */

		rB(timer, chan) = EMIOS_B(32768U);

		/* PWM mode */

		rC(timer, chan) = ((rC(timer, chan) & ~EMIOS_C_MODE_MASK) | EMIOS_C_MODE_OPWFMB_BMATCH);

		/* Edge polarity active low */

		//rC(timer,chan) |= EMIOS_C_EDPOL;

		/* Internal prescaler
		 * Div by 1, source prescaled lcok
		 */

		rC2(timer, chan) |= ((rC2(timer, chan) & ~EMIOS_C2_UCEXTPRE_MASK) | EMIOS_C2_UCEXTPRE(0));

		//rC2(timer,chan) |= EMIOS_C2_UCPRECLK;

		io_timer_set_PWM_mode(chan);
		timer_set_rate(chan, 50);

		rC(timer, chan) |= EMIOS_C_UCPREN;

		channel_handlers[channel].callback = channel_handler;
		channel_handlers[channel].context = context;
	}

	px4_leave_critical_section(flags);

	return rv;
}

int io_timer_set_enable(bool state, io_timer_channel_mode_t mode, io_timer_channel_allocation_t masks)
{
	switch (mode) {
	case IOTimerChanMode_NotUsed:
	case IOTimerChanMode_PWMOut:
	case IOTimerChanMode_OneShot:
	case IOTimerChanMode_Trigger:
		break;

	case IOTimerChanMode_PWMIn:
	case IOTimerChanMode_Capture:
		if (state) {
		}

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

	irqstate_t flags = px4_enter_critical_section();

	for (int chan_index = 0; masks != 0 && chan_index < MAX_TIMER_IO_CHANNELS; chan_index++) {
		if (masks & (1 << chan_index)) {
			masks &= ~(1 << chan_index);

			if (io_timer_validate_channel_index(chan_index) == 0) {
				uint32_t chan = timer_io_channels[chan_index].timer_channel - 1;
				uint32_t timer = channels_timer(chan_index);

				if ((state &&
				     (mode == IOTimerChanMode_PWMOut ||
				      mode == IOTimerChanMode_OneShot ||
				      mode == IOTimerChanMode_Trigger))) {
					rOUDIS(timer) &= ~EMIOS_OUDIS_OU(chan);

				} else {
					rOUDIS(timer) |= EMIOS_OUDIS_OU(chan);
				}

			}
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
			//FIXME why multiple by 2
			/* configure the channel */
			irqstate_t flags = px4_enter_critical_section();
			rA(channels_timer(channel), timer_io_channels[channel].timer_channel - 1) = EMIOS_A(value * 2);
			px4_leave_critical_section(flags);
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
			/* Read rALTA to fetch AS2 shadow register */
			value = (rALTA(channels_timer(channel), timer_io_channels[channel].timer_channel - 1) & EMIOS_A_MASK) / 2;
		}
	}

	return value;
}

uint32_t io_timer_get_group(unsigned timer)
{
	return get_timer_channels(timer);

}
