/****************************************************************************
 *
 *   Copyright (C) 2018 PX4 Development Team. All rights reserved.
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
 * Servo driver supporting PWM servos connected to imxrt FLEXPWM blocks.
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

#include <chip.h>
#include "hardware/imxrt_flexpwm.h"
#include "imxrt_periphclks.h"

static int io_timer_handler0(int irq, void *context, void *arg);
static int io_timer_handler1(int irq, void *context, void *arg);
static int io_timer_handler2(int irq, void *context, void *arg);
static int io_timer_handler3(int irq, void *context, void *arg);
static int io_timer_handler4(int irq, void *context, void *arg);
static int io_timer_handler5(int irq, void *context, void *arg);
static int io_timer_handler6(int irq, void *context, void *arg);
static int io_timer_handler7(int irq, void *context, void *arg);

#if !defined(BOARD_PWM_FREQ)
#define BOARD_PWM_FREQ 1000000
#endif

#if !defined(BOARD_ONESHOT_FREQ)
#define BOARD_ONESHOT_FREQ 8000000
#endif

#define FLEXPWM_SRC_CLOCK_FREQ 16000000

#define MAX_CHANNELS_PER_TIMER 2

#define SM_SPACING (IMXRT_FLEXPWM_SM1CNT_OFFSET-IMXRT_FLEXPWM_SM0CNT_OFFSET)

#define _REG(_addr)	(*(volatile uint16_t *)(_addr))
#define _REG16(_base, _reg)	(*(volatile uint16_t *)(_base + _reg))
#define REG(_tmr, _sm, _reg)		_REG16(io_timers[(_tmr)].base + ((_sm) * SM_SPACING), (_reg))


/* Timer register accessors */

#define rCNT(_tim, _sm)        REG(_tim, _sm,IMXRT_FLEXPWM_SM0CNT_OFFSET)        /* Counter Register */
#define rINIT(_tim, _sm)       REG(_tim, _sm,IMXRT_FLEXPWM_SM0INIT_OFFSET)       /* Initial Count Register */
#define rCTRL2(_tim, _sm)      REG(_tim, _sm,IMXRT_FLEXPWM_SM0CTRL2_OFFSET)      /* Control 2 Register */
#define rCTRL(_tim, _sm)       REG(_tim, _sm,IMXRT_FLEXPWM_SM0CTRL_OFFSET)       /* Control Register */
#define rVAL0(_tim, _sm)       REG(_tim, _sm,IMXRT_FLEXPWM_SM0VAL0_OFFSET)       /* Value Register 0 */
#define rFRACVAL1(_tim, _sm)   REG(_tim, _sm,IMXRT_FLEXPWM_SM0FRACVAL1_OFFSET)   /* Fractional Value Register 1 */
#define rVAL1(_tim, _sm)       REG(_tim, _sm,IMXRT_FLEXPWM_SM0VAL1_OFFSET)       /* Value Register 1 */
#define rFRACVAL2(_tim, _sm)   REG(_tim, _sm,IMXRT_FLEXPWM_SM0FRACVAL2_OFFSET)   /* Fractional Value Register 2 */
#define rVAL2(_tim, _sm)       REG(_tim, _sm,IMXRT_FLEXPWM_SM0VAL2_OFFSET)       /* Value Register 2 */
#define rFRACVAL3(_tim, _sm)   REG(_tim, _sm,IMXRT_FLEXPWM_SM0FRACVAL3_OFFSET)   /* Fractional Value Register 3 */
#define rVAL3(_tim, _sm)       REG(_tim, _sm,IMXRT_FLEXPWM_SM0VAL3_OFFSET)       /* Value Register 3 */
#define rFRACVAL4(_tim, _sm)   REG(_tim, _sm,IMXRT_FLEXPWM_SM0FRACVAL4_OFFSET)   /* Fractional Value Register 4 */
#define rVAL4(_tim, _sm)       REG(_tim, _sm,IMXRT_FLEXPWM_SM0VAL4_OFFSET)       /* Value Register 4 */
#define rFRACVAL5(_tim, _sm)   REG(_tim, _sm,IMXRT_FLEXPWM_SM0FRACVAL5_OFFSET)   /* Fractional Value Register 5 */
#define rVAL5(_tim, _sm)       REG(_tim, _sm,IMXRT_FLEXPWM_SM0VAL5_OFFSET)       /* Value Register 5 */
#define rFRCTRL(_tim, _sm)     REG(_tim, _sm,IMXRT_FLEXPWM_SM0FRCTRL_OFFSET)     /* Fractional Control Register */
#define rOCTRL(_tim, _sm)      REG(_tim, _sm,IMXRT_FLEXPWM_SM0OCTRL_OFFSET)      /* Output Control Register */
#define rSTS(_tim, _sm)        REG(_tim, _sm,IMXRT_FLEXPWM_SM0STS_OFFSET)        /* Status Register */
#define rINTEN(_tim, _sm)      REG(_tim, _sm,IMXRT_FLEXPWM_SM0INTEN_OFFSET)      /* Interrupt Enable Register */
#define rDMAEN(_tim, _sm)      REG(_tim, _sm,IMXRT_FLEXPWM_SM0DMAEN_OFFSET)      /* DMA Enable Register */
#define rTCTRL(_tim, _sm)      REG(_tim, _sm,IMXRT_FLEXPWM_SM0TCTRL_OFFSET)      /* Output Trigger Control Register */
#define rDISMAP0(_tim, _sm)    REG(_tim, _sm,IMXRT_FLEXPWM_SM0DISMAP0_OFFSET)    /* Fault Disable Mapping Register 0 */
#define rDISMAP1(_tim, _sm)    REG(_tim, _sm,IMXRT_FLEXPWM_SM0DISMAP1_OFFSET)    /* Fault Disable Mapping Register 1 */
#define rDTCNT0(_tim, _sm)     REG(_tim, _sm,IMXRT_FLEXPWM_SM0DTCNT0_OFFSET)     /* Deadtime Count Register 0 */
#define rDTCNT1(_tim, _sm)     REG(_tim, _sm,IMXRT_FLEXPWM_SM0DTCNT1_OFFSET)     /* Deadtime Count Register 1 */
#define rCAPTCTRLA(_tim, _sm)  REG(_tim, _sm,IMXRT_FLEXPWM_SM0CAPTCTRLA_OFFSET)  /* Capture Control A Register */
#define rCAPTCOMPA(_tim, _sm)  REG(_tim, _sm,IMXRT_FLEXPWM_SM0CAPTCOMPA_OFFSET)  /* Capture Compare A Register */
#define rCAPTCTRLB(_tim, _sm)  REG(_tim, _sm,IMXRT_FLEXPWM_SM0CAPTCTRLB_OFFSET)  /* Capture Control B Register */
#define rCAPTCOMPB(_tim, _sm)  REG(_tim, _sm,IMXRT_FLEXPWM_SM0CAPTCOMPB_OFFSET)  /* Capture Compare B Register */
#define rCAPTCTRLX(_tim, _sm)  REG(_tim, _sm,IMXRT_FLEXPWM_SM0CAPTCTRLX_OFFSET)  /* Capture Control X Register */
#define rCAPTCOMPX(_tim, _sm)  REG(_tim, _sm,IMXRT_FLEXPWM_SM0CAPTCOMPX_OFFSET)  /* Capture Compare X Register */
#define rCVAL0(_tim, _sm)      REG(_tim, _sm,IMXRT_FLEXPWM_SM0CVAL0_OFFSET)      /* Capture Value 0 Register */
#define rCVAL0CYC(_tim, _sm)   REG(_tim, _sm,IMXRT_FLEXPWM_SM0CVAL0CYC_OFFSET)   /* Capture Value 0 Cycle Register */
#define rCVAL1(_tim, _sm)      REG(_tim, _sm,IMXRT_FLEXPWM_SM0CVAL1_OFFSET)      /* Capture Value 1 Register */
#define rCVAL1CYC(_tim, _sm)   REG(_tim, _sm,IMXRT_FLEXPWM_SM0CVAL1CYC_OFFSET)   /* Capture Value 1 Cycle Register */
#define rCVAL2(_tim, _sm)      REG(_tim, _sm,IMXRT_FLEXPWM_SM0CVAL2_OFFSET)      /* Capture Value 2 Register */
#define rCVAL2CYC(_tim, _sm)   REG(_tim, _sm,IMXRT_FLEXPWM_SM0CVAL2CYC_OFFSET)   /* Capture Value 2 Cycle Register */
#define rCVAL3(_tim, _sm)      REG(_tim, _sm,IMXRT_FLEXPWM_SM0CVAL3_OFFSET)      /* Capture Value 3 Register */
#define rCVAL3CYC(_tim, _sm)   REG(_tim, _sm,IMXRT_FLEXPWM_SM0CVAL3CYC_OFFSET)   /* Capture Value 3 Cycle Register */
#define rCVAL4(_tim, _sm)      REG(_tim, _sm,IMXRT_FLEXPWM_SM0CVAL4_OFFSET)      /* Capture Value 4 Register */
#define rCVAL4CYC(_tim, _sm)   REG(_tim, _sm,IMXRT_FLEXPWM_SM0CVAL4CYC_OFFSET)   /* Capture Value 4 Cycle Register */
#define rCVAL5(_tim, _sm)      REG(_tim, _sm,IMXRT_FLEXPWM_SM0CVAL5_OFFSET)      /* Capture Value 5 Register */
#define rCVAL5CYC(_tim, _sm)   REG(_tim, _sm,IMXRT_FLEXPWM_SM0CVAL5CYC_OFFSET)   /* Capture Value 5 Cycle Register */

#define rOUTEN(_tim)           REG(_tim,  0, IMXRT_FLEXPWM_OUTEN_OFFSET)         /* Output Enable Register */
#define rMASK(_tim)            REG(_tim,  0, IMXRT_FLEXPWM_MASK_OFFSET)          /* Mask Register */
#define rSWCOUT(_tim)          REG(_tim,  0, IMXRT_FLEXPWM_SWCOUT_OFFSET)        /* Software Controlled Output Register */
#define rDTSRCSEL(_tim)        REG(_tim,  0, IMXRT_FLEXPWM_DTSRCSEL_OFFSET)      /* PWM Source Select Register */
#define rMCTRL(_tim)           REG(_tim,  0, IMXRT_FLEXPWM_MCTRL_OFFSET)         /* Master Control Register */
#define rMCTRL2(_tim)          REG(_tim,  0, IMXRT_FLEXPWM_MCTRL2_OFFSET)        /* Master Control 2 Register */
#define rFCTRL0(_tim)          REG(_tim,  0, IMXRT_FLEXPWM_FCTRL0_OFFSET)        /* Fault Control Register */
#define rFSTS0(_tim)           REG(_tim,  0, IMXRT_FLEXPWM_FSTS0_OFFSET)         /* Fault Status Register */
#define rFFILT0(_tim)          REG(_tim,  0, IMXRT_FLEXPWM_FFILT0_OFFSET)        /* Fault Filter Register */
#define rFTST0(_tim)           REG(_tim,  0, IMXRT_FLEXPWM_FTST0_OFFSET)         /* Fault Test Register */
#define rFCTRL20(_tim)         REG(_tim,  0, IMXRT_FLEXPWM_FCTRL20_OFFSET)       /* Fault Control 2 Register */


//												 				  NotUsed   PWMOut  PWMIn Capture OneShot Trigger Dshot LED PPS Other
io_timer_channel_allocation_t channel_allocations[IOTimerChanModeSize] = { UINT16_MAX,   0,  0,  0, 0, 0, 0, 0, 0, 0 };

typedef uint8_t io_timer_allocation_t; /* big enough to hold MAX_IO_TIMERS */

io_timer_channel_allocation_t timer_allocations[MAX_IO_TIMERS] = { };

/* Stats and handlers are only useful for Capture */

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
	// Not implemented yet
	UNUSED(io_timer_channel_stats);
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

static uint32_t get_channel_mask(unsigned channel)
{
	return io_timer_validate_channel_index(channel) == 0 ? 1 << channel : 0;
}

int io_timer_validate_channel_index(unsigned channel)
{
	int rv = -EINVAL;

	if (channel < MAX_TIMER_IO_CHANNELS && timer_io_channels[channel].val_offset != 0) {

		/* test timer for validity */

		if (io_timers[channels_timer(channel)].base != 0) {
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

	return timer_io_channels[channel].gpio_portpin | (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | IOMUX_CMOS_OUTPUT | IOMUX_PULL_KEEP
			| IOMUX_DRIVE_33OHM  | IOMUX_SPEED_MEDIUM | IOMUX_SLEW_FAST);
	return 0;
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
	rMCTRL(channels_timer(channel)) |= (timer_io_channels[channel].sub_module_bits >> MCTRL_LDOK_SHIFT) << MCTRL_CLDOK_SHIFT
					   ;
	rVAL1(channels_timer(channel), timer_io_channels[channel].sub_module) = (BOARD_PWM_FREQ / rate) - 1;
	rMCTRL(channels_timer(channel)) |= timer_io_channels[channel].sub_module_bits;
	px4_leave_critical_section(flags);
	return 0;
}

static inline void io_timer_set_oneshot_mode(unsigned channel)
{
	irqstate_t flags = px4_enter_critical_section();
	uint16_t rvalue = rCTRL(channels_timer(channel), timer_io_channels[channel].sub_module);
	rvalue &= ~SMCTRL_PRSC_MASK;
	rvalue |= SMCTRL_PRSC_DIV2 | SMCTRL_LDMOD;
	rMCTRL(channels_timer(channel)) |= (timer_io_channels[channel].sub_module_bits >> MCTRL_LDOK_SHIFT) << MCTRL_CLDOK_SHIFT
					   ;
	rCTRL(channels_timer(channel), timer_io_channels[channel].sub_module)  = rvalue;
	rMCTRL(channels_timer(channel)) |= timer_io_channels[channel].sub_module_bits;
	px4_leave_critical_section(flags);

}

static inline void io_timer_set_PWM_mode(unsigned channel)
{
	irqstate_t flags = px4_enter_critical_section();
	uint16_t rvalue = rCTRL(channels_timer(channel), timer_io_channels[channel].sub_module);
	rvalue &= ~(SMCTRL_PRSC_MASK | SMCTRL_LDMOD);
	rvalue |= SMCTRL_PRSC_DIV16;
	rMCTRL(channels_timer(channel)) |= (timer_io_channels[channel].sub_module_bits >> MCTRL_LDOK_SHIFT) << MCTRL_CLDOK_SHIFT
					   ;
	rCTRL(channels_timer(channel), timer_io_channels[channel].sub_module)  = rvalue;
	rMCTRL(channels_timer(channel)) |= timer_io_channels[channel].sub_module_bits;
	px4_leave_critical_section(flags);
}

void io_timer_trigger(unsigned channel_mask)
{
	// Any Channels in oneshot?

	int oneshots = io_timer_get_mode_channels(IOTimerChanMode_OneShot) & channel_mask;

	// Yes do triggering
	if (oneshots) {
		struct {
			uint32_t base;
			uint16_t triggers;
		} action_cache[MAX_IO_TIMERS] = {0};

		int actions = 0;

		// Get the Timer modules addresses
		for (int timer = 0; timer < MAX_IO_TIMERS && io_timers[timer].base != 0; timer++) {
			if (action_cache[actions].base == 0) {
				action_cache[actions].base = io_timers[timer].base;

			} else if (action_cache[actions].base != io_timers[timer].base) {
				actions++;
				action_cache[actions].base = io_timers[timer].base;
			}
		}


		/* Pre-calculate the list of channels to Trigger for each Timer module */

		int mask;
		uint32_t channel = 0;

		for (actions = 0; actions < MAX_IO_TIMERS; actions++) {
			if (action_cache[actions].base == 0) {
				// All have been processed
				break;
			}

			// Group the bits from channels on the same timer
			for (; channel < MAX_TIMER_IO_CHANNELS && timer_io_channels[channel].val_offset;  channel++) {

				if (io_timers[timer_io_channels[channel].timer_index].base != action_cache[actions].base) {
					break;
				}

				mask = get_channel_mask(channel);

				if (oneshots & mask) {
					action_cache[actions].triggers |= timer_io_channels[channel].sub_module_bits;
				}
			}
		}

		/* Now do them all with the shortest delay in between */

		irqstate_t flags = px4_enter_critical_section();

		for (actions = 0; action_cache[actions].base != 0 &&  actions < MAX_IO_TIMERS; actions++) {
			_REG16(action_cache[actions].base, IMXRT_FLEXPWM_MCTRL_OFFSET) |= (action_cache[actions].triggers >> MCTRL_LDOK_SHIFT)
					<< MCTRL_CLDOK_SHIFT ;
			_REG16(action_cache[actions].base, IMXRT_FLEXPWM_MCTRL_OFFSET) |= action_cache[actions].triggers;
		}

		px4_leave_critical_section(flags);
	}
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

		/* enable the timer clock before we try to talk to it */

		switch (io_timers[timer].base) {
		case IMXRT_FLEXPWM1_BASE:
			imxrt_clockall_pwm1();
			break;

		case IMXRT_FLEXPWM2_BASE:
			imxrt_clockall_pwm2();
			break;

		case IMXRT_FLEXPWM3_BASE:
			imxrt_clockall_pwm3();
			break;

		case IMXRT_FLEXPWM4_BASE:
			imxrt_clockall_pwm4();
			break;
		}

		uint32_t first_channel_index = io_timers_channel_mapping.element[timer].first_channel_index;
		uint32_t last_channel_index = first_channel_index + io_timers_channel_mapping.element[timer].channel_count;

		for (uint32_t chan = first_channel_index; chan < last_channel_index; chan++) {

			/* Clear all Faults */
			rFSTS0(timer) = FSTS_FFLAG_MASK;

			rCTRL2(timer, timer_io_channels[chan].sub_module) = SMCTRL2_CLK_SEL_EXT_CLK | SMCTRL2_DBGEN | SMCTRL2_INDEP;
			rCTRL(timer, timer_io_channels[chan].sub_module)  = SMCTRL_PRSC_DIV16 | SMCTRL_FULL;
			/* Edge aligned at 0 */
			rINIT(channels_timer(chan), timer_io_channels[chan].sub_module) = 0;
			rVAL0(channels_timer(chan), timer_io_channels[chan].sub_module) = 0;
			rVAL2(channels_timer(chan), timer_io_channels[chan].sub_module) = 0;
			rVAL4(channels_timer(chan), timer_io_channels[chan].sub_module) = 0;
			rFFILT0(timer) &= ~FFILT_FILT_PER_MASK;
			rDISMAP0(timer, timer_io_channels[chan].sub_module) = 0xf000;
			rDISMAP1(timer, timer_io_channels[chan].sub_module) = 0xf000;
			rOUTEN(timer) |= timer_io_channels[chan].val_offset == PWMA_VAL ? OUTEN_PWMA_EN(1 << timer_io_channels[chan].sub_module)
					 : OUTEN_PWMB_EN(1 << timer_io_channels[chan].sub_module);
			rDTSRCSEL(timer) = 0;
			rMCTRL(timer) = MCTRL_LDOK(1 << timer_io_channels[chan].sub_module);
			rMCTRL(timer) = timer_io_channels[chan].sub_module_bits;
			io_timer_set_PWM_mode(chan);
			timer_set_rate(chan, 50);
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

	uint32_t channels = get_channel_mask(timer) & (io_timer_get_mode_channels(IOTimerChanMode_OneShot) |
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

	uint32_t gpio = 0;

	/* figure out the GPIO config first */

	switch (mode) {

	case IOTimerChanMode_OneShot:
	case IOTimerChanMode_PWMOut:
	case IOTimerChanMode_Trigger:
		gpio = timer_io_channels[channel].gpio_out;
		break;

	case IOTimerChanMode_PWMIn:
	case IOTimerChanMode_Capture:
		return -EINVAL;
		break;

	case IOTimerChanMode_NotUsed:
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

		REG(timer, 0, IMXRT_FLEXPWM_MCTRL_OFFSET) |=  MCTRL_RUN(1 << timer_io_channels[channel].sub_module);

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

	if (masks) {
		struct {
			uint32_t sm_ens;
			uint32_t base;
			uint32_t io_index;
			uint32_t gpios[MAX_TIMER_IO_CHANNELS];
		} action_cache[MAX_IO_TIMERS];

		unsigned int actions = 0;
		memset(action_cache, 0, sizeof(action_cache));

		for (int timer = 0; timer < MAX_IO_TIMERS && io_timers[timer].base != 0; timer++) {
			if (action_cache[actions].base == 0) {
				action_cache[actions].base = io_timers[timer].base;

			} else if (action_cache[actions].base != io_timers[timer].base) {
				actions++;
				action_cache[actions].base = io_timers[timer].base;
			}
		}

		int chan_index = 0;

		for (actions = 0; actions < MAX_IO_TIMERS; actions++) {
			if (action_cache[actions].base == 0) {
				// All have been processed
				break;
			}

			for (; masks != 0 && chan_index < MAX_TIMER_IO_CHANNELS && timer_io_channels[chan_index].val_offset; chan_index++) {

				if (masks & (1 << chan_index)) {

					if (io_timers[timer_io_channels[chan_index].timer_index].base != action_cache[actions].base) {
						break;
					}

					masks &= ~(1 << chan_index);

					action_cache[actions].sm_ens |= MCTRL_RUN(1 << timer_io_channels[chan_index].sub_module) |
									timer_io_channels[chan_index].sub_module_bits;
					action_cache[actions].gpios[action_cache[actions].io_index++] = timer_io_channels[chan_index].gpio_out;
				}
			}
		}

		irqstate_t flags = px4_enter_critical_section();

		for (actions = 0; actions < arraySize(action_cache); actions++) {
			if (action_cache[actions].base == 0) {
				break;
			}

			for (unsigned int index = 0; index < action_cache[actions].io_index; index++) {
				if (action_cache[actions].gpios[index]) {
					px4_arch_configgpio(action_cache[actions].gpios[index]);
				}
			}

			_REG16(action_cache[actions].base, IMXRT_FLEXPWM_MCTRL_OFFSET) = action_cache[actions].sm_ens;
		}

		px4_leave_critical_section(flags);
	}

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
			irqstate_t flags = px4_enter_critical_section();
			rMCTRL(channels_timer(channel)) |= (timer_io_channels[channel].sub_module_bits >> MCTRL_LDOK_SHIFT) << MCTRL_CLDOK_SHIFT
							   ;
			REG(channels_timer(channel), timer_io_channels[channel].sub_module, timer_io_channels[channel].val_offset) = value - 1;
			rMCTRL(channels_timer(channel)) |= timer_io_channels[channel].sub_module_bits;
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
			value = REG(channels_timer(channel), timer_io_channels[channel].sub_module, timer_io_channels[channel].val_offset) + 1;
		}
	}

	return value;
}

// The rt has 1:1 group to channel
uint32_t io_timer_get_group(unsigned group)
{
	return get_channel_mask(group);
}
