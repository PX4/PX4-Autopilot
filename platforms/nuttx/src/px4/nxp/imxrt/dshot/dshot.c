/****************************************************************************
 *
 * Copyright (C) 2023 PX4 Development Team. All rights reserved.
 * Author: Peter van der Perk <peter.vanderperk@nxp.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/micro_hal.h>
#include <imxrt_flexio.h>
#include <px4_arch/dshot.h>
#include <px4_arch/io_timer.h>
#include <drivers/drv_dshot.h>
#include <stdio.h>

#include "arm_internal.h"

#define DSHOT_TIMERS			FLEXIO_SHIFTBUFNIS_COUNT
#define DSHOT_THROTTLE_POSITION		5u
#define DSHOT_TELEMETRY_POSITION	4u
#define NIBBLES_SIZE 			4u
#define DSHOT_NUMBER_OF_NIBBLES		3u

typedef struct dshot_handler_t {
	bool			init;
	uint32_t 		data_seg1;
	uint32_t 		irq_data;
} dshot_handler_t;

static dshot_handler_t dshot_inst[DSHOT_TIMERS] = {};

struct flexio_dev_s *flexio_s;

static int flexio_irq_handler(int irq, void *context, void *arg)
{

	uint32_t flags = flexio_s->ops->get_shifter_status_flags(flexio_s);
	uint32_t instance;

	for (instance = 0; flags && instance < DSHOT_TIMERS; instance++) {
		if (flags & (1 << instance)) {
			flexio_s->ops->disable_shifter_status_interrupts(flexio_s, (1 << instance));
			flexio_s->ops->disable_timer_status_interrupts(flexio_s, (1 << instance));

			if (dshot_inst[instance].irq_data != 0) {
				uint32_t buf_adr = flexio_s->ops->get_shifter_buffer_address(flexio_s, FLEXIO_SHIFTER_BUFFER, instance);
				putreg32(dshot_inst[instance].irq_data, IMXRT_FLEXIO1_BASE + buf_adr);
				dshot_inst[instance].irq_data = 0;
			}
		}
	}

	return OK;
}

int up_dshot_init(uint32_t channel_mask, unsigned dshot_pwm_freq)
{
	uint32_t timer_compare;

	if (dshot_pwm_freq == 150000) {
		timer_compare = 0x2F8A;

	} else if (dshot_pwm_freq == 300000) {
		timer_compare = 0x2F45;

	} else if (dshot_pwm_freq == 600000) {
		timer_compare = 0x2F22;

	} else if (dshot_pwm_freq == 1200000) {
		timer_compare = 0x2F11;

	} else {
		// Not supported Dshot frequency
		return 0;
	}

	/* Init FlexIO peripheral */

	flexio_s = imxrt_flexio_initialize(1);
	up_enable_irq(IMXRT_IRQ_FLEXIO1);
	irq_attach(IMXRT_IRQ_FLEXIO1, flexio_irq_handler, flexio_s);

	for (unsigned channel = 0; (channel_mask != 0) && (channel < DSHOT_TIMERS); channel++) {
		if (channel_mask & (1 << channel)) {
			uint8_t timer = timer_io_channels[channel].timer_index;

			if (io_timers[timer].dshot.pinmux == 0) { // board does not configure dshot on this pin
				continue;
			}

			imxrt_config_gpio(io_timers[timer].dshot.pinmux);

			struct flexio_shifter_config_s shft_cfg;
			shft_cfg.timer_select = channel;
			shft_cfg.timer_polarity = FLEXIO_SHIFTER_TIMER_POLARITY_ON_POSITIVE;
			shft_cfg.pin_config = FLEXIO_PIN_CONFIG_OUTPUT;
			shft_cfg.pin_select = io_timers[timer].dshot.flexio_pin;
			shft_cfg.pin_polarity = FLEXIO_PIN_ACTIVE_HIGH;
			shft_cfg.shifter_mode = FLEXIO_SHIFTER_MODE_TRANSMIT;
			shft_cfg.parallel_width = 0;
			shft_cfg.input_source = FLEXIO_SHIFTER_INPUT_FROM_PIN;
			shft_cfg.shifter_stop = FLEXIO_SHIFTER_STOP_BIT_LOW;
			shft_cfg.shifter_start = FLEXIO_SHIFTER_START_BIT_DISABLED_LOAD_DATA_ON_ENABLE;

			flexio_s->ops->set_shifter_config(flexio_s, channel, &shft_cfg);

			struct flexio_timer_config_s tmr_cfg;
			tmr_cfg.trigger_select = (4 * channel) + 1;
			tmr_cfg.trigger_polarity = FLEXIO_TIMER_TRIGGER_POLARITY_ACTIVE_LOW;
			tmr_cfg.trigger_source = FLEXIO_TIMER_TRIGGER_SOURCE_INTERNAL;
			tmr_cfg.pin_config = FLEXIO_PIN_CONFIG_OUTPUT_DISABLED;
			tmr_cfg.pin_select = 0;
			tmr_cfg.pin_polarity = FLEXIO_PIN_ACTIVE_LOW;
			tmr_cfg.timer_mode = FLEXIO_TIMER_MODE_DUAL8_BIT_BAUD_BIT;
			tmr_cfg.timer_output = FLEXIO_TIMER_OUTPUT_ONE_NOT_AFFECTED_BY_RESET;
			tmr_cfg.timer_decrement = FLEXIO_TIMER_DEC_SRC_ON_FLEX_IO_CLOCK_SHIFT_TIMER_OUTPUT;
			tmr_cfg.timer_reset = FLEXIO_TIMER_RESET_NEVER;
			tmr_cfg.timer_disable = FLEXIO_TIMER_DISABLE_ON_TIMER_COMPARE;
			tmr_cfg.timer_enable = FLEXIO_TIMER_ENABLE_ON_TRIGGER_HIGH;
			tmr_cfg.timer_stop = FLEXIO_TIMER_STOP_BIT_DISABLED;
			tmr_cfg.timer_start = FLEXIO_TIMER_START_BIT_DISABLED;
			tmr_cfg.timer_compare = timer_compare;
			flexio_s->ops->set_timer_config(flexio_s, channel, &tmr_cfg);

			dshot_inst[channel].init = true;
		}
	}

	flexio_s->ops->enable(flexio_s, true);

	return channel_mask;
}

void up_dshot_trigger(void)
{
	uint32_t buf_adr;

	for (uint8_t motor_number = 0; (motor_number < DSHOT_TIMERS); motor_number++) {
		if (dshot_inst[motor_number].init && dshot_inst[motor_number].data_seg1 != 0) {
			buf_adr = flexio_s->ops->get_shifter_buffer_address(flexio_s, FLEXIO_SHIFTER_BUFFER, motor_number);
			putreg32(dshot_inst[motor_number].data_seg1, IMXRT_FLEXIO1_BASE + buf_adr);
		}
	}

	flexio_s->ops->clear_timer_status_flags(flexio_s, 0xFF);
	flexio_s->ops->enable_shifter_status_interrupts(flexio_s, 0xFF);
}

uint64_t dshot_expand_data(uint16_t packet)
{
	unsigned int mask;
	unsigned int index = 0;
	uint64_t expanded = 0x0;

	for (mask = 0x8000; mask != 0; mask >>= 1) {
		if (packet & mask) {
			expanded = expanded | ((uint64_t)0x3 << index);

		} else {
			expanded = expanded | ((uint64_t)0x1 << index);
		}

		index = index + 3;
	}

	return expanded;
}

/**
* bits 	1-11	- throttle value (0-47 are reserved, 48-2047 give 2000 steps of throttle resolution)
* bit 	12		- dshot telemetry enable/disable
* bits 	13-16	- XOR checksum
**/
void dshot_motor_data_set(unsigned motor_number, uint16_t throttle, bool telemetry)
{
	if (motor_number < DSHOT_TIMERS && dshot_inst[motor_number].init) {
		uint16_t packet = 0;
		uint16_t checksum = 0;

		packet |= throttle << DSHOT_THROTTLE_POSITION;
		packet |= ((uint16_t)telemetry & 0x01) << DSHOT_TELEMETRY_POSITION;

		uint16_t csum_data = packet;

		/* XOR checksum calculation */
		csum_data >>= NIBBLES_SIZE;

		for (unsigned i = 0; i < DSHOT_NUMBER_OF_NIBBLES; i++) {
			checksum ^= (csum_data & 0x0F); // XOR data by nibbles
			csum_data >>= NIBBLES_SIZE;
		}

		packet |= (checksum & 0x0F);

		uint64_t dshot_expanded = dshot_expand_data(packet);
		dshot_inst[motor_number].data_seg1 = (uint32_t)(dshot_expanded & 0xFFFFFF);
		dshot_inst[motor_number].irq_data = (uint32_t)(dshot_expanded >> 24);
	}
}

int up_dshot_arm(bool armed)
{
	return io_timer_set_enable(armed, IOTimerChanMode_Dshot, IO_TIMER_ALL_MODES_CHANNELS);
}
