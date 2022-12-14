/****************************************************************************
 *
 *   Copyright (C) 2016, 2018 PX4 Development Team. All rights reserved.
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
 * @file timer_config.c
 *
 * Configuration data for the kinetis pwm_servo, input capture and pwm input driver.
 *
 * Note that these arrays must always be fully-sized.
 */

// TODO:Stubbed out for now
#include <stdint.h>

#include "hardware/s32k3xx_emios.h"

#include "board_config.h"

#include <drivers/drv_pwm_output.h>
#include <px4_arch/io_timer_hw_description.h>


constexpr io_timers_t io_timers[MAX_IO_TIMERS] = {
	initIOTimer(Timer::EMIOS0)
};

constexpr timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
	initIOTimerChannel(io_timers, {Timer::EMIOS0, Timer::Channel0}, PIN_EMIOS0_CH0_1),
	initIOTimerChannel(io_timers, {Timer::EMIOS0, Timer::Channel1}, PIN_EMIOS0_CH1_1),
	initIOTimerChannel(io_timers, {Timer::EMIOS0, Timer::Channel2}, PIN_EMIOS0_CH2_1),
	initIOTimerChannel(io_timers, {Timer::EMIOS0, Timer::Channel3}, PIN_EMIOS0_CH3_2),
	initIOTimerChannel(io_timers, {Timer::EMIOS0, Timer::Channel4}, PIN_EMIOS0_CH4_2),
	initIOTimerChannel(io_timers, {Timer::EMIOS0, Timer::Channel5}, PIN_EMIOS0_CH5_2),
	initIOTimerChannel(io_timers, {Timer::EMIOS0, Timer::Channel6}, PIN_EMIOS0_CH6_1),
	initIOTimerChannel(io_timers, {Timer::EMIOS0, Timer::Channel7}, PIN_EMIOS0_CH7_2),
};

constexpr io_timers_channel_mapping_t io_timers_channel_mapping =
	initIOTimerChannelMapping(io_timers, timer_io_channels);

const struct io_timers_t led_pwm_timers[MAX_LED_TIMERS] = {
	{
		.base = S32K3XX_EMIOS0_BASE,
		.clock_register = 0,
		.clock_bit = 0,
		.vectorno_0_3 = 0,
		.vectorno_4_7 = 0,
		.vectorno_8_11 = 0,
		.vectorno_12_15 = 0,
		.vectorno_16_19 = 0,
		.vectorno_20_23 = 0,
	},
	{
		.base = S32K3XX_EMIOS1_BASE,
		.clock_register = 0,
		.clock_bit = 0,
		.vectorno_0_3 = 0,
		.vectorno_4_7 = 0,
		.vectorno_8_11 = 0,
		.vectorno_12_15 = 0,
		.vectorno_16_19 = 0,
		.vectorno_20_23 = 0,
	},
};

const struct timer_io_channels_t led_pwm_channels[MAX_TIMER_LED_CHANNELS] = {
	{
		.gpio_out = GPIO_LED_R, // RGB_R
		.gpio_in  = 0,
		.timer_index = 0,
		.timer_channel = 19,
	},
	{
		.gpio_out = GPIO_LED_G, // RGB_G
		.gpio_in  = 0,
		.timer_index = 1,
		.timer_channel = 10,
	},
	{
		.gpio_out = GPIO_LED_B, // RGB_B
		.gpio_in  = 0,
		.timer_index = 1,
		.timer_channel = 5,
	},
};
