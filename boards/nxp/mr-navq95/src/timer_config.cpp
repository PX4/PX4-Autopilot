/****************************************************************************
 *
 *   Copyright (C) 2024 PX4 Development Team. All rights reserved.
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

#include <stdint.h>

#include <drivers/drv_pwm_output.h>
#include <px4_arch/io_timer_hw_description.h>

#include "board_config.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/


#ifdef __cplusplus
extern "C" {
#endif

extern const iomux_cfg_t GPIO_FLEXIO04_CH1;
extern const iomux_cfg_t GPIO_FLEXIO20_CH2;
extern const iomux_cfg_t GPIO_FLEXIO24_CH3;
extern const iomux_cfg_t GPIO_FLEXIO05_CH4;
extern const iomux_cfg_t GPIO_FLEXIO21_CH5;
extern const iomux_cfg_t GPIO_FLEXIO06_CH6;
extern const iomux_cfg_t GPIO_FLEXIO22_CH7;
extern const iomux_cfg_t GPIO_FLEXIO23_CH8;

#ifdef __cplusplus
}
#endif


constexpr io_timers_t io_timers[MAX_IO_TIMERS] = {
	initIOTimerDshot(Timer::TPM3, 133333333UL),
	initIOTimerDshot(Timer::TPM4, 1000000UL),
	initIOTimerDshot(Timer::TPM5, 1000000UL),
#if defined(CONFIG_BOARD_NAVQ95_IO23_PWM)
	initIOTimerDshot(Timer::TPM6, 1000000UL),
#endif
};

const timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
	initIOTimerChannelDshot(io_timers, {Timer::TPM3, Timer::Channel0}, {GPIO::Port2, GPIO::Pin4}, GPIO_FLEXIO04_CH1, 4),
	initIOTimerChannelDshot(io_timers, {Timer::TPM3, Timer::Channel1}, {GPIO::Port2, GPIO::Pin20}, GPIO_FLEXIO20_CH2, 20),
	initIOTimerChannelDshot(io_timers, {Timer::TPM3, Timer::Channel3}, {GPIO::Port2, GPIO::Pin24}, GPIO_FLEXIO24_CH3, 24),
	initIOTimerChannelDshot(io_timers, {Timer::TPM4, Timer::Channel0}, {GPIO::Port2, GPIO::Pin5}, GPIO_FLEXIO05_CH4, 5),
	initIOTimerChannelDshot(io_timers, {Timer::TPM4, Timer::Channel1}, {GPIO::Port2, GPIO::Pin21}, GPIO_FLEXIO21_CH5, 21),
	initIOTimerChannelDshot(io_timers, {Timer::TPM5, Timer::Channel0}, {GPIO::Port2, GPIO::Pin6}, GPIO_FLEXIO06_CH6, 6),
	initIOTimerChannelDshot(io_timers, {Timer::TPM5, Timer::Channel1}, {GPIO::Port2, GPIO::Pin22}, GPIO_FLEXIO22_CH7, 22),
#if defined(CONFIG_BOARD_NAVQ95_IO23_PWM)
	initIOTimerChannelDshot(io_timers, {Timer::TPM6, Timer::Channel1}, {GPIO::Port2, GPIO::Pin23}, GPIO_FLEXIO23_CH8, 23),
#endif
};

const io_timers_channel_mapping_t io_timers_channel_mapping =
	initIOTimerChannelMapping(io_timers, timer_io_channels);
