/****************************************************************************
 *
 *   Copyright (C) 2019,2024 PX4 Development Team. All rights reserved.
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

#pragma once


#include <px4_arch/io_timer.h>
#include <px4_arch/hw_description.h>
#include <px4_platform_common/constexpr_util.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform/io_timer_init.h>
#include <hardware/imx9_memorymap.h>

static inline constexpr timer_io_channels_t initIOTimerChannel(const io_timers_t io_timers_conf[MAX_IO_TIMERS],
		Timer::TimerChannel timer, GPIO::GPIOPin pin)
{
	timer_io_channels_t ret{};

	ret.gpio_in = getGPIOPort(pin.port) | getGPIOPin(pin.pin) | GPIO_INPUT;
	ret.gpio_out = getGPIOPort(pin.port) | getGPIOPin(pin.pin) | GPIO_OUTPUT;

	ret.timer_channel = (int)timer.channel;

	// find timer index
	ret.timer_index = 0xff;
	const uint32_t timer_base = timerBaseRegister(timer.timer);

	for (int i = 0; i < MAX_IO_TIMERS; ++i) {
		if (io_timers_conf[i].base == timer_base) {
			ret.timer_index = i;
			break;
		}
	}

	constexpr_assert(ret.timer_index != 0xff, "Timer not found");

	return ret;
}

static inline constexpr timer_io_channels_t initIOTimerChannelDshot(const io_timers_t io_timers_conf[MAX_IO_TIMERS],
		Timer::TimerChannel timer, GPIO::GPIOPin pin, iomux_cfg_t dshot_pinmux, uint32_t flexio_pin)
{
	timer_io_channels_t ret = initIOTimerChannel(io_timers_conf, timer, pin);

	ret.dshot.pinmux = dshot_pinmux;
	ret.dshot.flexio_pin = flexio_pin;
	return ret;
}

static inline constexpr io_timers_t initIOTimer(Timer::Timer timer, uint32_t clock_rate)
{
	io_timers_t ret{};

	ret.clock_rate = clock_rate;

	switch (timer) {
	case Timer::TPM1:
		ret.base = IMX9_TPM1_BASE;
		ret.vectorno = IMX9_IRQ_TPM1;
		break;

	case Timer::TPM2:
		ret.base = IMX9_TPM2_BASE;
		ret.vectorno = IMX9_IRQ_TPM2;
		break;

	case Timer::TPM3:
		ret.base = IMX9_TPM3_BASE;
		ret.vectorno = IMX9_IRQ_TPM3;
		break;

	case Timer::TPM4:
		ret.base = IMX9_TPM4_BASE;
		ret.vectorno = IMX9_IRQ_TPM4;
		break;


	case Timer::TPM5:
		ret.base = IMX9_TPM5_BASE;
		ret.vectorno = IMX9_IRQ_TPM5;
		break;

	case Timer::TPM6:
		ret.base = IMX9_TPM6_BASE;
		ret.vectorno = IMX9_IRQ_TPM6;
		break;
	}

	return ret;
}

static inline constexpr io_timers_t initIOTimerDshot(Timer::Timer timer, uint32_t clock_rate)
{
	return initIOTimer(timer, clock_rate);
}
