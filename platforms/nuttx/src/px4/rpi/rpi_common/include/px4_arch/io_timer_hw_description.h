/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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

static inline constexpr timer_io_channels_t initIOTimerChannel(const io_timers_t io_timers_conf[MAX_IO_TIMERS],
		Timer::TimerChannel timer, GPIO::GPIOPin pin)
{
	timer_io_channels_t ret{};

	uint32_t gpio_af = 0;

	switch (timer.channel) {
	case Timer::ChannelA:
		if (!(pin.pin & 1) && (pin.pin & 15) / 2 == (timer.timer - 1)) {
			gpio_af = getGPIOPin(pin.pin) | GPIO_FUN(RP2040_GPIO_FUNC_PWM);
		}

		break;

	case Timer::ChannelB:
		if ((pin.pin & 1) && (pin.pin & 15) / 2 == (timer.timer - 1)) {
			gpio_af = getGPIOPin(pin.pin) | GPIO_FUN(RP2040_GPIO_FUNC_PWM);
		}

		break;

	default:
		break;
	}

	ret.gpio_in = gpio_af;
	ret.gpio_out = gpio_af;

	ret.timer_channel = timer.channel;

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

static inline constexpr io_timers_t initIOTimer(Timer::Timer timer)
{
	bool nuttx_config_timer_enabled = false;
	io_timers_t ret{};

	switch (timer) {
	case Timer::Timer0:
		ret.base = timerBaseRegister(timer);
#ifdef CONFIG_RP2040_PWM_CH0				// Currently Nuttx doesn't have PWM support for RP2040. This is for possible future use.
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::Timer1:
		ret.base = timerBaseRegister(timer);
#ifdef CONFIG_RP2040_PWM_CH1				// Currently Nuttx doesn't have PWM support for RP2040. This is for possible future use.
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::Timer2:
		ret.base = timerBaseRegister(timer);
#ifdef CONFIG_RP2040_PWM_CH2				// Currently Nuttx doesn't have PWM support for RP2040. This is for possible future use.
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::Timer3:
		ret.base = timerBaseRegister(timer);
#ifdef CONFIG_RP2040_PWM_CH3				// Currently Nuttx doesn't have PWM support for RP2040. This is for possible future use.
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::Timer4:
		ret.base = timerBaseRegister(timer);
#ifdef CONFIG_RP2040_PWM_CH4				// Currently Nuttx doesn't have PWM support for RP2040. This is for possible future use.
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::Timer5:
		ret.base = timerBaseRegister(timer);
#ifdef CONFIG_RP2040_PWM_CH5				// Currently Nuttx doesn't have PWM support for RP2040. This is for possible future use.
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::Timer6:
		ret.base = timerBaseRegister(timer);
#ifdef CONFIG_RP2040_PWM_CH6				// Currently Nuttx doesn't have PWM support for RP2040. This is for possible future use.
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::Timer7:
		ret.base = timerBaseRegister(timer);
#ifdef CONFIG_RP2040_PWM_CH7				// Currently Nuttx doesn't have PWM support for RP2040. This is for possible future use.
		nuttx_config_timer_enabled = true;
#endif
		break;
	}

	// This is not strictly required, but for consistency let's make sure NuttX timers are disabled
	constexpr_assert(!nuttx_config_timer_enabled, "IO Timer requires NuttX timer config to be disabled (KINETIS_FTMx)");

	return ret;
}
