/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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
#include "hardware/s32k3xx_emios.h"

/* TO DO: This mess only supports FTM 0-5, not 6-7 (only available on S32K148).
 * It's also hard to check if it is correct. *Should* work for S32K146, not confirmed for other S32K1XX MCUs. */

static inline constexpr timer_io_channels_t initIOTimerChannel(const io_timers_t io_timers_conf[MAX_IO_TIMERS],
		Timer::TimerChannel timer, uint32_t pinmux)
{
	timer_io_channels_t ret{};

	ret.gpio_in = pinmux;
	ret.gpio_out = pinmux;

	ret.timer_channel = (int)timer.channel + 1;

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

/* eMIOS has 24 channels and 4 channels per IRQ, io_timers_t can only specify one vector
 * However we can follow the pattern using the intitial IRQ vector
 * Ch0  - Ch3  = vectorno
 * Ch4  - Ch7  = vectorno - 1
 * Ch8  - Ch11 = vectorno - 2
 * Ch12 - Ch15 = vectorno - 3
 * Ch16 - Ch19 = vectorno - 4
 * Ch20 - Ch23 = vectorno - 5
 */

static inline constexpr io_timers_t initIOTimer(Timer::Timer timer)
{
	bool nuttx_config_timer_enabled = false;
	io_timers_t ret{};

	switch (timer) {
	case Timer::EMIOS0:
		ret.base = S32K3XX_EMIOS0_BASE;
		ret.clock_register = 0;
		ret.clock_bit = 0;
		ret.vectorno_0_3 = S32K3XX_IRQ_EMIOS0_0_3;
		ret.vectorno_4_7 = S32K3XX_IRQ_EMIOS0_4_7;
		ret.vectorno_8_11 = S32K3XX_IRQ_EMIOS0_8_11;
		ret.vectorno_12_15 = S32K3XX_IRQ_EMIOS0_12_15;
		ret.vectorno_16_19 = S32K3XX_IRQ_EMIOS0_16_19;
		ret.vectorno_20_23 = S32K3XX_IRQ_EMIOS0_20_23;
#ifdef CONFIG_S32K3XX_EMIOS0
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::EMIOS1:
		ret.base = S32K3XX_EMIOS1_BASE;
		ret.clock_register = 0;
		ret.clock_bit = 0;
		ret.vectorno_0_3 = S32K3XX_IRQ_EMIOS1_0_3;
		ret.vectorno_4_7 = S32K3XX_IRQ_EMIOS1_4_7;
		ret.vectorno_8_11 = S32K3XX_IRQ_EMIOS1_8_11;
		ret.vectorno_12_15 = S32K3XX_IRQ_EMIOS1_12_15;
		ret.vectorno_16_19 = S32K3XX_IRQ_EMIOS1_16_19;
		ret.vectorno_20_23 = S32K3XX_IRQ_EMIOS1_20_23;
#ifdef CONFIG_S32K3XX_EMIOS1
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::EMIOS2:
		ret.base = S32K3XX_EMIOS2_BASE;
		ret.clock_register = 0;
		ret.clock_bit = 0;
		ret.vectorno_0_3 = S32K3XX_IRQ_EMIOS2_0_3;
		ret.vectorno_4_7 = S32K3XX_IRQ_EMIOS2_4_7;
		ret.vectorno_8_11 = S32K3XX_IRQ_EMIOS2_8_11;
		ret.vectorno_12_15 = S32K3XX_IRQ_EMIOS2_12_15;
		ret.vectorno_16_19 = S32K3XX_IRQ_EMIOS2_16_19;
		ret.vectorno_20_23 = S32K3XX_IRQ_EMIOS2_20_23;
#ifdef CONFIG_S32K3XX_EMIOS2
		nuttx_config_timer_enabled = true;
#endif
		break;
	}

	// This is not strictly required, but for consistency let's make sure NuttX timers are disabled
	constexpr_assert(!nuttx_config_timer_enabled, "IO Timer requires NuttX timer config to be disabled (S32K1XX_FTMx)");

	return ret;
}
