/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file drv_io_timer_gpio.h
 *
 * Shared helper for drivers that claim an individual timer/IO channel as a
 * plain digital (push-pull) output.
 */

#pragma once

#include <px4_arch/io_timer.h>

/**
 * Claims a channel and configures it as a plain digital output.
 *
 * @param channel      the IO timer channel index
 * @param mode         the allocation mode
 * @param initial_high initial output level
 * @return the configured pin descriptor on success, 0 if the channel is
 *         invalid or already allocated to a different mode.
 */
static inline uint32_t io_timer_channel_claim_as_gpio_output(unsigned channel, io_timer_channel_mode_t mode,
		bool initial_high)
{
	uint32_t gpio = io_timer_channel_get_gpio_output(channel);

	if (gpio == 0) {
		return 0;
	}

	if (io_timer_allocate_channel(channel, mode) != 0) {
		return 0;
	}

#if defined(PX4_MAKE_GPIO_OUTPUT_SET)

	// put the initial level into the pin config,
	// (configgpio would otherwise drive it low first)
	if (initial_high) {
		gpio = PX4_MAKE_GPIO_OUTPUT_SET(gpio);
	}

#endif
	px4_arch_configgpio(gpio);
	px4_arch_gpiowrite(gpio, initial_high);
	return gpio;
}
