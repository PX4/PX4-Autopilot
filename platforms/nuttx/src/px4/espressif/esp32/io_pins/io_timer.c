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


uint32_t io_timer_get_group(unsigned timer)
{
	if (timer == 0) {

#if defined(CONFIG_ESP32_LEDC_TIM0_CHANNELS)
		return (1 << CONFIG_ESP32_LEDC_TIM0_CHANNELS) - 1;
#endif
		return -1;

	} else if (timer == 1) {

#if defined(CONFIG_ESP32_LEDC_TIM1_CHANNELS)
		return (1 << CONFIG_ESP32_LEDC_TIM1_CHANNELS) - 1;
#endif
		return -1;

	} else if (timer == 2) {
#if defined(CONFIG_ESP32_LEDC_TIM2_CHANNELS)
		return (1 << CONFIG_ESP32_LEDC_TIM2_CHANNELS) - 1;
#endif
		return -1;

	} else if (timer == 3) {
#if defined(CONFIG_ESP32_LEDC_TIM3_CHANNELS)
		return (1 << CONFIG_ESP32_LEDC_TIM3_CHANNELS) - 1;
#endif
		return -1;
	}

	return -1;

}

uint32_t io_timer_channel_get_gpio_output(unsigned channel)
{
	return timer_io_channels[channel].gpio_out | GPIO_OUTPUT | GPIO_PULLUP;
}
