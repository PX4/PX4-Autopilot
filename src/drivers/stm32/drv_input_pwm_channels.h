/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file drv_input_pwm.h
 *
 * stm32-specific input pwm capture.
 */

#pragma once

#include <drivers/drv_input_pwm.h>

/* configuration limits */
#define INPUT_PWM_MAX_TIMERS	3
#define INPUT_PWM_MAX_CHANNELS_PER_TIMER 4
#define INPUT_PWM_MAX_CHANNELS   (INPUT_PWM_MAX_TIMERS * INPUT_PWM_MAX_CHANNELS_PER_TIMER)

/* array of timers dedicated to PWM input use */
struct input_pwm_timer {
	uint32_t	base;
	uint32_t	clock_register;
	uint32_t	clock_bit;
	uint32_t	clock_freq;
	int		irq_vector;
};

/* array of channels in logical order */
struct input_pwm_channel {
	uint32_t	gpio;
	uint8_t		timer_index;
	uint8_t		timer_channel;
};

/* supplied by board-specific code */
__EXPORT extern const struct input_pwm_timer input_pwm_timers[INPUT_PWM_MAX_TIMERS];
__EXPORT extern const struct input_pwm_channel input_pwm_channels[INPUT_PWM_MAX_CHANNELS];

int up_input_pwm_timer_isr(uint8_t timer, uint8_t *channel, uint16_t *value);
