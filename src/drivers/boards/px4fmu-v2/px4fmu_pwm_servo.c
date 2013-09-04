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

/*
 * @file px4fmu_pwm_servo.c
 *
 * Configuration data for the stm32 pwm_servo driver.
 *
 * Note that these arrays must always be fully-sized.
 */

#include <stdint.h>

#include <stm32.h>
#include <stm32_gpio.h>
#include <stm32_tim.h>

#include <drivers/stm32/drv_pwm_servo.h>
#include <drivers/drv_pwm_output.h>

#include "board_config.h"

__EXPORT const struct pwm_servo_timer pwm_timers[PWM_SERVO_MAX_TIMERS] = {
	{
		.base = STM32_TIM1_BASE,
		.clock_register = STM32_RCC_APB2ENR,
		.clock_bit = RCC_APB2ENR_TIM1EN,
		.clock_freq = STM32_APB2_TIM1_CLKIN
	},
	{
		.base = STM32_TIM4_BASE,
		.clock_register = STM32_RCC_APB1ENR,
		.clock_bit = RCC_APB1ENR_TIM4EN,
		.clock_freq = STM32_APB1_TIM4_CLKIN
	}
};

__EXPORT const struct pwm_servo_channel pwm_channels[PWM_SERVO_MAX_CHANNELS] = {
	{
		.gpio = GPIO_TIM1_CH4OUT,
		.timer_index = 0,
		.timer_channel = 4,
		.default_value = 1000,
	},
	{
		.gpio = GPIO_TIM1_CH3OUT,
		.timer_index = 0,
		.timer_channel = 3,
		.default_value = 1000,
	},
	{
		.gpio = GPIO_TIM1_CH2OUT,
		.timer_index = 0,
		.timer_channel = 2,
		.default_value = 1000,
	},
	{
		.gpio = GPIO_TIM1_CH1OUT,
		.timer_index = 0,
		.timer_channel = 1,
		.default_value = 1000,
	},
	{
		.gpio = GPIO_TIM4_CH2OUT,
		.timer_index = 1,
		.timer_channel = 2,
		.default_value = 1000,
	},
	{
		.gpio = GPIO_TIM4_CH3OUT,
		.timer_index = 1,
		.timer_channel = 3,
		.default_value = 1000,
	}
};
