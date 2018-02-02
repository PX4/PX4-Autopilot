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
 * @file board_pwm_input.c
 *
 * Configuration data for the stm32 pwm_servo driver.
 *
 * Note that these arrays must always be fully-sized.
 */

#include <stdint.h>

#include <stm32.h>
#include <stm32_gpio.h>
#include <stm32_tim.h>

#include <drivers/stm32/drv_pwm_input.h>
#include <drivers/drv_rc_input.h>

#include "board_config.h"

#if CONFIG_RC_INPUTS_TYPE(RC_INPUT_PWM)

/* PWM Input
* RC4 PB5  Timer 3 Channel 2 (AF2)
* RC5 PB0  Timer 3 Channel 3 (AF2)
* RC6 PB1  Timer 3 Channel 4 (AF2)
* RC7 PD13 Timer 4 Channel 2 (AF2)
* RC8 PD14 Timer 4 Channel 1 (AF2)
*/

__EXPORT const struct pwm_input_timer pwm_input_timers[PWM_INPUT_MAX_TIMERS] = {
	{
		.base = STM32_TIM3_BASE,
		.clock_register = STM32_RCC_APB1ENR,
		.clock_bit = RCC_APB1ENR_TIM3EN,
		.vector	= STM32_IRQ_TIM3,
		.clock_freq = STM32_APB1_TIM3_CLKIN
	},
	{
		.base = STM32_TIM4_BASE,
		.clock_register = STM32_RCC_APB1ENR,
		.clock_bit = RCC_APB1ENR_TIM4EN,
		.vector	= STM32_IRQ_TIM4,
		.clock_freq = STM32_APB1_TIM4_CLKIN
	}
};

__EXPORT const struct pwm_input_channel pwm_input_channels[PWM_INPUT_MAX_CHANNELS] = {
	{
		.gpio = GPIO_TIM3_CH2IN,
		.timer_index = 0,
		.timer_channel = 2,
	},
	{
		.gpio = GPIO_TIM3_CH3IN,
		.timer_index = 0,
		.timer_channel = 3,
	},
	{
		.gpio = GPIO_TIM3_CH4IN,
		.timer_index = 0,
		.timer_channel = 4,
	},
	{
		.gpio = GPIO_TIM4_CH2IN,
		.timer_index = 1,
		.timer_channel = 2,
	},
	{
		.gpio = GPIO_TIM4_CH1IN,
		.timer_index = 1,
		.timer_channel = 1,
	}
};

#endif
