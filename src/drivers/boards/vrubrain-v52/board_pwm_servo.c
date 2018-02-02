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
 * @file board_pwm_servo.c
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

/* PWM Output
* RC1 PE9  Timer 1 Channel 1 (AF1)
* RC2 PE11 Timer 1 Channel 2 (AF1)
* RC3 PE13 Timer 1 Channel 3 (AF1)
* RC4 PE14 Timer 1 Channel 4 (AF1)
* RC5 PE5  Timer 9 Channel 1 (AF3)
* RC6 PE6  Timer 9 Channel 2 (AF3)
* RC7 PC8  Timer 8 Channel 3 (AF3)
* RC8 PC9  Timer 8 Channel 4 (AF3)
* RC4 PB5  Timer 3 Channel 2 (AF2)
* RC5 PB0  Timer 3 Channel 3 (AF2)
* RC6 PB1  Timer 3 Channel 4 (AF2)
*/

__EXPORT const struct pwm_servo_timer pwm_timers[PWM_SERVO_MAX_TIMERS] = {
#if CONFIG_RC_INPUTS_TYPE(RC_INPUT_PWM)
	{
		.base	= STM32_TIM1_BASE,
		.clock_register	= STM32_RCC_APB2ENR,
		.clock_bit	= RCC_APB2ENR_TIM1EN,
		.clock_freq	= STM32_APB2_TIM1_CLKIN
	},
	{
		.base = STM32_TIM9_BASE,
		.clock_register = STM32_RCC_APB2ENR,
		.clock_bit = RCC_APB2ENR_TIM9EN,
		.clock_freq = STM32_APB2_TIM9_CLKIN
	},
	{
		.base = STM32_TIM8_BASE,
		.clock_register = STM32_RCC_APB2ENR,
		.clock_bit = RCC_APB2ENR_TIM8EN,
		.clock_freq = STM32_APB2_TIM8_CLKIN
	}
#else
	{
		.base	= STM32_TIM1_BASE,
		.clock_register	= STM32_RCC_APB2ENR,
		.clock_bit	= RCC_APB2ENR_TIM1EN,
		.clock_freq	= STM32_APB2_TIM1_CLKIN
	},
	{
		.base = STM32_TIM9_BASE,
		.clock_register = STM32_RCC_APB2ENR,
		.clock_bit = RCC_APB2ENR_TIM9EN,
		.clock_freq = STM32_APB2_TIM9_CLKIN
	},
	{
		.base = STM32_TIM8_BASE,
		.clock_register = STM32_RCC_APB2ENR,
		.clock_bit = RCC_APB2ENR_TIM8EN,
		.clock_freq = STM32_APB2_TIM8_CLKIN
	},
	{
		.base = STM32_TIM3_BASE,
		.clock_register = STM32_RCC_APB1ENR,
		.clock_bit = RCC_APB1ENR_TIM3EN,
		.clock_freq = STM32_APB1_TIM3_CLKIN
	}
#endif
};

__EXPORT const struct pwm_servo_channel pwm_channels[PWM_SERVO_MAX_CHANNELS] = {
#if CONFIG_RC_INPUTS_TYPE(RC_INPUT_PWM)
	{
		.gpio = GPIO_TIM1_CH1OUT,
		.timer_index = 0,
		.timer_channel = 1,
		.default_value = 0,
	},
	{
		.gpio = GPIO_TIM1_CH2OUT,
		.timer_index = 0,
		.timer_channel = 2,
		.default_value = 0,
	},
	{
		.gpio = GPIO_TIM1_CH3OUT,
		.timer_index = 0,
		.timer_channel = 3,
		.default_value = 0,
	},
	{
		.gpio = GPIO_TIM1_CH4OUT,
		.timer_index = 0,
		.timer_channel = 4,
		.default_value = 0,
	},
	{
		.gpio = GPIO_TIM9_CH1OUT,
		.timer_index = 1,
		.timer_channel = 1,
		.default_value = 0,
	},
	{
		.gpio = GPIO_TIM9_CH2OUT,
		.timer_index = 1,
		.timer_channel = 2,
		.default_value = 0,
	},
	{
		.gpio = GPIO_TIM8_CH3OUT,
		.timer_index = 2,
		.timer_channel = 3,
		.default_value = 0,
	},
	{
		.gpio = GPIO_TIM8_CH4OUT,
		.timer_index = 2,
		.timer_channel = 4,
		.default_value = 0,
	}
#else
	{
		.gpio = GPIO_TIM1_CH1OUT,
		.timer_index = 0,
		.timer_channel = 1,
		.default_value = 0,
	},
	{
		.gpio = GPIO_TIM1_CH2OUT,
		.timer_index = 0,
		.timer_channel = 2,
		.default_value = 0,
	},
	{
		.gpio = GPIO_TIM1_CH3OUT,
		.timer_index = 0,
		.timer_channel = 3,
		.default_value = 0,
	},
	{
		.gpio = GPIO_TIM1_CH4OUT,
		.timer_index = 0,
		.timer_channel = 4,
		.default_value = 0,
	},
	{
		.gpio = GPIO_TIM9_CH1OUT,
		.timer_index = 1,
		.timer_channel = 1,
		.default_value = 0,
	},
	{
		.gpio = GPIO_TIM9_CH2OUT,
		.timer_index = 1,
		.timer_channel = 2,
		.default_value = 0,
	},
	{
		.gpio = GPIO_TIM8_CH3OUT,
		.timer_index = 2,
		.timer_channel = 3,
		.default_value = 0,
	},
	{
		.gpio = GPIO_TIM8_CH4OUT,
		.timer_index = 2,
		.timer_channel = 4,
		.default_value = 0,
	},
	{
		.gpio = GPIO_TIM3_CH2OUT,
		.timer_index = 3,
		.timer_channel = 2,
		.default_value = 0,
	},
	{
		.gpio = GPIO_TIM3_CH3OUT,
		.timer_index = 3,
		.timer_channel = 3,
		.default_value = 0,
	},
	{
		.gpio = GPIO_TIM3_CH4OUT,
		.timer_index = 3,
		.timer_channel = 4,
		.default_value = 0,
	}
#endif
};
