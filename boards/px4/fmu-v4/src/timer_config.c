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
 * @file timer_config.c
 *
 * Configuration data for the stm32 pwm_servo, input capture and pwm input driver.
 *
 * Note that these arrays must always be fully-sized.
 */

#include <stdint.h>

#include <stm32.h>
#include <stm32_gpio.h>
#include <stm32_tim.h>

#include <drivers/drv_pwm_output.h>
#include <px4_arch/io_timer.h>

#include "board_config.h"

__EXPORT const io_timers_t io_timers[MAX_IO_TIMERS] = {
	{
		.base = STM32_TIM1_BASE,
		.clock_register = STM32_RCC_APB2ENR,
		.clock_bit = RCC_APB2ENR_TIM1EN,
		.clock_freq = STM32_APB2_TIM1_CLKIN,
		.vectorno =  STM32_IRQ_TIM1CC,
		.dshot = {
			.dma_base = STM32_DMA2_BASE,
			.dmamap = DMAMAP_TIM1_UP,
		}

	},
	{
		.base = STM32_TIM4_BASE,
		.clock_register = STM32_RCC_APB1ENR,
		.clock_bit = RCC_APB1ENR_TIM4EN,
		.clock_freq = STM32_APB1_TIM4_CLKIN,
		.vectorno =  STM32_IRQ_TIM4,
		.dshot = {
			.dma_base = STM32_DMA1_BASE,
			.dmamap = DMAMAP_TIM4_UP,
		}

	}
};

__EXPORT const io_timers_channel_mapping_t io_timers_channel_mapping = {
	.element = {
		{
			.first_channel_index = 0,
			.channel_count = 4,
		},
		{
			.first_channel_index = 4,
			.channel_count = 2,
		}
	}
};

__EXPORT const timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
	{
		.gpio_out = GPIO_TIM1_CH4OUT,
		.gpio_in  = GPIO_TIM1_CH4IN,
		.timer_index = 0,
		.timer_channel = 4,
		.ccr_offset = STM32_GTIM_CCR4_OFFSET,
		.masks  = GTIM_SR_CC4IF | GTIM_SR_CC4OF
	},
	{
		.gpio_out = GPIO_TIM1_CH3OUT,
		.gpio_in = GPIO_TIM1_CH3IN,
		.timer_index = 0,
		.timer_channel = 3,
		.ccr_offset = STM32_GTIM_CCR3_OFFSET,
		.masks  = GTIM_SR_CC3IF | GTIM_SR_CC3OF
	},
	{
		.gpio_out = GPIO_TIM1_CH2OUT,
		.gpio_in = GPIO_TIM1_CH2IN,
		.timer_index = 0,
		.timer_channel = 2,
		.ccr_offset = STM32_GTIM_CCR2_OFFSET,
		.masks  = GTIM_SR_CC2IF | GTIM_SR_CC2OF
	},
	{
		.gpio_out = GPIO_TIM1_CH1OUT,
		.gpio_in = GPIO_TIM1_CH1IN,
		.timer_index = 0,
		.timer_channel = 1,
		.ccr_offset = STM32_GTIM_CCR1_OFFSET,
		.masks  = GTIM_SR_CC1IF | GTIM_SR_CC1OF
	},
	{
		.gpio_out = GPIO_TIM4_CH2OUT,
		.gpio_in = GPIO_TIM4_CH2IN,
		.timer_index = 1,
		.timer_channel = 2,
		.ccr_offset = STM32_GTIM_CCR2_OFFSET,
		.masks  = GTIM_SR_CC2IF | GTIM_SR_CC2OF
	},
	{
		.gpio_out = GPIO_TIM4_CH3OUT,
		.gpio_in = GPIO_TIM4_CH3IN,
		.timer_index = 1,
		.timer_channel = 3,
		.ccr_offset = STM32_GTIM_CCR3_OFFSET,
		.masks  = GTIM_SR_CC3IF | GTIM_SR_CC3OF
	}
};
