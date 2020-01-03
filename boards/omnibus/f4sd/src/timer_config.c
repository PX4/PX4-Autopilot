/****************************************************************************
 *
 *   Copyright (C) 2018 PX4 Development Team. All rights reserved.
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
		.base = STM32_TIM2_BASE,
		.clock_register = STM32_RCC_APB1ENR,
		.clock_bit = RCC_APB1ENR_TIM2EN,
		.clock_freq = STM32_APB1_TIM2_CLKIN,
		.vectorno =  STM32_IRQ_TIM2,
		.dshot = {
			.dma_base = STM32_DMA1_BASE,
			.dmamap = DMAMAP_TIM2_UP_1,
		}
	},
	{
		.base = STM32_TIM3_BASE,
		.clock_register = STM32_RCC_APB1ENR,
		.clock_bit = RCC_APB1ENR_TIM3EN,
		.clock_freq = STM32_APB1_TIM3_CLKIN,
		.vectorno =  STM32_IRQ_TIM3,
		.dshot = {
			.dma_base = STM32_DMA1_BASE,
			.dmamap = DMAMAP_TIM3_UP,
		}
	}
};

__EXPORT const io_timers_channel_mapping_t io_timers_channel_mapping = {
	.element = {
		{
			.first_channel_index = 0,
			.channel_count = 2,
		},
		{
			.first_channel_index = 2,
			.channel_count = 2,
		}
	}
};


/*
 * OUTPUTS:
 *  M3 : PA3 : TIM2_CH3
 *  M4 : PA2 : TIM2_CH4
 *  M1 : PB0 : TIM3_CH3
 *  M2 : PB1 : TIM3_CH4
 */

__EXPORT const timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
	//PB0 S1_OUT D1_ST7
	{
		.gpio_out = GPIO_TIM3_CH3OUT,
		.gpio_in = GPIO_TIM3_CH3IN,
		.timer_index = 1,
		.timer_channel = 3,
		.ccr_offset = STM32_GTIM_CCR3_OFFSET,
		.masks  = GTIM_SR_CC3IF | GTIM_SR_CC3OF
	},
	//PB1 S2_OUT D1_ST2
	{
		.gpio_out = GPIO_TIM3_CH4OUT,
		.gpio_in = GPIO_TIM3_CH4IN,
		.timer_index = 1,
		.timer_channel = 4,
		.ccr_offset = STM32_GTIM_CCR4_OFFSET,
		.masks  = GTIM_SR_CC4IF | GTIM_SR_CC4OF
	},
	//PA3 S3_OUT D1_ST6
	{
		.gpio_out = GPIO_TIM2_CH4OUT,
		.gpio_in = GPIO_TIM2_CH4IN,
		.timer_index = 0,
		.timer_channel = 4,
		.ccr_offset = STM32_GTIM_CCR4_OFFSET,
		.masks  = GTIM_SR_CC4IF | GTIM_SR_CC4OF
	},
	//PA2 S4_OUT D1_ST1
	{
		.gpio_out = GPIO_TIM2_CH3OUT,
		.gpio_in = GPIO_TIM2_CH3IN,
		.timer_index = 0,
		.timer_channel = 3,
		.ccr_offset = STM32_GTIM_CCR3_OFFSET,
		.masks  = GTIM_SR_CC3IF | GTIM_SR_CC3OF
	}
};
