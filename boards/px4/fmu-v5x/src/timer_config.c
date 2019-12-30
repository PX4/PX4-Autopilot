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
 * @file px4fmu_timer_config.c
 *
 * Configuration data for the stm32 pwm_servo, input capture and pwm input driver.
 *
 * Note that these arrays must always be fully-sized.
 */

#include <stdint.h>

#include <chip.h>
#include <stm32_gpio.h>
#include <stm32_tim.h>

#include <drivers/drv_pwm_output.h>
#include <px4_arch/io_timer.h>

#include "board_config.h"

/* Timer allocation
 *
 * TIM1_CH4  T FMU_CH1
 * TIM1_CH3  T FMU_CH2
 * TIM1_CH2  T FMU_CH3
 * TIM1_CH1  T FMU_CH4
 *
 * TIM4_CH2  T FMU_CH5
 * TIM4_CH3  T FMU_CH6
 *
 * TIM12_CH1 T FMU_CH7
 * TIM12_CH2 T FMU_CH8
 *
 * TIM5_CH4  T FMU_CAP1               < Capture
 * TIM5_CH3  T SPI2_DRDY2_ISM330_INT2 < Capture or GPIO INT
 * TIM5_CH1  T SPIX_SYNC              > Pulse or GPIO strobe
 * TIM2_CH3  T HEATER                 > PWM OUT or GPIO
 *
 * TIM14_CH1 T BUZZER_1              - Driven by other driver
 * TIM8_CH1_IN T FMU_PPM_INPUT       - Sampled byt HRT by other driver

 */

__EXPORT const io_timers_t io_timers[MAX_IO_TIMERS] = {
	{
		.base = STM32_TIM1_BASE,
		.clock_register = STM32_RCC_APB2ENR,
		.clock_bit = RCC_APB2ENR_TIM1EN,
		.clock_freq = STM32_APB2_TIM1_CLKIN,
		.first_channel_index = 0,
		.last_channel_index = 3,
		.handler = io_timer_handler0,
		.vectorno =  STM32_IRQ_TIM1CC,
		.dshot = {
			.dma_base = STM32_DMA2_BASE,
			.dmamap = DMAMAP_TIM1_UP,
			.start_ccr_register = TIM_DMABASE_CCR1,
			.channels_number = 4u /* CCR1, CCR2, CCR3 and CCR4 */
		}
	},
	{
		.base = STM32_TIM4_BASE,
		.clock_register = STM32_RCC_APB1ENR,
		.clock_bit = RCC_APB1ENR_TIM4EN,
		.clock_freq = STM32_APB1_TIM4_CLKIN,
		.first_channel_index = 4,
		.last_channel_index = 5,
		.handler = io_timer_handler1,
		.vectorno =  STM32_IRQ_TIM4,
		.dshot = {
			.dma_base = STM32_DMA1_BASE,
			.dmamap = DMAMAP_TIM4_UP,
			.start_ccr_register = TIM_DMABASE_CCR2,
			.channels_number = 2u /* CCR2 and CCR3 */
		}
	},
	{
		.base = STM32_TIM12_BASE,
		.clock_register = STM32_RCC_APB1ENR,
		.clock_bit = RCC_APB1ENR_TIM12EN,
		.clock_freq = STM32_APB1_TIM12_CLKIN,
		.first_channel_index = 6,
		.last_channel_index = 7,
		.handler = io_timer_handler2,
		.vectorno =  STM32_IRQ_TIM12,
	},
	{
		.base = STM32_TIM5_BASE,
		.clock_register = STM32_RCC_APB1ENR,
		.clock_bit = RCC_APB1ENR_TIM5EN,
		.clock_freq = STM32_APB1_TIM5_CLKIN,
		.first_channel_index = 8,
		.last_channel_index = 10,
		.handler = io_timer_handler3,
		.vectorno =  STM32_IRQ_TIM5,
	},
	{
		.base = STM32_TIM2_BASE,
		.clock_register = STM32_RCC_APB1ENR,
		.clock_bit = RCC_APB1ENR_TIM2EN,
		.clock_freq = STM32_APB1_TIM2_CLKIN,
		.first_channel_index = 11,
		.last_channel_index = 11,
		.handler = io_timer_handler4,
		.vectorno =  STM32_IRQ_TIM2,
	},
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
	},
	{
		.gpio_out = GPIO_TIM12_CH1OUT,
		.gpio_in = GPIO_TIM12_CH1IN,
		.timer_index = 2,
		.timer_channel = 1,
		.ccr_offset = STM32_GTIM_CCR1_OFFSET,
		.masks  = GTIM_SR_CC1IF | GTIM_SR_CC1OF
	},
	{
		.gpio_out = GPIO_TIM12_CH2OUT,
		.gpio_in = GPIO_TIM12_CH2IN,
		.timer_index = 2,
		.timer_channel = 2,
		.ccr_offset = STM32_GTIM_CCR2_OFFSET,
		.masks  = GTIM_SR_CC2IF | GTIM_SR_CC2OF
	},
// todo:Need to support MAX_TIMER_IO_CHANNELS == 12
#if MAX_TIMER_IO_CHANNELS == 12
	{
		.gpio_out = GPIO_TIM5_CH4OUT,
		.gpio_in = GPIO_TIM5_CH4IN,
		.timer_index = 3,
		.timer_channel = 4,
		.ccr_offset = STM32_GTIM_CCR4_OFFSET,
		.masks  = GTIM_SR_CC4IF | GTIM_SR_CC4OF
	},
	{
		.gpio_out = 0,
		.gpio_in = GPIO_TIM5_CH3IN,
		.timer_index = 3,
		.timer_channel = 3,
		.ccr_offset = STM32_GTIM_CCR3_OFFSET,
		.masks  = GTIM_SR_CC3IF | GTIM_SR_CC3OF
	},
	{
		.gpio_out = GPIO_TIM5_CH1OUT,
		.gpio_in = 0,
		.timer_index = 3,
		.timer_channel = 1,
		.ccr_offset = STM32_GTIM_CCR1_OFFSET,
		.masks  = GTIM_SR_CC1IF | GTIM_SR_CC1OF
	},
	{
		.gpio_out = GPIO_TIM2_CH3OUT,
		.gpio_in = 0,
		.timer_index = 4,
		.timer_channel = 3,
		.ccr_offset = STM32_GTIM_CCR3_OFFSET,
		.masks  = GTIM_SR_CC3IF | GTIM_SR_CC3OF
	}
#endif
};
