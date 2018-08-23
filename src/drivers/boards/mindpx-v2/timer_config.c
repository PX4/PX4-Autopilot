/****************************************************************************
 *
 *   Copyright (c) 2015, 2016 PX4 Development Team. All rights reserved.
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
 * @file mindpx_timer_config.c
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
#include <drivers/stm32/drv_io_timer.h>

#include "board_config.h"

__EXPORT const io_timers_t io_timers[MAX_IO_TIMERS] = {
//MAIN pwm timers;
    {
        .base = STM32_TIM1_BASE,
        .clock_register = STM32_RCC_APB2ENR,
        .clock_bit = RCC_APB2ENR_TIM1EN,
        .clock_freq = STM32_APB2_TIM1_CLKIN,
        .first_channel_index = 0,
        .last_channel_index = 3,
        .handler = io_timer_handler0,
        .vectorno =  STM32_IRQ_TIM1CC,
    },
    {
        .base = STM32_TIM4_BASE,
        .clock_register = STM32_RCC_APB1ENR,
        .clock_bit = RCC_APB1ENR_TIM4EN,
        .clock_freq = STM32_APB1_TIM4_CLKIN,
        .first_channel_index = 4,
        .last_channel_index = 7,
        .handler = io_timer_handler1,
        .vectorno =  STM32_IRQ_TIM4
    },
//AUX pwm timers;
    {
        .base = STM32_TIM3_BASE,
        .clock_register = STM32_RCC_APB1ENR,
        .clock_bit = RCC_APB1ENR_TIM3EN,
        .clock_freq = STM32_APB1_TIM3_CLKIN,
        .first_channel_index = 8,
        .last_channel_index = 11,
        .handler = io_timer_handler2,
        .vectorno =  STM32_IRQ_TIM3
    },
    {
        .base = STM32_TIM2_BASE,
        .clock_register = STM32_RCC_APB1ENR,
        .clock_bit = RCC_APB1ENR_TIM2EN,
        .clock_freq = STM32_APB1_TIM2_CLKIN,
        .first_channel_index = 12,
        .last_channel_index = 15,
        .handler = io_timer_handler3,
        .vectorno =  STM32_IRQ_TIM2,
    },
    
};

__EXPORT const timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
//MAIN pwm channels;
	{
		.gpio_out = GPIO_TIM1_CH1OUT,
		.gpio_in  = GPIO_TIM1_CH1IN,
		.timer_index = 0,
		.timer_channel = 1,
		.ccr_offset = STM32_GTIM_CCR1_OFFSET,
		.masks  = GTIM_SR_CC1IF | GTIM_SR_CC1OF
	},
	{
		.gpio_out = GPIO_TIM1_CH2OUT,
		.gpio_in  = GPIO_TIM1_CH2IN,
		.timer_index = 0,
		.timer_channel = 2,
		.ccr_offset = STM32_GTIM_CCR2_OFFSET,
		.masks  = GTIM_SR_CC2IF | GTIM_SR_CC2OF
	},
	{
		.gpio_out = GPIO_TIM1_CH3OUT,
		.gpio_in  = GPIO_TIM1_CH3IN,
		.timer_index = 0,
		.timer_channel = 3,
		.ccr_offset = STM32_GTIM_CCR3_OFFSET,
		.masks  = GTIM_SR_CC3IF | GTIM_SR_CC3OF
	},
	{
		.gpio_out = GPIO_TIM1_CH4OUT,
		.gpio_in  = GPIO_TIM1_CH4IN,
		.timer_index = 0,
		.timer_channel = 4,
		.ccr_offset = STM32_GTIM_CCR4_OFFSET,
		.masks  = GTIM_SR_CC4IF | GTIM_SR_CC4OF
	},
	{
		.gpio_out = GPIO_TIM4_CH1OUT,
		.gpio_in  = GPIO_TIM4_CH1IN,
		.timer_index = 1,
		.timer_channel = 1,
		.ccr_offset = STM32_GTIM_CCR1_OFFSET,
		.masks  = GTIM_SR_CC1IF | GTIM_SR_CC1OF
	},
	{
		.gpio_out = GPIO_TIM4_CH2OUT,
		.gpio_in  = GPIO_TIM4_CH2IN,
		.timer_index = 1,
		.timer_channel = 2,
		.ccr_offset = STM32_GTIM_CCR2_OFFSET,
		.masks  = GTIM_SR_CC2IF | GTIM_SR_CC2OF
	},
	{
		.gpio_out = GPIO_TIM4_CH3OUT,
		.gpio_in  = GPIO_TIM4_CH3IN,
		.timer_index = 1,
		.timer_channel = 3,
		.ccr_offset = STM32_GTIM_CCR3_OFFSET,
		.masks  = GTIM_SR_CC3IF | GTIM_SR_CC3OF
	},
	{
		.gpio_out = GPIO_TIM4_CH4OUT,
		.gpio_in  = GPIO_TIM4_CH4IN,
		.timer_index = 1,
		.timer_channel = 4,
		.ccr_offset = STM32_GTIM_CCR4_OFFSET,
		.masks  = GTIM_SR_CC4IF | GTIM_SR_CC4OF
	},
//AUX pwm channels;
    {
        .gpio_out = GPIO_TIM3_CH1OUT,
        .gpio_in  = GPIO_TIM3_CH1IN,
        .timer_index = 2,
        .timer_channel = 1,
        .ccr_offset = STM32_GTIM_CCR1_OFFSET,
        .masks  = GTIM_SR_CC1IF | GTIM_SR_CC1OF
    },
    {
        .gpio_out = GPIO_TIM3_CH2OUT,
        .gpio_in  = GPIO_TIM3_CH2IN,
        .timer_index = 2,
        .timer_channel = 2,
        .ccr_offset = STM32_GTIM_CCR2_OFFSET,
        .masks  = GTIM_SR_CC2IF | GTIM_SR_CC2OF
    },
    {
        .gpio_out = GPIO_TIM3_CH3OUT,
        .gpio_in  = GPIO_TIM3_CH3IN,
        .timer_index = 2,
        .timer_channel = 3,
        .ccr_offset = STM32_GTIM_CCR3_OFFSET,
        .masks  = GTIM_SR_CC3IF | GTIM_SR_CC3OF
    },
    {
        .gpio_out = GPIO_TIM3_CH4OUT,
        .gpio_in  = GPIO_TIM3_CH4IN,
        .timer_index = 2,
        .timer_channel = 4,
        .ccr_offset = STM32_GTIM_CCR4_OFFSET,
        .masks  = GTIM_SR_CC4IF | GTIM_SR_CC4OF
    },
    {
        .gpio_out = GPIO_TIM2_CH1OUT,
        .gpio_in  = GPIO_TIM2_CH1IN,
        .timer_index = 3,
        .timer_channel = 1,
        .ccr_offset = STM32_GTIM_CCR1_OFFSET,
        .masks  = GTIM_SR_CC1IF | GTIM_SR_CC1OF
    },
    {
        .gpio_out = GPIO_TIM2_CH2OUT,
        .gpio_in  = GPIO_TIM2_CH2IN,
        .timer_index = 3,
        .timer_channel = 2,
        .ccr_offset = STM32_GTIM_CCR2_OFFSET,
        .masks  = GTIM_SR_CC1IF | GTIM_SR_CC1OF
    },
    {
        .gpio_out = GPIO_TIM2_CH3OUT,
        .gpio_in  = GPIO_TIM2_CH3IN,
        .timer_index = 3,
        .timer_channel = 3,
        .ccr_offset = STM32_GTIM_CCR3_OFFSET,
        .masks  = GTIM_SR_CC1IF | GTIM_SR_CC1OF
    },
    {
        .gpio_out = GPIO_TIM2_CH4OUT,
        .gpio_in  = GPIO_TIM2_CH4IN,
        .timer_index = 3,
        .timer_channel = 4,
        .ccr_offset = STM32_GTIM_CCR4_OFFSET,
        .masks  = GTIM_SR_CC1IF | GTIM_SR_CC1OF
    },
};

#define CCER_C1_NUM_BITS   4
#define ACTIVE_LOW(c)      (GTIM_CCER_CC1P << (((c)-1) * CCER_C1_NUM_BITS))
#define ACTIVE_HIGH(c)     0

#if defined(BOARD_LED_PWM_DRIVE_ACTIVE_LOW)
#  define POLARITY(c)      ACTIVE_LOW(c)
#  define DRIVE_TYPE(p)    ((p)|GPIO_OPENDRAIN)
#else
#  define POLARITY(c)      ACTIVE_HIGH((c))
#  define DRIVE_TYPE(p)    (p)
#endif

#if defined(BOARD_UI_LED_PWM_DRIVE_ACTIVE_LOW)
#  define UI_POLARITY(c)    ACTIVE_LOW(c)
#  define UI_DRIVE_TYPE(p)  ((p)|GPIO_OPENDRAIN)
#else
#  define UI_POLARITY(c)    ACTIVE_HIGH((c))
#  define UI_DRIVE_TYPE(p)  (p)
#endif

__EXPORT const struct io_timers_t led_pwm_timers[MAX_LED_TIMERS] = {
    {
        .base             = STM32_TIM2_BASE,
        .clock_register     = STM32_RCC_APB1ENR,
        .clock_bit         = RCC_APB1ENR_TIM2EN,
        .clock_freq         = STM32_APB1_TIM2_CLKIN,
        .vectorno         = 0,
        .first_channel_index = 1,
        .last_channel_index = 3,
    }
};

__EXPORT const struct timer_io_channels_t led_pwm_channels[MAX_TIMER_LED_CHANNELS] = {
    {
        .gpio_out = LED_TIM2_CH2OUT,
        .gpio_in  = 0,
        .timer_index = 0,
        .timer_channel = 2,
        .masks = POLARITY(2),

    },
    {
        .gpio_out = LED_TIM2_CH3OUT,
        .gpio_in  = 0,
        .timer_index = 0,
        .timer_channel = 3,
        .masks = POLARITY(3),

    },
    {
        .gpio_out = LED_TIM2_CH4OUT,
        .gpio_in  = 0,
        .timer_index = 0,
        .timer_channel = 4,
        .masks = POLARITY(4),

    }
};

__EXPORT const uint8_t main_group_timer_map = 0x03 ;
__EXPORT const uint32_t main_group_channel_map = 0x00FF;

__EXPORT const uint8_t aux_group_timer_map = 0x0C ;
__EXPORT const uint32_t aux_group_channel_map = 0xFF00;

__EXPORT const uint8_t led_group_timer_map = 0x08 ;
__EXPORT const uint32_t led_group_channel_map = 0xE000;

__EXPORT const uint8_t heater_group_timer_map = 0x20 ;


