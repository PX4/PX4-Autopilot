/****************************************************************************
 *
 *   Copyright (c) 2015, 2016 Airmind Development Team. All rights reserved.
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
 * 3. Neither the name Airmind nor the names of its contributors may be
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
* @file drv_led_pwm.cpp
*
*
*/

#include <nuttx/config.h>


#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <ctype.h>


#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>

#include <drivers/drv_pwm_output.h>
#include <drivers/stm32/drv_io_timer.h>


#include <board_config.h>


#define REG(_tmr, _reg) (*(volatile uint32_t *)(led_pwm_timers[_tmr].base + _reg))

#define rCR1(_tmr)      REG(_tmr, STM32_GTIM_CR1_OFFSET)
#define rCR2(_tmr)      REG(_tmr, STM32_GTIM_CR2_OFFSET)
#define rSMCR(_tmr)     REG(_tmr, STM32_GTIM_SMCR_OFFSET)
#define rDIER(_tmr)     REG(_tmr, STM32_GTIM_DIER_OFFSET)
#define rSR(_tmr)       REG(_tmr, STM32_GTIM_SR_OFFSET)
#define rEGR(_tmr)      REG(_tmr, STM32_GTIM_EGR_OFFSET)
#define rCCMR1(_tmr)    REG(_tmr, STM32_GTIM_CCMR1_OFFSET)
#define rCCMR2(_tmr)    REG(_tmr, STM32_GTIM_CCMR2_OFFSET)
#define rCCER(_tmr)     REG(_tmr, STM32_GTIM_CCER_OFFSET)
#define rCNT(_tmr)      REG(_tmr, STM32_GTIM_CNT_OFFSET)
#define rPSC(_tmr)      REG(_tmr, STM32_GTIM_PSC_OFFSET)
#define rARR(_tmr)      REG(_tmr, STM32_GTIM_ARR_OFFSET)
#define rCCR1(_tmr)     REG(_tmr, STM32_GTIM_CCR1_OFFSET)
#define rCCR2(_tmr)     REG(_tmr, STM32_GTIM_CCR2_OFFSET)
#define rCCR3(_tmr)     REG(_tmr, STM32_GTIM_CCR3_OFFSET)
#define rCCR4(_tmr)     REG(_tmr, STM32_GTIM_CCR4_OFFSET)
#define rDCR(_tmr)      REG(_tmr, STM32_GTIM_DCR_OFFSET)
#define rDMAR(_tmr)     REG(_tmr, STM32_GTIM_DMAR_OFFSET)
#define rBDTR(_tmr)     REG(_tmr, STM32_ATIM_BDTR_OFFSET)

static void             led_pwm_timer_init(unsigned timer);
static void             led_pwm_timer_set_rate(unsigned timer, unsigned rate);
static void             led_pwm_channel_init(unsigned channel);

int led_pwm_servo_set(unsigned channel, uint8_t  value);
unsigned led_pwm_servo_get(unsigned channel);
int led_pwm_servo_init(void);
void led_pwm_servo_deinit(void);
void led_pwm_servo_arm(bool armed);
unsigned led_pwm_timer_get_period(unsigned timer);


const struct io_timers_t led_pwm_timers[3] = {
	{
		.base = STM32_TIM3_BASE,
		.clock_register = STM32_RCC_APB1ENR,
		.clock_bit = RCC_APB1ENR_TIM3EN,
		.clock_freq = STM32_APB1_TIM3_CLKIN,
		.vectorno =  STM32_IRQ_TIM3,
		.first_channel_index = 4,
		.last_channel_index = 7,
		.handler = io_timer_handler0,
	},
	{
		.base = STM32_TIM3_BASE,
		.clock_register = STM32_RCC_APB1ENR,
		.clock_bit = RCC_APB1ENR_TIM3EN,
		.clock_freq = STM32_APB1_TIM3_CLKIN,
		.vectorno =  STM32_IRQ_TIM3,
		.first_channel_index = 4,
		.last_channel_index = 7,
		.handler = io_timer_handler1,
	},
	{
		.base = STM32_TIM3_BASE,
		.clock_register = STM32_RCC_APB1ENR,
		.clock_bit = RCC_APB1ENR_TIM3EN,
		.clock_freq = STM32_APB1_TIM3_CLKIN,
		.vectorno =  STM32_IRQ_TIM3,
		.first_channel_index = 4,
		.last_channel_index = 7,
		.handler = io_timer_handler2,
	}
};

#define LED_TIM3_CH3OUT  (GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_OPENDRAIN|GPIO_PORTB|GPIO_PIN0)
#define LED_TIM3_CH2OUT  (GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_OPENDRAIN|GPIO_PORTC|GPIO_PIN7)
#define LED_TIM3_CH4OUT  (GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_OPENDRAIN|GPIO_PORTB|GPIO_PIN1)
#define LED_TIM3_CH3IN  (GPIO_INPUT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_OPENDRAIN|GPIO_PORTB|GPIO_PIN0)
#define LED_TIM3_CH2IN  (GPIO_INPUT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_OPENDRAIN|GPIO_PORTC|GPIO_PIN7)
#define LED_TIM3_CH4IN  (GPIO_INPUT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_OPENDRAIN|GPIO_PORTB|GPIO_PIN1)

const struct timer_io_channels_t led_pwm_channels[3] = {
	{
		.gpio_out = LED_TIM3_CH3OUT,
		.gpio_in  = LED_TIM3_CH3IN,
		.timer_index = 0,
		.timer_channel = 3,
		.masks	= GTIM_SR_CC3IF | GTIM_SR_CC3OF,
		.ccr_offset = STM32_GTIM_CCR3_OFFSET,
	},
	{
		.gpio_out = LED_TIM3_CH2OUT,
		.gpio_in  = LED_TIM3_CH2IN,
		.timer_index = 1,
		.timer_channel = 2,
		.masks	= GTIM_SR_CC2IF | GTIM_SR_CC2OF,
		.ccr_offset = STM32_GTIM_CCR2_OFFSET,
	},
	{
		.gpio_out = LED_TIM3_CH4OUT,
		.gpio_in  = LED_TIM3_CH4IN,
		.timer_index = 2,
		.timer_channel = 4,
		.masks	= GTIM_SR_CC4IF | GTIM_SR_CC4OF,
		.ccr_offset = STM32_GTIM_CCR4_OFFSET,
	}
};


static void
led_pwm_timer_init(unsigned timer)
{
	/* enable the timer clock before we try to talk to it */
	modifyreg32(led_pwm_timers[timer].clock_register, 0, led_pwm_timers[timer].clock_bit);

	/* disable and configure the timer */
	rCR1(timer) = 0;
	rCR2(timer) = 0;
	rSMCR(timer) = 0;
	rDIER(timer) = 0;
	rCCER(timer) = 0;
	rCCMR1(timer) = 0;
	rCCMR2(timer) = 0;
	rCCER(timer) = 0;
	rDCR(timer) = 0;

	if ((led_pwm_timers[timer].base == STM32_TIM1_BASE) || (led_pwm_timers[timer].base == STM32_TIM8_BASE)) {
		/* master output enable = on */
		rBDTR(timer) = ATIM_BDTR_MOE;
	}

	/* configure the timer to free-run at 1MHz */
	rPSC(timer) = (led_pwm_timers[timer].clock_freq / 1000000) - 1;

	/* default to updating at 50Hz */
	led_pwm_timer_set_rate(timer, 50);

	/* note that the timer is left disabled - arming is performed separately */
}
unsigned
led_pwm_timer_get_period(unsigned timer)
{
	return (rARR(timer));
}
static void
led_pwm_timer_set_rate(unsigned timer, unsigned rate)
{
	/* configure the timer to update at the desired rate */
	rARR(timer) = 1000000 / rate;

	/* generate an update event; reloads the counter and all registers */
	rEGR(timer) = GTIM_EGR_UG;
}


static void
led_pwm_channel_init(unsigned channel)
{
	unsigned timer = led_pwm_channels[channel].timer_index;

	/* configure the GPIO first */
	stm32_configgpio(led_pwm_channels[channel].gpio_out);

	/* configure the channel */
	switch (led_pwm_channels[channel].timer_channel) {
	case 1:
		rCCMR1(timer) |= (GTIM_CCMR_MODE_PWM1 << GTIM_CCMR1_OC1M_SHIFT) | GTIM_CCMR1_OC1PE;
		//rCCR1(timer) = led_pwm_channels[channel].default_value;
		rCCER(timer) |= GTIM_CCER_CC1E;
		break;

	case 2:
		rCCMR1(timer) |= (GTIM_CCMR_MODE_PWM1 << GTIM_CCMR1_OC2M_SHIFT) | GTIM_CCMR1_OC2PE;
		//rCCR2(timer) = led_pwm_channels[channel].default_value;
		rCCER(timer) |= GTIM_CCER_CC2E;
		break;

	case 3:
		rCCMR2(timer) |= (GTIM_CCMR_MODE_PWM1 << GTIM_CCMR2_OC3M_SHIFT) | GTIM_CCMR2_OC3PE;
		//rCCR3(timer) = led_pwm_channels[channel].default_value;
		rCCER(timer) |= GTIM_CCER_CC3E;
		break;

	case 4:
		rCCMR2(timer) |= (GTIM_CCMR_MODE_PWM1 << GTIM_CCMR2_OC4M_SHIFT) | GTIM_CCMR2_OC4PE;
		//rCCR4(timer) = led_pwm_channels[channel].default_value;
		rCCER(timer) |= GTIM_CCER_CC4E;
		break;
	}
}

int
led_pwm_servo_set(unsigned channel, uint8_t  cvalue)
{
	if (channel >= 3) {
		return -1;
	}

	unsigned timer = led_pwm_channels[channel].timer_index;

	/* test timer for validity */
	if ((led_pwm_timers[timer].base == 0) ||
	    (led_pwm_channels[channel].gpio_out == 0)) {
		return -1;
	}

	unsigned period = led_pwm_timer_get_period(timer);

	unsigned value = period - (unsigned)cvalue * period / 255;

	/* configure the channel */
	if (value > 0) {
		value--;
	}


	switch (led_pwm_channels[channel].timer_channel) {
	case 1:
		rCCR1(timer) = value;
		break;

	case 2:
		rCCR2(timer) = value;
		break;

	case 3:
		rCCR3(timer) = value;
		break;

	case 4:
		rCCR4(timer) = value;
		break;

	default:
		return -1;
	}

	return 0;
}
unsigned
led_pwm_servo_get(unsigned channel)
{
	if (channel >= 3) {
		return 0;
	}

	unsigned timer = led_pwm_channels[channel].timer_index;
	servo_position_t value = 0;

	/* test timer for validity */
	if ((led_pwm_timers[timer].base == 0) ||
	    (led_pwm_channels[channel].timer_channel == 0)) {
		return 0;
	}

	/* configure the channel */
	switch (led_pwm_channels[channel].timer_channel) {
	case 1:
		value = rCCR1(timer);
		break;

	case 2:
		value = rCCR2(timer);
		break;

	case 3:
		value = rCCR3(timer);
		break;

	case 4:
		value = rCCR4(timer);
		break;
	}

	unsigned period = led_pwm_timer_get_period(timer);
	return ((value + 1) * 255 / period);
}
int
led_pwm_servo_init(void)
{
	/* do basic timer initialisation first */
	for (unsigned i = 0; i < 3; i++) {
		led_pwm_timer_init(i);
	}

	/* now init channels */
	for (unsigned i = 0; i < 3; i++) {
		led_pwm_channel_init(i);
	}

	led_pwm_servo_arm(true);
	return OK;
}

void
led_pwm_servo_deinit(void)
{
	/* disable the timers */
	led_pwm_servo_arm(false);
}
void
led_pwm_servo_arm(bool armed)
{
	/* iterate timers and arm/disarm appropriately */
	for (unsigned i = 0; i < 3; i++) {
		if (led_pwm_timers[i].base != 0) {
			if (armed) {
				/* force an update to preload all registers */
				rEGR(i) = GTIM_EGR_UG;

				/* arm requires the timer be enabled */
				rCR1(i) |= GTIM_CR1_CEN | GTIM_CR1_ARPE;

			} else {
				// XXX This leads to FMU PWM being still active
				// but uncontrollable. Just disable the timer
				// and risk a runt.
				///* on disarm, just stop auto-reload so we don't generate runts */
				//rCR1(i) &= ~GTIM_CR1_ARPE;
				rCR1(i) = 0;
			}
		}
	}
}

