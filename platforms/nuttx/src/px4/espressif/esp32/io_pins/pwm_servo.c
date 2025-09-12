/****************************************************************************
 *
 *   Copyright (C) 2012, 2017 PX4 Development Team. All rights reserved.
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
 * @file drv_pwm_servo.c
 *
 * Servo driver supporting PWM servos connected to STM32 timer blocks.
 *
 * Works with any of the 'generic' or 'advanced' STM32 timers that
 * have output pins, does not require an interrupt.
 */

#include <px4_platform_common/px4_config.h>
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

// #include <esp32_ledc.h>
#include "xtensa.h"


#define DR_REG_DPORT_BASE                       0x3ff00000
#define DPORT_PERIP_CLK_EN_REG          (DR_REG_DPORT_BASE + 0x0C0)
#define DPORT_PERIP_RST_EN_REG          (DR_REG_DPORT_BASE + 0x0C4)

#define DR_REG_LEDC_BASE                0x3ff59000
#define LEDC_LSTIMER0_CONF_REG          (DR_REG_LEDC_BASE + 0x0160)
#define LEDC_LSTIMER1_CONF_REG          (DR_REG_LEDC_BASE + 0x0168)
#define LEDC_LSCH1_CONF0_REG          (DR_REG_LEDC_BASE + 0x00B4)
#define LEDC_LSCH0_CONF0_REG          (DR_REG_LEDC_BASE + 0x00A0)
#define LEDC_LSCH0_CONF1_REG          (DR_REG_LEDC_BASE + 0x00AC)
#define LEDC_LSCH0_HPOINT_REG          (DR_REG_LEDC_BASE + 0x00A4)
#define LEDC_LSCH0_DUTY_REG          (DR_REG_LEDC_BASE + 0x00A8)
#define LEDC_CONF_REG          (DR_REG_LEDC_BASE + 0x0190)

#define LEDC_SIG_OUT_EN_LSCH0  1 << 2
#define LEDC_PARA_UP_LSCH0  1 << 4
#define DPORT_LEDC_CLK_EN   1 << 11
#define DPORT_LEDC_RST   1 << 11
#define LEDC_LSTIMER0_PAUSE  1 << 23
#define LEDC_LSTIMER0_RST  1 << 24
#define LEDC_TICK_SEL_LSTIMER0  1 << 25
#define LEDC_LSTIMER0_PARA_UP  1 << 26
#define LEDC_DUTY_START_LSCH0  1 << 31

#define LEDC_DIV_NUM_LSTIMER0_S  5
#define LEDC_LSTIMER0_DUTY_RES_S  0

#define b16HALF         0x00008000               /* 0.5 */
#define b16toi(a)       ((a) >> 16)              /* Conversion to integer */

/* LEDC clock resource */
#define LEDC_CLK_RES              (1)         /* APB clock */

/* LEDC timer max reload */
#define LEDC_RELOAD_MAX           (1048576)   /* 2^20 */

/* LEDC timer max clock divider parameter */
#define LEDC_CLKDIV_MAX           (1024)      /* 2^10 */

/* LEDC timer registers mapping */
#define LEDC_TIMER_REG(r, n)      ((r) + (n) * (LEDC_LSTIMER1_CONF_REG - LEDC_LSTIMER0_CONF_REG))

/* LEDC timer channel registers mapping */
#define setbits(bs, a)            modifyreg32(a, 0, bs)
#define resetbits(bs, a)          modifyreg32(a, bs, 0)

#define LEDC_CHAN_REG(r, n)       ((r) + (n) * (LEDC_LSCH1_CONF0_REG - LEDC_LSCH0_CONF0_REG))

#define SET_TIMER_BITS(t, r, b)   setbits(b, LEDC_TIMER_REG(r, t));
#define SET_TIMER_REG(t, r, v)    putreg32(v, LEDC_TIMER_REG(r, t));

#define SET_CHAN_BITS(c, r, b)    setbits(b, LEDC_CHAN_REG(r, c));
#define SET_CHAN_REG(c, r, v)     putreg32(v, LEDC_CHAN_REG(r, c));

uint32_t reload = 0;
uint32_t prescaler = 0;
uint32_t shift = 0;
uint32_t timer_rate = 0;

void get_optimal_timer_setup(uint32_t desired_freq)
{

	uint32_t shifted = 1;
	timer_rate = desired_freq;
	uint64_t pwm_clk = 80000000;
	reload = (pwm_clk * 256 / desired_freq + LEDC_CLKDIV_MAX) / LEDC_CLKDIV_MAX;

	if (reload == 0) {
		reload = 1;

	} else if (reload > LEDC_RELOAD_MAX) {
		reload = LEDC_RELOAD_MAX;
	}

	for (uint32_t c = 2; c <= LEDC_RELOAD_MAX; c *= 2) {
		if (c * 2 > reload) {
			reload = c;
			break;
		}

		shifted++;
	}

	shift = shifted;
	prescaler = (pwm_clk * 256 / reload) / desired_freq;
}

int up_pwm_servo_set(unsigned channel, uint16_t value)
{
	uint32_t duty = (value * timer_rate) * 0.065535;
	uint32_t regval = b16toi(duty * reload + b16HALF);

	irqstate_t flags;
	flags = px4_enter_critical_section();
	SET_CHAN_REG(channel, LEDC_LSCH0_CONF0_REG, 0);
	SET_CHAN_REG(channel, LEDC_LSCH0_CONF1_REG, 0);

	/* Set pulse phase 0 */
	SET_CHAN_REG(channel, LEDC_LSCH0_HPOINT_REG, 0);
	SET_CHAN_REG(channel, LEDC_LSCH0_DUTY_REG, regval << 4);

	SET_CHAN_BITS(channel, LEDC_LSCH0_CONF0_REG, LEDC_SIG_OUT_EN_LSCH0);

	/* Start Duty counter  */
	SET_CHAN_BITS(channel, LEDC_LSCH0_CONF1_REG, LEDC_DUTY_START_LSCH0);

	/* Update duty and phase to hardware */
	SET_CHAN_BITS(channel, LEDC_LSCH0_CONF0_REG, LEDC_PARA_UP_LSCH0);


	px4_leave_critical_section(flags);

	return OK;
}

uint16_t up_pwm_servo_get(unsigned channel)
{
	return 0;
}

int up_pwm_servo_init(uint32_t channel_mask)
{
	// pause the timer
	SET_TIMER_BITS(io_timers[0].base, LEDC_LSTIMER0_CONF_REG, LEDC_LSTIMER0_PAUSE);
	// reset the timer
	SET_TIMER_BITS(io_timers[0].base, LEDC_LSTIMER0_CONF_REG, LEDC_LSTIMER0_RST);

	// We are going to use REF_Tick as our source, which is set at 1Mhz.
	// our prescaler is thus 0
	irqstate_t flags;
	prescaler = 0;
	shift = 0;
	get_optimal_timer_setup(400);

	flags = px4_enter_critical_section();

	setbits(DPORT_LEDC_CLK_EN, DPORT_PERIP_CLK_EN_REG);
	resetbits(DPORT_LEDC_RST, DPORT_PERIP_RST_EN_REG);
	putreg32(LEDC_CLK_RES, LEDC_CONF_REG);

	uint32_t regval = (shift << LEDC_LSTIMER0_DUTY_RES_S) | (prescaler << LEDC_DIV_NUM_LSTIMER0_S);
	SET_TIMER_REG(io_timers[0].base, LEDC_LSTIMER0_CONF_REG, regval);

	/* Setup to timer to use APB clock (80MHz) */
	SET_TIMER_BITS(io_timers[0].base, LEDC_LSTIMER0_CONF_REG, LEDC_TICK_SEL_LSTIMER0);

	/* Update clock divide and reload to hardware */
	SET_TIMER_BITS(io_timers[0].base, LEDC_LSTIMER0_CONF_REG, LEDC_LSTIMER0_PARA_UP);

	px4_leave_critical_section(flags);

	return channel_mask;
}

void up_pwm_servo_deinit(uint32_t channel_mask)
{
	/* disable the timers */
	up_pwm_servo_arm(false, channel_mask);
}

int up_pwm_servo_set_rate_group_update(unsigned group, unsigned rate)
{
	if (group == 0 || group == 1 || group == 2 || group == 3) {
		irqstate_t flags;
		shift = 0;
		prescaler = 0;
		get_optimal_timer_setup(rate);

		flags = px4_enter_critical_section();

		uint32_t regval = (shift << LEDC_LSTIMER0_DUTY_RES_S) | (prescaler << LEDC_DIV_NUM_LSTIMER0_S);
		SET_TIMER_REG(io_timers[0].base, LEDC_LSTIMER0_CONF_REG, regval);

		/* Setup to timer to use APB clock (80MHz) */
		SET_TIMER_BITS(io_timers[0].base, LEDC_LSTIMER0_CONF_REG, LEDC_TICK_SEL_LSTIMER0);

		/* Update clock divide and reload to hardware */
		SET_TIMER_BITS(io_timers[0].base, LEDC_LSTIMER0_CONF_REG, LEDC_LSTIMER0_PARA_UP);

		px4_leave_critical_section(flags);

		return OK;

	}

	return ERROR;
}

void up_pwm_update(unsigned channels_mask)
{
	//syslog(LOG_INFO, "up_pwm_update channels_mask: %d\n", channels_mask);
	// pwm->ops->start(pwm,&pwm_info);
}

uint32_t up_pwm_servo_get_rate_group(unsigned group)
{

	if (group == 0) {
#if defined(CONFIG_ESP32_LEDC_TIM0_CHANNELS)
		return (1 << CONFIG_ESP32_LEDC_TIM0_CHANNELS) - 1;
#endif
		return -1;

	} else if (group == 1) {

#if defined(CONFIG_ESP32_LEDC_TIM1_CHANNELS)
		return (1 << CONFIG_ESP32_LEDC_TIM1_CHANNELS) - 1;
#endif
		return -1;

	} else if (group == 2) {
#if defined(CONFIG_ESP32_LEDC_TIM2_CHANNELS)
		return (1 << CONFIG_ESP32_LEDC_TIM2_CHANNELS) - -1;
#endif
		return -1;

	} else if (group == 3) {
#if defined(CONFIG_ESP32_LEDC_TIM3_CHANNELS)
		return (1 << CONFIG_ESP32_LEDC_TIM3_CHANNELS) - 1;
#endif
		return -1;
	}

	return -1;
}

void
up_pwm_servo_arm(bool armed, uint32_t channel_mask)
{
	if (armed) {
		for (uint8_t chan = 0; chan < 4; chan++) {

			irqstate_t flags;
			flags = px4_enter_critical_section();

			/* Reset config 0 & 1 registers */
			SET_CHAN_REG(chan, LEDC_LSCH0_CONF0_REG, 0);
			SET_CHAN_REG(chan, LEDC_LSCH0_CONF1_REG, 0);

			/* Set pulse phase 0 */
			SET_CHAN_REG(chan, LEDC_LSCH0_HPOINT_REG, 0);

			/* Start GPIO output  */
			SET_CHAN_BITS(chan, LEDC_LSCH0_CONF0_REG, LEDC_SIG_OUT_EN_LSCH0);

			/* Start Duty counter  */
			SET_CHAN_BITS(chan, LEDC_LSCH0_CONF1_REG, LEDC_DUTY_START_LSCH0);

			/* Update duty and phase to hardware */
			SET_CHAN_BITS(chan, LEDC_LSCH0_CONF0_REG, LEDC_PARA_UP_LSCH0);

			px4_leave_critical_section(flags);
		}

	} else {
		irqstate_t flags;

		flags = px4_enter_critical_section();

		/* Stop timer */
		SET_TIMER_BITS(io_timers[0].base, LEDC_LSTIMER0_CONF_REG, LEDC_LSTIMER0_PAUSE);

		/* Reset timer */
		SET_TIMER_BITS(io_timers[0].base, LEDC_LSTIMER0_CONF_REG, LEDC_LSTIMER0_RST);

		px4_leave_critical_section(flags);
	}
}
