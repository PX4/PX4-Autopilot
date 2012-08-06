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
 * @file Servo driver supporting PWM servos connected to STM32 timer blocks.
 *
 * Works with any of the 'generic' or 'advanced' STM32 timers that
 * have output pins, does not require an interrupt.
 */

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <sys/types.h>
#include <stdbool.h>

#include <debug.h>
#include <errno.h>

#include <arch/board/board.h>
#include <arch/board/drv_pwm_servo.h>

#include "stm32_gpio.h"
#include "stm32_tim.h"

#ifdef CONFIG_PWM_SERVO

static const struct pwm_servo_config	*cfg;

#define REG(_tmr, _reg)	(*(volatile uint32_t *)(cfg->timers[_tmr].base + _reg))

#define rCR1(_tmr)    	REG(_tmr, STM32_GTIM_CR1_OFFSET)
#define rCR2(_tmr)    	REG(_tmr, STM32_GTIM_CR2_OFFSET)
#define rSMCR(_tmr)   	REG(_tmr, STM32_GTIM_SMCR_OFFSET)
#define rDIER(_tmr)   	REG(_tmr, STM32_GTIM_DIER_OFFSET)
#define rSR(_tmr)     	REG(_tmr, STM32_GTIM_SR_OFFSET)
#define rEGR(_tmr)    	REG(_tmr, STM32_GTIM_EGR_OFFSET)
#define rCCMR1(_tmr)  	REG(_tmr, STM32_GTIM_CCMR1_OFFSET)
#define rCCMR2(_tmr)  	REG(_tmr, STM32_GTIM_CCMR2_OFFSET)
#define rCCER(_tmr)   	REG(_tmr, STM32_GTIM_CCER_OFFSET)
#define rCNT(_tmr)    	REG(_tmr, STM32_GTIM_CNT_OFFSET)
#define rPSC(_tmr)    	REG(_tmr, STM32_GTIM_PSC_OFFSET)
#define rARR(_tmr)    	REG(_tmr, STM32_GTIM_ARR_OFFSET)
#define rCCR1(_tmr)   	REG(_tmr, STM32_GTIM_CCR1_OFFSET)
#define rCCR2(_tmr)   	REG(_tmr, STM32_GTIM_CCR2_OFFSET)
#define rCCR3(_tmr)   	REG(_tmr, STM32_GTIM_CCR3_OFFSET)
#define rCCR4(_tmr)   	REG(_tmr, STM32_GTIM_CCR4_OFFSET)
#define rDCR(_tmr)    	REG(_tmr, STM32_GTIM_DCR_OFFSET)
#define rDMAR(_tmr)   	REG(_tmr, STM32_GTIM_DMAR_OFFSET)

static void
pwm_timer_init(unsigned timer)
{
	/* enable the timer clock before we try to talk to it */
	modifyreg32(cfg->timers[timer].clock_register, 0, cfg->timers[timer].clock_bit);

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

	/* configure the timer to free-run at 1MHz */
	rPSC(timer) = (cfg->timers[timer].clock_freq / 1000000) -1;

	/* and update at the desired rate */
	rARR(timer) = (1000000 / cfg->update_rate) - 1;

	/* generate an update event; reloads the counter and all registers */
	rEGR(timer) = GTIM_EGR_UG;

	/* note that the timer is left disabled - arming is performed separately */
}

static void
pwm_servos_arm(bool armed)
{
	/* iterate timers and arm/disarm appropriately */
	for (unsigned i = 0; i < PWM_SERVO_MAX_TIMERS; i++) {
		if (cfg->timers[i].base != 0)
			rCR1(i) = armed ? GTIM_CR1_CEN : 0;
	}
}

static void
pwm_channel_init(unsigned channel)
{
	unsigned timer = cfg->channels[channel].timer_index;

	/* configure the GPIO first */
	stm32_configgpio(cfg->channels[channel].gpio);

	/* configure the channel */
	switch (cfg->channels[channel].timer_channel) {
		case 1:
			rCCMR1(timer) |= (6 << 4);
			rCCR1(timer) = cfg->channels[channel].default_value;
			rCCER(timer) |= (1 << 0);
			break;
		case 2:
			rCCMR1(timer) |= (6 << 12);
			rCCR2(timer) = cfg->channels[channel].default_value;
			rCCER(timer) |= (1 << 4);
			break;
		case 3:
			rCCMR2(timer) |= (6 << 4);
			rCCR3(timer) = cfg->channels[channel].default_value;
			rCCER(timer) |= (1 << 8);
			break;
		case 4:
			rCCMR2(timer) |= (6 << 12);
			rCCR4(timer) = cfg->channels[channel].default_value;
			rCCER(timer) |= (1 << 12);
			break;
	}
}

static void
pwm_channel_set(unsigned channel, servo_position_t value)
{
	if (channel >= PWM_SERVO_MAX_CHANNELS) {
		lldbg("pwm_channel_set: bogus channel %u\n", channel);
		return;
	}

	unsigned timer = cfg->channels[channel].timer_index;

	/* test timer for validity */
	if ((cfg->timers[timer].base == 0) ||
	    (cfg->channels[channel].gpio == 0))
		return;

	/* configure the channel */
	if (value > 0)
		value--;
	switch (cfg->channels[channel].timer_channel) {
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
	}
}

static servo_position_t
pwm_channel_get(unsigned channel)
{
	if (channel >= PWM_SERVO_MAX_CHANNELS) {
		lldbg("pwm_channel_get: bogus channel %u\n", channel);
		return 0;
	}

	unsigned timer = cfg->channels[channel].timer_index;
	servo_position_t value = 0;

	/* test timer for validity */
	if ((cfg->timers[timer].base == 0) ||
	    (cfg->channels[channel].gpio == 0))
		return 0;

	/* configure the channel */
	switch (cfg->channels[channel].timer_channel) {
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
	return value;
}

static int	pwm_servo_write(struct file *filp, const char *buffer, size_t len);
static int	pwm_servo_read(struct file *filp, char *buffer, size_t len);
static int	pwm_servo_ioctl(struct file *filep, int cmd, unsigned long arg);

static const struct file_operations pwm_servo_fops = {
	.write = pwm_servo_write,
	.read = pwm_servo_read,
	.ioctl = pwm_servo_ioctl
};

static int
pwm_servo_write(struct file *filp, const char *buffer, size_t len)
{
	unsigned channels = len / sizeof(servo_position_t);
	servo_position_t *pdata = (servo_position_t *)buffer;
	unsigned i;

	if (channels > PWM_SERVO_MAX_CHANNELS)
		return -EIO;

	for (i = 0; i < channels; i++)
		pwm_channel_set(i, pdata[i]);

	return i * sizeof(servo_position_t);
}

static int
pwm_servo_read(struct file *filp, char *buffer, size_t len)
{
	unsigned channels = len / sizeof(servo_position_t);
	servo_position_t *pdata = (servo_position_t *)buffer;
	unsigned i;

	if (channels > PWM_SERVO_MAX_CHANNELS)
		return -EIO;

	for (i = 0; i < channels; i++)
		pdata[i] = pwm_channel_get(i);

	return i * sizeof(servo_position_t);
}

static int
pwm_servo_ioctl(struct file *filep, int cmd, unsigned long arg)
{
	/* regular ioctl? */
	switch (cmd) {
		case PWM_SERVO_ARM:
			pwm_servos_arm(true);
			return 0;

		case PWM_SERVO_DISARM:
			pwm_servos_arm(false);
			return 0;
	}

	/* channel set? */
	if ((cmd >= PWM_SERVO_SET(0)) && (cmd < PWM_SERVO_SET(PWM_SERVO_MAX_CHANNELS))) {
		/* XXX sanity-check value? */
		pwm_channel_set(cmd - PWM_SERVO_SET(0), (servo_position_t)arg);
		return 0;
	}

	/* channel get? */
	if ((cmd >= PWM_SERVO_GET(0)) && (cmd < PWM_SERVO_GET(PWM_SERVO_MAX_CHANNELS))) {
		/* XXX sanity-check value? */
		*(servo_position_t *)arg = pwm_channel_get(cmd - PWM_SERVO_GET(0));
		return 0;
	}

	/* not a recognised value */
	return -ENOTTY;
}


int
pwm_servo_init(const struct pwm_servo_config *config)
{
	/* save a pointer to the configuration */
	cfg = config;

	/* do basic timer initialisation first */
	for (unsigned i = 0; i < PWM_SERVO_MAX_TIMERS; i++) {
		if (cfg->timers[i].base != 0)
			pwm_timer_init(i);
	}

	/* now init channels */
	for (unsigned i = 0; i < PWM_SERVO_MAX_CHANNELS; i++) {
		if (cfg->channels[i].gpio != 0)
			pwm_channel_init(i);
	}

	/* register the device */
	return register_driver("/dev/pwm_servo", &pwm_servo_fops, 0666, NULL);
}

#endif /* CONFIG_PWM_SERVO */