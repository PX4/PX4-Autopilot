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

#include "esp32_ledc.h"

struct pwm_info_s pwm_info;
struct pwm_lowerhalf_s *pwm;

// duty cycle = duty / 65536 * reload (fractional value)
int up_pwm_servo_set(unsigned channel, uint16_t value)
{
	//syslog(LOG_INFO, "PWM set ch: %d value:%d\n", channel,value);
	// pwm_info.channels[channel].duty = (value*pwm_info.frequency)/(1000000/65535);
	pwm_info.channels[channel].duty = value/((1/pwm_info.frequency)*1e6);
	// pwm->ops->start(pwm, &pwm_info);
	return OK;
}

uint16_t up_pwm_servo_get(unsigned channel)
{
	return (pwm_info.frequency * pwm_info.channels[channel].duty)*1e-6f;
}

int up_pwm_servo_init(uint32_t channel_mask)
{

  	pwm = esp32_ledc_init(io_timers[0].base);

  	if (!pwm)
    	{
      		syslog(LOG_ERR, "[boot] Failed to get the LEDC PWM 0 lower half\n");
    	}

	pwm->ops->setup(pwm);

	pwm_info.frequency=50;
	pwm_info.channels[0].duty = 80;//channel_mask & 0b1 ? 0.5 : 0.0;
	pwm_info.channels[1].duty = channel_mask  & 0b10 ? 0.5 : 0.0;
	pwm_info.channels[2].duty = channel_mask & 0b100 ? 0.5 : 0.0;
	pwm_info.channels[3].duty = channel_mask & 0b1000 ? 0.5 : 0.0;
	pwm->ops->start(pwm, &pwm_info);

	return channel_mask;
}

void up_pwm_servo_deinit(uint32_t channel_mask)
{
	/* disable the timers */
	up_pwm_servo_arm(false, channel_mask);
}

int up_pwm_servo_set_rate_group_update(unsigned group, unsigned rate)
{
	if(group == 0 || group == 1 || group == 2 || group == 3)
	{
		printf("new rate: %d\n", rate);
		pwm_info.frequency = rate;
		pwm->ops->start(pwm, &pwm_info);
		return OK;
	}
	return ERROR;
}

void up_pwm_update(unsigned channels_mask)
{
	//syslog(LOG_INFO, "up_pwm_update channels_mask: %d\n", channels_mask);
	pwm->ops->start(pwm,&pwm_info);
}

uint32_t up_pwm_servo_get_rate_group(unsigned group)
{

	if(group == 0){
		#if defined(CONFIG_ESP32_LEDC_TIM0_CHANNELS)
			return (1 << CONFIG_ESP32_LEDC_TIM0_CHANNELS) - 1;
		#endif
		return -1;
	}
	else if(group == 1){

		#if defined(CONFIG_ESP32_LEDC_TIM1_CHANNELS)
			return ( 1 << CONFIG_ESP32_LEDC_TIM1_CHANNELS) -1;
		#endif
		return -1;
	}
	else if(group == 2){
		#if defined(CONFIG_ESP32_LEDC_TIM2_CHANNELS)
			return (1 << CONFIG_ESP32_LEDC_TIM2_CHANNELS) - -1;
		#endif
		return -1;
	}
	else if(group == 3){
		#if defined(CONFIG_ESP32_LEDC_TIM3_CHANNELS)
			return (1 << CONFIG_ESP32_LEDC_TIM3_CHANNELS) -1;
		#endif
		return -1;
	}

	return -1;
}

void
up_pwm_servo_arm(bool armed, uint32_t channel_mask)
{
	if(armed)
	{
		pwm->ops->start(pwm,&pwm_info);
	}else
	{
		pwm->ops->stop(pwm);
	}
}
